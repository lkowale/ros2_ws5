#!/usr/bin/env python3
"""
m3_sim_logger.py — Thorough logger for the M3 RS-planner test suite.

Subscribes to every relevant topic and writes timestamped, structured
log lines to stdout AND to logs/m3_sim/logger_<timestamp>.log.

Logged domains:
  ODOM    – robot pose (map frame via TF), velocity, heading
  GPS     – NavSatFix fix quality, lat/lon, covariance
  EKF     – /odom topic (filtered odometry x/y/yaw/vel)
  NAV     – NavigateToPose action feedback (distance_remaining, recoveries)
  PLAN    – /plan_forward /plan_reverse path stats (length, n_poses, start/end)
  CTRL    – /cmd_vel (commanded twist to robot)
  GOAL    – goal_checker status via /navigate_to_pose/_action/status
  NAVSAT  – navsat health via /rosout filtered messages
  TF      – map→odom→base_footprint TF staleness warnings

Usage (separate terminal, sim already running):
    bash m3_sim_logger.sh           # wrapper that sources workspace
  or
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws5/install/setup.bash
    python3 ~/ros2_ws5/m3_sim_logger.py [--hz 2]

Options:
    --hz N   How often to sample ODOM/EKF/CTRL in Hz (default 2)
    --all    Log every sample (no rate-limiting)
"""
import argparse
import math
import os
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy)
from rclpy.time import Time as RosTime
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Log as RosoutLog
from sensor_msgs.msg import NavSatFix, Imu
from tf2_ros import Buffer, TransformListener, TransformException

# ── helpers ──────────────────────────────────────────────────────────────────

def _quat_to_yaw(q):
    """quaternion → yaw in degrees"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def _path_stats(path: Path):
    """Return (n_poses, length_m, start_xy, end_xy) for a nav_msgs/Path."""
    poses = path.poses
    n = len(poses)
    if n == 0:
        return 0, 0.0, None, None
    length = 0.0
    for i in range(1, n):
        dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
        dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
        length += math.hypot(dx, dy)
    s = poses[0].pose.position
    e = poses[-1].pose.position
    s_yaw = _quat_to_yaw(poses[0].pose.orientation)
    e_yaw = _quat_to_yaw(poses[-1].pose.orientation)
    return n, length, (s.x, s.y, s_yaw), (e.x, e.y, e_yaw)


_TRANSIENT_LOCAL = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

_BEST_EFFORT = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# ── logger node ───────────────────────────────────────────────────────────────

class M3Logger(Node):
    def __init__(self, sample_hz: float, log_all: bool, log_file):
        super().__init__('m3_sim_logger')
        self._log_file = log_file
        self._sample_period = 1.0 / sample_hz
        self._log_all = log_all
        self._lock = threading.Lock()

        # rate-limiting: last print times per category
        self._last_t: dict[str, float] = {}

        # TF
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._tf_warn_interval = 5.0
        self._last_tf_warn = 0.0

        # state cache
        self._ekf_odom: Odometry | None = None
        self._cmd_vel: TwistStamped | None = None
        self._gps_fix: NavSatFix | None = None

        # ── subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',
                                 self._cb_ekf_odom, 10)

        self.create_subscription(NavSatFix, '/gps/fix',
                                 self._cb_gps, _BEST_EFFORT)

        # cmd_vel may be TwistStamped or plain Twist depending on nav2 version
        try:
            self.create_subscription(TwistStamped, '/cmd_vel',
                                     self._cb_cmd_vel, 10)
        except Exception:
            pass
        # plain Twist fallback
        from geometry_msgs.msg import Twist
        self.create_subscription(Twist, '/cmd_vel',
                                 self._cb_cmd_vel_plain, 10)

        # RS planner paths — transient_local so we get them even after publish
        self.create_subscription(Path, '/plan_forward',
                                 self._cb_plan_forward, _TRANSIENT_LOCAL)
        self.create_subscription(Path, '/plan_reverse',
                                 self._cb_plan_reverse, _TRANSIENT_LOCAL)
        self.create_subscription(Path, '/plan_swath',
                                 self._cb_plan_swath, _TRANSIENT_LOCAL)
        self.create_subscription(Path, '/plan_turn',
                                 self._cb_plan_turn, _TRANSIENT_LOCAL)

        # NavigateToPose action status
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status',
                                 self._cb_nav_status, 10)

        # /rosout — filter for navsat / ekf / controller messages
        self.create_subscription(RosoutLog, '/rosout',
                                 self._cb_rosout, 100)

        # periodic sampler for ODOM/EKF/CTRL/TF
        self.create_timer(self._sample_period, self._sample_tick)

        self._log('INIT', f'logger started  sample_hz={sample_hz}  log_all={log_all}')
        self._log('INIT', f'subscribing to /odom /gps/fix /cmd_vel '
                           '/plan_forward /plan_reverse /plan_swath /plan_turn '
                           '/navigate_to_pose/_action/status /rosout')

    # ── logging util ─────────────────────────────────────────────────────────

    def _log(self, category: str, msg: str):
        now = datetime.now()
        ts = now.strftime('%H:%M:%S.%f')[:-3]
        line = f'[{ts}] [{category:8s}] {msg}'
        with self._lock:
            print(line, flush=True)
            if self._log_file:
                print(line, file=self._log_file, flush=True)

    def _should_log(self, key: str) -> bool:
        """Rate-limit a category to sample_period; always True if --all."""
        if self._log_all:
            return True
        now = time.monotonic()
        if now - self._last_t.get(key, 0.0) >= self._sample_period:
            self._last_t[key] = now
            return True
        return False

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_ekf_odom(self, msg: Odometry):
        self._ekf_odom = msg
        if self._should_log('ekf'):
            p = msg.pose.pose.position
            yaw = _quat_to_yaw(msg.pose.pose.orientation)
            v = msg.twist.twist.linear
            w = msg.twist.twist.angular.z
            cov_xx = msg.pose.covariance[0]
            cov_yy = msg.pose.covariance[7]
            self._log('EKF',
                f'x={p.x:8.3f}  y={p.y:8.3f}  yaw={yaw:7.2f}°  '
                f'vx={v.x:6.3f}  vy={v.y:6.3f}  wz={w:6.3f}  '
                f'cov_xy=({cov_xx:.4f},{cov_yy:.4f})')

    def _cb_gps(self, msg: NavSatFix):
        self._gps_fix = msg
        if self._should_log('gps'):
            status = msg.status.status   # -1=no fix, 0=fix, 1=sbas, 2=gbas
            status_str = {-1: 'NO_FIX', 0: 'FIX', 1: 'SBAS', 2: 'GBAS'}.get(
                status, str(status))
            cov_diag = (msg.position_covariance[0],
                        msg.position_covariance[4],
                        msg.position_covariance[8])
            self._log('GPS',
                f'status={status_str}  '
                f'lat={msg.latitude:.8f}  lon={msg.longitude:.8f}  '
                f'alt={msg.altitude:.2f}  '
                f'cov_diag=({cov_diag[0]:.4f},{cov_diag[1]:.4f},{cov_diag[2]:.4f})')

    def _cb_cmd_vel(self, msg: TwistStamped):
        self._cmd_vel = msg
        if self._should_log('ctrl'):
            t = msg.twist
            self._log('CTRL',
                f'linear=({t.linear.x:6.3f},{t.linear.y:6.3f},{t.linear.z:6.3f})  '
                f'angular=({t.angular.x:6.3f},{t.angular.y:6.3f},{t.angular.z:6.3f})')

    def _cb_cmd_vel_plain(self, msg):
        if self._should_log('ctrl'):
            self._log('CTRL',
                f'linear=({msg.linear.x:6.3f},{msg.linear.y:6.3f},{msg.linear.z:6.3f})  '
                f'angular=({msg.angular.x:6.3f},{msg.angular.y:6.3f},{msg.angular.z:6.3f})')

    def _cb_plan_forward(self, msg: Path):
        n, length, start, end = _path_stats(msg)
        if n == 0:
            self._log('PLAN', '/plan_forward  EMPTY')
            return
        self._log('PLAN',
            f'/plan_forward  n={n:4d}  len={length:7.3f}m  '
            f'start=({start[0]:.3f},{start[1]:.3f},{start[2]:.1f}°)  '
            f'end=({end[0]:.3f},{end[1]:.3f},{end[2]:.1f}°)')

    def _cb_plan_reverse(self, msg: Path):
        n, length, start, end = _path_stats(msg)
        if n == 0:
            self._log('PLAN', '/plan_reverse  EMPTY')
            return
        self._log('PLAN',
            f'/plan_reverse  n={n:4d}  len={length:7.3f}m  '
            f'start=({start[0]:.3f},{start[1]:.3f},{start[2]:.1f}°)  '
            f'end=({end[0]:.3f},{end[1]:.3f},{end[2]:.1f}°)')

    def _cb_plan_swath(self, msg: Path):
        n, length, start, end = _path_stats(msg)
        self._log('PLAN',
            f'/plan_swath   n={n:4d}  len={length:7.3f}m  '
            + (f'start=({start[0]:.3f},{start[1]:.3f},{start[2]:.1f}°)  '
               f'end=({end[0]:.3f},{end[1]:.3f},{end[2]:.1f}°)' if start else 'EMPTY'))

    def _cb_plan_turn(self, msg: Path):
        n, length, start, end = _path_stats(msg)
        self._log('PLAN',
            f'/plan_turn    n={n:4d}  len={length:7.3f}m  '
            + (f'start=({start[0]:.3f},{start[1]:.3f},{start[2]:.1f}°)  '
               f'end=({end[0]:.3f},{end[1]:.3f},{end[2]:.1f}°)' if start else 'EMPTY'))

    _STATUS_STRS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING',
                    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

    def _cb_nav_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        for gs in msg.status_list:
            s = self._STATUS_STRS.get(gs.status, str(gs.status))
            uid = gs.goal_info.goal_id.uuid.tobytes().hex()[:8]
            if self._should_log(f'nav_status_{uid}'):
                self._log('NAV',
                    f'goal {uid}  status={s}({gs.status})')

    def _cb_rosout(self, msg: RosoutLog):
        # Capture navsat_transform, ekf, controller_server, planner_server messages
        name = msg.name.lower()
        text = msg.msg
        keywords = ('navsat', 'ekf', 'controller_server', 'planner_server',
                    'bt_navigator', 'goal_checker', 'progress_checker',
                    'reeds', 'swath', 'followpath', 'recovery', 'oscillat',
                    'reached', 'succeeded', 'aborted', 'canceled', 'failed',
                    'heading factor', 'corrected', 'transform')
        if any(kw in name or kw in text.lower() for kw in keywords):
            level_str = {10: 'DEBUG', 20: 'INFO', 30: 'WARN',
                         40: 'ERROR', 50: 'FATAL'}.get(msg.level, str(msg.level))
            self._log('ROSOUT',
                f'[{level_str}] [{msg.name}] {text}')

    # ── periodic sampler (TF + summary) ──────────────────────────────────────

    def _sample_tick(self):
        self._log_tf_pose()

    def _log_tf_pose(self):
        try:
            # map → base_footprint (full localization pose)
            t_map = self._tf_buf.lookup_transform(
                'map', 'base_footprint', RosTime())
            tr = t_map.transform.translation
            ro = t_map.transform.rotation
            yaw = _quat_to_yaw(ro)
            age_s = (self.get_clock().now() -
                     RosTime.from_msg(t_map.header.stamp)).nanoseconds * 1e-9
            if self._should_log('tf_map'):
                self._log('ODOM',
                    f'map→base  x={tr.x:8.3f}  y={tr.y:8.3f}  yaw={yaw:7.2f}°  '
                    f'tf_age={age_s*1000:.1f}ms')

            # odom → base_footprint (raw odometry pose)
            t_odom = self._tf_buf.lookup_transform(
                'odom', 'base_footprint', RosTime())
            tr2 = t_odom.transform.translation
            yaw2 = _quat_to_yaw(t_odom.transform.rotation)
            if self._should_log('tf_odom'):
                self._log('ODOM',
                    f'odom→base x={tr2.x:8.3f}  y={tr2.y:8.3f}  yaw={yaw2:7.2f}°')

        except TransformException as e:
            now_mono = time.monotonic()
            if now_mono - self._last_tf_warn > self._tf_warn_interval:
                self._last_tf_warn = now_mono
                self._log('TF', f'TF lookup failed: {e}')


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='M3 sim thorough logger')
    parser.add_argument('--hz', type=float, default=2.0,
                        help='Sample rate for ODOM/EKF/CTRL (Hz, default 2)')
    parser.add_argument('--all', dest='log_all', action='store_true',
                        help='Log every message without rate-limiting')
    args = parser.parse_args()

    log_dir = os.path.expanduser('~/ros2_ws5/logs/m3_sim')
    os.makedirs(log_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(log_dir, f'logger_{ts}.log')
    latest_path = os.path.join(log_dir, 'logger_latest.log')

    log_file = open(log_path, 'w', buffering=1)
    try:
        os.unlink(latest_path)
    except FileNotFoundError:
        pass
    os.symlink(log_path, latest_path)

    print(f'M3 sim logger')
    print(f'Log file : {log_path}')
    print(f'Symlink  : {latest_path}')
    print(f'Sample   : {args.hz} Hz  log_all={args.log_all}')
    print(f'Ctrl-C to stop.')
    print()

    rclpy.init()
    node = M3Logger(sample_hz=args.hz, log_all=args.log_all, log_file=log_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        log_file.close()
        print(f'\nLog saved to {log_path}')


if __name__ == '__main__':
    main()
