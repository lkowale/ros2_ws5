#!/usr/bin/env python3
"""
rs_controller_logger.py — CSV logger for RsPathController tuning.

Writes to ~/ros2_ws5/logs/m3_sim/rs_ctrl_<timestamp>.csv at 20 Hz.

Columns:
  time_sec, wall_time
  -- current job --
  job_path_index, job_path_label, job_dist_remaining
  -- robot pose (map frame via TF) --
  robot_x, robot_y, robot_yaw_deg
  tool_x, tool_y, tool_yaw_deg       (tool_link / rear axle)
  -- odometry --
  odom_x, odom_y, odom_yaw_deg, odom_vx, odom_wz
  -- GPS --
  gps_lat, gps_lon, gps_fix, gps_cov_x
  -- commands --
  cmd_vx, cmd_wz
  -- controller internals (from /rs_ctrl_debug) --
  ctrl_idx, ctrl_n, ctrl_rev
  ctrl_cte, ctrl_heading_err_deg, ctrl_stanley_deg
  ctrl_v_cmd, ctrl_w_cmd, ctrl_dist_to_end
  -- nav status --
  nav_status

Run:
  source ~/ros2_ws5/install/setup.bash
  python3 ~/ros2_ws5/rs_controller_logger.py
"""

import csv
import math
import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                        QoSProfile, QoSReliabilityPolicy)
from rclpy.time import Time as RosTime

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist as TwistPlain
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from solbot5_msgs.action import RunRsTest

_BEST_EFFORT = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

_STATUS_STRS = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}


def _quat_to_yaw_deg(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def _nan():
    return float('nan')


class RsCtrlLogger(Node):
    def __init__(self, csv_path: str):
        super().__init__('rs_controller_logger')
        self._lock = threading.Lock()

        # TF
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # state
        self._robot_x = _nan(); self._robot_y = _nan(); self._robot_yaw_deg = _nan()
        self._tool_x  = _nan(); self._tool_y  = _nan(); self._tool_yaw_deg  = _nan()

        self._odom_x = _nan(); self._odom_y = _nan(); self._odom_yaw_deg = _nan()
        self._odom_vx = _nan(); self._odom_wz = _nan()

        self._gps_lat = _nan(); self._gps_lon = _nan()
        self._gps_fix = -1;     self._gps_cov_x = _nan()

        self._cmd_vx = 0.0; self._cmd_wz = 0.0

        # current job (from RunRsTest action feedback)
        self._job_path_index   = -1
        self._job_path_label   = ''
        self._job_dist_remaining = _nan()

        # controller debug fields
        self._ctrl_idx       = -1
        self._ctrl_n         = -1
        self._ctrl_rev       = 0
        self._ctrl_cte       = _nan()
        self._ctrl_h_err     = _nan()
        self._ctrl_lookahead = _nan()
        self._ctrl_stanley   = _nan()
        self._ctrl_v_cmd     = _nan()
        self._ctrl_w_cmd     = _nan()
        self._ctrl_dist      = _nan()
        self._ctrl_dist_path = _nan()

        self._nav_status = ''

        # subscriptions
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._cb_gps, _BEST_EFFORT)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cb_cmd_stamped, 10)
        self.create_subscription(TwistPlain,   '/cmd_vel', self._cb_cmd_plain,   10)
        self.create_subscription(String, '/rs_ctrl_debug', self._cb_debug, 10)
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._cb_nav_status, 10)
        # RS test suite feedback — gives path index and label
        self.create_subscription(
            RunRsTest.Impl.FeedbackMessage,
            '/run_rs_test/_action/feedback',
            self._cb_rs_feedback, 10)

        # CSV
        self._csv_file = open(csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            'time_sec', 'wall_time',
            'job_path_index', 'job_path_label', 'job_dist_remaining',
            'robot_x', 'robot_y', 'robot_yaw_deg',
            'tool_x',  'tool_y',  'tool_yaw_deg',
            'odom_x',  'odom_y',  'odom_yaw_deg',
            'odom_vx', 'odom_wz',
            'gps_lat', 'gps_lon', 'gps_fix', 'gps_cov_x',
            'cmd_vx',  'cmd_wz',
            'ctrl_idx', 'ctrl_n', 'ctrl_rev',
            'ctrl_cte', 'ctrl_heading_err_deg', 'ctrl_lookahead',
            'ctrl_delta_deg', 'ctrl_v_cmd', 'ctrl_w_cmd',
            'ctrl_dist_to_end', 'ctrl_dist_to_path',
            'nav_status',
        ])

        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info(f'rs_controller_logger: writing to {csv_path}')

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            p = msg.pose.pose.position
            self._odom_x         = p.x
            self._odom_y         = p.y
            self._odom_yaw_deg   = _quat_to_yaw_deg(msg.pose.pose.orientation)
            self._odom_vx        = msg.twist.twist.linear.x
            self._odom_wz        = msg.twist.twist.angular.z

    def _cb_gps(self, msg: NavSatFix):
        with self._lock:
            self._gps_lat   = msg.latitude
            self._gps_lon   = msg.longitude
            self._gps_fix   = msg.status.status
            self._gps_cov_x = msg.position_covariance[0]

    def _cb_cmd_stamped(self, msg: TwistStamped):
        with self._lock:
            self._cmd_vx = msg.twist.linear.x
            self._cmd_wz = msg.twist.angular.z

    def _cb_cmd_plain(self, msg: TwistPlain):
        with self._lock:
            self._cmd_vx = msg.linear.x
            self._cmd_wz = msg.angular.z

    def _cb_debug(self, msg: String):
        # format: "idx,n,rev,cte,heading_err_deg,lookahead,delta_deg,v_cmd,w_cmd,dist_to_end,dist_to_path"
        try:
            parts = msg.data.split(',')
            with self._lock:
                self._ctrl_idx     = int(parts[0])
                self._ctrl_n       = int(parts[1])
                self._ctrl_rev     = int(parts[2])
                self._ctrl_cte     = float(parts[3])
                self._ctrl_h_err     = float(parts[4])   # already degrees
                self._ctrl_lookahead = float(parts[5])
                self._ctrl_stanley   = float(parts[6])   # delta_deg
                self._ctrl_v_cmd     = float(parts[7])
                self._ctrl_w_cmd     = float(parts[8])
                self._ctrl_dist      = float(parts[9])
                self._ctrl_dist_path = float(parts[10])
        except Exception:
            pass

    def _cb_nav_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        last = msg.status_list[-1]
        with self._lock:
            self._nav_status = _STATUS_STRS.get(last.status, str(last.status))

    def _cb_rs_feedback(self, msg: RunRsTest.Impl.FeedbackMessage):
        fb = msg.feedback
        with self._lock:
            self._job_path_index    = int(fb.current_goal_index)
            self._job_path_label    = fb.current_label
            self._job_dist_remaining = float(fb.distance_remaining)

    # ── TF lookups ─────────────────────────────────────────────────────────────

    def _update_tf(self):
        try:
            t = self._tf_buf.lookup_transform('map', 'base_footprint', RosTime())
            tr = t.transform.translation
            with self._lock:
                self._robot_x       = tr.x
                self._robot_y       = tr.y
                self._robot_yaw_deg = _quat_to_yaw_deg(t.transform.rotation)
        except TransformException:
            pass

        try:
            t = self._tf_buf.lookup_transform('map', 'tool_link', RosTime())
            tr = t.transform.translation
            with self._lock:
                self._tool_x       = tr.x
                self._tool_y       = tr.y
                self._tool_yaw_deg = _quat_to_yaw_deg(t.transform.rotation)
        except TransformException:
            pass

    # ── timer ──────────────────────────────────────────────────────────────────

    def _tick(self):
        self._update_tf()
        ros_sec = self.get_clock().now().nanoseconds / 1e9
        wall_time = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        with self._lock:
            self._writer.writerow([
                f'{ros_sec:.4f}',
                wall_time,
                self._job_path_index,
                self._job_path_label,
                f'{self._job_dist_remaining:.3f}',
                f'{self._robot_x:.4f}',   f'{self._robot_y:.4f}',
                f'{self._robot_yaw_deg:.3f}',
                f'{self._tool_x:.4f}',    f'{self._tool_y:.4f}',
                f'{self._tool_yaw_deg:.3f}',
                f'{self._odom_x:.4f}',    f'{self._odom_y:.4f}',
                f'{self._odom_yaw_deg:.3f}',
                f'{self._odom_vx:.4f}',   f'{self._odom_wz:.4f}',
                f'{self._gps_lat:.8f}',   f'{self._gps_lon:.8f}',
                self._gps_fix,            f'{self._gps_cov_x:.5f}',
                f'{self._cmd_vx:.4f}',    f'{self._cmd_wz:.4f}',
                self._ctrl_idx,           self._ctrl_n,
                self._ctrl_rev,
                f'{self._ctrl_cte:.5f}',
                f'{self._ctrl_h_err:.3f}',
                f'{self._ctrl_lookahead:.3f}',
                f'{self._ctrl_stanley:.3f}',
                f'{self._ctrl_v_cmd:.4f}', f'{self._ctrl_w_cmd:.4f}',
                f'{self._ctrl_dist:.3f}',  f'{self._ctrl_dist_path:.3f}',
                self._nav_status,
            ])
            self._csv_file.flush()

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main():
    log_dir = os.path.expanduser('~/ros2_ws5/logs/m3_sim')
    os.makedirs(log_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(log_dir, f'rs_ctrl_{ts}.csv')
    latest   = os.path.join(log_dir, 'rs_ctrl_latest.csv')
    try:
        os.unlink(latest)
    except FileNotFoundError:
        pass
    os.symlink(csv_path, latest)

    print(f'RS controller logger')
    print(f'CSV : {csv_path}')
    print(f'Link: {latest}')
    print()

    rclpy.init()
    node = RsCtrlLogger(csv_path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print(f'\nCSV saved: {csv_path}')


if __name__ == '__main__':
    main()
