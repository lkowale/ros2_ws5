#!/usr/bin/env python3
"""
heading_fuser — offset-based IMU/GPS heading fusion.

Maintains a single IMU-offset: ground_heading = imu_yaw_raw + offset.

Offset calibration sources:

  1. BOOTSTRAP — at startup, seed offset from BNO080 absolute orientation to keep
       TF tree connected.  Then wait for 1 m straight travel (low yaw rate +
       fresh GPS heading) to compute a GPS-based offset.

  2. SWATH BLIND — triggered by /swath_path arrival (published by RunSwathAction
       before the controller starts, so it fires at true swath start).  Published
       heading is locked to the path bearing for the entire swath — IMU
       integration continues but its output is ignored.  EKF sees a perfectly
       constant heading so navsat_transform cannot perturb GPS position mid-swath.
       On swath exit (/to_start_path or turn-end) offset is re-synced to the
       swath bearing so IMU integration resumes from a known-good base.

  3. LINE SNAP — while following to_start_path (approach), snap every 2 m of
       travel with cross-track < 0.15 m.  Disabled during swath and turn.

During TURN (MoveSequence active): offset frozen, no snaps.

Subscribes:
    /imu                          sensor_msgs/Imu             raw yaw rate (BNO080)
    /imu/gps_heading              sensor_msgs/Imu             GPS velocity heading reference
    /move_sequence/_action/status action_msgs/GoalStatusArray detect TURN active
    /swath_path                   nav_msgs/Path               swath start → blind mode entry
    /swath_cross_track            std_msgs/Float32            cross-track error from controller
    /swath_bearing                std_msgs/Float32            path bearing (rad, ENU)

Publishes:
    /imu/fused_heading            sensor_msgs/Imu       absolute heading + yaw rate
    /heading_fuser/pose           geometry_msgs/PoseStamped  heading arrow in map frame
    /heading_fuser/mode           std_msgs/String       current operating mode

Parameters:
    bootstrap_dist_m          straight distance required for GPS cal  [1.0]
    bootstrap_max_yaw_rate    max yaw rate to count as straight        [0.10]
    line_snap_interval_m      distance between line snaps              [2.0]
    line_snap_ct_m            max cross-track to qualify for snap      [0.15]
    publish_covariance        yaw covariance on output                 [0.01]
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String
from action_msgs.msg import GoalStatusArray, GoalStatus
import tf2_ros


def _quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quat(yaw):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0))


def _angle_wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def _action_is_active(msg: GoalStatusArray) -> bool:
    for s in msg.status_list:
        if s.status == GoalStatus.STATUS_EXECUTING:
            return True
    return False


class HeadingFuser(Node):

    def __init__(self):
        super().__init__('heading_fuser')

        self._bootstrap_dist = self.declare_parameter('bootstrap_dist_m', 1.0).value
        self._bootstrap_max_yaw_rate = self.declare_parameter('bootstrap_max_yaw_rate', 0.10).value
        self._snap_interval = self.declare_parameter('line_snap_interval_m', 2.0).value
        self._snap_ct_max = self.declare_parameter('line_snap_ct_m', 0.15).value
        self._pub_cov = self.declare_parameter('publish_covariance', 0.01).value

        # Raw IMU yaw integral (seeded at 0, unbounded)
        self._imu_yaw_raw = None
        self._prev_imu_time = None
        self._yaw_rate_now = 0.0

        # offset: ground_heading = imu_yaw_raw + offset
        self._offset = None
        self._offset_calibrated = False

        # Bootstrap state
        self._bootstrap_dist_accum = 0.0
        self._bootstrap_done = False
        self._last_gps_heading = None
        self._last_gps_heading_time = 0.0

        # Freeze flags
        self._turn_active = False       # True while MoveSequence is executing
        self._turn_path_active = False  # True while arc turn path is active
        self._swath_active = False      # True from /swath_path arrival until turn-end or approach
        self._swath_bearing_rad = None  # bearing held constant during swath

        # Line snap state — active only during approach (to_start_path)
        self._snap_dist_accum = 0.0
        self._last_cross_track = None
        self._last_bearing = None
        self._last_bearing_time = 0.0

        _latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(Imu, '/imu/fused_heading', 10)
        self._pose_pub = self.create_publisher(PoseStamped, '/heading_fuser/pose', 10)
        self._mode_pub = self.create_publisher(String, '/heading_fuser/mode', 10)
        self._event_pub = self.create_publisher(String, '/heading_fuser/event', 10)
        self._offset_pub = self.create_publisher(Float32, '/heading_fuser/offset_deg', 10)

        self.create_subscription(Imu, '/imu', self._cb_imu, 10)
        self.create_subscription(Imu, '/imu/gps_heading', self._cb_gps_heading, 10)
        self.create_subscription(
            GoalStatusArray, '/move_sequence/_action/status', self._cb_turn_status, 10)
        self.create_subscription(Path, '/swath_path', self._cb_swath_path, 10)
        self.create_subscription(Path, '/to_start_path', self._cb_approach_path, _latched_qos)
        self.create_subscription(Path, '/turn_path', self._cb_turn_path, _latched_qos)
        self.create_subscription(Float32, '/swath_cross_track', self._cb_cross_track, 10)
        self.create_subscription(Float32, '/swath_bearing', self._cb_bearing, 10)

        self.get_logger().info(
            f'heading_fuser: waiting for {self._bootstrap_dist}m straight travel to calibrate')

    # ── offset calibration ────────────────────────────────────────────────────

    def _apply_offset(self, ground_heading_rad: float, source: str):
        if self._imu_yaw_raw is None:
            return
        new_offset = _angle_wrap(ground_heading_rad - self._imu_yaw_raw)
        old_heading = math.degrees(_angle_wrap(self._imu_yaw_raw + self._offset)) \
            if self._offset is not None else float('nan')
        old_offset = math.degrees(self._offset) if self._offset is not None else float('nan')
        self._offset = new_offset
        self._offset_calibrated = True
        new_heading = math.degrees(ground_heading_rad)
        new_offset_deg = math.degrees(new_offset)
        self.get_logger().info(
            f'heading_fuser: calibrated [{source}]  '
            f'ground={new_heading:.1f}°  '
            f'was={old_heading:.1f}°  '
            f'offset={new_offset_deg:.1f}°')
        event = json.dumps({
            'source': source,
            'heading_new_deg': round(new_heading, 2),
            'heading_old_deg': round(old_heading, 2),
            'offset_new_deg': round(new_offset_deg, 2),
            'offset_old_deg': round(old_offset, 2),
            'correction_deg': round(new_offset_deg - old_offset, 2),
        })
        self._event_pub.publish(String(data=event))
        self._publish_mode()

    def _current_heading(self) -> float:
        if self._imu_yaw_raw is None or self._offset is None:
            return 0.0
        return _angle_wrap(self._imu_yaw_raw + self._offset)

    # ── path bearing from first two poses ─────────────────────────────────────

    @staticmethod
    def _path_bearing(msg: Path):
        if len(msg.poses) < 2:
            return None
        p1 = msg.poses[0].pose.position
        p2 = msg.poses[1].pose.position
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        if math.hypot(dx, dy) < 1e-6:
            return None
        return math.atan2(dy, dx)

    # ── mode string ───────────────────────────────────────────────────────────

    def _mode_str(self) -> str:
        if not self._offset_calibrated:
            return f'BOOTSTRAP|{self._bootstrap_dist_accum:.2f}/{self._bootstrap_dist}m'
        if self._turn_active:
            return 'TURN|offset frozen'
        if self._swath_active:
            return f'SWATH|fixed {math.degrees(self._swath_bearing_rad):.1f}°' \
                if self._swath_bearing_rad is not None else 'SWATH|fixed'
        now = time.monotonic()
        if (self._last_bearing is not None
                and now - self._last_bearing_time < 1.0):
            return f'APPROACH|snap in {max(0.0, self._snap_interval - self._snap_dist_accum):.1f}m'
        return 'CALIBRATED|IMU+offset'

    def _publish_mode(self):
        self._mode_pub.publish(String(data=self._mode_str()))

    # ── context callbacks ─────────────────────────────────────────────────────

    def _cb_turn_status(self, msg: GoalStatusArray):
        was = self._turn_active
        self._turn_active = _action_is_active(msg)
        if self._turn_active and not was:
            self._snap_dist_accum = 0.0
            self._publish_mode()
            self.get_logger().info('heading_fuser: TURN mode — offset frozen')
        elif not self._turn_active and was:
            self._snap_dist_accum = 0.0
            self._publish_mode()
            self.get_logger().info('heading_fuser: turn ended — offset frozen until swath start')

    def _cb_swath_path(self, msg: Path):
        """Swath path published by RunSwathAction — enter blind mode immediately."""
        bearing = self._path_bearing(msg)
        if bearing is None:
            return
        self._swath_bearing_rad = bearing
        self._swath_active = True
        self._turn_path_active = False
        self._snap_dist_accum = 0.0
        self._apply_offset(bearing, 'SWATH start')
        self.get_logger().info(
            f'heading_fuser: SWATH blind — locked to {math.degrees(bearing):.1f}°')

    def _cb_approach_path(self, msg: Path):
        """Approach path — exit swath/turn blind mode, resync offset, enable line snaps."""
        self._swath_active = False
        self._turn_path_active = False
        self._snap_dist_accum = 0.0
        if self._swath_bearing_rad is not None:
            self._apply_offset(self._swath_bearing_rad, 'SWATH exit resync')
            self.get_logger().info(
                f'heading_fuser: APPROACH — resynced to {math.degrees(self._swath_bearing_rad):.1f}°, '
                f'line snaps enabled')
        else:
            self._publish_mode()
            self.get_logger().info('heading_fuser: APPROACH — line snaps enabled')

    def _cb_turn_path(self, msg: Path):
        """Turn path — exit swath blind mode, freeze offset for arc turn duration."""
        self._swath_active = False
        self._turn_path_active = True
        self._snap_dist_accum = 0.0
        self._publish_mode()

    def _cb_cross_track(self, msg: Float32):
        self._last_cross_track = msg.data

    def _cb_bearing(self, msg: Float32):
        self._last_bearing = msg.data
        self._last_bearing_time = time.monotonic()

    def _cb_gps_heading(self, msg: Imu):
        self._last_gps_heading = _quat_to_yaw(msg.orientation)
        self._last_gps_heading_time = time.monotonic()

    # ── IMU integration (core loop) ───────────────────────────────────────────

    def _cb_imu(self, msg: Imu):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._prev_imu_time is None:
            self._prev_imu_time = stamp_sec
            self._imu_yaw_raw = 0.0
            imu_yaw = _quat_to_yaw(msg.orientation)
            self._offset = _angle_wrap(imu_yaw - self._imu_yaw_raw)
            self.get_logger().info(
                f'heading_fuser: seeded from BNO080 {math.degrees(imu_yaw):.1f}° '
                f'(waiting for {self._bootstrap_dist}m straight GPS bootstrap)')
            return

        dt = stamp_sec - self._prev_imu_time
        self._prev_imu_time = stamp_sec

        if dt <= 0.0 or dt > 1.0:
            return

        yaw_rate = msg.angular_velocity.z
        self._yaw_rate_now = abs(yaw_rate)
        self._imu_yaw_raw += yaw_rate * dt

        if not self._bootstrap_done:
            self._try_bootstrap(dt)

        # Line snaps only during approach (not swath, not any kind of turn)
        if not self._turn_active and not self._turn_path_active and not self._swath_active:
            self._try_line_snap(dt)

        # During swath: publish fixed bearing — IMU integration continues but is ignored for output
        if self._swath_active and self._swath_bearing_rad is not None:
            heading = self._swath_bearing_rad
        else:
            heading = self._current_heading()

        if self._offset is not None:
            self._offset_pub.publish(Float32(data=float(math.degrees(self._offset))))

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_footprint'
        out.orientation = _yaw_to_quat(heading)
        out.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, self._pub_cov if self._offset_calibrated else 1.0,
        ]
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
        out.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        self._pub.publish(out)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'map'
        pose.pose.orientation = _yaw_to_quat(heading)
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
        except tf2_ros.TransformException:
            pass
        self._pose_pub.publish(pose)

    # ── bootstrap (1 m straight → GPS heading) ───────────────────────────────

    def _try_bootstrap(self, dt: float):
        if self._turn_active:
            self._bootstrap_dist_accum = 0.0
            return
        if self._yaw_rate_now > self._bootstrap_max_yaw_rate:
            self._bootstrap_dist_accum = 0.0
            return
        now = time.monotonic()
        if (self._last_gps_heading is None
                or now - self._last_gps_heading_time > 3.0):
            return
        self._bootstrap_dist_accum += 0.3 * dt
        if self._bootstrap_dist_accum >= self._bootstrap_dist:
            self._apply_offset(self._last_gps_heading, 'BOOTSTRAP 1m straight')
            self._bootstrap_done = True

    # ── line snap (every 2 m while on approach, ct < threshold) ─────────────

    def _try_line_snap(self, dt: float):
        if self._last_cross_track is None or self._last_bearing is None:
            return
        if time.monotonic() - self._last_bearing_time > 1.0:
            self._snap_dist_accum = 0.0
            return

        ct = abs(self._last_cross_track)
        if ct < self._snap_ct_max:
            self._snap_dist_accum += 0.4 * dt
        else:
            self._snap_dist_accum = 0.0

        if self._snap_dist_accum >= self._snap_interval:
            self._apply_offset(self._last_bearing, f'LINE snap (ct={ct:.3f}m)')
            self._snap_dist_accum = 0.0


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HeadingFuser()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
