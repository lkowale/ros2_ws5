#!/usr/bin/env python3
"""
auto_imu_cal — automatic IMU yaw offset calibration at startup

Passively monitors GPS and IMU while the robot moves in a straight line.
Computes gps_bearing - imu_yaw offset (circular mean), adds it to the
existing stored offset, and publishes the corrected value to /imu_yaw_offset
which imu_bridge applies immediately and saves to config.yaml.

Runs once per session: collects num_samples samples then shuts down.
If timeout_s elapses without enough samples, exits silently (keeps old offset).

The formula is:
    new_offset = old_offset + circular_mean(gps_bearing - published_imu_yaw)

This is correct regardless of what the old offset is, because:
    published_imu_yaw = raw_imu_yaw + old_offset
    gps_bearing - published_imu_yaw = gps_bearing - raw_imu_yaw - old_offset
    new_offset = old_offset + (gps_bearing - raw_imu_yaw - old_offset)
               = gps_bearing - raw_imu_yaw   (the true mounting offset)
"""

import math
import os
import yaml

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String


R_EARTH = 6_371_000.0


def _haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return R_EARTH * 2 * math.asin(math.sqrt(a))


def _gps_bearing_enu(lat1, lon1, lat2, lon2):
    """Bearing from point1 to point2 in ROS ENU convention (rad).
    ENU: x=East y=North → yaw=0 is East, yaw=π/2 is North.
    """
    cos_lat = math.cos(math.radians(lat1))
    dx = math.radians(lon2 - lon1) * cos_lat * R_EARTH  # East
    dy = math.radians(lat2 - lat1) * R_EARTH             # North
    return math.atan2(dy, dx)


def _yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _circular_mean(angles):
    sin_sum = sum(math.sin(a) for a in angles)
    cos_sum = sum(math.cos(a) for a in angles)
    return math.atan2(sin_sum, cos_sum)


CONFIG_PATH = os.path.expanduser(
    '~/ros2_ws4/src/control/solbot5_control/config/config.yaml')


def _load_old_offset():
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
        return float(cfg.get('imu', {}).get('yaw_offset', 0.0))
    except Exception:
        return 0.0


class ImuPassiveCal(Node):

    def __init__(self):
        super().__init__('imu_passive_cal')

        self._min_speed    = self.declare_parameter('min_speed_mps',   0.3).value
        self._max_wz       = self.declare_parameter('max_angular_z',   0.15).value
        self._min_gps_move = self.declare_parameter('min_gps_move_m',  0.3).value
        self._num_samples  = self.declare_parameter('num_samples',     20).value
        self._timeout_s    = self.declare_parameter('timeout_s',       120.0).value

        self._old_offset = _load_old_offset()
        self.get_logger().info(
            f'auto_imu_cal started  old_offset={math.degrees(self._old_offset):.2f}°  '
            f'need {self._num_samples} straight-line samples')

        self._lat = self._lon = None
        self._imu_yaw = None
        self._odom_speed = self._odom_wz = None
        self._prev_lat = self._prev_lon = None
        self._samples = []
        self._done = False

        self._pub        = self.create_publisher(Float32, '/imu_yaw_offset', 10)
        self._status_pub = self.create_publisher(String,  '/auto_imu_cal/status', 10)

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(NavSatFix, '/gps/fix',  self._cb_fix,  best_effort)
        self.create_subscription(Imu,       '/imu',      self._cb_imu,  10)
        self.create_subscription(Odometry,  '/odom',     self._cb_odom, 10)

        self._start_time = self.get_clock().now()
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_status)

    def _cb_fix(self, msg: NavSatFix):
        self._lat = msg.latitude
        self._lon = msg.longitude

    def _cb_imu(self, msg: Imu):
        q = msg.orientation
        self._imu_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _cb_odom(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._odom_speed = math.sqrt(vx * vx + vy * vy)
        self._odom_wz    = abs(msg.twist.twist.angular.z)

    def _tick(self):
        if self._done:
            return

        # Timeout check
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed > self._timeout_s:
            self.get_logger().warn(
                f'auto_imu_cal timeout after {elapsed:.0f}s — '
                f'keeping old offset ({math.degrees(self._old_offset):.2f}°)')
            self._shutdown()
            return

        # Need GPS, IMU, odom
        if (self._lat is None or self._lon is None
                or self._imu_yaw is None
                or self._odom_speed is None):
            return

        # Motion quality gates
        if self._odom_speed < self._min_speed:
            return
        if self._odom_wz is not None and self._odom_wz > self._max_wz:
            return

        # GPS movement gate
        if self._prev_lat is None:
            self._prev_lat, self._prev_lon = self._lat, self._lon
            return

        dist = _haversine(self._prev_lat, self._prev_lon, self._lat, self._lon)
        if dist < self._min_gps_move:
            return

        bearing = _gps_bearing_enu(self._prev_lat, self._prev_lon, self._lat, self._lon)
        self._prev_lat, self._prev_lon = self._lat, self._lon

        # Sample: discrepancy between GPS bearing and published IMU yaw
        sample = _normalize(bearing - self._imu_yaw)
        self._samples.append(sample)

        n = len(self._samples)
        self.get_logger().info(
            f'sample {n}/{self._num_samples}  '
            f'gps_bearing={math.degrees(bearing):.1f}°  '
            f'imu_yaw={math.degrees(self._imu_yaw):.1f}°  '
            f'diff={math.degrees(sample):.1f}°')

        if n >= self._num_samples:
            self._finish()

    def _finish(self):
        correction = _circular_mean(self._samples)
        new_offset = _normalize(self._old_offset + correction)

        self.get_logger().info(
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info(
            f'auto_imu_cal DONE  samples={len(self._samples)}')
        self.get_logger().info(
            f'  old offset : {math.degrees(self._old_offset):+.2f}°')
        self.get_logger().info(
            f'  correction : {math.degrees(correction):+.2f}°')
        self.get_logger().info(
            f'  new offset : {math.degrees(new_offset):+.2f}°')
        self.get_logger().info(
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

        msg = Float32()
        msg.data = float(new_offset)
        for _ in range(5):
            self._pub.publish(msg)

        self._shutdown()

    def _publish_status(self):
        n = len(self._samples)
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        if self._done and n >= self._num_samples:
            correction = _circular_mean(self._samples)
            new_offset = _normalize(self._old_offset + correction)
            status = {
                'state': 'done',
                'samples': n,
                'needed': self._num_samples,
                'correction_deg': round(math.degrees(correction), 1),
                'new_offset_deg': round(math.degrees(new_offset), 1),
                'elapsed_s': round(elapsed, 0),
            }
        elif self._done:
            status = {
                'state': 'timeout',
                'samples': n,
                'needed': self._num_samples,
                'elapsed_s': round(elapsed, 0),
            }
        elif n == 0:
            status = {
                'state': 'waiting',
                'samples': 0,
                'needed': self._num_samples,
                'elapsed_s': round(elapsed, 0),
            }
        else:
            correction = _circular_mean(self._samples)
            status = {
                'state': 'collecting',
                'samples': n,
                'needed': self._num_samples,
                'correction_deg': round(math.degrees(correction), 1),
                'elapsed_s': round(elapsed, 0),
            }

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def _shutdown(self):
        self._done = True
        self._publish_status()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ImuPassiveCal()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
