#!/usr/bin/env python3
"""
Sim adapter: publish a correct /gps/fix from Gazebo ground truth.

The gz-sim NavSat sensor's lat/lon output does not track the world ENU frame
consistently (its reported displacement is rotated by a heading-dependent
amount relative to ground truth — verified directly against /odometry/gazebo).
That rotation silently corrupts navsat_transform's odometry.

This node bypasses the gz NavSat sensor entirely: it takes the robot's
ground-truth ENU position from /odometry/gazebo, applies the gps_link antenna
lever-arm (forward of base_footprint), and converts ENU -> lat/lon about the
world datum using the same WGS84/equirectangular model navsat_transform expects.
The result is a /gps/fix that is correct by construction, so the localization
path downstream (navsat_transform + EKF) is exercised on clean data — and the
sim matches the real robot at the /gps/fix boundary.

Subscribes: /odometry/gazebo  (nav_msgs/Odometry, ground-truth ENU pose)
Publishes:  /gps/fix          (sensor_msgs/NavSatFix, reliable — see QoS note below)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

# WGS84
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2 - _F)


class SimGpsFix(Node):

    def __init__(self):
        super().__init__('sim_gps_fix_publisher')

        # World datum (must match world spherical_coordinates in empty.sdf).
        self._lat0 = math.radians(self.declare_parameter('datum_lat', 53.5204991).value)
        self._lon0 = math.radians(self.declare_parameter('datum_lon', 17.8258532).value)
        self._alt0 = self.declare_parameter('datum_alt', 100.0).value

        # Antenna offset from base_footprint, in the body frame (m). base_footprint
        # is defined to coincide with the front antenna (gps_link), so this is 0.
        self._ant_x = self.declare_parameter('antenna_x', 0.0).value
        self._ant_y = self.declare_parameter('antenna_y', 0.0).value

        self._rate_hz = self.declare_parameter('rate_hz', 10.0).value
        self._frame_id = self.declare_parameter('frame_id', 'gps_link').value

        # Meridian/parallel lengths (m per radian) at the datum.
        sin_lat = math.sin(self._lat0)
        denom = math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
        self._m_per_rad_lat = _A * (1.0 - _E2) / (denom ** 3)
        self._m_per_rad_lon = _A * math.cos(self._lat0) / denom

        # RELIABLE so Mapviz's navsat plugin (which subscribes reliable) receives
        # it; best_effort subscribers (navsat_transform, navsat_init) are still
        # served fine since reliable satisfies best_effort.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(NavSatFix, '/gps/fix', qos)

        self._pose = None
        self.create_subscription(Odometry, '/odometry/gazebo', self._cb, 10)
        self.create_timer(1.0 / self._rate_hz, self._tick)

        self.get_logger().info(
            f'sim_gps_fix_publisher started  datum=({math.degrees(self._lat0):.7f},'
            f'{math.degrees(self._lon0):.7f})  antenna=({self._ant_x},{self._ant_y})  '
            f'rate={self._rate_hz} Hz')

    def _cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # base_footprint ENU position
        bx = msg.pose.pose.position.x
        by = msg.pose.pose.position.y
        # antenna position = base + body-frame offset rotated into ENU
        ax = bx + self._ant_x * math.cos(yaw) - self._ant_y * math.sin(yaw)
        ay = by + self._ant_x * math.sin(yaw) + self._ant_y * math.cos(yaw)
        self._pose = (ax, ay, msg.pose.pose.position.z)

    def _tick(self):
        if self._pose is None:
            return
        e, nn, up = self._pose
        lat = self._lat0 + nn / self._m_per_rad_lat
        lon = self._lon0 + e / self._m_per_rad_lon

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = math.degrees(lat)
        fix.longitude = math.degrees(lon)
        fix.altitude = self._alt0 + up
        fix.position_covariance = [0.0] * 9
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimGpsFix()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
