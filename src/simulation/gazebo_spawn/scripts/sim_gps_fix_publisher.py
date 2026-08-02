#!/usr/bin/env python3
"""
Sim adapter: publish a correct /gps/fix from Gazebo ground truth.

The gz-sim NavSat sensor's lat/lon output does not track the world ENU frame
consistently (its reported displacement is rotated by a heading-dependent
amount relative to ground truth — verified directly against /odometry/gazebo).

This node bypasses the gz NavSat sensor entirely: it reads the robot's TRUE
world-frame position from /gz/robot_world_odom (gz OdometryPublisher plugin
with odom_frame=world), then converts ENU -> lat/lon about the world datum for
navsat_transform.

Why not /odometry/gazebo:
  That comes from the AckermannSteering plugin (odom_frame=odom) — wheel
  odometry, NOT ground truth. It accumulates >10 m of drift during a 10-min
  field run, corrupting the GPS fix and causing EKF drift.

Subscribes: /gz/robot_world_odom  (nav_msgs/Odometry, OdometryPublisher world frame)
Publishes:  /gps/fix              (sensor_msgs/NavSatFix, reliable)
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


def _quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimGpsFix(Node):

    def __init__(self):
        super().__init__('sim_gps_fix_publisher')

        # World datum (must match world spherical_coordinates in the SDF).
        self._lat0 = math.radians(self.declare_parameter('datum_lat', 53.5204991).value)
        self._lon0 = math.radians(self.declare_parameter('datum_lon', 17.8258532).value)
        self._alt0 = self.declare_parameter('datum_alt', 100.0).value

        # Antenna offset from base_footprint, in the body frame (m).
        # In simulation: antenna_x=0 so datum is anchored at base_footprint,
        # aligning map frame with the Gazebo world frame.
        self._ant_x = self.declare_parameter('antenna_x', 0.0).value
        self._ant_y = self.declare_parameter('antenna_y', 0.0).value

        self._rate_hz = self.declare_parameter('rate_hz', 10.0).value
        self._frame_id = self.declare_parameter('frame_id', 'base_footprint').value

        # Meridian/parallel lengths (m per radian) at the datum.
        sin_lat = math.sin(self._lat0)
        denom = math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
        self._m_per_rad_lat = _A * (1.0 - _E2) / (denom ** 3)
        self._m_per_rad_lon = _A * math.cos(self._lat0) / denom

        # RELIABLE so Mapviz's navsat plugin receives it; best_effort subscribers
        # (navsat_transform, navsat_init) are still served fine.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(NavSatFix, '/gps/fix', qos)

        # Ground-truth world position and yaw from OdometryPublisher (world frame).
        self._world_pose = None

        self.create_subscription(Odometry, '/gz/robot_world_odom', self._world_odom_cb, 10)
        self.create_timer(1.0 / self._rate_hz, self._tick)

        self.get_logger().info(
            f'sim_gps_fix_publisher started  datum=({math.degrees(self._lat0):.7f},'
            f'{math.degrees(self._lon0):.7f})  antenna=({self._ant_x},{self._ant_y})  '
            f'rate={self._rate_hz} Hz')

    def _world_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._world_pose = (p.x, p.y, _quat_yaw(q))

    def _tick(self):
        if self._world_pose is None:
            return
        bx, by, yaw = self._world_pose
        # Apply antenna lever-arm in world frame (rotate body offset by yaw).
        ax = bx + self._ant_x * math.cos(yaw) - self._ant_y * math.sin(yaw)
        ay = by + self._ant_x * math.sin(yaw) + self._ant_y * math.cos(yaw)

        lat = self._lat0 + ay / self._m_per_rad_lat
        lon = self._lon0 + ax / self._m_per_rad_lon

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = math.degrees(lat)
        fix.longitude = math.degrees(lon)
        fix.altitude = self._alt0
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
