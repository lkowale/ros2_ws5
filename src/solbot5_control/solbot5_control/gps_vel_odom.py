#!/usr/bin/env python3
"""
gps_vel_odom — convert UBX-NAV-VELNED to heading IMU for EKF.

When driving above a speed threshold, computes heading from GPS velocity
direction (atan2(vel_n, vel_e) in ENU) and publishes it as an IMU message
on /imu/gps_heading. The global EKF fuses this as an absolute yaw
measurement, providing heading observability without needing a magnetometer.

Also publishes Odometry on /odometry/gps_vel (for diagnostics) and
PoseStamped on /gps_vel_heading (for mapviz visualization).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion


class GpsVelOdom(Node):

    def __init__(self):
        super().__init__('gps_vel_odom')

        self._min_speed = self.declare_parameter('min_speed_mps', 0.3).value
        self._vel_cov = self.declare_parameter('velocity_covariance', 0.04).value
        self._heading_cov = self.declare_parameter('heading_covariance', 0.05).value   # ~13 deg — soft correction
        self._max_yaw_rate = self.declare_parameter('max_yaw_rate_rads', 0.15).value   # suppress during turns
        self._heading_interval = self.declare_parameter('heading_publish_interval_s', 2.0).value
        self._last_heading_pub = 0.0

        from ublox_ubx_msgs.msg import UBXNavVelNED
        self._UBXNavVelNED = UBXNavVelNED

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._wheel_vx = 0.0
        self._yaw_rate = 0.0
        self._odom_pub = self.create_publisher(Odometry, '/odometry/gps_vel', 10)
        self._heading_imu_pub = self.create_publisher(Imu, '/imu/gps_heading', 10)
        self._heading_pub = self.create_publisher(PoseStamped, '/gps_vel_heading', 10)

        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(
            UBXNavVelNED, '/ubx_nav_vel_ned', self._cb_vel, 10)
        self.create_subscription(
            Odometry, '/odometry/global', self._cb_odom, best_effort)
        self.create_subscription(
            Odometry, '/odom/wheel', self._cb_wheel, 10)
        self.create_subscription(
            Imu, '/imu', self._cb_imu, 10)

        self.get_logger().info(
            f'gps_vel_odom started  min_speed={self._min_speed} m/s  '
            f'vel_cov={self._vel_cov}  heading_cov={self._heading_cov}  '
            f'max_yaw_rate={self._max_yaw_rate} rad/s')

    def _cb_odom(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _cb_wheel(self, msg):
        self._wheel_vx = msg.twist.twist.linear.x

    def _cb_imu(self, msg):
        self._yaw_rate = abs(msg.angular_velocity.z)

    def _cb_vel(self, msg):
        # NED cm/s → ENU m/s
        vel_e = msg.vel_e / 100.0
        vel_n = msg.vel_n / 100.0
        vx_enu = vel_e
        vy_enu = vel_n
        speed = math.hypot(vx_enu, vy_enu)

        # Publish diagnostic odometry (ENU velocities, not for EKF fusion)
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'map'
        odom.twist.twist.linear.x = vx_enu
        odom.twist.twist.linear.y = vy_enu
        cov = [0.0] * 36
        cov[0] = self._vel_cov
        cov[7] = self._vel_cov
        for i in range(2, 6):
            cov[i * 7] = 1e6
        odom.twist.covariance = cov
        pos_cov = [0.0] * 36
        for i in range(6):
            pos_cov[i * 7] = 1e6
        odom.pose.covariance = pos_cov
        self._odom_pub.publish(odom)

        # Publish heading as IMU only when driving forward above threshold
        # and not turning (low yaw rate = straight ahead).
        # When reversing, GPS velocity direction ≠ robot heading.
        now = self.get_clock().now().nanoseconds * 1e-9
        if (speed >= self._min_speed
                and self._wheel_vx > 0.0
                and self._yaw_rate <= self._max_yaw_rate
                and now - self._last_heading_pub >= self._heading_interval):
            # ENU heading: atan2(east, north) would give compass bearing,
            # but ROS ENU yaw = atan2(y, x) = atan2(north, east)
            heading_rad = math.atan2(vy_enu, vx_enu)

            imu_msg = Imu()
            imu_msg.header.stamp = msg.header.stamp
            imu_msg.header.frame_id = 'base_footprint'
            imu_msg.orientation = _yaw_to_quat(heading_rad)
            # Orientation covariance: only yaw is valid
            imu_msg.orientation_covariance = [
                1e6, 0.0, 0.0,
                0.0, 1e6, 0.0,
                0.0, 0.0, self._heading_cov,
            ]
            # Mark angular velocity and linear accel as unknown
            imu_msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
            imu_msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8

            self._heading_imu_pub.publish(imu_msg)
            self._last_heading_pub = now

            # Mapviz heading arrow
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = self._robot_x
            pose.pose.position.y = self._robot_y
            pose.pose.orientation = _yaw_to_quat(heading_rad)
            self._heading_pub.publish(pose)


def _yaw_to_quat(yaw_rad):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw_rad / 2.0),
        w=math.cos(yaw_rad / 2.0))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GpsVelOdom()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
