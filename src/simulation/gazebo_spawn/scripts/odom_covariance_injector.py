#!/usr/bin/env python3
"""
Odometry covariance injector for GPS odometry.

The navsat_transform node may output zero covariance when GPS sensor
reports zero noise. This node adds reasonable covariance values to
ensure the EKF can properly weight GPS measurements.

Subscribes to: odometry/gps_raw (nav_msgs/Odometry)
Publishes to: odometry/gps (nav_msgs/Odometry)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomCovarianceInjector(Node):
    def __init__(self):
        super().__init__('odom_covariance_injector')

        # Covariance parameters (diagonal values)
        self.declare_parameter('pose_covariance_x', 0.01)  # meters^2
        self.declare_parameter('pose_covariance_y', 0.01)  # meters^2
        self.declare_parameter('pose_covariance_z', 0.1)   # meters^2
        self.declare_parameter('pose_covariance_roll', 0.1)   # rad^2
        self.declare_parameter('pose_covariance_pitch', 0.1)  # rad^2
        self.declare_parameter('pose_covariance_yaw', 0.05)   # rad^2

        self.pose_cov = [
            self.get_parameter('pose_covariance_x').value,
            self.get_parameter('pose_covariance_y').value,
            self.get_parameter('pose_covariance_z').value,
            self.get_parameter('pose_covariance_roll').value,
            self.get_parameter('pose_covariance_pitch').value,
            self.get_parameter('pose_covariance_yaw').value,
        ]

        self.subscription = self.create_subscription(
            Odometry,
            'odometry/gps_raw',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            Odometry,
            'odometry/gps',
            10
        )

        self.get_logger().info(
            f'Odometry covariance injector started. '
            f'Pose covariance diag: {self.pose_cov[:2]}'
        )

    def odom_callback(self, msg: Odometry):
        # Check if covariance is all zeros (or very small)
        if all(abs(c) < 1e-9 for c in msg.pose.covariance):
            # Inject diagonal covariance for pose
            # Covariance is 6x6 matrix in row-major order: [x, y, z, roll, pitch, yaw]
            msg.pose.covariance[0] = self.pose_cov[0]   # x
            msg.pose.covariance[7] = self.pose_cov[1]   # y
            msg.pose.covariance[14] = self.pose_cov[2]  # z
            msg.pose.covariance[21] = self.pose_cov[3]  # roll
            msg.pose.covariance[28] = self.pose_cov[4]  # pitch
            msg.pose.covariance[35] = self.pose_cov[5]  # yaw

        # Publish with covariance
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
