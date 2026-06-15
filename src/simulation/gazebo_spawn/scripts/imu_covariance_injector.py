#!/usr/bin/env python3
"""
IMU covariance injector for Gazebo simulation.

The Gazebo IMU sensor may output zero covariance. This node adds
reasonable covariance values to ensure the EKF can properly weight
IMU measurements.

Subscribes to: imu_raw (sensor_msgs/Imu)
Publishes to: imu (sensor_msgs/Imu)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceInjector(Node):
    def __init__(self):
        super().__init__('imu_covariance_injector')

        # Covariance parameters (diagonal values)
        self.declare_parameter('orientation_covariance', 0.01)  # rad^2
        self.declare_parameter('angular_velocity_covariance', 0.01)  # (rad/s)^2
        self.declare_parameter('linear_acceleration_covariance', 0.1)  # (m/s^2)^2

        self.orientation_cov = self.get_parameter('orientation_covariance').value
        self.angular_vel_cov = self.get_parameter('angular_velocity_covariance').value
        self.linear_acc_cov = self.get_parameter('linear_acceleration_covariance').value

        self.subscription = self.create_subscription(
            Imu,
            'imu_raw',
            self.imu_callback,
            10
        )

        self.publisher = self.create_publisher(
            Imu,
            'imu',
            10
        )

        self.get_logger().info(
            f'IMU covariance injector started. '
            f'Orientation: {self.orientation_cov}, '
            f'Angular vel: {self.angular_vel_cov}, '
            f'Linear acc: {self.linear_acc_cov}'
        )

    def imu_callback(self, msg: Imu):
        # Check if orientation covariance is all zeros
        if all(abs(c) < 1e-9 for c in msg.orientation_covariance):
            # 3x3 matrix in row-major: [roll, pitch, yaw]
            msg.orientation_covariance[0] = self.orientation_cov
            msg.orientation_covariance[4] = self.orientation_cov
            msg.orientation_covariance[8] = self.orientation_cov

        # Check if angular velocity covariance is all zeros
        if all(abs(c) < 1e-9 for c in msg.angular_velocity_covariance):
            msg.angular_velocity_covariance[0] = self.angular_vel_cov
            msg.angular_velocity_covariance[4] = self.angular_vel_cov
            msg.angular_velocity_covariance[8] = self.angular_vel_cov

        # Check if linear acceleration covariance is all zeros
        if all(abs(c) < 1e-9 for c in msg.linear_acceleration_covariance):
            msg.linear_acceleration_covariance[0] = self.linear_acc_cov
            msg.linear_acceleration_covariance[4] = self.linear_acc_cov
            msg.linear_acceleration_covariance[8] = self.linear_acc_cov

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
