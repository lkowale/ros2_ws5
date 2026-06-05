#!/usr/bin/env python3
"""
Sim adapter: derive GPS-like velocity odometry from Gazebo ground truth.

Converts body-frame velocity from the Gazebo Ackermann plugin odometry
to ENU-frame velocity on /odometry/gps_vel, matching what gps_vel_odom
publishes on the real robot from the F9P NAV-VELNED.

Subscribes: /odometry/gazebo (nav_msgs/Odometry, body-frame twist)
Publishes:  /odometry/gps_vel (nav_msgs/Odometry, odom-frame twist)
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class SimGpsVelOdom(Node):

    def __init__(self):
        super().__init__('sim_gps_vel_odom')

        self._vel_cov = self.declare_parameter('velocity_covariance', 0.04).value

        self._pub = self.create_publisher(Odometry, '/odometry/gps_vel', 10)
        self.create_subscription(Odometry, '/odometry/gazebo', self._cb, 10)

        self.get_logger().info('sim_gps_vel_odom started')

    def _cb(self, msg):
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Body-frame velocity → odom/ENU frame
        vx_body = msg.twist.twist.linear.x
        vy_body = msg.twist.twist.linear.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_enu = vx_body * cos_yaw - vy_body * sin_yaw
        vy_enu = vx_body * sin_yaw + vy_body * cos_yaw

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'odom'

        odom.twist.twist.linear.x = vx_enu
        odom.twist.twist.linear.y = vy_enu

        cov = [0.0] * 36
        cov[0] = self._vel_cov
        cov[7] = self._vel_cov
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = 1e6
        odom.twist.covariance = cov

        pos_cov = [0.0] * 36
        for i in range(6):
            pos_cov[i * 7] = 1e6
        odom.pose.covariance = pos_cov

        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimGpsVelOdom()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
