#!/usr/bin/env python3
"""
ackermann_odom — dead-reckoning odometry from wheel speed feedback.

Subscribes to /wheel_speeds (JSON with FL, FR, RL, RR motor RPM from ODrive)
published by drive.py and computes nav_msgs/Odometry for the local EKF.

Uses front axle differential speed for ackermann yaw rate estimation:
    v_left  = FL wheel linear speed
    v_right = FR wheel linear speed
    vx      = (v_left + v_right) / 2
    yaw_rate = (v_right - v_left) / track_width

Parameters:
    wheelbase   : front-to-rear axle distance (default 1.25 m)
    track_width : left-to-right wheel distance (default 0.7 m)
    wheel_diameter : wheel diameter (default 0.45 m)
    gear_ratio  : motor-to-wheel gear ratio (default 1.0)
    publish_rate : odometry publish rate in Hz (default 20.0)
"""

import math
import json

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String


class AckermannOdom(Node):

    def __init__(self):
        super().__init__('ackermann_odom')

        self._wheelbase = self.declare_parameter('wheelbase', 1.25).value
        self._track_width = self.declare_parameter('track_width', 0.7).value
        self._wheel_diameter = self.declare_parameter('wheel_diameter', 0.45).value
        self._gear_ratio = self.declare_parameter('gear_ratio', 1.0).value
        self._rate = self.declare_parameter('publish_rate', 20.0).value

        # Direction config: sign to make positive = forward.
        # Both FL and FR report positive RPM when driving forward.
        self._speed_sign = {
            'FL': 1.0,
            'FR': 1.0,
            'RL': 1.0,
            'RR': 1.0,
        }

        # State
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vyaw = 0.0
        self._last_time = self.get_clock().now()

        # Latest wheel speeds (motor RPM from ODrive)
        self._fl_mps = 0.0
        self._fr_mps = 0.0

        self._odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
        self.create_subscription(String, '/wheel_speeds', self._cb_wheel_speeds, 10)
        self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f'ackermann_odom started  wheelbase={self._wheelbase}m  '
            f'track={self._track_width}m  diameter={self._wheel_diameter}m  '
            f'rate={self._rate}Hz')

    def _rpm_to_mps(self, motor_rpm):
        """Convert motor RPM to wheel linear speed in m/s."""
        wheel_rpm = motor_rpm * self._gear_ratio
        return wheel_rpm * math.pi * self._wheel_diameter / 60.0

    def _cb_wheel_speeds(self, msg):
        """Update wheel speeds from drive.py JSON feedback."""
        try:
            speeds = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return

        fl_rpm = speeds.get('FL', 0.0) * self._speed_sign['FL']
        fr_rpm = speeds.get('FR', 0.0) * self._speed_sign['FR']

        self._fl_mps = self._rpm_to_mps(fl_rpm)
        self._fr_mps = self._rpm_to_mps(fr_rpm)

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0 or dt > 1.0:
            return

        vx = (self._fl_mps + self._fr_mps) / 2.0
        vyaw = (self._fr_mps - self._fl_mps) / self._track_width

        self._vx = vx
        self._vyaw = vyaw

        # Integrate pose
        self._yaw += vyaw * dt
        self._x += vx * math.cos(self._yaw) * dt
        self._y += vx * math.sin(self._yaw) * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = _yaw_to_quat(self._yaw)

        # Pose covariance — wheel odom drifts over time
        pc = [0.0] * 36
        pc[0] = 0.1    # x
        pc[7] = 0.1    # y
        pc[14] = 1e6   # z
        pc[21] = 1e6   # roll
        pc[28] = 1e6   # pitch
        pc[35] = 0.2   # yaw
        odom.pose.covariance = pc

        # Twist in body frame
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw

        tc = [0.0] * 36
        tc[0] = 0.05   # vx
        tc[7] = 0.01   # vy (non-holonomic: vy≈0 always)
        tc[14] = 1e6   # vz
        tc[21] = 1e6   # wx
        tc[28] = 1e6   # wy
        tc[35] = 0.1   # wz
        odom.twist.covariance = tc

        self._odom_pub.publish(odom)


def _yaw_to_quat(yaw_rad):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw_rad / 2.0),
        w=math.cos(yaw_rad / 2.0))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AckermannOdom()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
