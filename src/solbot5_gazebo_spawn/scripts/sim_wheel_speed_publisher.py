#!/usr/bin/env python3
"""
Sim adapter: convert Gazebo joint_states to /wheel_speeds JSON.

Gazebo's JointStatePublisher outputs angular velocities (rad/s) for each
wheel joint. This node converts them to the motor RPM format that
ackermann_odom expects on the /wheel_speeds topic.

Joint name mapping (Gazebo → real robot):
    front_left_wheel  → FL
    front_right_wheel → FR
    rear_left_wheel   → RL
    rear_right_wheel  → RR

The real ODrive reports motor RPM where:
    FL/RL (dir=0, CW):  positive RPM = forward
    FR/RR (dir=1, CCW): negative RPM = forward

Gazebo wheels all report positive angular velocity for forward motion,
so we negate FR/RR to match the real robot sign convention.

Subscribes: /joint_states (sensor_msgs/JointState)
Publishes:  /wheel_speeds (std_msgs/String, JSON)
"""

import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


# Gazebo joint name → real robot wheel name
JOINT_MAP = {
    'front_left_wheel': 'FL',
    'front_right_wheel': 'FR',
    'rear_left_wheel': 'RL',
    'rear_right_wheel': 'RR',
}

# Sign convention: match real ODrive reporting
# FL/RL are CW (positive = forward in Gazebo, positive RPM on real robot)
# FR/RR are CCW (positive = forward in Gazebo, but negative RPM on real robot)
SPEED_SIGN = {
    'FL': 1.0,
    'FR': -1.0,
    'RL': 1.0,
    'RR': -1.0,
}


class SimWheelSpeedPublisher(Node):

    def __init__(self):
        super().__init__('sim_wheel_speed_publisher')

        self._wheel_radius = self.declare_parameter('wheel_radius', 0.20).value
        self._gear_ratio = self.declare_parameter('gear_ratio', 1.0).value

        self._speeds = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        self._pub = self.create_publisher(String, '/wheel_speeds', 10)
        self.create_subscription(JointState, '/joint_states', self._cb, 10)

        self.get_logger().info(
            f'sim_wheel_speed_publisher started  '
            f'wheel_radius={self._wheel_radius}m  gear_ratio={self._gear_ratio}')

    def _cb(self, msg):
        for i, name in enumerate(msg.name):
            wheel = JOINT_MAP.get(name)
            if wheel and i < len(msg.velocity):
                # angular velocity (rad/s) → motor RPM
                wheel_rps = msg.velocity[i] / (2.0 * math.pi)
                motor_rpm = (wheel_rps * 60.0) / self._gear_ratio
                self._speeds[wheel] = motor_rpm * SPEED_SIGN[wheel]

        out = String()
        out.data = json.dumps(self._speeds)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimWheelSpeedPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
