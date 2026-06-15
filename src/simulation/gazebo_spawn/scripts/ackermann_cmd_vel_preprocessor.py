#!/usr/bin/env python3
"""
Ackermann cmd_vel preprocessor for Gazebo simulation.

The Gazebo AckermannSteering plugin doesn't properly handle reverse steering.
When driving in reverse (negative linear.x), the steering angle calculation
assumes forward motion kinematics. This node inverts angular.z when linear.x
is negative to achieve correct reverse steering behavior.

Subscribes to: cmd_vel (geometry_msgs/Twist)
Publishes to: cmd_vel_ackermann (geometry_msgs/Twist)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AckermannCmdVelPreprocessor(Node):
    def __init__(self):
        super().__init__('ackermann_cmd_vel_preprocessor')

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel_ackermann',
            10
        )

        self.get_logger().info('Ackermann cmd_vel preprocessor started')

    def cmd_vel_callback(self, msg: Twist):
        output_msg = Twist()
        output_msg.linear.x = msg.linear.x
        output_msg.linear.y = msg.linear.y
        output_msg.linear.z = msg.linear.z
        output_msg.angular.x = msg.angular.x
        output_msg.angular.y = msg.angular.y

        # Invert angular.z when driving in reverse
        if msg.linear.x < 0:
            output_msg.angular.z = -msg.angular.z
        else:
            output_msg.angular.z = msg.angular.z

        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannCmdVelPreprocessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
