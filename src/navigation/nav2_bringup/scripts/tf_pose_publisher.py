#!/usr/bin/env python3
"""Republish base_footprint TF as PoseStamped on /robot_pose.

Mapviz's robot_image plugin calls GetTransform(now()) which fails under
sim time during clock hiccups. This node uses tf2's lookup_transform with
Time(0) (latest available) and publishes a PoseStamped topic that the
mapviz odometry plugin can display stably instead.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class TfPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_pose_publisher')
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._pub = self.create_publisher(PoseStamped, '/robot_pose', 1)
        self.create_timer(0.05, self._cb)  # 20 Hz

    def _cb(self):
        try:
            t = self._tf_buf.lookup_transform('map', 'base_footprint', Time())
            pose = PoseStamped()
            pose.header = t.header
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            self._pub.publish(pose)
        except Exception:
            pass


def main():
    rclpy.init()
    rclpy.spin(TfPosePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
