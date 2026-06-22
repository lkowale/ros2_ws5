#!/usr/bin/env python3
"""Republish TF frames as PoseStamped topics for lag-free Mapviz display.

Mapviz's TF plugin calls GetTransform(now()) which fails under sim time
during clock hiccups. This node uses lookup_transform with Time(0) (latest
available) and publishes stable PoseStamped topics instead.

Topics published (20 Hz):
  /robot_pose      — map → base_footprint  (rear axle / Ackermann pivot / RS planner reference)
  /tool_link_pose  — map → tool_link       (alias for base_footprint; same position)
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

        self._pubs = {
            'base_footprint': self.create_publisher(PoseStamped, '/robot_pose', 1),
            'tool_link':      self.create_publisher(PoseStamped, '/tool_link_pose', 1),
        }

        self.create_timer(0.05, self._cb)  # 20 Hz

    def _cb(self):
        for frame, pub in self._pubs.items():
            try:
                t = self._tf_buf.lookup_transform('map', frame, Time())
                pose = PoseStamped()
                pose.header = t.header
                pose.pose.position.x = t.transform.translation.x
                pose.pose.position.y = t.transform.translation.y
                pose.pose.position.z = t.transform.translation.z
                pose.pose.orientation = t.transform.rotation
                pub.publish(pose)
            except Exception:
                pass


def main():
    rclpy.init()
    rclpy.spin(TfPosePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
