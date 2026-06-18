#!/usr/bin/env python3
"""Publish start (green) and goal (red) arrow markers for every NavigateToPose goal.

Listens to:
  /navigate_to_pose/_action/send_goal  — NavigateToPose SendGoal request (goal pose+yaw)
  /odom                                — EKF odometry (start pose at goal receipt time)

Publishes visualization_msgs/MarkerArray on /goal_markers (transient_local).
All markers accumulate so the full test suite is visible at once in mapviz.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action._navigate_to_pose import NavigateToPose_SendGoal_Request
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


_TL_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST, depth=100,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _arrow_marker(mid, ns, frame, x, y, yaw, r, g, b):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.1
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = math.sin(yaw / 2.0)
    m.pose.orientation.w = math.cos(yaw / 2.0)
    m.scale.x = 2.5   # arrow length
    m.scale.y = 0.5   # shaft diameter
    m.scale.z = 0.5   # head diameter
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 1.0
    m.lifetime.sec = 0  # persist forever
    return m


class GoalMarkerPublisher(Node):
    def __init__(self):
        super().__init__('goal_marker_publisher')
        self._pub = self.create_publisher(MarkerArray, '/goal_markers', _TL_QOS)
        self._all_markers: list[Marker] = []
        self._marker_id = 0

        self._latest_odom: Odometry | None = None

        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)

        # NavigateToPose action send_goal carries the full goal PoseStamped
        self.create_subscription(
            NavigateToPose_SendGoal_Request,
            '/navigate_to_pose/_action/send_goal',
            self._cb_send_goal,
            10)

        self.get_logger().info('goal_marker_publisher ready → /goal_markers')

    def _cb_odom(self, msg: Odometry):
        self._latest_odom = msg

    def _cb_send_goal(self, msg: NavigateToPose_SendGoal_Request):
        goal_pose = msg.goal.pose  # geometry_msgs/PoseStamped
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        gyaw = _quat_to_yaw(goal_pose.pose.orientation)

        # Start = current robot pose from latest EKF odom
        if self._latest_odom is not None:
            sx = self._latest_odom.pose.pose.position.x
            sy = self._latest_odom.pose.pose.position.y
            syaw = _quat_to_yaw(self._latest_odom.pose.pose.orientation)
        else:
            sx, sy, syaw = gx, gy, gyaw  # fallback: no odom yet

        frame = goal_pose.header.frame_id or 'map'
        now = self.get_clock().now().to_msg()

        start_m = _arrow_marker(
            self._marker_id,     'start', frame, sx, sy, syaw,
            r=0.0, g=1.0, b=0.0)   # green
        goal_m  = _arrow_marker(
            self._marker_id + 1, 'goal',  frame, gx, gy, gyaw,
            r=1.0, g=0.15, b=0.0)   # red-orange

        start_m.header.stamp = now
        goal_m.header.stamp  = now

        self._all_markers.append(start_m)
        self._all_markers.append(goal_m)
        self._marker_id += 2

        ma = MarkerArray()
        ma.markers = self._all_markers
        self._pub.publish(ma)

        self.get_logger().info(
            f'Goal {self._marker_id // 2}: '
            f'start=({sx:.2f},{sy:.2f},{math.degrees(syaw):.1f}°) '
            f'→ goal=({gx:.2f},{gy:.2f},{math.degrees(gyaw):.1f}°)')


def main():
    rclpy.init()
    node = GoalMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
