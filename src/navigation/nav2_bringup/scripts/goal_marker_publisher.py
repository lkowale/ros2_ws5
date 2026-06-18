#!/usr/bin/env python3
"""Publish start (green) and goal (red) arrow markers for every NavigateToPose goal.

Subscribes to /navigate_to_pose/_action/status + bt_navigator /rosout to get
the start position, and to the action goal topic for the goal pose.
Publishes visualization_msgs/MarkerArray on /goal_markers (transient_local).

Mapviz displays this with mapviz_plugins/marker.
"""
import math
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_SendGoal_Request
from rcl_interfaces.msg import Log as RosoutLog
from visualization_msgs.msg import Marker, MarkerArray


_TL_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST, depth=100,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _arrow_marker(mid, frame, x, y, yaw, r, g, b, ns, scale=1.0):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.1
    m.pose.orientation.z = math.sin(yaw / 2)
    m.pose.orientation.w = math.cos(yaw / 2)
    m.scale.x = 2.0 * scale   # arrow length
    m.scale.y = 0.4 * scale   # arrow width
    m.scale.z = 0.4 * scale   # arrow height
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 1.0
    m.lifetime.sec = 0         # persist until replaced
    return m


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalMarkerPublisher(Node):
    def __init__(self):
        super().__init__('goal_marker_publisher')
        self._pub = self.create_publisher(MarkerArray, '/goal_markers', _TL_QOS)
        self._markers = MarkerArray()
        self._goal_id = 0

        # Parse "Begin navigating from current location (sx, sy) to (gx, gy)"
        # from bt_navigator rosout — this gives us both start and goal in one line.
        self.create_subscription(RosoutLog, '/rosout', self._cb_rosout, 100)

        # Also subscribe to the raw goal topic to get the goal yaw
        # (rosout only has x,y, not heading).
        self.create_subscription(
            PoseStamped,
            '/navigate_to_pose/_action/send_goal',
            self._cb_send_goal, 10)
        self._pending_goal_pose: PoseStamped | None = None

        self.get_logger().info('goal_marker_publisher ready → /goal_markers')

    def _cb_send_goal(self, msg: PoseStamped):
        # Store the latest goal pose so we have the yaw when rosout fires.
        self._pending_goal_pose = msg

    def _cb_rosout(self, msg: RosoutLog):
        if 'bt_navigator' not in msg.name:
            return
        text = msg.msg
        # "Begin navigating from current location (sx, sy) to (gx, gy)"
        m = re.search(
            r'Begin navigating from current location \(([^,]+),\s*([^)]+)\)'
            r' to \(([^,]+),\s*([^)]+)\)', text)
        if not m:
            return

        sx, sy = float(m.group(1)), float(m.group(2))
        gx, gy = float(m.group(3)), float(m.group(4))

        # Goal yaw from the pending pose if available, else point toward goal.
        if self._pending_goal_pose is not None:
            gyaw = _quat_to_yaw(self._pending_goal_pose.pose.orientation)
        else:
            gyaw = math.atan2(gy - sy, gx - sx)

        # Start yaw: point toward goal (we don't have the actual robot yaw here,
        # but the rosout message is fired after navigation starts so EKF is live;
        # pointing start→goal is a reasonable visual indicator).
        syaw = math.atan2(gy - sy, gx - sx)

        frame = 'map'
        base_id = self._goal_id * 2
        self._goal_id += 1

        start_marker = _arrow_marker(
            base_id, frame, sx, sy, syaw,
            r=0.0, g=1.0, b=0.0,   # green = start
            ns='start')
        goal_marker = _arrow_marker(
            base_id + 1, frame, gx, gy, gyaw,
            r=1.0, g=0.2, b=0.0,   # red = goal
            ns='goal')

        # Keep all historical markers so the full test suite is visible at once.
        # Replace any existing marker with the same id.
        existing_ids = {(mk.ns, mk.id) for mk in self._markers.markers}
        for mk in [start_marker, goal_marker]:
            if (mk.ns, mk.id) not in existing_ids:
                self._markers.markers.append(mk)
            else:
                for i, existing in enumerate(self._markers.markers):
                    if existing.ns == mk.ns and existing.id == mk.id:
                        self._markers.markers[i] = mk
                        break

        # Stamp all markers with current time so mapviz accepts them.
        now = self.get_clock().now().to_msg()
        for mk in self._markers.markers:
            mk.header.stamp = now

        self._pub.publish(self._markers)
        self.get_logger().info(
            f'Goal {self._goal_id}: start=({sx:.2f},{sy:.2f}) '
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
