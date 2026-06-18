#!/usr/bin/env python3
"""Publish start (green) and goal (red) arrow markers for every NavigateToPose goal.

Listens to:
  /received_global_plan  — nav_msgs/Path: first pose = path start, last pose = goal
  /odom                  — EKF odometry: robot pose at goal receipt time

Publishes visualization_msgs/MarkerArray on /goal_markers (transient_local).
All markers accumulate so the full test suite is visible at once in mapviz.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray


_TL_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST, depth=100,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _arrow(mid, ns, frame, x, y, yaw, r, g, b, length=2.5):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.1
    m.pose.orientation.z = math.sin(yaw / 2.0)
    m.pose.orientation.w = math.cos(yaw / 2.0)
    m.scale.x = length  # shaft length
    m.scale.y = 0.4     # shaft diameter
    m.scale.z = 0.4     # head diameter
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
        self._last_goal_xy: tuple | None = None  # (gx, gy) of last published goal

        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)

        # /received_global_plan is republished every controller cycle — deduplicate
        # by goal position (last pose) so we only emit markers on a genuinely new goal.
        self.create_subscription(Path, '/received_global_plan',
                                 self._cb_plan, 10)

        self.get_logger().info('goal_marker_publisher ready → /goal_markers')

    def _cb_odom(self, msg: Odometry):
        self._latest_odom = msg

    def _cb_plan(self, msg: Path):
        if not msg.poses:
            return

        # Deduplicate by goal position — controller republishes the same plan every
        # cycle with a fresh stamp, so stamp-based dedup doesn't work.
        gx_r = round(gx, 2)
        gy_r = round(gy, 2)
        if self._last_goal_xy == (gx_r, gy_r):
            return
        self._last_goal_xy = (gx_r, gy_r)

        # Goal = last pose of the plan
        goal_p = msg.poses[-1].pose
        gx = goal_p.position.x
        gy = goal_p.position.y
        gyaw = _quat_to_yaw(goal_p.orientation)

        # Start = current robot pose from EKF
        if self._latest_odom is not None:
            sp = self._latest_odom.pose.pose
            sx = sp.position.x
            sy = sp.position.y
            syaw = _quat_to_yaw(sp.orientation)
        else:
            # Fallback: use first path pose
            fp = msg.poses[0].pose
            sx, sy = fp.position.x, fp.position.y
            syaw = _quat_to_yaw(fp.orientation)

        frame = msg.header.frame_id or 'map'
        now = self.get_clock().now().to_msg()

        start_m = _arrow(self._marker_id,     'start', frame,
                         sx, sy, syaw, r=0.0, g=1.0, b=0.0)
        goal_m  = _arrow(self._marker_id + 1, 'goal',  frame,
                         gx, gy, gyaw, r=1.0, g=0.15, b=0.0)
        start_m.header.stamp = now
        goal_m.header.stamp  = now

        self._all_markers += [start_m, goal_m]
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
