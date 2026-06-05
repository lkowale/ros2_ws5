#!/usr/bin/env python3
"""
Sim adapter: fake ublox UBX-NAV-RELPOSNED from Gazebo ground truth.

Lets the real ``relposned_heading`` node run unchanged in simulation, so the
solbot5 dual-antenna heading path is identical sim↔real. Converts Gazebo
ground-truth yaw to a baseline NED bearing and publishes a fully-populated
RELPOSNED with RTK-fixed flags on ``/ubx_nav_rel_pos_ned``.

The conversion is the exact inverse of relposned_heading:

    bearing_ned = pi/2 - (yaw_enu - offset)

``sim_heading_offset_deg`` injects a deliberate mounting offset so calibration
of relposned_heading's ``heading_offset_deg`` can be exercised in sim. Set both
to 0 for a clean baseline.

Subscribes: /odometry/gazebo  (nav_msgs/Odometry, ground-truth pose)
Publishes:  /ubx_nav_rel_pos_ned  (ublox_ubx_msgs/UBXNavRelPosNED)
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry

from ublox_ubx_msgs.msg import UBXNavRelPosNED, CarrSoln


# True antenna spacing on solbot5 (front gps_link +0.95 X, rear gps_rear_link
# -0.95 X) → 1.90 m baseline.
BASELINE_M = 1.90


class SimRelPosNed(Node):

    def __init__(self):
        super().__init__('sim_relposned_publisher')

        self._offset_rad = math.radians(
            self.declare_parameter('sim_heading_offset_deg', 0.0).value)
        self._noise_deg = self.declare_parameter('heading_noise_deg', 0.0).value
        self._rate_hz = self.declare_parameter('rate_hz', 8.0).value

        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(
            UBXNavRelPosNED, '/ubx_nav_rel_pos_ned', best_effort)

        self._last_yaw = None
        self.create_subscription(Odometry, '/odometry/gazebo', self._cb, 10)
        self.create_timer(1.0 / self._rate_hz, self._tick)

        self.get_logger().info(
            f'sim_relposned_publisher started  '
            f'sim_offset={math.degrees(self._offset_rad):.1f} deg  '
            f'noise={self._noise_deg} deg  rate={self._rate_hz} Hz')

    def _cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._last_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _tick(self):
        if self._last_yaw is None:
            return

        yaw = self._last_yaw
        if self._noise_deg > 0.0:
            yaw += math.radians(random.gauss(0.0, self._noise_deg))

        # ENU yaw → NED compass bearing of the baseline (inverse of the node).
        bearing_ned = _wrap(math.pi / 2.0 - (yaw - self._offset_rad))
        bearing_deg = math.degrees(bearing_ned) % 360.0

        msg = UBXNavRelPosNED()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.rel_pos_length = int(round(BASELINE_M * 100.0))   # cm
        msg.rel_pos_heading = int(round(bearing_deg / 1e-5))  # deg × 1e-5
        msg.acc_heading = int(round(0.3 / 1e-5))              # ~0.3 deg accuracy
        msg.acc_length = 5                                    # mm × 0.1
        msg.gnss_fix_ok = True
        msg.rel_pos_valid = True
        msg.rel_pos_heading_valid = True
        msg.carr_soln = CarrSoln(
            status=CarrSoln.CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES)
        self._pub.publish(msg)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimRelPosNed()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
