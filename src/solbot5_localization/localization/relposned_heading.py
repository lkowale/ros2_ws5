#!/usr/bin/env python3
"""
relposned_heading — convert ublox UBX-NAV-RELPOSNED to a heading IMU for the EKF.

solbot5 carries two GNSS antennas (gps_link front, gps_rear_link rear). The
ublox moving-base + rover pair produces a RELPOSNED message whose
``rel_pos_heading`` is the absolute bearing of the antenna baseline. Unlike
solbot4's GPS-velocity heading, this is valid at standstill and while reversing,
so we publish it continuously (gated only on solution quality).

The heading is published as an ``Imu`` on ``/imu/gps_heading`` with only yaw
valid — identical interface to solbot4's gps_vel_odom, so the EKF config stays
the same.

Frame conversion
----------------
``rel_pos_heading`` is a NED compass bearing (0 = North, clockwise positive) of
the baseline vector (rover relative to base). ROS uses ENU yaw (0 = East,
counter-clockwise positive):

    yaw_enu = pi/2 - bearing_ned

The antenna baseline is not necessarily aligned with the robot's +X axis (and
which antenna is base vs rover is a wiring detail), so a configurable
``heading_offset_deg`` is added and calibrated in simulation against
ground-truth yaw.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion

from ublox_ubx_msgs.msg import UBXNavRelPosNED, CarrSoln


class RelPosNedHeading(Node):

    def __init__(self):
        super().__init__('relposned_heading')

        # Mounting/wiring offset between the antenna baseline and robot +X,
        # calibrated in sim against ground-truth yaw. Degrees, ENU.
        self._offset_rad = math.radians(
            self.declare_parameter('heading_offset_deg', 0.0).value)

        # Covariance (rad^2) used when carrier solution is RTK-fixed. Float/None
        # solutions are inflated by the multipliers below.
        self._fixed_cov = self.declare_parameter('heading_covariance', 0.0003).value  # ~1 deg
        self._float_mult = self.declare_parameter('float_cov_multiplier', 25.0).value
        self._use_acc_heading = self.declare_parameter('use_acc_heading', True).value

        # Plausibility gate on baseline length (cm). The true antenna spacing is
        # 1.90 m; reject wildly wrong baselines that indicate a bad solution.
        self._min_len_cm = self.declare_parameter('min_baseline_cm', 100.0).value
        self._max_len_cm = self.declare_parameter('max_baseline_cm', 300.0).value

        self._require_fixed = self.declare_parameter('require_rtk_fixed', False).value

        self._frame_id = self.declare_parameter('frame_id', 'base_footprint').value

        self._heading_pub = self.create_publisher(Imu, '/imu/gps_heading', 10)
        self._viz_pub = self.create_publisher(PoseStamped, '/gps_baseline_heading', 10)

        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            UBXNavRelPosNED, '/ubx_nav_rel_pos_ned', self._cb_relpos, best_effort)

        self._n_pub = 0
        self._n_rejected = 0
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f'relposned_heading started  offset={math.degrees(self._offset_rad):.1f} deg  '
            f'fixed_cov={self._fixed_cov}  require_fixed={self._require_fixed}  '
            f'baseline=[{self._min_len_cm:.0f},{self._max_len_cm:.0f}] cm')

    def _log_stats(self):
        if self._n_pub or self._n_rejected:
            self.get_logger().info(
                f'heading: {self._n_pub} published, {self._n_rejected} rejected (last 5 s)')
            self._n_pub = 0
            self._n_rejected = 0

    def _cb_relpos(self, msg):
        # Validity gates.
        if not (msg.gnss_fix_ok and msg.rel_pos_valid and msg.rel_pos_heading_valid):
            self._n_rejected += 1
            return

        carr = msg.carr_soln.status
        if self._require_fixed and carr != CarrSoln.CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES:
            self._n_rejected += 1
            return

        if not (self._min_len_cm <= msg.rel_pos_length <= self._max_len_cm):
            self._n_rejected += 1
            return

        # NED compass bearing (deg) → ENU yaw (rad), plus mounting offset.
        bearing_ned = math.radians(msg.rel_pos_heading * 1e-5)
        yaw = _wrap(math.pi / 2.0 - bearing_ned + self._offset_rad)

        # Covariance: prefer the receiver's reported accuracy if enabled,
        # otherwise a fixed value inflated for non-fixed solutions.
        if self._use_acc_heading and msg.acc_heading > 0:
            sigma = math.radians(msg.acc_heading * 1e-5)
            cov = max(sigma * sigma, self._fixed_cov)
        else:
            cov = self._fixed_cov
        if carr != CarrSoln.CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES:
            cov *= self._float_mult

        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self._frame_id
        imu.orientation = _yaw_to_quat(yaw)
        imu.orientation_covariance = [1e6, 0.0, 0.0,
                                      0.0, 1e6, 0.0,
                                      0.0, 0.0, cov]
        imu.angular_velocity_covariance = [-1.0] + [0.0] * 8
        imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        self._heading_pub.publish(imu)
        self._n_pub += 1

        viz = PoseStamped()
        viz.header.stamp = msg.header.stamp
        viz.header.frame_id = 'base_footprint'
        viz.pose.orientation = imu.orientation
        self._viz_pub.publish(viz)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RelPosNedHeading()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
