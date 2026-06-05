#!/usr/bin/env python3
"""
Correct GPS lateral position for robot roll tilt.

When the robot tilts by roll angle, the GPS antenna (height h above wheel plane,
laterally centred) shifts laterally in the robot body frame by h*sin(roll).
In the map frame this becomes:
    dx = -h * sin(roll) * sin(heading)
    dy =  h * sin(roll) * cos(heading)

Heading source (in priority order):
  1. /swath_bearing  (std_msgs/Float32) — ENU yaw of the active swath segment,
     published by GpsLineFollowerController at the controller rate.  Used while
     fresh (< swath_bearing_timeout_s).  This is the path heading, not the robot
     heading, so it is immune to heading-fuser noise/steps.
  2. /imu/fused_heading — fallback when not on a swath or bearing is stale.

Subscribes:
  /gps/fix           (sensor_msgs/NavSatFix)  — raw GPS
  /imu               (sensor_msgs/Imu)         — for roll
  /swath_bearing     (std_msgs/Float32)        — swath path heading (ENU yaw, rad)
  /imu/fused_heading (sensor_msgs/Imu)         — fallback heading

Publishes:
  /gps/fix_corrected (sensor_msgs/NavSatFix)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32

EARTH_RADIUS = 6378137.0  # WGS-84 semi-major axis [m]

# How long a swath bearing is considered fresh (seconds).
# Controller publishes at ~20 Hz so 0.5 s gives plenty of margin.
_SWATH_BEARING_TIMEOUT_S = 0.5


class GpsRollCorrectionNode(Node):
    def __init__(self):
        super().__init__('gps_roll_correction')

        self.declare_parameter('antenna_height', 0.97)
        self.declare_parameter('enable_roll_correction', True)
        self.antenna_height = self.get_parameter('antenna_height').value
        self.enable_roll_correction = self.get_parameter('enable_roll_correction').value

        self.roll_rad = 0.0
        self._fused_heading_rad = 0.0
        self._swath_bearing_rad = 0.0
        self._swath_bearing_time = 0.0  # monotonic, 0 = never received

        gps_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self.pub = self.create_publisher(NavSatFix, '/gps/fix_corrected', 10)

        self.create_subscription(NavSatFix, '/gps/fix', self._cb_fix, gps_qos)
        self.create_subscription(Imu, '/imu', self._cb_imu_roll, 10)
        self.create_subscription(Float32, '/swath_bearing', self._cb_swath_bearing, 10)
        self.create_subscription(Imu, '/imu/fused_heading', self._cb_imu_heading, 10)

        self.get_logger().info(
            f'gps_roll_correction started  antenna_height={self.antenna_height:.3f}m  '
            f'enable_roll_correction={self.enable_roll_correction}  '
            f'swath_bearing_timeout={_SWATH_BEARING_TIMEOUT_S}s')

    def _cb_imu_roll(self, msg: Imu):
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        self.roll_rad = math.atan2(sinr, cosr)

    def _cb_swath_bearing(self, msg: Float32):
        self._swath_bearing_rad = msg.data
        self._swath_bearing_time = time.monotonic()

    def _cb_imu_heading(self, msg: Imu):
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        self._fused_heading_rad = math.atan2(2.0 * (w * z + x * y),
                                             1.0 - 2.0 * (y * y + z * z))

    def _cb_fix(self, msg: NavSatFix):
        if not self.enable_roll_correction:
            self.pub.publish(msg)
            return

        h = self.antenna_height
        roll = self.roll_rad

        # Use swath path bearing when fresh — immune to heading-fuser steps.
        swath_fresh = (time.monotonic() - self._swath_bearing_time) < _SWATH_BEARING_TIMEOUT_S
        yaw = self._swath_bearing_rad if swath_fresh else self._fused_heading_rad

        lateral_m = h * math.sin(roll)

        dx = -lateral_m * math.sin(yaw)
        dy =  lateral_m * math.cos(yaw)

        lat_rad = math.radians(msg.latitude)
        dlat = math.degrees(dy / EARTH_RADIUS)
        dlon = math.degrees(dx / (EARTH_RADIUS * math.cos(lat_rad)))

        corrected = NavSatFix()
        corrected.header = msg.header
        corrected.status = msg.status
        corrected.latitude  = msg.latitude  + dlat
        corrected.longitude = msg.longitude + dlon
        corrected.altitude  = msg.altitude
        corrected.position_covariance = msg.position_covariance
        corrected.position_covariance_type = msg.position_covariance_type

        self.pub.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = GpsRollCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
