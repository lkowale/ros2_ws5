#!/usr/bin/env python3
"""
gps_to_map — convert GPS fix directly to map-frame odometry for EKF.

Subscribes to /gps/fix (NavSatFix) and publishes /odometry/gps_map (Odometry)
in the map frame using a local ENU projection from the first GPS fix.

This breaks the circular dependency in the dual EKF architecture:
- navsat_transform publishes odometry/gps in the odom frame
- The map EKF (world_frame=map) must transform odom→map using its own
  map→odom output, creating a positive feedback loop with absolute GPS
- This node publishes GPS positions directly in the map frame, so the
  map EKF uses them without any frame transform → no feedback loop
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


# WGS84 ellipsoid
_A = 6378137.0            # semi-major axis (m)
_F = 1.0 / 298.257223563  # flattening
_E2 = 2 * _F - _F * _F    # first eccentricity squared


def _geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Convert geodetic (lat, lon, alt) to local ENU relative to (lat0, lon0, alt0).

    Uses closed-form ECEF intermediate, accurate to <1cm within 10km of origin.
    """
    # Geodetic to ECEF
    def _to_ecef(la, lo, al):
        la_r = math.radians(la)
        lo_r = math.radians(lo)
        sin_la = math.sin(la_r)
        cos_la = math.cos(la_r)
        sin_lo = math.sin(lo_r)
        cos_lo = math.cos(lo_r)
        n = _A / math.sqrt(1.0 - _E2 * sin_la * sin_la)
        x = (n + al) * cos_la * cos_lo
        y = (n + al) * cos_la * sin_lo
        z = (n * (1.0 - _E2) + al) * sin_la
        return x, y, z

    x, y, z = _to_ecef(lat, lon, alt)
    x0, y0, z0 = _to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0

    lat0_r = math.radians(lat0)
    lon0_r = math.radians(lon0)
    sin_lat = math.sin(lat0_r)
    cos_lat = math.cos(lat0_r)
    sin_lon = math.sin(lon0_r)
    cos_lon = math.cos(lon0_r)

    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz

    return e, n, u


class GpsToMap(Node):

    def __init__(self):
        super().__init__('gps_to_map')

        self._pos_cov = self.declare_parameter('position_covariance', 2.0).value

        self._lat0 = None
        self._lon0 = None
        self._alt0 = None

        self._pub = self.create_publisher(Odometry, '/odometry/gps_map', 10)

        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(NavSatFix, '/gps/fix', self._cb_fix, best_effort)

        self.get_logger().info(
            f'gps_to_map started  pos_cov={self._pos_cov}')

    def _cb_fix(self, msg):
        if msg.status.status < 0:
            return  # no fix

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        # Use first fix as ENU origin
        if self._lat0 is None:
            self._lat0 = lat
            self._lon0 = lon
            self._alt0 = alt
            self.get_logger().info(
                f'ENU datum set: lat={lat:.8f} lon={lon:.8f} alt={alt:.2f}')

        e, n, u = _geodetic_to_enu(lat, lon, alt, self._lat0, self._lon0, self._alt0)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = e
        odom.pose.pose.position.y = n
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # Use GPS-reported covariance if available, otherwise use parameter
        cov = [0.0] * 36
        if msg.position_covariance_type > 0 and msg.position_covariance[0] < 100.0:
            cov[0] = msg.position_covariance[0]   # east variance
            cov[7] = msg.position_covariance[4]   # north variance
        else:
            cov[0] = self._pos_cov
            cov[7] = self._pos_cov
        cov[14] = 1e6  # z — don't care (2D mode)
        for i in range(3, 6):
            cov[i * 7] = 1e6  # orientation — not set
        odom.pose.covariance = cov

        # Twist not set — only position is meaningful
        twist_cov = [0.0] * 36
        for i in range(6):
            twist_cov[i * 7] = 1e6
        odom.twist.covariance = twist_cov

        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GpsToMap()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
