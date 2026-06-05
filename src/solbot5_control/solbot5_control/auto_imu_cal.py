#!/usr/bin/env python3
"""
auto_imu_cal — continuous IMU yaw offset calibration using line bearing as ground truth.

When the robot is running a swath (/swath_path active) and ON_LINE (/ctrl_mode) with
cross-track error < threshold, the known geojson line bearing is used as ground truth:

    offset_sample = line_bearing - imu_yaw_360

A sliding window of N samples is maintained. When stdev < stdev_threshold, the median
is applied as a correction via /imu_yaw_offset. imu_bridge applies it immediately and
saves to config.yaml.

Runs continuously — re-collects and re-applies on every good opportunity.
Publishes status to /auto_imu_cal/status (JSON string).

Parameters:
    geojson_file      : path to line geojson (required)
    window_size       : sliding window size (default 8)
    stdev_threshold   : max stdev° to accept correction (default 5.0)
    cross_track_max_m : max cross-track to geojson line (default 0.10)
    swath_timeout_s   : max age of /swath_path before ignoring (default 60.0)
    min_apply_interval_s : minimum seconds between offset applications (default 30.0)
"""

import json
import math
import os
import time
import collections
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String


def _bearing_deg(lon1, lat1, lon2, lat2):
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360.0


def _cross_track_m(lat, lon, lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2); phi = math.radians(lat)
    lam1 = math.radians(lon1); lam2 = math.radians(lon2); lam = math.radians(lon)
    d13 = 2 * math.asin(math.sqrt(
        math.sin((phi - phi1) / 2)**2 +
        math.cos(phi1) * math.cos(phi) * math.sin((lam - lam1) / 2)**2))
    t13 = math.atan2(
        math.sin(lam - lam1) * math.cos(phi),
        math.cos(phi1) * math.sin(phi) - math.sin(phi1) * math.cos(phi) * math.cos(lam - lam1))
    t12 = math.atan2(
        math.sin(lam2 - lam1) * math.cos(phi2),
        math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(lam2 - lam1))
    return math.asin(math.sin(d13) * math.sin(t13 - t12)) * R


def _adiff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def _normalize_rad(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class AutoImuCal(Node):

    def __init__(self):
        super().__init__('auto_imu_cal')

        geojson = self.declare_parameter('geojson_file', '').value
        self._window      = self.declare_parameter('window_size',          8).value
        self._stdev_thr   = self.declare_parameter('stdev_threshold',      5.0).value
        self._ct_max      = self.declare_parameter('cross_track_max_m',    0.10).value
        self._swath_tmo   = self.declare_parameter('swath_timeout_s',      60.0).value
        self._min_apply_interval = self.declare_parameter('min_apply_interval_s', 30.0).value

        if not geojson:
            self.get_logger().error('geojson_file parameter required')
            raise RuntimeError('geojson_file not set')

        with open(os.path.expanduser(geojson)) as f:
            data = json.load(f)
        coords = data['features'][0]['geometry']['coordinates']
        self._lon1, self._lat1 = coords[0][0],  coords[0][1]
        self._lon2, self._lat2 = coords[-1][0], coords[-1][1]
        self._fwd = _bearing_deg(self._lon1, self._lat1, self._lon2, self._lat2)
        self._rev = (self._fwd + 180.0) % 360.0

        self.get_logger().info(
            f'auto_imu_cal started  fwd={self._fwd:.1f}°  rev={self._rev:.1f}°  '
            f'window={self._window}  stdev<{self._stdev_thr}°  ct<{self._ct_max*100:.0f}cm')

        self._imu_yaw    = None
        self._gps_lat    = None
        self._gps_lon    = None
        self._ctrl_mode  = None
        self._swath_last = 0.0
        self._settled    = 0
        self._window_buf = collections.deque(maxlen=self._window)
        self._last_apply = 0.0
        self._apply_count = 0

        self._offset_pub = self.create_publisher(Float32, '/imu_yaw_offset', 10)
        self._status_pub = self.create_publisher(String,  '/auto_imu_cal/status', 10)

        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Imu,       '/imu',           self._cb_imu,   best_effort)
        self.create_subscription(NavSatFix, '/gps/fix',       self._cb_gps,   best_effort)
        self.create_subscription(String,    '/ctrl_mode',     self._cb_mode,  10)
        self.create_subscription(Path,      '/swath_path',    self._cb_swath, transient)

        self.create_timer(0.2, self._tick)
        self.create_timer(2.0, self._publish_status)

    def _cb_imu(self, msg):
        q = msg.orientation
        self._imu_yaw = math.degrees(math.atan2(
            2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))

    def _cb_gps(self, msg):
        self._gps_lat = msg.latitude
        self._gps_lon = msg.longitude

    def _cb_mode(self, msg):
        if msg.data != 'ON_LINE':
            self._settled = 0
        self._ctrl_mode = msg.data

    def _cb_swath(self, msg):
        # Ignore stale TRANSIENT_LOCAL replays from previous runs.
        # Accept only if the message stamp is within 30s of wall time.
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall_sec = time.time()
        if stamp_sec > 0 and abs(wall_sec - stamp_sec) > 30.0:
            return
        self._swath_last = time.monotonic()

    def _tick(self):
        # Gate 1: ctrl_mode ON_LINE
        if self._ctrl_mode != 'ON_LINE':
            return
        # Gate 2: swath active
        if time.monotonic() - self._swath_last > self._swath_tmo:
            self._settled = 0
            return
        # Gate 3: data available
        if self._gps_lat is None or self._imu_yaw is None:
            return
        # Gate 4: cross-track
        ct = _cross_track_m(
            self._gps_lat, self._gps_lon,
            self._lat1, self._lon1, self._lat2, self._lon2)
        if abs(ct) > self._ct_max:
            self._settled = 0
            return

        self._settled += 1
        if self._settled < 5:   # require 1s settled before collecting
            return

        # Determine direction and compute offset sample
        imu_360 = self._imu_yaw % 360.0
        bearing = self._fwd if abs(_adiff(self._fwd, imu_360)) <= abs(_adiff(self._rev, imu_360)) else self._rev
        offset = _adiff(bearing, imu_360)
        self._window_buf.append(offset)

        if len(self._window_buf) < self._window:
            return

        # Check convergence
        stdev = statistics.stdev(self._window_buf)
        if stdev > self._stdev_thr:
            return

        # Rate limit applications
        now = time.monotonic()
        if now - self._last_apply < self._min_apply_interval:
            return

        # Apply correction
        median = statistics.median(self._window_buf)
        self._apply_correction(median, stdev)
        self._last_apply = now
        self._apply_count += 1
        self._window_buf.clear()

    def _apply_correction(self, correction_deg, stdev_deg):
        # correction_deg = line_bearing - imu_yaw = amount to add to current offset
        # But imu_yaw already has old_offset applied, so:
        # new_offset = old_offset + correction_deg  (in degrees, then convert)
        import yaml
        config_path = os.path.expanduser(
            '~/ros2_ws4/src/control/solbot5_control/config/config.yaml')
        try:
            with open(config_path) as f:
                cfg = yaml.safe_load(f) or {}
            old_offset_rad = float(cfg.get('imu', {}).get('yaw_offset', 0.0))
        except Exception:
            old_offset_rad = 0.0

        old_deg = math.degrees(old_offset_rad)
        new_deg = old_deg + correction_deg
        new_rad = _normalize_rad(math.radians(new_deg))

        # Publish to heading_fuser — applied immediately at runtime
        msg = Float32()
        msg.data = float(new_rad)
        self._offset_pub.publish(msg)

        # Persist to config.yaml so it survives restarts
        try:
            with open(config_path) as f:
                cfg = yaml.safe_load(f) or {}
            cfg.setdefault('imu', {})['yaw_offset'] = round(float(new_rad), 6)
            with open(config_path, 'w') as f:
                yaml.dump(cfg, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f'auto_imu_cal: failed to save config: {e}')

        self.get_logger().info(
            f'━━━ auto_imu_cal APPLIED #{self._apply_count + 1} ━━━')
        self.get_logger().info(
            f'  old={old_deg:+.2f}°  correction={correction_deg:+.2f}°  '
            f'new={new_deg:+.2f}°  stdev={stdev_deg:.2f}°  n={self._window}')

    def _publish_status(self):
        buf = list(self._window_buf)
        status = {
            'state': 'applied' if self._apply_count > 0 and not buf else 'collecting',
            'window': len(buf),
            'window_size': self._window,
            'settled': self._settled,
            'apply_count': self._apply_count,
            'ctrl_mode': self._ctrl_mode,
            'stdev_deg': round(statistics.stdev(buf), 2) if len(buf) > 1 else None,
            'median_deg': round(statistics.median(buf), 2) if buf else None,
            'imu_yaw_deg': round(self._imu_yaw, 1) if self._imu_yaw is not None else None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutoImuCal()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
