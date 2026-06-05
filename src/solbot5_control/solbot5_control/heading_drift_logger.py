#!/usr/bin/env python3
"""
heading_drift_logger — CSV logger for heading pipeline investigation.

Records every signal in the heading pipeline from BNO080 → heading_fuser →
EKF → /odom at 10 Hz. Use during field nav to capture heading drift events.

Columns:
  time_sec
  -- Raw IMU (/imu) --
  imu_gyro_z            raw yaw rate from BNO080 [rad/s]
  imu_quat_yaw          yaw extracted from BNO080 orientation quaternion [deg]
  -- Heading fuser output (/imu/fused_heading) --
  fused_yaw             heading_fuser output yaw [deg]
  fused_yaw_cov         yaw covariance (0.01=calibrated, 1.0=uncalibrated)
  fused_gyro_z          yaw rate passed through [rad/s]
  -- Heading fuser mode --
  fuser_mode            BOOTSTRAP/LINE/TURN/CALIBRATED string
  -- GPS heading (/imu/gps_heading) --
  gps_heading_yaw       GPS velocity heading [deg] (nan if not published)
  gps_heading_age_s     seconds since last GPS heading publication
  -- Line following (/swath_bearing, /swath_cross_track) --
  swath_bearing_deg     path bearing [deg] (nan if controller silent)
  cross_track_m         cross-track error [m]
  -- navsat_transform output (/odometry/gps) --
  navsat_x              GPS odometry x [m]
  navsat_y              GPS odometry y [m]
  -- EKF output (/odom) --
  odom_x                EKF position x [m]
  odom_y                EKF position y [m]
  odom_yaw              EKF heading yaw [deg]
  odom_yaw_cov          EKF yaw covariance
  odom_vx               EKF linear velocity x [m/s]
  odom_gyro_z           EKF angular velocity z [rad/s]
  -- GPS fix (/gps/fix) --
  gps_lat
  gps_lon
  gps_fix               status (-1=no fix, 0=fix, 1=sbas, 2=rtk float, 3=rtk fix)
  gps_cov_x             position covariance xx [m²]

Run:
  ros2 run solbot5_control heading_drift_logger
"""

import math
import csv
import os
import time
from datetime import datetime

import rclpy
import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String


def _quat_to_yaw_deg(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


class HeadingDriftLogger(Node):
    def __init__(self):
        super().__init__('heading_drift_logger')

        # /imu
        self._imu_gyro_z = float('nan')
        self._imu_quat_yaw = float('nan')

        # /imu/fused_heading
        self._fused_yaw = float('nan')
        self._fused_yaw_cov = float('nan')
        self._fused_gyro_z = float('nan')

        # /heading_fuser/mode
        self._fuser_mode = ''

        # /imu/gps_heading
        self._gps_heading_yaw = float('nan')
        self._gps_heading_time = 0.0

        # /swath_bearing, /swath_cross_track
        self._swath_bearing = float('nan')
        self._swath_bearing_time = 0.0
        self._cross_track = float('nan')

        # /odometry/gps
        self._navsat_x = float('nan')
        self._navsat_y = float('nan')

        # /odom
        self._odom_x = float('nan')
        self._odom_y = float('nan')
        self._odom_yaw = float('nan')
        self._odom_yaw_cov = float('nan')
        self._odom_vx = float('nan')
        self._odom_gyro_z = float('nan')

        # /gps/fix
        self._gps_lat = float('nan')
        self._gps_lon = float('nan')
        self._gps_fix = -1
        self._gps_cov_x = float('nan')

        # Subscriptions
        self.create_subscription(Imu, '/imu', self._cb_imu, 10)
        self.create_subscription(Imu, '/imu/fused_heading', self._cb_fused, 10)
        self.create_subscription(String, '/heading_fuser/mode', self._cb_mode, 10)
        self.create_subscription(Imu, '/imu/gps_heading', self._cb_gps_heading, 10)
        self.create_subscription(Float32, '/swath_bearing', self._cb_bearing, 10)
        self.create_subscription(Float32, '/swath_cross_track', self._cb_cross_track, 10)
        self.create_subscription(Odometry, '/odometry/gps', self._cb_navsat, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(
            NavSatFix, '/gps/fix', self._cb_gps_fix,
            rclpy.qos.qos_profile_sensor_data)

        # CSV
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = f'/tmp/heading_drift_{ts}.csv'
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            'time_sec',
            'imu_gyro_z', 'imu_quat_yaw',
            'fused_yaw', 'fused_yaw_cov', 'fused_gyro_z',
            'fuser_mode',
            'gps_heading_yaw', 'gps_heading_age_s',
            'swath_bearing_deg', 'cross_track_m',
            'navsat_x', 'navsat_y',
            'odom_x', 'odom_y', 'odom_yaw', 'odom_yaw_cov', 'odom_vx', 'odom_gyro_z',
            'gps_lat', 'gps_lon', 'gps_fix', 'gps_cov_x',
        ])

        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'heading_drift_logger: writing to {self._csv_path}')

    def _cb_imu(self, msg: Imu):
        self._imu_gyro_z = msg.angular_velocity.z
        self._imu_quat_yaw = _quat_to_yaw_deg(msg.orientation)

    def _cb_fused(self, msg: Imu):
        self._fused_yaw = _quat_to_yaw_deg(msg.orientation)
        self._fused_yaw_cov = msg.orientation_covariance[8]
        self._fused_gyro_z = msg.angular_velocity.z

    def _cb_mode(self, msg: String):
        self._fuser_mode = msg.data

    def _cb_gps_heading(self, msg: Imu):
        self._gps_heading_yaw = _quat_to_yaw_deg(msg.orientation)
        self._gps_heading_time = time.monotonic()

    def _cb_bearing(self, msg: Float32):
        self._swath_bearing = math.degrees(msg.data)
        self._swath_bearing_time = time.monotonic()

    def _cb_cross_track(self, msg: Float32):
        self._cross_track = msg.data

    def _cb_navsat(self, msg: Odometry):
        self._navsat_x = msg.pose.pose.position.x
        self._navsat_y = msg.pose.pose.position.y

    def _cb_odom(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = _quat_to_yaw_deg(msg.pose.pose.orientation)
        self._odom_yaw_cov = msg.pose.covariance[35]  # yaw-yaw element
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_gyro_z = msg.twist.twist.angular.z

    def _cb_gps_fix(self, msg: NavSatFix):
        self._gps_lat = msg.latitude
        self._gps_lon = msg.longitude
        self._gps_fix = msg.status.status
        self._gps_cov_x = msg.position_covariance[0]

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        mono = time.monotonic()

        gps_heading_age = mono - self._gps_heading_time if self._gps_heading_time > 0 else float('nan')
        # Show bearing as nan if controller is silent
        swath_bearing = self._swath_bearing if (mono - self._swath_bearing_time < 2.0) else float('nan')

        def f(v, p=4):
            return f'{v:.{p}f}' if not math.isnan(v) else 'nan'

        self._writer.writerow([
            f'{now:.3f}',
            f(self._imu_gyro_z, 5), f(self._imu_quat_yaw, 2),
            f(self._fused_yaw, 2), f(self._fused_yaw_cov, 4), f(self._fused_gyro_z, 5),
            self._fuser_mode,
            f(self._gps_heading_yaw, 2), f(gps_heading_age, 1),
            f(swath_bearing, 2), f(self._cross_track, 4),
            f(self._navsat_x, 4), f(self._navsat_y, 4),
            f(self._odom_x, 4), f(self._odom_y, 4),
            f(self._odom_yaw, 2), f(self._odom_yaw_cov, 6),
            f(self._odom_vx, 4), f(self._odom_gyro_z, 5),
            f(self._gps_lat, 8), f(self._gps_lon, 8),
            self._gps_fix, f(self._gps_cov_x, 6),
        ])
        self._csv_file.flush()

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(f'heading_drift_logger: closed {self._csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeadingDriftLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
