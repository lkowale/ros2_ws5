#!/usr/bin/env python3
"""
field_nav_logger — CSV logger for field nav debugging.

Records to /tmp/field_nav_log_<timestamp>.csv:
  time_sec, gps_lat, gps_lon, gps_fix, gps_cov_x,
  odom_x, odom_y, odom_heading_deg,
  cmd_vel_lx, cmd_vel_az,
  wheel_FL, wheel_FR, wheel_RL, wheel_RR,
  handbrake_cmd, handbrake_state,
  pause_active, pause_source, pause_reason,
  move_seq_segment, move_seq_dist_traveled

Run standalone:
  ros2 run solbot5_control field_nav_logger
"""

import math
import csv
import time
import os
from datetime import datetime

import rclpy
import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from solbot4_msgs.msg import Pause


class FieldNavLogger(Node):
    def __init__(self):
        super().__init__('field_nav_logger')

        # State
        self._gps_lat = float('nan')
        self._gps_lon = float('nan')
        self._gps_fix = -1
        self._gps_cov_x = float('nan')

        self._odom_x = float('nan')
        self._odom_y = float('nan')
        self._odom_heading_deg = float('nan')

        self._cmd_lx = 0.0
        self._cmd_az = 0.0

        self._wheels = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        self._handbrake_cmd = False
        self._handbrake_state = False

        self._pause_active = False
        self._pause_source = ''
        self._pause_reason = ''

        self._move_seq_segment = -1
        self._move_seq_dist = 0.0

        # Subscriptions
        self.create_subscription(NavSatFix, '/gps/fix', self._cb_gps,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self.create_subscription(Bool, '/handbrake', self._cb_hb_cmd, 10)
        self.create_subscription(Bool, '/handbrake_state', self._cb_hb_state, 10)
        self.create_subscription(Pause, '/pause', self._cb_pause, 10)
        self.create_subscription(String, '/wheel_speeds', self._cb_wheels, 10)
        # MoveSequence feedback comes via action feedback topic
        # Use the move_sequence action feedback - try both topic paths
        try:
            from solbot4_msgs.action import MoveSequence as MS
            self.create_subscription(
                MS.Impl.FeedbackMessage,
                '/move_sequence/_action/feedback',
                self._cb_move_seq_fb, 10)
        except Exception:
            pass

        # CSV setup
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = f'/tmp/field_nav_log_{ts}.csv'
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            'time_sec',
            'gps_lat', 'gps_lon', 'gps_fix', 'gps_cov_x',
            'odom_x', 'odom_y', 'odom_heading_deg',
            'cmd_lx', 'cmd_az',
            'wheel_FL', 'wheel_FR', 'wheel_RL', 'wheel_RR',
            'handbrake_cmd', 'handbrake_state',
            'pause_active', 'pause_source', 'pause_reason',
            'move_seq_segment', 'move_seq_dist_traveled',
        ])

        self.create_timer(0.1, self._tick)  # 10 Hz log rate
        self.get_logger().info(f'field_nav_logger: writing to {self._csv_path}')

    def _cb_gps(self, msg):
        self._gps_lat = msg.latitude
        self._gps_lon = msg.longitude
        self._gps_fix = msg.status.status
        self._gps_cov_x = msg.position_covariance[0]

    def _cb_odom(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_heading_deg = math.degrees(math.atan2(siny, cosy))

    def _cb_cmd_vel(self, msg):
        self._cmd_lx = msg.linear.x
        self._cmd_az = msg.angular.z

    def _cb_hb_cmd(self, msg):
        self._handbrake_cmd = msg.data

    def _cb_hb_state(self, msg):
        self._handbrake_state = msg.data

    def _cb_pause(self, msg):
        self._pause_active = msg.paused
        self._pause_source = msg.source
        self._pause_reason = msg.reason

    def _cb_wheels(self, msg):
        import json
        try:
            d = json.loads(msg.data)
            self._wheels = {k: d.get(k, 0.0) for k in ('FL', 'FR', 'RL', 'RR')}
        except Exception:
            pass

    def _cb_move_seq_fb(self, msg):
        self._move_seq_segment = msg.feedback.current_segment
        self._move_seq_dist = msg.feedback.distance_traveled

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self._writer.writerow([
            f'{now:.3f}',
            f'{self._gps_lat:.8f}', f'{self._gps_lon:.8f}',
            self._gps_fix, f'{self._gps_cov_x:.4f}',
            f'{self._odom_x:.4f}', f'{self._odom_y:.4f}',
            f'{self._odom_heading_deg:.2f}',
            f'{self._cmd_lx:.3f}', f'{self._cmd_az:.3f}',
            f'{self._wheels["FL"]:.2f}', f'{self._wheels["FR"]:.2f}',
            f'{self._wheels["RL"]:.2f}', f'{self._wheels["RR"]:.2f}',
            int(self._handbrake_cmd), int(self._handbrake_state),
            int(self._pause_active), self._pause_source, self._pause_reason,
            self._move_seq_segment, f'{self._move_seq_dist:.3f}',
        ])
        self._csv_file.flush()

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(f'field_nav_logger: closed {self._csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FieldNavLogger()
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
