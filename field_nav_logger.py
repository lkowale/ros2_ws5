#!/usr/bin/env python3
"""
field_nav_logger.py — CSV logger for field_nav / RS planner headland turns.

Writes to ~/ros2_ws5/logs/field_sim/field_nav_<timestamp>.csv at 5 Hz.

Columns:
  t_s            wall-clock seconds since logger start
  ros_t_s        ROS time (seconds)
  -- GPS --
  lat, lon
  fix_status     NavSatFix status (-1=none 0=fix 1=sbas 2=gbas)
  -- Pose (TF: map → base_footprint) --
  map_x, map_y, map_yaw_deg
  -- Odometry --
  odom_vx        forward velocity (m/s)
  odom_wz        yaw rate (rad/s)
  -- Commands --
  cmd_vx, cmd_wz
  -- RS planner --
  rs_constraint  last /rs_planner_constraints message
  -- Nav stage (inferred from active action goal status) --
  nav_status     IDLE / EXECUTING / SUCCEEDED / ABORTED / CANCELED

Run (new terminal, sim already running):
  source ~/ros2_ws5/install/setup.bash
  python3 ~/ros2_ws5/field_nav_logger.py
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                        QoSProfile, QoSReliabilityPolicy)

from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

_BEST_EFFORT = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

_LATCHED = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

CSV_COLUMNS = [
    't_s', 'ros_t_s',
    'lat', 'lon', 'fix_status',
    'map_x', 'map_y', 'map_yaw_deg',
    'odom_vx', 'odom_wz',
    'cmd_vx', 'cmd_wz',
    'rs_constraint',
    'nav_status',
]

_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN:   'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED:  'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED:  'CANCELED',
    GoalStatus.STATUS_ABORTED:   'ABORTED',
}


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


class FieldNavLogger(Node):

    def __init__(self):
        super().__init__('field_nav_logger')

        log_dir = os.path.expanduser('~/ros2_ws5/logs/field_sim')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(log_dir, f'field_nav_{ts}.csv')

        self._csv_f = open(csv_path, 'w', newline='')
        self._writer = csv.DictWriter(self._csv_f, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._csv_f.flush()

        self._t0 = time.time()
        self._row = {c: '' for c in CSV_COLUMNS}

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self.create_subscription(NavSatFix, '/gps/fix',     self._cb_fix,    _BEST_EFFORT)
        self.create_subscription(Odometry,  '/odom',        self._cb_odom,   10)
        self.create_subscription(Twist,     '/cmd_vel',     self._cb_cmd,    10)
        self.create_subscription(String,    '/rs_planner_constraints',
                                 self._cb_rs, _LATCHED)
        self.create_subscription(GoalStatusArray, '/run_field/_action/status',
                                 self._cb_status, 10)

        self.create_timer(0.2, self._write_row)  # 5 Hz

        self.get_logger().info(f'Logging to {csv_path}')

    def _cb_fix(self, msg: NavSatFix):
        self._row['lat'] = f'{msg.latitude:.9f}'
        self._row['lon'] = f'{msg.longitude:.9f}'
        self._row['fix_status'] = str(msg.status.status)

    def _cb_odom(self, msg: Odometry):
        self._row['odom_vx'] = f'{msg.twist.twist.linear.x:.4f}'
        self._row['odom_wz'] = f'{msg.twist.twist.angular.z:.4f}'

    def _cb_cmd(self, msg: Twist):
        self._row['cmd_vx'] = f'{msg.linear.x:.4f}'
        self._row['cmd_wz'] = f'{msg.angular.z:.4f}'

    def _cb_rs(self, msg: String):
        self._row['rs_constraint'] = msg.data

    def _cb_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            self._row['nav_status'] = 'IDLE'
            return
        # report the most active status
        priority = [
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_SUCCEEDED,
        ]
        statuses = {s.status for s in msg.status_list}
        for p in priority:
            if p in statuses:
                self._row['nav_status'] = _STATUS_NAMES.get(p, str(p))
                return

    def _write_row(self):
        # TF lookup: map → base_footprint
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            t = tf.transform.translation
            self._row['map_x'] = f'{t.x:.4f}'
            self._row['map_y'] = f'{t.y:.4f}'
            self._row['map_yaw_deg'] = f'{_yaw_from_quat(tf.transform.rotation):.2f}'
        except TransformException:
            pass

        now = time.time()
        self._row['t_s'] = f'{now - self._t0:.3f}'
        self._row['ros_t_s'] = f'{self.get_clock().now().nanoseconds / 1e9:.3f}'
        self._writer.writerow(self._row)
        self._csv_f.flush()

    def destroy_node(self):
        self._csv_f.close()
        super().destroy_node()


def main():
    rclpy.init()
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
