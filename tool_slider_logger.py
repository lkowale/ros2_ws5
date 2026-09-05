#!/usr/bin/env python3
"""
tool_slider_logger.py — CSV logger for the tool slider vision/control chain.

Writes to ~/ros2_ws5/logs/tool_slider/tool_slider_<timestamp>.csv at 20 Hz.

Columns:
  t_s               wall-clock seconds since logger start
  ros_t_s           ROS time (seconds)
  plant_row_offset  /plant_row_offset (m, +right; NaN if row lost) — vision
                    node's target: row-midpoint minus implement position
  tool_bar_cmd      /tool_bar_position_cmd (m, +right) — controller output
  tool_bar_pos      /tool_bar_position (m, +right) — actuator's lagged actual
                    position, fed back from tool_actuator_sim
  row_lost          /tool_slider/row_lost (bool)
  nav_stage         current_stage from /run_field's action feedback:
                    approach / swath / turn / '' (no goal active). PID tuning
                    should only be evaluated on nav_stage=='swath' rows — the
                    implement has nothing to track during approach/turn, so
                    offset there is not a meaningful tracking-error sample.
  -- from /crop_row_vision/diagnostics (raw per-frame detector state) --
  img_w, img_h      camera image size (px)
  row_mid_px        chosen row-midpoint pixel column (blank if not found)
  implement_px      chosen implement marker pixel column (blank if not found)
  n_greens          number of green (crop row) blobs detected this frame
  n_implements      number of blue (implement_center) blobs detected this frame
  greens_json       raw [{cx,cy,area}, ...] for every green blob
  implements_json   raw [{cx,cy,area}, ...] for every blue implement blob

Run (new terminal, sim already running):
  source ~/ros2_ws5/install/setup.bash
  python3 ~/ros2_ws5/tool_slider_logger.py
"""

import csv
import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, Float32, String
from solbot5_msgs.action import RunField

_BEST_EFFORT = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

CSV_COLUMNS = [
    't_s', 'ros_t_s',
    'plant_row_offset', 'tool_bar_cmd', 'tool_bar_pos', 'row_lost', 'nav_stage',
    'img_w', 'img_h', 'row_mid_px', 'implement_px', 'n_greens', 'n_implements',
    'greens_json', 'implements_json',
]


class ToolSliderLogger(Node):
    def __init__(self):
        super().__init__('tool_slider_logger')

        log_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws5', 'logs', 'tool_slider')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(log_dir, f'tool_slider_{ts}.csv')
        latest_path = os.path.join(log_dir, 'latest.csv')

        self._csv_f = open(csv_path, 'w', newline='')
        self._writer = csv.DictWriter(self._csv_f, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._csv_f.flush()

        try:
            if os.path.lexists(latest_path):
                os.remove(latest_path)
            os.symlink(csv_path, latest_path)
        except OSError:
            pass

        self._start_wall = time.time()

        self._plant_row_offset = math.nan
        self._tool_bar_cmd = math.nan
        self._tool_bar_pos = math.nan
        self._row_lost = False
        self._diag = {}
        self._nav_stage = ''

        self.create_subscription(
            Float32, 'plant_row_offset', self._cb_row_offset, _BEST_EFFORT)
        self.create_subscription(
            Float32, 'tool_bar_position_cmd', self._cb_cmd, _BEST_EFFORT)
        self.create_subscription(
            Float32, 'tool_bar_position', self._cb_pos, _BEST_EFFORT)
        self.create_subscription(
            Bool, 'tool_slider/row_lost', self._cb_row_lost, _BEST_EFFORT)
        self.create_subscription(
            String, 'crop_row_vision/diagnostics', self._cb_diag, _BEST_EFFORT)
        # Action feedback is published as a plain topic under <action>/_action/feedback —
        # subscribe directly rather than via ActionClient so we don't need to own the goal.
        self.create_subscription(
            RunField.Impl.FeedbackMessage, 'run_field/_action/feedback',
            self._cb_nav_feedback, 10)

        self.create_timer(1.0 / 20.0, self._tick)

        self.get_logger().info(f'Logging to {csv_path}')

    def _cb_row_offset(self, msg):
        self._plant_row_offset = msg.data

    def _cb_cmd(self, msg):
        self._tool_bar_cmd = msg.data

    def _cb_pos(self, msg):
        self._tool_bar_pos = msg.data

    def _cb_row_lost(self, msg):
        self._row_lost = msg.data

    def _cb_diag(self, msg):
        try:
            self._diag = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            self._diag = {}

    def _cb_nav_feedback(self, msg):
        self._nav_stage = msg.feedback.current_stage

    def _tick(self):
        d = self._diag
        greens = d.get('greens', [])
        implements = d.get('implements', [])
        row = {
            't_s': f'{time.time() - self._start_wall:.3f}',
            'ros_t_s': f'{self.get_clock().now().nanoseconds / 1e9:.3f}',
            'plant_row_offset': f'{self._plant_row_offset:.4f}',
            'tool_bar_cmd': f'{self._tool_bar_cmd:.4f}',
            'tool_bar_pos': f'{self._tool_bar_pos:.4f}',
            'row_lost': int(self._row_lost),
            'nav_stage': self._nav_stage,
            'img_w': d.get('img_w', ''),
            'img_h': d.get('img_h', ''),
            'row_mid_px': d.get('row_mid_px', ''),
            'implement_px': d.get('implement_px', ''),
            'n_greens': len(greens),
            'n_implements': len(implements),
            'greens_json': json.dumps(greens),
            'implements_json': json.dumps(implements),
        }
        self._writer.writerow(row)
        self._csv_f.flush()

    def destroy_node(self):
        try:
            self._csv_f.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ToolSliderLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
