#!/usr/bin/env python3
"""
gz_rl_recorder — synchronised Gazebo-world vs RL diagnostic recorder.

Records one CSV row every 0.5 s containing:

  Gazebo world frame (from bridged odometry/gazebo topic — zero subprocess cost):
    robot_gz_x/y/yaw       — solbot5 world pose
    cam_gz_x/y             — camera ground-footprint centre

  ROS RL / map frame (EKF truth):
    robot_map_x/y/yaw      — map→base_footprint TF
    robot_odom_x/y/yaw     — odom→base_footprint TF
    cam_map_x/y            — camera footprint in map frame

  Derived divergence:
    map_vs_gz_x/y/dist     — per-axis and Euclidean map vs gz divergence
    odom_vs_map_x/y        — should be 0 (map==odom)
    map_gz_yaw_diff_deg    — heading divergence between frames

Parameters:
  world_name   (str)   Gazebo world name          default: house_short_crop_rows
  robot_model  (str)   Gazebo model name (unused) default: solbot5
  gz_odom_topic (str)  bridged gz odometry topic  default: odometry/gazebo
  cam_fwd_m    (float) camera fwd offset in body  default: 0.5
  out_csv      (str)   output file path           default: /tmp/gz_rl_record.csv
  rate_hz      (float) recording rate             default: 2.0

Run:
  ros2 run gazebo_crop_rows gz_rl_recorder --ros-args \\
    -p world_name:=house_short_crop_rows \\
    -p out_csv:=/tmp/gz_rl_record.csv
"""

import csv
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def quat_yaw(q):
    return math.atan2(
        2.0 * (q[3] * q[2] + q[0] * q[1]),
        1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2)
    )


def tf_pose(buf, parent, child):
    """Return (x, y, yaw_rad) from TF or None."""
    try:
        t = buf.lookup_transform(parent, child, rclpy.time.Time())
        tr = t.transform.translation
        ro = t.transform.rotation
        yaw = quat_yaw((ro.x, ro.y, ro.z, ro.w))
        return tr.x, tr.y, yaw
    except (LookupException, ConnectivityException, ExtrapolationException):
        return None


def camera_ground_xy(robot_x, robot_y, robot_yaw, cam_fwd_m):
    cx = robot_x + cam_fwd_m * math.cos(robot_yaw)
    cy = robot_y + cam_fwd_m * math.sin(robot_yaw)
    return cx, cy


FIELDS = [
    'time_s',
    # Gazebo world frame (from bridged odometry topic)
    'robot_gz_x', 'robot_gz_y', 'robot_gz_yaw_deg',
    'cam_gz_x', 'cam_gz_y',
    # RL / map frame
    'robot_map_x', 'robot_map_y', 'robot_map_yaw_deg',
    'robot_odom_x', 'robot_odom_y', 'robot_odom_yaw_deg',
    'cam_map_x', 'cam_map_y',
    # Divergence
    'map_vs_gz_x', 'map_vs_gz_y', 'map_vs_gz_dist',
    'odom_vs_map_x', 'odom_vs_map_y',
    'map_gz_yaw_diff_deg',
]


class GzRlRecorder(Node):
    def __init__(self):
        super().__init__('gz_rl_recorder')

        self.declare_parameter('world_name',    'house_short_crop_rows')
        self.declare_parameter('robot_model',   'solbot5')
        self.declare_parameter('gz_odom_topic', 'odometry/gazebo')
        self.declare_parameter('cam_fwd_m',     0.5)
        self.declare_parameter('out_csv',       '/tmp/gz_rl_record.csv')
        self.declare_parameter('rate_hz',       2.0)

        self._cam_fwd  = self.get_parameter('cam_fwd_m').value
        self._out_csv  = self.get_parameter('out_csv').value
        rate_hz        = self.get_parameter('rate_hz').value
        gz_odom_topic  = self.get_parameter('gz_odom_topic').value

        # Latest Gazebo odometry (bridged from /model/solbot5/odometry)
        self._gz_pose = None
        self.create_subscription(Odometry, gz_odom_topic, self._gz_odom_cb, 10)

        self._tf_buf = Buffer()
        self._tf_lst = TransformListener(self._tf_buf, self)

        self._csv_file = open(self._out_csv, 'w', newline='')
        self._writer = csv.DictWriter(self._csv_file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._csv_file.flush()
        self._t0 = time.time()
        self._row_count = 0

        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Recording to {self._out_csv} at {rate_hz} Hz '
            f'(gz pose from {gz_odom_topic})')

    def _gz_odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = quat_yaw((o.x, o.y, o.z, o.w))
        self._gz_pose = (p.x, p.y, yaw)

    def _tick(self):
        now = time.time() - self._t0
        row = {f: '' for f in FIELDS}
        row['time_s'] = f'{now:.2f}'

        # ── Gazebo world pose ───────────────────────────────────────────────
        gz_pose = self._gz_pose
        if gz_pose:
            gx, gy, gyaw = gz_pose
            row['robot_gz_x']       = f'{gx:.4f}'
            row['robot_gz_y']       = f'{gy:.4f}'
            row['robot_gz_yaw_deg'] = f'{math.degrees(gyaw):.2f}'
            cgx, cgy = camera_ground_xy(gx, gy, gyaw, self._cam_fwd)
            row['cam_gz_x'] = f'{cgx:.4f}'
            row['cam_gz_y'] = f'{cgy:.4f}'
        else:
            gx = gy = gyaw = None

        # ── TF: map and odom frame poses ────────────────────────────────────
        map_pose  = tf_pose(self._tf_buf, 'map',  'base_footprint')
        odom_pose = tf_pose(self._tf_buf, 'odom', 'base_footprint')

        if map_pose:
            mx, my, myaw = map_pose
            row['robot_map_x']       = f'{mx:.4f}'
            row['robot_map_y']       = f'{my:.4f}'
            row['robot_map_yaw_deg'] = f'{math.degrees(myaw):.2f}'
            cmx, cmy = camera_ground_xy(mx, my, myaw, self._cam_fwd)
            row['cam_map_x'] = f'{cmx:.4f}'
            row['cam_map_y'] = f'{cmy:.4f}'
        else:
            mx = my = myaw = None

        if odom_pose:
            ox, oy, oyaw = odom_pose
            row['robot_odom_x']       = f'{ox:.4f}'
            row['robot_odom_y']       = f'{oy:.4f}'
            row['robot_odom_yaw_deg'] = f'{math.degrees(oyaw):.2f}'

        # ── Divergence metrics ──────────────────────────────────────────────
        if gx is not None and mx is not None:
            dvx = mx - gx
            dvy = my - gy
            row['map_vs_gz_x']    = f'{dvx:.4f}'
            row['map_vs_gz_y']    = f'{dvy:.4f}'
            row['map_vs_gz_dist'] = f'{math.hypot(dvx, dvy):.4f}'
            yaw_diff = math.degrees(math.atan2(
                math.sin(myaw - gyaw), math.cos(myaw - gyaw)))
            row['map_gz_yaw_diff_deg'] = f'{yaw_diff:.2f}'

        if odom_pose and map_pose:
            row['odom_vs_map_x'] = f'{ox - mx:.4f}'
            row['odom_vs_map_y'] = f'{oy - my:.4f}'

        self._writer.writerow(row)
        self._csv_file.flush()
        self._row_count += 1

        if self._row_count % 10 == 0:
            dvd = row.get('map_vs_gz_dist', '?')
            self.get_logger().info(
                f't={now:.0f}s  map_vs_gz={dvd}m  '
                f'map=({row.get("robot_map_x","?")},{row.get("robot_map_y","?")})  '
                f'gz=({row.get("robot_gz_x","?")},{row.get("robot_gz_y","?")})'
            )

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(f'Saved {self._row_count} rows to {self._out_csv}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GzRlRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
