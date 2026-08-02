#!/usr/bin/env python3
"""
gz_rl_recorder — synchronised Gazebo-world vs RL diagnostic recorder.

Records one CSV row every tick containing:

  Gazebo TRUE world frame (from /gz/world_poses TFMessage — dynamic_pose/info bridge):
    robot_gz_x/y/yaw_deg   — solbot5 world pose (ground truth)

  Gazebo odom frame (from odometry/gazebo — Ackermann plugin):
    robot_odom_gz_x/y/yaw_deg — robot in gz odom frame (what spawner uses)

  ROS map frame (EKF):
    robot_map_x/y/yaw_deg  — map→base_footprint TF

  Spawned crop row boxes (from crop_row_spawner/events JSON topic):
    last_spawn_name         — name of most recently spawned box
    last_spawn_map_x/y     — box centre in map frame
    last_spawn_gz_x/y      — box centre in gz world frame (as placed)
    last_spawn_yaw_deg      — box orientation in gz frame
    active_box_count        — number of currently alive boxes

  Camera:
    Images saved as /tmp/gz_rl_frames/<timestamp_ms>.jpg when image arrives

  Derived:
    map_vs_gz_world_x/y    — map position minus gz world position
    gz_odom_vs_world_x/y   — gz odom position minus gz world position
    map_gz_yaw_diff_deg     — map yaw minus gz world yaw

Parameters:
  out_csv       str   output CSV path           default: /tmp/gz_rl_record2.csv
  img_dir       str   camera frame output dir   default: /tmp/gz_rl_frames
  gz_odom_topic str   bridged odom topic        default: odometry/gazebo
  gz_pose_topic str   world pose TF topic       default: /gz/world_poses
  robot_model   str   gz model name             default: solbot5
  rate_hz       float recording rate            default: 4.0

Run:
  ros2 run gazebo_crop_rows gz_rl_recorder --ros-args \\
    -p out_csv:=/tmp/gz_rl_record2.csv \\
    -p img_dir:=/tmp/gz_rl_frames
"""

import csv
import json
import math
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def quat_yaw(q):
    return math.atan2(
        2.0 * (q[3] * q[2] + q[0] * q[1]),
        1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))


def tf_pose(buf, parent, child):
    try:
        t = buf.lookup_transform(parent, child, rclpy.time.Time())
        tr = t.transform.translation
        ro = t.transform.rotation
        return tr.x, tr.y, quat_yaw((ro.x, ro.y, ro.z, ro.w))
    except (LookupException, ConnectivityException, ExtrapolationException):
        return None


FIELDS = [
    'time_s',
    # True Gazebo world frame (from dynamic_pose/info via TFMessage bridge)
    'robot_gz_world_x', 'robot_gz_world_y', 'robot_gz_world_yaw_deg',
    # Gz odom frame (from Ackermann plugin — what spawner uses)
    'robot_gz_odom_x', 'robot_gz_odom_y', 'robot_gz_odom_yaw_deg',
    # EKF map frame
    'robot_map_x', 'robot_map_y', 'robot_map_yaw_deg',
    # Spawned box info (most recent)
    'last_spawn_name',
    'last_spawn_map_x', 'last_spawn_map_y',
    'last_spawn_gz_x', 'last_spawn_gz_y', 'last_spawn_yaw_deg',
    'active_box_count',
    # Derived divergence
    'map_vs_gz_world_x', 'map_vs_gz_world_y',
    'gz_odom_vs_world_x', 'gz_odom_vs_world_y',
    'map_gz_yaw_diff_deg',
    # Camera frame filename (basename only)
    'img_file',
]


class GzRlRecorder(Node):
    def __init__(self):
        super().__init__('gz_rl_recorder')

        self.declare_parameter('out_csv',              '/tmp/gz_rl_record2.csv')
        self.declare_parameter('img_dir',              '/tmp/gz_rl_frames')
        self.declare_parameter('gz_odom_topic',        'odometry/gazebo')
        self.declare_parameter('gz_world_odom_topic',  '/gz/robot_world_odom')
        self.declare_parameter('robot_model',          'solbot5')
        self.declare_parameter('rate_hz',              4.0)

        self._out_csv     = self.get_parameter('out_csv').value
        self._img_dir     = self.get_parameter('img_dir').value
        self._robot_model = self.get_parameter('robot_model').value
        rate_hz           = self.get_parameter('rate_hz').value

        os.makedirs(self._img_dir, exist_ok=True)

        # State
        self._gz_odom_pose  = None   # (x, y, yaw) from Ackermann odometry
        self._gz_world_pose = None   # (x, y, yaw) from dynamic_pose/info TF bridge
        self._last_image    = None   # (cv2_bgr, timestamp_ms)
        self._spawn_events: dict[str, dict] = {}  # name → {map_x, map_y, gz_x, gz_y, yaw}
        self._last_spawn    = None   # most recent spawn event dict

        self._bridge = CvBridge()

        # Subscriptions
        self.create_subscription(
            Odometry, self.get_parameter('gz_odom_topic').value,
            self._gz_odom_cb, 10)

        # True world-frame position from OdometryPublisher (odom_frame=world).
        # Replaces the broken Pose_V TFMessage approach (child_frame_id always empty).
        self.create_subscription(
            Odometry, self.get_parameter('gz_world_odom_topic').value,
            self._gz_world_odom_cb, 10)

        self.create_subscription(
            String, 'crop_row_spawner/events',
            self._spawn_event_cb, 50)

        self.create_subscription(
            Image, '/oakd/rgb/image',
            self._image_cb, rclpy.qos.QoSProfile(
                depth=2,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))

        self._tf_buf = Buffer()
        self._tf_lst = TransformListener(self._tf_buf, self)

        self._csv_file = open(self._out_csv, 'w', newline='')
        self._writer   = csv.DictWriter(self._csv_file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._csv_file.flush()
        self._t0       = time.time()
        self._row_count = 0

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'Recording to {self._out_csv}, images to {self._img_dir}/ at {rate_hz} Hz')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _gz_odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._gz_odom_pose = (p.x, p.y, quat_yaw((o.x, o.y, o.z, o.w)))

    def _gz_world_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._gz_world_pose = (p.x, p.y, quat_yaw((o.x, o.y, o.z, o.w)))

    def _spawn_event_cb(self, msg: String):
        try:
            ev = json.loads(msg.data)
        except Exception:
            return
        name = ev.get('name', '')
        if ev.get('ev') == 'spawn':
            self._spawn_events[name] = ev
            self._last_spawn = ev
        elif ev.get('ev') == 'remove':
            self._spawn_events.pop(name, None)

    def _image_cb(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts_ms = int(time.time() * 1000)
            self._last_image = (bgr, ts_ms)
        except Exception as e:
            self.get_logger().warn(f'Image convert error: {e}', throttle_duration_sec=5.0)

    # ── tick ───────────────────────────────────────────────────────────────

    def _tick(self):
        now = time.time() - self._t0
        row = {f: '' for f in FIELDS}
        row['time_s'] = f'{now:.3f}'

        # True gz world pose
        gwp = self._gz_world_pose
        if gwp:
            row['robot_gz_world_x']       = f'{gwp[0]:.4f}'
            row['robot_gz_world_y']       = f'{gwp[1]:.4f}'
            row['robot_gz_world_yaw_deg'] = f'{math.degrees(gwp[2]):.2f}'

        # Gz odom pose (what spawner uses)
        gop = self._gz_odom_pose
        if gop:
            row['robot_gz_odom_x']       = f'{gop[0]:.4f}'
            row['robot_gz_odom_y']       = f'{gop[1]:.4f}'
            row['robot_gz_odom_yaw_deg'] = f'{math.degrees(gop[2]):.2f}'

        # EKF map pose
        mp = tf_pose(self._tf_buf, 'map', 'base_footprint')
        if mp:
            row['robot_map_x']       = f'{mp[0]:.4f}'
            row['robot_map_y']       = f'{mp[1]:.4f}'
            row['robot_map_yaw_deg'] = f'{math.degrees(mp[2]):.2f}'

        # Spawn events
        row['active_box_count'] = str(len(self._spawn_events))
        ls = self._last_spawn
        if ls:
            row['last_spawn_name']    = ls.get('name', '')
            row['last_spawn_map_x']   = f"{ls.get('map_x', ''):.4f}" if 'map_x' in ls else ''
            row['last_spawn_map_y']   = f"{ls.get('map_y', ''):.4f}" if 'map_y' in ls else ''
            row['last_spawn_gz_x']    = f"{ls.get('gz_x', ''):.4f}"  if 'gz_x'  in ls else ''
            row['last_spawn_gz_y']    = f"{ls.get('gz_y', ''):.4f}"  if 'gz_y'  in ls else ''
            row['last_spawn_yaw_deg'] = f"{math.degrees(ls.get('yaw', 0)):.2f}" if 'yaw' in ls else ''

        # Divergence
        if gwp and mp:
            dx = mp[0] - gwp[0]
            dy = mp[1] - gwp[1]
            row['map_vs_gz_world_x'] = f'{dx:.4f}'
            row['map_vs_gz_world_y'] = f'{dy:.4f}'
            yaw_diff = math.degrees(math.atan2(
                math.sin(mp[2] - gwp[2]), math.cos(mp[2] - gwp[2])))
            row['map_gz_yaw_diff_deg'] = f'{yaw_diff:.3f}'

        if gwp and gop:
            row['gz_odom_vs_world_x'] = f'{gop[0] - gwp[0]:.4f}'
            row['gz_odom_vs_world_y'] = f'{gop[1] - gwp[1]:.4f}'

        # Camera image — save latest frame if available
        img_data = self._last_image
        if img_data:
            bgr, ts_ms = img_data
            fname = f'{ts_ms}.jpg'
            fpath = os.path.join(self._img_dir, fname)
            if not os.path.exists(fpath):
                cv2.imwrite(fpath, bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            row['img_file'] = fname
            self._last_image = None  # consume so we only write new frames

        self._writer.writerow(row)
        self._csv_file.flush()
        self._row_count += 1

        if self._row_count % 20 == 0:
            self.get_logger().info(
                f't={now:.0f}s  rows={self._row_count}  '
                f'boxes={row["active_box_count"]}  '
                f'gz_world={row.get("robot_gz_world_x","?")},'
                f'{row.get("robot_gz_world_y","?")}  '
                f'map={row.get("robot_map_x","?")},{row.get("robot_map_y","?")}  '
                f'map_vs_world=({row.get("map_vs_gz_world_x","?")},{row.get("map_vs_gz_world_y","?")})')

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
