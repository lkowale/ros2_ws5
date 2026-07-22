#!/usr/bin/env python3
"""
gz_rl_recorder — synchronised Gazebo-world vs RL diagnostic recorder.

Records one CSV row every 0.5 s containing:

  Gazebo world frame (physics truth, via gz service):
    robot_gz_x/y/yaw       — solbot5 world pose
    cam_gz_x/y             — camera ground-footprint centre (robot + 0.5 m fwd in gz_yaw)
    row_gz_x/y/yaw/name    — nearest spawned gcr_* model to camera footprint
    row_dist_gz            — distance from cam_footprint to nearest row centre

  ROS RL / map frame (EKF truth):
    robot_map_x/y/yaw      — map→base_footprint TF
    robot_odom_x/y/yaw     — odom→base_footprint TF
    cam_map_x/y            — camera footprint in map frame (same formula)

  Derived divergence:
    map_vs_gz_x/y          — robot_map - (robot_gz + gz_offset) per axis
                             should be 0 if EKF matches physics exactly
    map_vs_gz_dist         — Euclidean magnitude of above
    odom_vs_map_x/y        — robot_odom - robot_map  (should be 0 if map=odom)
    map_gz_yaw_diff_deg    — robot_map_yaw - robot_gz_yaw
    nearest_row_map_dist   — distance from cam_map to nearest row_gz centre
                             (tells you how far the CAMERA SEES from row centre)
    swath_id               — swath the spawner is tracking (from /crop_row_diag topic)
    dot                    — alignment dot product (from /crop_row_diag)

Parameters:
  world_name   (str)  Gazebo world name         default: house_short_crop_rows
  robot_model  (str)  Gazebo model name          default: solbot5
  gz_offset_x  (float) map→gz X offset          default: -1.35
  gz_offset_y  (float) map→gz Y offset          default: 0.0
  cam_fwd_m    (float) camera fwd offset in body default: 0.5
  cam_height_m (float) camera height             default: 0.8
  out_csv      (str)  output file path           default: /tmp/gz_rl_record.csv
  rate_hz      (float) recording rate            default: 2.0

Run:
  ros2 run gazebo_crop_rows gz_rl_recorder --ros-args \\
    -p world_name:=house_short_crop_rows \\
    -p gz_offset_x:=-1.35 \\
    -p gz_offset_y:=0.0 \\
    -p out_csv:=/tmp/gz_rl_record.csv
"""

import csv
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


# ── helpers ──────────────────────────────────────────────────────────────────

def quat_yaw(q):
    return math.atan2(
        2.0 * (q[3] * q[2] + q[0] * q[1]),
        1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2)
    )


def gz_model_pose(model_name, world_name):
    """Return (x, y, yaw_rad) of a Gazebo model or None on failure."""
    try:
        r = subprocess.run(
            ['gz', 'model', '-m', model_name, '-p'],
            capture_output=True, text=True, timeout=1.5
        )
        # Warnings go to stderr; parse only stdout lines that look like [x y z]
        xyz_line = None; rpy_line = None
        found_xyz_header = False
        for line in r.stdout.splitlines():
            s = line.strip()
            if 'XYZ' in s:
                found_xyz_header = True
                continue
            if found_xyz_header and xyz_line is None and s.startswith('['):
                xyz_line = s
                continue
            if xyz_line and 'RPY' in s and '(rad)' in s:
                # next bracketed line is RPY
                continue
            if xyz_line and rpy_line is None and s.startswith('[') and '.' in s:
                rpy_line = s
                break
        if xyz_line and rpy_line:
            x, y, _ = map(float, xyz_line.strip('[]').split())
            _, _, yaw = map(float, rpy_line.strip('[]').split())
            return x, y, yaw
    except Exception:
        pass
    return None


def gz_list_models(world_name):
    """Return list of model names in the Gazebo world."""
    try:
        r = subprocess.run(
            ['gz', 'model', '--list'],
            capture_output=True, text=True, timeout=2.0
        )
        names = []
        for line in r.stdout.splitlines():
            s = line.strip()
            if s.startswith('- '):
                names.append(s[2:])
        return names
    except Exception:
        return []


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
    """Project camera ground-footprint centre (camera points straight down)."""
    cx = robot_x + cam_fwd_m * math.cos(robot_yaw)
    cy = robot_y + cam_fwd_m * math.sin(robot_yaw)
    return cx, cy


def nearest_row(row_poses, cx, cy):
    """Find nearest (name, x, y, yaw, dist) from list of (name,x,y,yaw)."""
    best = None; best_d = 1e9
    for name, rx, ry, ryaw in row_poses:
        d = math.hypot(rx - cx, ry - cy)
        if d < best_d:
            best_d = d
            best = (name, rx, ry, ryaw, d)
    return best  # None if no rows


# ── node ─────────────────────────────────────────────────────────────────────

FIELDS = [
    'time_s',
    # Gazebo world frame
    'robot_gz_x', 'robot_gz_y', 'robot_gz_yaw_deg',
    'cam_gz_x', 'cam_gz_y',
    'row_gz_name', 'row_gz_x', 'row_gz_y', 'row_gz_yaw_deg', 'row_dist_gz',
    # RL / map frame
    'robot_map_x', 'robot_map_y', 'robot_map_yaw_deg',
    'robot_odom_x', 'robot_odom_y', 'robot_odom_yaw_deg',
    'cam_map_x', 'cam_map_y',
    # Divergence
    'map_vs_gz_x', 'map_vs_gz_y', 'map_vs_gz_dist',
    'odom_vs_map_x', 'odom_vs_map_y',
    'map_gz_yaw_diff_deg',
    'nearest_row_map_dist',
    # Camera footprint vs row alignment
    'cam_gz_to_row_bearing_deg',   # bearing from cam_gz to nearest row in gz world
    'robot_gz_heading_deg',        # redundant alias, easier to read
    'bearing_vs_heading_diff_deg', # cam→row bearing vs robot heading
]


class GzRlRecorder(Node):
    def __init__(self):
        super().__init__('gz_rl_recorder')

        self.declare_parameter('world_name',   'house_short_crop_rows')
        self.declare_parameter('robot_model',  'solbot5')
        self.declare_parameter('gz_offset_x',  -1.35)
        self.declare_parameter('gz_offset_y',   0.0)
        self.declare_parameter('cam_fwd_m',     0.5)
        self.declare_parameter('cam_height_m',  0.8)
        self.declare_parameter('out_csv',       '/tmp/gz_rl_record.csv')
        self.declare_parameter('rate_hz',       2.0)

        self._world     = self.get_parameter('world_name').value
        self._robot_mdl = self.get_parameter('robot_model').value
        self._gz_off_x  = self.get_parameter('gz_offset_x').value
        self._gz_off_y  = self.get_parameter('gz_offset_y').value
        self._cam_fwd   = self.get_parameter('cam_fwd_m').value
        self._out_csv   = self.get_parameter('out_csv').value
        rate_hz         = self.get_parameter('rate_hz').value

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
        self.get_logger().info(f'Recording to {self._out_csv} at {rate_hz} Hz')

    def _tick(self):
        now = time.time() - self._t0
        row = {f: '' for f in FIELDS}
        row['time_s'] = f'{now:.2f}'

        # ── Gazebo world pose ───────────────────────────────────────────────
        gz_pose = gz_model_pose(self._robot_mdl, self._world)
        if gz_pose:
            gx, gy, gyaw = gz_pose
            row['robot_gz_x']       = f'{gx:.4f}'
            row['robot_gz_y']       = f'{gy:.4f}'
            row['robot_gz_yaw_deg'] = f'{math.degrees(gyaw):.2f}'
            row['robot_gz_heading_deg'] = f'{math.degrees(gyaw):.2f}'

            cgx, cgy = camera_ground_xy(gx, gy, gyaw, self._cam_fwd)
            row['cam_gz_x'] = f'{cgx:.4f}'
            row['cam_gz_y'] = f'{cgy:.4f}'
        else:
            gx = gy = gyaw = cgx = cgy = None

        # ── Spawned row poses (fetch at most 4 most-recent gcr_ models) ────
        models = gz_list_models(self._world)
        row_models = sorted([m for m in models if m.startswith('gcr_')],
                            key=lambda n: int(n.split('_')[1]) if n.split('_')[1].isdigit() else 0,
                            reverse=True)[:4]

        row_poses = []
        for name in row_models:
            p = gz_model_pose(name, self._world)
            if p:
                row_poses.append((name, p[0], p[1], p[2]))

        if row_poses and cgx is not None:
            nr = nearest_row(row_poses, cgx, cgy)
            if nr:
                rname, rrx, rry, rryaw, rdist = nr
                row['row_gz_name']    = rname
                row['row_gz_x']       = f'{rrx:.4f}'
                row['row_gz_y']       = f'{rry:.4f}'
                row['row_gz_yaw_deg'] = f'{math.degrees(rryaw):.2f}'
                row['row_dist_gz']    = f'{rdist:.4f}'

                # Bearing from camera footprint to nearest row centre
                brg = math.degrees(math.atan2(rry - cgy, rrx - cgx))
                row['cam_gz_to_row_bearing_deg'] = f'{brg:.2f}'
                if gyaw is not None:
                    diff = math.degrees(math.atan2(
                        math.sin(math.radians(brg) - gyaw),
                        math.cos(math.radians(brg) - gyaw)))
                    row['bearing_vs_heading_diff_deg'] = f'{diff:.2f}'
        else:
            nr = None

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

            # Distance from cam_map to nearest row gz centre
            if nr:
                mrd = math.hypot(rrx - cmx, rry - cmy)
                row['nearest_row_map_dist'] = f'{mrd:.4f}'
        else:
            mx = my = myaw = None

        if odom_pose:
            ox, oy, oyaw = odom_pose
            row['robot_odom_x']       = f'{ox:.4f}'
            row['robot_odom_y']       = f'{oy:.4f}'
            row['robot_odom_yaw_deg'] = f'{math.degrees(oyaw):.2f}'

        # ── Divergence metrics ──────────────────────────────────────────────
        if gx is not None and mx is not None:
            # map frame should equal gz_world + gz_offset
            # gz_world = map - gz_offset  =>  map - gz_offset = gz_world
            # divergence = map_pos - (gz_pos + gz_offset)
            # gz_offset = position of gz_world_origin in map frame
            # so: expected_map_from_gz = gz_pos - gz_offset  (gz_offset is negative)
            dvx = mx - (gx - self._gz_off_x)
            dvy = my - (gy - self._gz_off_y)
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

        # Console summary every 10 rows
        if self._row_count % 10 == 0:
            dvd = row.get('map_vs_gz_dist', '?')
            rnd = row.get('nearest_row_map_dist', '?')
            brg = row.get('bearing_vs_heading_diff_deg', '?')
            self.get_logger().info(
                f't={now:.0f}s  map_vs_gz={dvd}m  '
                f'nearest_row_map={rnd}m  '
                f'cam→row_vs_heading={brg}°  '
                f'rows_in_world={len(row_poses)}'
            )

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(
            f'Saved {self._row_count} rows to {self._out_csv}')
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
