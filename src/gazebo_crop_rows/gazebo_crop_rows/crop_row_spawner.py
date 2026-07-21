"""crop_row_spawner — sliding window Gazebo crop row spawner.

Reads swath lines from a GeoJSON field file. For each swath computes two crop
row positions (±row_offset_m perpendicular to swath) in the ROS map frame,
which equals the Gazebo world frame when both share the same WGS84 datum.

Spawns rows via `gz service` when they enter a lookahead window ahead of the
robot. Removes them once they fall behind a removal threshold.

Subscribes:
  /tf  (robot pose via base_footprint→map lookup)

Parameters:
  field_file        path to *_directed_turns.geojson
  world_name        Gazebo world name (default: house_short_crop_rows)
  spawn_ahead_m     spawn rows this far ahead of robot (default: 3.0)
  remove_behind_m   remove rows this far behind robot (default: 2.0)
  row_offset_m      lateral offset from swath centre (default: 0.18)
  row_width_m       visual width of each row box (default: 0.06)
  datum_lat         WGS84 origin latitude  (default: 53.5204991)
  datum_lon         WGS84 origin longitude (default: 17.8258532)
"""

import json
import math
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros


WGS84_A = 6378137.0


def wgs84_to_enu(lon, lat, datum_lon, datum_lat):
    x = math.radians(lon - datum_lon) * WGS84_A * math.cos(math.radians(datum_lat))
    y = math.radians(lat - datum_lat) * WGS84_A
    return x, y


def normalize_angle(a):
    while a > math.pi / 2:
        a -= math.pi
    while a <= -math.pi / 2:
        a += math.pi
    return a


def _sdf(name, x, y, yaw, length, width):
    return (
        f'<sdf version="1.7">'
        f'<model name="{name}">'
        f'<static>true</static>'
        f'<pose>{x:.4f} {y:.4f} 0.005 0 0 {yaw:.6f}</pose>'
        f'<link name="link">'
        f'<visual name="visual">'
        f'<geometry><box><size>{length:.4f} {width:.4f} 0.01</size></box></geometry>'
        f'<material>'
        f'<ambient>0.13 0.55 0.13 1</ambient>'
        f'<diffuse>0.13 0.55 0.13 1</diffuse>'
        f'<specular>0 0 0 1</specular>'
        f'</material>'
        f'</visual>'
        f'</link>'
        f'</model>'
        f'</sdf>'
    )


class CropRowSpawner(Node):

    def __init__(self):
        super().__init__('crop_row_spawner')

        self.declare_parameter('field_file', '')
        self.declare_parameter('world_name', 'house_short_crop_rows')
        self.declare_parameter('spawn_ahead_m', 3.0)
        self.declare_parameter('remove_behind_m', 2.0)
        self.declare_parameter('row_offset_m', 0.18)
        self.declare_parameter('row_width_m', 0.06)
        self.declare_parameter('datum_lat', 53.5204991)
        self.declare_parameter('datum_lon', 17.8258532)

        field_file  = self.get_parameter('field_file').value
        self._world  = self.get_parameter('world_name').value
        self._ahead  = self.get_parameter('spawn_ahead_m').value
        self._behind = self.get_parameter('remove_behind_m').value
        self._width  = self.get_parameter('row_width_m').value
        datum_lat    = self.get_parameter('datum_lat').value
        datum_lon    = self.get_parameter('datum_lon').value

        if not field_file:
            self.get_logger().error('field_file parameter not set')
            return

        self._rows = self._load_rows(
            field_file, datum_lon, datum_lat,
            self.get_parameter('row_offset_m').value)
        self.get_logger().info(
            f'Loaded {len(self._rows)} crop rows from {field_file}')

        self._spawned: set[str] = set()
        self._pending: set[str] = set()  # async ops in flight
        self._lock = threading.Lock()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_timer(0.2, self._update)

    def _load_rows(self, field_file, datum_lon, datum_lat, offset):
        with open(field_file) as f:
            d = json.load(f)
        rows = []
        for si, feat in enumerate(d['features']):
            coords = feat['geometry']['coordinates']
            x0, y0 = wgs84_to_enu(coords[0][0], coords[0][1], datum_lon, datum_lat)
            x1, y1 = wgs84_to_enu(coords[1][0], coords[1][1], datum_lon, datum_lat)

            dx, dy = x1 - x0, y1 - y0
            length = math.hypot(dx, dy)
            ux, uy = dx / length, dy / length
            px, py = -uy, ux
            yaw = normalize_angle(math.atan2(dy, dx))
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2

            for side, sign in [('L', +1), ('R', -1)]:
                rows.append({
                    'name':   f'gcr_{si}_{side}',
                    'cx':     cx + sign * offset * px,
                    'cy':     cy + sign * offset * py,
                    'yaw':    yaw,
                    'length': length,
                    'ux': ux, 'uy': uy,
                })
        return rows

    def _get_robot_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def _along_axis(self, row, rx, ry):
        return (rx - row['cx']) * row['ux'] + (ry - row['cy']) * row['uy']

    def _update(self):
        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry = pose

        to_spawn = []
        to_remove = []

        with self._lock:
            for row in self._rows:
                dist = self._along_axis(row, rx, ry)
                name = row['name']
                in_window = -self._behind <= dist <= self._ahead
                if in_window and name not in self._spawned and name not in self._pending:
                    to_spawn.append(row)
                    self._pending.add(name)
                elif not in_window and name in self._spawned and name not in self._pending:
                    to_remove.append(row)
                    self._pending.add(name)

        for row in to_spawn:
            threading.Thread(target=self._spawn_row, args=(row,), daemon=True).start()
        for row in to_remove:
            threading.Thread(target=self._remove_row, args=(row,), daemon=True).start()

    def _spawn_row(self, row):
        sdf = _sdf(row['name'], row['cx'], row['cy'], row['yaw'],
                   row['length'], self._width)
        req = f'sdf: "{sdf.replace(chr(34), chr(92)+chr(34))}"'
        result = subprocess.run(
            ['gz', 'service',
             '-s', f'/world/{self._world}/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', req],
            capture_output=True, text=True)
        with self._lock:
            self._pending.discard(row['name'])
            if 'true' in result.stdout:
                self._spawned.add(row['name'])
                self.get_logger().debug(f"Spawned {row['name']}")
            else:
                self.get_logger().warn(
                    f"Failed to spawn {row['name']}: {result.stderr.strip()}")

    def _remove_row(self, row):
        result = subprocess.run(
            ['gz', 'service',
             '-s', f'/world/{self._world}/remove',
             '--reqtype', 'gz.msgs.Entity',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', f'name: "{row["name"]}" type: MODEL'],
            capture_output=True, text=True)
        with self._lock:
            self._pending.discard(row['name'])
            if 'true' in result.stdout:
                self._spawned.discard(row['name'])
                self.get_logger().debug(f"Removed {row['name']}")
            else:
                self.get_logger().warn(
                    f"Failed to remove {row['name']}: {result.stderr.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = CropRowSpawner()
    rclpy.spin(node)
    rclpy.shutdown()
