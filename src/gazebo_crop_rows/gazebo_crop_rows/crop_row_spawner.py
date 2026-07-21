"""crop_row_spawner — sliding window Gazebo crop row spawner.

Places short crop row segments in Gazebo at the robot's current localization
position. Rows are positioned in map frame then transformed to Gazebo world
frame via the odom→map TF correction, so they track RL even as EKF drifts.

All gz service calls are serialised through a single worker thread with a
bounded queue so they never overwhelm the simulator.

Parameters:
  field_file        path to *_directed_turns.geojson
  world_name        Gazebo world name (default: house_short_crop_rows)
  spawn_ahead_m     distance ahead to spawn segments (default: 4.0)
  remove_behind_m   distance behind to remove segments (default: 3.0)
  row_offset_m      lateral offset from swath (default: 0.18)
  row_width_m       visual width of row box (default: 0.06)
  segment_length_m  length of each spawned segment (default: 2.0)
  min_move_m        minimum robot movement before re-spawning (default: 1.5)
  datum_lat         WGS84 origin latitude  (default: 53.5204991)
  datum_lon         WGS84 origin longitude (default: 17.8258532)
"""

import json
import math
import queue
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


def _map_to_world(mx, my, t_map_odom):
    tx = t_map_odom.transform.translation.x
    ty = t_map_odom.transform.translation.y
    q = t_map_odom.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    wx = tx + mx * math.cos(yaw) - my * math.sin(yaw)
    wy = ty + mx * math.sin(yaw) + my * math.cos(yaw)
    return wx, wy


class CropRowSpawner(Node):

    def __init__(self):
        super().__init__('crop_row_spawner')

        self.declare_parameter('field_file', '')
        self.declare_parameter('world_name', 'house_short_crop_rows')
        self.declare_parameter('spawn_ahead_m', 4.0)
        self.declare_parameter('remove_behind_m', 3.0)
        self.declare_parameter('row_offset_m', 0.18)
        self.declare_parameter('row_width_m', 0.06)
        self.declare_parameter('segment_length_m', 2.0)
        self.declare_parameter('min_move_m', 1.5)
        self.declare_parameter('datum_lat', 53.5204991)
        self.declare_parameter('datum_lon', 17.8258532)

        field_file    = self.get_parameter('field_file').value
        self._world   = self.get_parameter('world_name').value
        self._ahead   = self.get_parameter('spawn_ahead_m').value
        self._behind  = self.get_parameter('remove_behind_m').value
        self._width   = self.get_parameter('row_width_m').value
        self._seg_len = self.get_parameter('segment_length_m').value
        self._min_move = self.get_parameter('min_move_m').value
        datum_lat     = self.get_parameter('datum_lat').value
        datum_lon     = self.get_parameter('datum_lon').value

        if not field_file:
            self.get_logger().error('field_file parameter not set')
            return

        self._swaths = self._load_swaths(
            field_file, datum_lon, datum_lat,
            self.get_parameter('row_offset_m').value)
        self.get_logger().info(
            f'Loaded {len(self._swaths)} swaths from {field_file}')

        # spawned: name → along-swath position when spawned
        self._spawned: dict[str, float] = {}
        self._lock = threading.Lock()
        self._spawn_counter = 0
        self._last_spawn_along: float | None = None  # last along-pos where we spawned

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Single worker thread serialises all gz service calls
        self._gz_queue: queue.Queue = queue.Queue(maxsize=20)
        self._worker = threading.Thread(target=self._gz_worker, daemon=True)
        self._worker.start()

        self.create_timer(0.5, self._update)

    def _load_swaths(self, field_file, datum_lon, datum_lat, offset):
        with open(field_file) as f:
            d = json.load(f)
        swaths = []
        for feat in d['features']:
            coords = feat['geometry']['coordinates']
            x0, y0 = wgs84_to_enu(coords[0][0], coords[0][1], datum_lon, datum_lat)
            x1, y1 = wgs84_to_enu(coords[1][0], coords[1][1], datum_lon, datum_lat)
            dx, dy = x1 - x0, y1 - y0
            length = math.hypot(dx, dy)
            ux, uy = dx / length, dy / length
            px, py = -uy, ux
            yaw = normalize_angle(math.atan2(dy, dx))
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            swaths.append({
                'cx': cx, 'cy': cy,
                'ux': ux, 'uy': uy,
                'px': px, 'py': py,
                'yaw': yaw, 'length': length, 'offset': offset,
            })
        return swaths

    def _get_transforms(self):
        try:
            t_robot = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            t_map_odom = self._tf_buffer.lookup_transform(
                'odom', 'map',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None
        rx = t_robot.transform.translation.x
        ry = t_robot.transform.translation.y
        q = t_robot.transform.rotation
        ryaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return rx, ry, ryaw, t_map_odom

    def _nearest_swath(self, rx, ry, ryaw):
        best, best_perp = None, float('inf')
        for sw in self._swaths:
            perp = abs((rx - sw['cx']) * (-sw['uy']) + (ry - sw['cy']) * sw['ux'])
            dot = math.cos(ryaw) * sw['ux'] + math.sin(ryaw) * sw['uy']
            if abs(dot) < 0.5:
                continue
            if perp < best_perp:
                best_perp = perp
                best = sw
        return best

    def _along(self, sw, rx, ry):
        return (rx - sw['cx']) * sw['ux'] + (ry - sw['cy']) * sw['uy']

    def _update(self):
        result = self._get_transforms()
        if result is None:
            return
        rx, ry, ryaw, t_map_odom = result

        sw = self._nearest_swath(rx, ry, ryaw)
        if sw is None:
            return

        along = self._along(sw, rx, ry)
        spawn_along = along + self._ahead

        # Only spawn if robot has moved min_move_m since last spawn
        if self._last_spawn_along is not None:
            if abs(spawn_along - self._last_spawn_along) < self._min_move:
                # Still check removals even if not spawning
                self._queue_removals(along)
                return

        half = sw['length'] / 2
        if abs(spawn_along) > half + self._seg_len:
            self._queue_removals(along)
            return

        seg_cx_map = sw['cx'] + spawn_along * sw['ux']
        seg_cy_map = sw['cy'] + spawn_along * sw['uy']

        with self._lock:
            ctr = self._spawn_counter
            self._spawn_counter += 1
        self._last_spawn_along = spawn_along

        for side, sign in [('L', +1), ('R', -1)]:
            mx = seg_cx_map + sign * sw['offset'] * sw['px']
            my = seg_cy_map + sign * sw['offset'] * sw['py']
            wx, wy = _map_to_world(mx, my, t_map_odom)
            name = f'gcr_{ctr}_{side}'
            try:
                self._gz_queue.put_nowait(('spawn', name, wx, wy, sw['yaw'], spawn_along))
            except queue.Full:
                self.get_logger().warn('gz queue full, skipping spawn')

        self._queue_removals(along)

    def _queue_removals(self, along):
        with self._lock:
            to_remove = [n for n, a in self._spawned.items()
                         if along - a > self._behind]
        for name in to_remove:
            try:
                self._gz_queue.put_nowait(('remove', name))
            except queue.Full:
                pass

    def _gz_worker(self):
        while True:
            item = self._gz_queue.get()
            if item[0] == 'spawn':
                _, name, x, y, yaw, seg_along = item
                sdf = _sdf(name, x, y, yaw, self._seg_len, self._width)
                req = f'sdf: "{sdf.replace(chr(34), chr(92)+chr(34))}"'
                r = subprocess.run(
                    ['gz', 'service',
                     '-s', f'/world/{self._world}/create',
                     '--reqtype', 'gz.msgs.EntityFactory',
                     '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000',
                     '--req', req],
                    capture_output=True, text=True)
                if 'true' in r.stdout:
                    with self._lock:
                        self._spawned[name] = seg_along
                    self.get_logger().debug(f'Spawned {name}')
                else:
                    self.get_logger().warn(f'Spawn failed {name}: {r.stderr.strip()}')

            elif item[0] == 'remove':
                name = item[1]
                with self._lock:
                    if name not in self._spawned:
                        continue
                r = subprocess.run(
                    ['gz', 'service',
                     '-s', f'/world/{self._world}/remove',
                     '--reqtype', 'gz.msgs.Entity',
                     '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000',
                     '--req', f'name: "{name}" type: MODEL'],
                    capture_output=True, text=True)
                if 'true' in r.stdout:
                    with self._lock:
                        self._spawned.pop(name, None)
                    self.get_logger().debug(f'Removed {name}')
                else:
                    self.get_logger().warn(f'Remove failed {name}: {r.stderr.strip()}')

            self._gz_queue.task_done()


def main(args=None):
    rclpy.init(args=args)
    node = CropRowSpawner()
    rclpy.spin(node)
    rclpy.shutdown()
