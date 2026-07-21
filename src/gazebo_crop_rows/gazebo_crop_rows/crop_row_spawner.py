"""crop_row_spawner — sliding window Gazebo crop row spawner.

Reads swath lines from a GeoJSON field file. For each swath computes two crop
row positions (±row_offset_m perpendicular to swath) in the ROS map frame,
which equals the Gazebo world frame when both share the same WGS84 datum.

Spawns rows when they enter a lookahead window ahead of the robot.
Removes rows when they fall behind a removal threshold behind the camera.

Subscribes:
  /tf  (robot pose via base_footprint→map lookup)

Services called (Gazebo):
  /world/<world_name>/create  (ros_gz_interfaces/srv/SpawnEntity)
  /world/<world_name>/remove  (ros_gz_interfaces/srv/DestroyEntity)

Parameters:
  field_file        path to *_directed_turns.geojson
  world_name        Gazebo world name
  spawn_ahead_m     spawn rows this far ahead of robot (default: 3.0)
  remove_behind_m   remove rows this far behind robot (default: 2.0)
  row_offset_m      lateral offset from swath centre (default: 0.18)
  row_width_m       visual width of each row box (default: 0.06)
  datum_lat         WGS84 origin latitude  (default: 53.5204991)
  datum_lon         WGS84 origin longitude (default: 17.8258532)
"""

import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from geometry_msgs.msg import TransformStamped

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity


WGS84_A = 6378137.0  # semi-major axis


def wgs84_to_enu(lon, lat, datum_lon, datum_lat):
    """WGS84 → local ENU (same formula as Gazebo spherical_coordinates)."""
    x = math.radians(lon - datum_lon) * WGS84_A * math.cos(math.radians(datum_lat))
    y = math.radians(lat - datum_lat) * WGS84_A
    return x, y


def normalize_angle(a):
    """Fold angle into (-pi/2, pi/2] — rows have no direction."""
    while a > math.pi / 2:
        a -= math.pi
    while a <= -math.pi / 2:
        a += math.pi
    return a


def row_sdf(name, x, y, yaw, length, width):
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
        world_name  = self.get_parameter('world_name').value
        self._ahead  = self.get_parameter('spawn_ahead_m').value
        self._behind = self.get_parameter('remove_behind_m').value
        self._offset = self.get_parameter('row_offset_m').value
        self._width  = self.get_parameter('row_width_m').value
        datum_lat    = self.get_parameter('datum_lat').value
        datum_lon    = self.get_parameter('datum_lon').value

        if not field_file:
            self.get_logger().error('field_file parameter not set')
            return

        # Build row descriptors from field file
        self._rows = self._load_rows(field_file, datum_lon, datum_lat)
        self.get_logger().info(
            f'Loaded {len(self._rows)} crop rows from {field_file}')

        # Track which rows are currently spawned
        self._spawned: set[str] = set()
        self._lock = threading.Lock()

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Gazebo services
        spawn_srv  = f'/world/{world_name}/create'
        remove_srv = f'/world/{world_name}/delete'
        self._spawn_cli  = self.create_client(SpawnEntity,  spawn_srv)
        self._remove_cli = self.create_client(DeleteEntity, remove_srv)

        self._ready = False
        self.get_logger().info(
            f'Waiting for Gazebo services {spawn_srv}, {remove_srv}...')
        # Poll for service availability from the timer so the executor can spin
        self.create_timer(0.2, self._update)

    def _load_rows(self, field_file, datum_lon, datum_lat):
        """Return list of dicts: name, cx, cy, yaw, length."""
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
            px, py = -uy, ux  # left-perpendicular
            yaw = normalize_angle(math.atan2(dy, dx))
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2

            for side, sign in [('L', +1), ('R', -1)]:
                rows.append({
                    'name':   f'gcr_{si}_{side}',
                    'cx':     cx + sign * self.get_parameter('row_offset_m').value * px,
                    'cy':     cy + sign * self.get_parameter('row_offset_m').value * py,
                    'yaw':    yaw,
                    'length': length,
                    # swath axis for distance projection
                    'ux': ux, 'uy': uy,
                    # swath start/end for along-axis range
                    'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1,
                })
        return rows

    def _get_robot_pose(self):
        """Return (x, y, yaw) in map frame, or None."""
        try:
            t: TransformStamped = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            x = t.transform.translation.x
            y = t.transform.translation.y
            return x, y
        except Exception:
            return None

    def _along_axis_distance(self, row, rx, ry):
        """Signed distance of robot along swath axis relative to row centre."""
        return (rx - row['cx']) * row['ux'] + (ry - row['cy']) * row['uy']

    def _update(self):
        if not self._ready:
            if (self._spawn_cli.service_is_ready() and
                    self._remove_cli.service_is_ready()):
                self._ready = True
                self.get_logger().info('Gazebo services ready')
            return

        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry = pose

        to_spawn = []
        to_remove = []

        with self._lock:
            for row in self._rows:
                dist = self._along_axis_distance(row, rx, ry)
                # dist > 0 means row centre is ahead of robot along swath
                # spawn when row centre is within spawn_ahead_m ahead
                # remove when row centre is more than remove_behind_m behind
                name = row['name']
                if -self._behind <= dist <= self._ahead:
                    if name not in self._spawned:
                        to_spawn.append(row)
                else:
                    if name in self._spawned:
                        to_remove.append(row)

        for row in to_spawn:
            self._spawn_row(row)

        for row in to_remove:
            self._remove_row(row)

    def _spawn_row(self, row):
        req = SpawnEntity.Request()
        req.entity_factory.sdf = row_sdf(
            row['name'], row['cx'], row['cy'], row['yaw'],
            row['length'], self._width)
        req.entity_factory.name = row['name']
        future = self._spawn_cli.call_async(req)
        future.add_done_callback(
            lambda f, n=row['name']: self._on_spawned(f, n))

    def _on_spawned(self, future, name):
        try:
            resp = future.result()
            if resp.success:
                with self._lock:
                    self._spawned.add(name)
                self.get_logger().debug(f'Spawned {name}')
            else:
                self.get_logger().warn(f'Failed to spawn {name}')
        except Exception as e:
            self.get_logger().warn(f'Spawn call failed for {name}: {e}')

    def _remove_row(self, row):
        req = DeleteEntity.Request()
        req.entity.name = row['name']
        req.entity.type = 2  # MODEL
        future = self._remove_cli.call_async(req)
        future.add_done_callback(
            lambda f, n=row['name']: self._on_removed(f, n))

    def _on_removed(self, future, name):
        try:
            resp = future.result()
            if resp.success:
                with self._lock:
                    self._spawned.discard(name)
                self.get_logger().debug(f'Removed {name}')
            else:
                self.get_logger().warn(f'Failed to remove {name}')
        except Exception as e:
            self.get_logger().warn(f'Remove call failed for {name}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CropRowSpawner()
    rclpy.spin(node)
    rclpy.shutdown()
