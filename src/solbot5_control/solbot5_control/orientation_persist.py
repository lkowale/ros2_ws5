#!/usr/bin/env python3
"""
orientation_persist — persist robot world-frame orientation across power cycles.

SAVING (continuous, while running):
    Looks up the TF transform odom→base_footprint every second and saves the
    yaw (robot heading in world frame) to ~/.ros/solbot_orientation.json.

RESTORING (once at startup):
    Reads the saved world-frame yaw.  Waits for the first /imu messages
    (which carry random_raw_imu_yaw + old_offset, as loaded by imu_bridge from
    config.yaml).  Computes the correct offset:

        raw_imu_yaw = current_imu_yaw - old_offset
        new_offset  = saved_world_yaw - raw_imu_yaw
                    = old_offset + (saved_world_yaw - current_imu_yaw)

    Publishes new_offset to /imu_yaw_offset; imu_bridge applies it immediately
    and saves it to config.yaml.

The IMU re-initialises to a random heading on every power cycle, so the stored
offset in config.yaml becomes invalid.  This node corrects it using the last
known world-frame pose from TF.
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

PERSIST_FILE = os.path.expanduser('~/.ros/solbot_orientation.json')
CONFIG_FILE = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    'config', 'config.yaml')

_WARMUP_MSGS = 5   # /imu messages to collect before applying saved orientation


def _yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _load_config_offset():
    try:
        import yaml
        with open(CONFIG_FILE) as f:
            cfg = yaml.safe_load(f) or {}
        return float(cfg.get('imu', {}).get('yaw_offset', 0.0))
    except Exception:
        return 0.0


class OrientationPersist(Node):

    def __init__(self):
        super().__init__('orientation_persist')

        self._saved_yaw = self._load_saved_yaw()

        # TF for reading world-frame orientation while running
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # IMU state for startup restore
        self._imu_samples = []
        self._offset_applied = False

        self._offset_pub = self.create_publisher(Float32, '/imu_yaw_offset', 10)
        self.create_subscription(Imu, '/imu', self._cb_imu, 10)

        self.create_timer(1.0, self._save_orientation)

        if self._saved_yaw is not None:
            self.get_logger().info(
                f'Saved world-frame yaw: {math.degrees(self._saved_yaw):.1f}° — '
                f'will restore offset once IMU is ready')
        else:
            self.get_logger().info(
                'No saved orientation — will start saving once TF odom→base_footprint is available')

    # ── save world-frame yaw via TF every second ───────────────────────────────

    def _save_orientation(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'odom', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return   # TF not yet available — skip silently

        q = tf.transform.rotation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

        try:
            now = self.get_clock().now().to_msg()
            data = {
                'yaw_world_rad': round(yaw, 6),
                'yaw_world_deg': round(math.degrees(yaw), 2),
                'saved_at':      f'{now.sec}.{now.nanosec // 1_000_000:03d}',
            }
            os.makedirs(os.path.dirname(PERSIST_FILE), exist_ok=True)
            with open(PERSIST_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Failed to save orientation: {e}')

    # ── load saved yaw ─────────────────────────────────────────────────────────

    def _load_saved_yaw(self):
        try:
            with open(PERSIST_FILE) as f:
                data = json.load(f)
            yaw = float(data['yaw_world_rad'])
            self.get_logger().info(
                f'Loaded saved world-frame yaw: {math.degrees(yaw):.1f}° '
                f'(saved at {data.get("saved_at", "unknown")})')
            return yaw
        except FileNotFoundError:
            self.get_logger().info(f'No saved orientation file at {PERSIST_FILE}')
            return None
        except Exception as e:
            self.get_logger().warn(f'Failed to load saved orientation: {e}')
            return None

    # ── IMU callback — collect samples then apply saved orientation ────────────

    def _cb_imu(self, msg: Imu):
        if self._offset_applied or self._saved_yaw is None:
            return

        q = msg.orientation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._imu_samples.append(yaw)

        if len(self._imu_samples) >= _WARMUP_MSGS:
            self._apply_saved_orientation()

    def _apply_saved_orientation(self):
        # Average the warmup samples (circular mean) for a stable reading
        sins = sum(math.sin(a) for a in self._imu_samples)
        coss = sum(math.cos(a) for a in self._imu_samples)
        current_imu_yaw = math.atan2(sins, coss)

        # current_imu_yaw = random_raw_imu_yaw + old_offset
        # new_offset = saved_world_yaw - random_raw_imu_yaw
        #            = old_offset + (saved_world_yaw - current_imu_yaw)
        old_offset = _load_config_offset()
        correction = _normalize(self._saved_yaw - current_imu_yaw)
        new_offset = _normalize(old_offset + correction)

        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('orientation_persist: restoring world-frame heading')
        self.get_logger().info(
            f'  saved world yaw   : {math.degrees(self._saved_yaw):+.1f}°')
        self.get_logger().info(
            f'  current imu yaw   : {math.degrees(current_imu_yaw):+.1f}°  '
            f'(avg of {len(self._imu_samples)} samples)')
        self.get_logger().info(
            f'  old offset        : {math.degrees(old_offset):+.1f}°')
        self.get_logger().info(
            f'  correction        : {math.degrees(correction):+.1f}°')
        self.get_logger().info(
            f'  new offset        : {math.degrees(new_offset):+.1f}°')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

        msg = Float32()
        msg.data = float(new_offset)
        for _ in range(5):
            self._offset_pub.publish(msg)

        self._offset_applied = True


def main(args=None):
    rclpy.init(args=args)
    node = OrientationPersist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
