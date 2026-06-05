#!/usr/bin/env python3

import json
import os
import math
import ssl
import yaml

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

HIVEMQ_HOST = "ae4cb1b10ad84e53af8887dd32476b04.s2.eu.hivemq.cloud"
HIVEMQ_PORT = 8883
HIVEMQ_USER = "aargideon"
HIVEMQ_PASS = "para!234"


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw from quaternion without tf_transformations dependency."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ImuHeadingCalibrator(Node):
    def __init__(self):
        super().__init__('imu_heading_calibrator')

        # Parameters
        self.declare_parameter('min_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.05)
        self.declare_parameter('num_samples', 50)

        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.num_samples = self.get_parameter('num_samples').value

        # Config file path
        self.config_file = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'config', 'config.yaml'
        )

        # State
        self.calibrating = False
        self.offset_samples = []
        self.latest_imu_yaw = None
        self.current_offset = 0.0

        # Load existing offset
        self.load_offset()

        # Publisher for imu_bridge runtime update
        self.offset_pub = self.create_publisher(Float32, 'imu_yaw_offset', 10)

        # Publish loaded offset after a short delay so imu_bridge is ready
        self.create_timer(2.0, self.publish_loaded_offset_once)
        self._published_initial = False

        # Subscribers
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Calibration service
        self.create_service(Trigger, 'calibrate_imu_heading', self.calibrate_callback)

        # HiveMQ e-stop
        self.estop = False
        self._hive = mqtt.Client(client_id="imu_cal_estop", protocol=mqtt.MQTTv5)
        self._hive.username_pw_set(HIVEMQ_USER, HIVEMQ_PASS)
        self._hive.tls_set(tls_version=ssl.PROTOCOL_TLS_CLIENT)
        self._hive.on_message = self._on_hivemq_msg
        try:
            self._hive.connect(HIVEMQ_HOST, HIVEMQ_PORT, 60)
            self._hive.subscribe("outer/#")
            self._hive.loop_start()
            self.get_logger().info('HiveMQ e-stop armed')
        except Exception as e:
            self.get_logger().warn(f'HiveMQ connect failed: {e} — e-stop not available')

        self.get_logger().info(
            f'IMU heading calibrator ready. '
            f'Loaded offset: {self.current_offset:.4f} rad '
            f'({math.degrees(self.current_offset):.2f} deg)'
        )
        self.get_logger().info(
            f'Call /calibrate_imu_heading service to start calibration while driving straight'
        )

    def publish_loaded_offset_once(self):
        if not self._published_initial:
            self._published_initial = True
            msg = Float32()
            msg.data = self.current_offset
            self.offset_pub.publish(msg)
            self.get_logger().info(
                f'Published loaded offset to /imu_yaw_offset: '
                f'{self.current_offset:.4f} rad'
            )

    def load_offset(self):
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f) or {}
                imu_config = config.get('imu', {})
                self.current_offset = float(imu_config.get('yaw_offset', 0.0))
                self.get_logger().info(
                    f'Loaded yaw_offset from {self.config_file}: '
                    f'{self.current_offset:.4f} rad'
                )
            else:
                self.get_logger().warn(f'Config file not found: {self.config_file}')
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}')
            self.current_offset = 0.0

    def save_offset(self):
        try:
            config = {}
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f) or {}

            if 'imu' not in config:
                config['imu'] = {}
            config['imu']['yaw_offset'] = round(float(self.current_offset), 6)

            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)

            self.get_logger().info(
                f'Saved yaw_offset to {self.config_file}: '
                f'{self.current_offset:.4f} rad ({math.degrees(self.current_offset):.2f} deg)'
            )
        except Exception as e:
            self.get_logger().error(f'Error saving config: {e}')

    def _on_hivemq_msg(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            if msg.topic.startswith("outer/") and data.get("data", 1) == 0:
                self.estop = True
                if self.calibrating:
                    self.calibrating = False
                    self.get_logger().error(
                        f'E-STOP — calibration aborted '
                        f'({len(self.offset_samples)}/{self.num_samples} samples)')
        except Exception:
            pass

    def imu_callback(self, msg):
        q = msg.orientation
        self.latest_imu_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def odom_callback(self, msg):
        if not self.calibrating or self.latest_imu_yaw is None or self.estop:
            return

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        speed = math.sqrt(vx**2 + vy**2)

        # Check straight-line conditions
        if speed < self.min_linear_speed:
            return
        if abs(wz) > self.max_angular_speed:
            return

        # Get actual travel heading from odom pose (more stable than velocity direction)
        q = msg.pose.pose.orientation
        odom_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # Offset = actual heading - IMU heading
        offset = self.normalize_angle(odom_yaw - self.latest_imu_yaw)
        self.offset_samples.append(offset)

        self.get_logger().info(
            f'Sample {len(self.offset_samples)}/{self.num_samples}: '
            f'odom_yaw={math.degrees(odom_yaw):.1f} '
            f'imu_yaw={math.degrees(self.latest_imu_yaw):.1f} '
            f'offset={math.degrees(offset):.1f} deg'
        )

        if len(self.offset_samples) >= self.num_samples:
            self.finish_calibration()

    def finish_calibration(self):
        self.calibrating = False

        # Use circular mean to handle angle wrapping
        sin_sum = sum(math.sin(s) for s in self.offset_samples)
        cos_sum = sum(math.cos(s) for s in self.offset_samples)
        self.current_offset = math.atan2(sin_sum, cos_sum)

        self.get_logger().info(
            f'Calibration complete! '
            f'Offset: {self.current_offset:.4f} rad ({math.degrees(self.current_offset):.2f} deg) '
            f'from {len(self.offset_samples)} samples'
        )

        # Save to config file
        self.save_offset()

        # Publish to imu_bridge
        msg = Float32()
        msg.data = self.current_offset
        self.offset_pub.publish(msg)
        self.get_logger().info('Published new offset to /imu_yaw_offset')

    def calibrate_callback(self, request, response):
        if self.calibrating:
            response.success = False
            response.message = (
                f'Calibration already in progress: '
                f'{len(self.offset_samples)}/{self.num_samples} samples'
            )
        else:
            self.calibrating = True
            self.offset_samples = []
            response.success = True
            response.message = (
                f'Calibration started. Drive the robot in a straight line. '
                f'Collecting {self.num_samples} samples '
                f'(min speed: {self.min_linear_speed} m/s)'
            )
            self.get_logger().info(response.message)
        return response

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ImuHeadingCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._hive.loop_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
