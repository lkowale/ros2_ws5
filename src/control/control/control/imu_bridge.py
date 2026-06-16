#!/usr/bin/env python3

import rclpy
import json
import paho.mqtt.client as mqtt
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        self.broker_address = self.declare_parameter("broker_ip_address", 't630').value
        self.mqttclient = mqtt.Client(client_id="imu_bridge", userdata=None, protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username="mark", password="pass")
        self.mqttclient.connect(self.broker_address)
        self.mqttclient.subscribe("imu")
        self.mqttclient.on_message = self.topic_get

        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.mqttclient.loop_start()

        self.get_logger().info('imu_bridge started (differential mode — GPS heading provides absolute yaw)')

    def __del__(self):
        try:
            self.mqttclient.loop_stop()
            self.mqttclient.disconnect()
        except Exception:
            pass

    def topic_get(self, client, userdata, msg):
        if msg.topic == "imu":
            message = msg.payload.decode("utf-8")
            msg_in = json.loads(message)
            self.publish_imu_message(msg_in)

    def publish_imu_message(self, msg_in):
        imu_msg = Imu()

        gyro = Vector3()
        gyro.x, gyro.y, gyro.z = [float(i) for i in msg_in["angular_velocity"]]
        imu_msg.angular_velocity = gyro
        imu_msg.angular_velocity_covariance[0] = 0.00001
        imu_msg.angular_velocity_covariance[4] = 0.00001
        imu_msg.angular_velocity_covariance[8] = 0.00001

        accel = Vector3()
        accel.x, accel.y, accel.z = [float(i) for i in msg_in["linear_acceleration"]]
        imu_msg.linear_acceleration = accel
        imu_msg.linear_acceleration_covariance[0] = 0.00001
        imu_msg.linear_acceleration_covariance[4] = 0.00001
        imu_msg.linear_acceleration_covariance[8] = 0.00001

        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = [float(i) for i in msg_in["orientation"]]
        imu_msg.orientation = quat
        imu_msg.orientation_covariance[0] = 0.00001  # roll
        imu_msg.orientation_covariance[4] = 0.00001  # pitch
        imu_msg.orientation_covariance[8] = 1.0      # yaw — BNO080 magnetometer unreliable

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        self.imu_publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuBridge()
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
