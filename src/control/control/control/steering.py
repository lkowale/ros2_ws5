#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import os
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import time
from ament_index_python.packages import get_package_share_directory

class SteeringControl(Node):
# ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
    def __init__(self):
        super().__init__('steering_control')
        try:
            pkg_dir = get_package_share_directory('control')
            config_path = os.path.join(pkg_dir, 'config', 'config.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        self.config_path = config_path
        with open(config_path, 'r') as file:
            self.yaml_params = yaml.safe_load(file)

        self.reload_srv = self.create_service(
            Trigger, 'steering/reload_config', self.reload_config_cb)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

        self.broker_address= self.declare_parameter("broker_ip_address", 't630').value
        self.mqttclient = mqtt.Client(client_id="steering_outer_bridge", userdata=None, protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username="mark", password="pass")
        self.mqttclient.on_disconnect = self.on_disconnect
        self.mqttclient.on_connect = self.on_connect
        self.mqttclient.on_message = self.on_message
        self.mqttclient.connect(self.broker_address, keepalive=0)
        self.mqttclient.loop_start()

        self._steering_state_pub = self.create_publisher(String, '/steering_state', 10)
        #    def on_disconnect(client, userdata, flags, reason_code, properties):
    def on_connect(self, client, userdata, flags, reason_code, properties):
        client.subscribe('ads')

    def on_message(self, client, userdata, msg):
        try:
            ros_msg = String()
            ros_msg.data = msg.payload.decode()
            self._steering_state_pub.publish(ros_msg)
        except Exception:
            pass

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        FIRST_RECONNECT_DELAY = 1
        RECONNECT_RATE = 2
        MAX_RECONNECT_COUNT = 12
        MAX_RECONNECT_DELAY = 60
        # self.get_logger().info(f'My log message {num}', once=True)
        self.get_logger().info(f"Disconnected with result code: {reason_code}")
        # logging.info("Disconnected with result code: %s", rc)
        logging = self.get_logger()
        reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
        while reconnect_count < MAX_RECONNECT_COUNT:
            self.get_logger().info(f"Reconnecting in {reconnect_delay} seconds...")
            time.sleep(reconnect_delay)

            try:
                client.reconnect()
                logging.info("Reconnected successfully!")
                return
            except Exception as err:
                logging.error(f"{err} Reconnect failed. Retrying...")

            reconnect_delay *= RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
            reconnect_count += 1
        logging.info(f"Reconnect failed after {reconnect_count} attempts. Exiting...")

    def reload_config_cb(self, request, response):
        with open(self.config_path, 'r') as f:
            self.yaml_params = yaml.safe_load(f)
        zero = self.yaml_params['steer']['zero']
        self.get_logger().info(f'Config reloaded: steer.zero={zero}')
        response.success = True
        response.message = f'steer.zero={zero}'
        return response

    def listener_callback(self, msg):
        twist_cmd = msg.angular.z
        steer_pos = self.yaml_params['steer']['zero']
        # turn right is negative angular.z
        if twist_cmd<0:
            steer_pos = self.map_range(abs(twist_cmd), 0, 0.6, self.yaml_params['steer']['zero'],self.yaml_params['steer']['max_right'])
        # turn left is positive angular.z
        if twist_cmd>0:
            steer_pos = self.map_range(twist_cmd, 0, 0.6, self.yaml_params['steer']['zero'], self.yaml_params['steer']['max_left'])
        if twist_cmd == 0:
            steer_pos = self.yaml_params['steer']['zero']
        # steer_position	{"data": 450}
        msg = "{\"data\": " + str(steer_pos) + "}"
        self.mqttclient.publish('steer_position', msg)
        # self.get_logger().info('I heard: "%f"' % msg.angular.z)
        
    def map_range(self, x, in_min, in_max, out_min, out_max):
        return round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def main(args=None):
    rclpy.init(args=args)

    steering_control = SteeringControl()

    rclpy.spin(steering_control)

    steering_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()