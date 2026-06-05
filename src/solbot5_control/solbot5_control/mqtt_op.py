#!/usr/bin/env python3

import rclpy
import rclpy.executors
from rclpy.time import Time
from rclpy.timer import Timer

import time
import paho.mqtt.client as mqtt
from rclpy.node import Node
import uuid
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import Int32, Float32, String, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import (Quaternion, Vector3)
from tf2_ros import TransformBroadcaster
import json
from json.decoder import JSONDecodeError
from solbot4_msgs.msg import Pause
from solbot4_msgs.srv import PlanterControl, ResumePause

class MQTT_op(Node):
    def __init__(self):
        super().__init__('mqtt_op_bridge')
        self._shutdown_requested = False

        self.rate = 10
        self.r = self.create_rate(self.rate)
        # self.mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "mqtt_op_bridge")
        # Generate unique client ID to prevent conflicts
        unique_id = str(uuid.uuid4())[:8]
        client_id = f"mqtt_op_bridge_{unique_id}"
        self.mqttclient = mqtt.Client(client_id=client_id, userdata=None, protocol=mqtt.MQTTv5)
        self.get_logger().info(f"MQTT Client ID: {client_id}")
        self.mqttclient.on_connect = self.on_connect
        self.mqttclient.on_disconnect = self.on_disconnect
        self.mqttclient.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
        self.mqttclient.username_pw_set(username="aargideon", password="para!234")
        self.mqttclient.connect("ae4cb1b10ad84e53af8887dd32476b04.s2.eu.hivemq.cloud",8883)                  
        self.mqttclient.subscribe("outer/#")        
        self.mqttclient.on_message = self.topic_get  

        self.get_logger().info('mqtt_op_bridge started...')
        self.steer = 0.0
        self.speed = 0.0
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.handbrake_publisher = self.create_publisher(Bool, '/handbrake', 10)
        self.create_timer(1.0, self._repeat_cmd_vel)  # 1 Hz — keeps robot moving within 2s cmd_timeout
        self.lift_pos_publisher = self.create_publisher(Int32, 'lift/set_position', 10) 
        self.lift_cmd_publisher = self.create_publisher(String, 'lift/cmd', 10)           
        self.job_paused_publisher = self.create_publisher(Pause, '/pause_request', 10)    
        # Planter control service client
        self.planter_client = self.create_client(PlanterControl, '/planter_control')
        # Resume pause service client (clears all HIGH severity conditions)
        self.resume_pause_client = self.create_client(ResumePause, '/resume_pause')
        # self.tf_broadcaster = TransformBroadcaster(self)

        self.mqttclient.loop_start()

    def topic_get(self, client, userdata, msg):
        topic = msg.topic
        try:
            self._topic_get(topic, msg)
        except Exception as e:
            self.get_logger().error(f"topic_get error on '{topic}': {e}")

    def _topic_get(self, topic, msg):
        if topic == "outer/steer":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            self.steer =  float(msg_in["data"])
            self.publish_cmd_vel_msg()

        if topic == "outer/speed":
            message = msg.payload.decode("utf-8")
            self.get_logger().info(f"Received speed msg: {message}")
            msg_in=json.loads(message)
            self.speed =  float(msg_in["data"])
            self.get_logger().info(f"Publishing cmd_vel: linear.x={self.speed}, angular.z={self.steer}")
            self.publish_cmd_vel_msg()
            pause_msg = Pause()
            pause_msg.source = "mqtt"
            if self.speed == 0.0:
                pause_msg.paused = True
                pause_msg.reason = "Speed zero — operator e-stop"
                self.get_logger().info("Speed=0 received — publishing pause (e-stop)")
            else:
                pause_msg.paused = False
                pause_msg.reason = "Speed non-zero — operator resume"
            self.job_paused_publisher.publish(pause_msg)

        if topic == "outer/lift_pos":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            lift_pos =  int(msg_in["data"])
            out_msg = Int32()
            out_msg.data = lift_pos
            self.lift_pos_publisher.publish(out_msg)      

        if topic == "outer/lift_cmd":
            message = msg.payload.decode("utf-8")
            msg_in=json.loads(message)
            lift_cmd =  msg_in["data"]
            out_msg = String()
            out_msg.data = lift_cmd
            self.lift_cmd_publisher.publish(out_msg)    

        if topic == "outer/job_pause":
            message = msg.payload.decode("utf-8")
            msg_in = json.loads(message)
            pause_cmd = int(msg_in["data"])
            out_msg = Pause()
            out_msg.paused = bool(pause_cmd)
            out_msg.source = "mqtt"
            out_msg.reason = "User triggered"
            self.job_paused_publisher.publish(out_msg)
            if not pause_cmd:
                # Also call /resume_pause to clear any HIGH severity diagnostic conditions
                if self.resume_pause_client.service_is_ready():
                    req = ResumePause.Request()
                    req.operator_id = "mqtt"
                    future = self.resume_pause_client.call_async(req)
                    future.add_done_callback(self._resume_pause_callback)
                else:
                    self.get_logger().warn("resume_pause service not available")

        if topic == "outer/planter_cmd":
            # Expected format: {"data": "start"} or {"data": "stop", "planter": "PL1"}
            message = msg.payload.decode("utf-8")
            msg_in = json.loads(message)
            planter_cmd = msg_in["data"]
            planter_name = msg_in.get("planter", "")  # Empty = all planters
            self.call_planter_service(planter_cmd, planter_name)

        if topic == "outer/planter_aospr":
            # Expected format: {"data": 12} or {"data": 12, "planter": "PL1"}
            message = msg.payload.decode("utf-8")
            msg_in = json.loads(message)
            aospr = int(msg_in["data"])
            planter_name = msg_in.get("planter", "")
            self.call_planter_service("set_aospr", planter_name, aospr=aospr)

        if topic == "outer/handbrake":
            message = msg.payload.decode("utf-8")
            msg_in = json.loads(message)
            out_msg = Bool()
            out_msg.data = bool(int(msg_in["data"]))
            self.handbrake_publisher.publish(out_msg)

        if topic == "outer/planter_dd":
            # Expected format: {"data": 10} or {"data": 10, "planter": "PL1"}
            message = msg.payload.decode("utf-8")
            msg_in = json.loads(message)
            dd = int(msg_in["data"])
            planter_name = msg_in.get("planter", "")
            self.call_planter_service("set_density", planter_name, desired_density=dd)

    def _repeat_cmd_vel(self):
        """1 Hz republish while moving (keeps robot within 2s cmd_timeout window)."""
        if abs(self.speed) > 0.001:
            self.publish_cmd_vel_msg()

    def publish_cmd_vel_msg(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = self.speed
        cmd_msg.angular.z = self.steer

        #self.get_logger().info('Publishing imu message')
        #self.pub_imu_raw.publish(imu_raw_msg)
        self.cmd_vel_publisher.publish(cmd_msg)

    def call_planter_service(self, command, planter_name="", desired_density=0, aospr=0):
        """Call the planter control service asynchronously."""
        if not self.planter_client.service_is_ready():
            self.get_logger().warn("Planter service not available")
            return

        request = PlanterControl.Request()
        request.command = command
        request.planter_name = planter_name
        request.desired_density = desired_density
        request.aospr = aospr

        self.get_logger().info(f"Calling planter service: cmd={command}, planter={planter_name or 'ALL'}")

        future = self.planter_client.call_async(request)
        future.add_done_callback(self.planter_service_callback)

    def planter_service_callback(self, future):
        """Handle planter service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Planter service: {response.message}")
            else:
                self.get_logger().warn(f"Planter service failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Planter service call failed: {e}")

    def _resume_pause_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"resume_pause: {response.message}")
        except Exception as e:
            self.get_logger().error(f"resume_pause call failed: {e}")

    def on_connect(self, client, userdata, flags, rc, properties=None):
        # print("CONNACK received with code %s." % rc)
        if rc == 0:
            self.get_logger().info("Successfully connected to HiveMQ cloud broker", once=True)
        else:
            self.get_logger().error(f"Connection to hivemq failed with code: {rc}")

    def on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        # Don't attempt reconnection if shutdown was requested
        if self._shutdown_requested:
            return

        FIRST_RECONNECT_DELAY = 1
        RECONNECT_RATE = 2
        MAX_RECONNECT_COUNT = 12
        MAX_RECONNECT_DELAY = 60

        print(f"[mqtt_op] Disconnected with result code: {reason_code}")
        reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY

        while reconnect_count < MAX_RECONNECT_COUNT and not self._shutdown_requested:
            print(f"[mqtt_op] Reconnecting in {reconnect_delay} seconds...")
            time.sleep(reconnect_delay)

            if self._shutdown_requested:
                break

            try:
                client.reconnect()
                print("[mqtt_op] Reconnected successfully!")
                return
            except Exception as err:
                print(f"[mqtt_op] {err} Reconnect failed. Retrying...")

            reconnect_delay *= RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
            reconnect_count += 1

        if not self._shutdown_requested:
            print(f"[mqtt_op] Reconnect failed after {reconnect_count} attempts.")

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self._shutdown_requested = True
        self.mqttclient.loop_stop()
        self.mqttclient.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    relay_ros2_mqtt = None
    try:
        relay_ros2_mqtt = MQTT_op()
        rclpy.spin(relay_ros2_mqtt)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if relay_ros2_mqtt is not None:
            relay_ros2_mqtt.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
