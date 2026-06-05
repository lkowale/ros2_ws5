#!/usr/bin/env python3
"""
Lift Control Service Node with Self-Diagnostics

Provides a ROS2 service interface for controlling the lift with MQTT backend.
Reports diagnostic conditions to PauseManager.

Publishes /lift/state (LiftStateMsg) for real-time monitoring by ComponentStateSupervisor
and robot_state_viz. State is derived from actual sensor readings received from ESP32
over MQTT topic 'ads'.

Diagnostic conditions reported:
- mqtt_disconnected (MEDIUM): MQTT broker connection lost
- position_out_of_range (HIGH): Requested position outside safe limits
- sensor_timeout (HIGH): No sensor data from ESP32 for >3 seconds
- sensor_out_of_range (HIGH): Sensor reading outside physical limits
- actuator_jammed (HIGH): Lift not reaching commanded position within timeout

Parameters:
  - broker_ip_address: MQTT broker address (default: t630)

Service: /lift_control (solbot4_msgs/srv/LiftControl)
Commands: lift_up, lift_down, set_position, get_status
"""

import json
import os
import time

import paho.mqtt.client as mqtt
import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from std_msgs.msg import Int32, String
from solbot4_msgs.msg import LiftStateMsg
from solbot4_msgs.srv import LiftControl
from solbot_telemetry.diagnostic_mixin import DiagnosticMixin


class LiftServiceNode(Node, DiagnosticMixin):
    # Diagnostic condition IDs
    COND_MQTT_DISCONNECTED = "mqtt_disconnected"
    COND_POSITION_OUT_OF_RANGE = "position_out_of_range"
    COND_SENSOR_TIMEOUT = "sensor_timeout"
    COND_SENSOR_OUT_OF_RANGE = "sensor_out_of_range"
    COND_ACTUATOR_JAMMED = "actuator_jammed"

    # Failure detection thresholds
    SENSOR_TIMEOUT_SEC = 3.0      # No data from ESP32 for this long
    JAM_DETECTION_SEC = 10.0      # Time to reach position before declaring jammed
    JAM_POSITION_TOLERANCE = 50   # ADC units - if further than this, still not there
    STUCK_SENSOR_SEC = 10.0       # Position unchanged for this long while moving
    STUCK_SENSOR_TOLERANCE = 2    # ADC units - change less than this = stuck

    def __init__(self):
        super().__init__('lift_service')
        self._shutdown_requested = False

        # Setup diagnostics
        self.setup_diagnostics('lift_service')

        # Load configuration
        try:
            pkg_dir = get_package_share_directory('solbot5_control')
            config_path = os.path.join(pkg_dir, 'config', 'config.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')

        with open(config_path, 'r') as file:
            self.yaml_params = yaml.safe_load(file)

        # Lift positions from config
        self.up_position = self.yaml_params.get('lift', {}).get('up_position', 800)
        self.down_position = self.yaml_params.get('lift', {}).get('down_position', 1100)
        self.commanded_position = self.up_position  # Last commanded position

        # Position safety limits (with some margin)
        self.min_position = self.yaml_params.get('lift', {}).get('min_position', 500)
        self.max_position = self.yaml_params.get('lift', {}).get('max_position', 1500)

        self.get_logger().info(f"Lift positions - up: {self.up_position}, down: {self.down_position}")
        self.get_logger().info(f"Lift limits - min: {self.min_position}, max: {self.max_position}")

        # Actual position from ESP32 sensor (via MQTT 'ads' topic)
        self.actual_position = self.up_position
        self.last_sensor_time = 0.0     # 0 means never received
        self.position_commanded_time = 0.0
        self.last_stable_position = self.up_position
        self.position_stable_since = time.time()

        # MQTT setup
        self.broker_address = self.declare_parameter("broker_ip_address", 't630').value
        self.get_logger().info(f"MQTT broker address: {self.broker_address}")

        # MQTT client
        self.mqttclient = mqtt.Client(client_id="lift_service", userdata=None, protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username="mark", password="pass")
        self.mqttclient.on_disconnect = self.on_disconnect
        self.mqttclient.on_connect = self.on_connect
        self.mqttclient.on_message = self.on_mqtt_message

        try:
            self.mqttclient.connect(self.broker_address, keepalive=60)
            self.mqttclient.loop_start()
            self.mqtt_connected = True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            self.mqtt_connected = False
            self.report_condition(
                self.COND_MQTT_DISCONNECTED,
                self.SEVERITY_MEDIUM,
                f"Lift MQTT connection failed: {e}",
                active=True,
                auto_clearable=True
            )

        # State publisher
        self.state_pub = self.create_publisher(LiftStateMsg, '/lift/state', 10)

        # Subscribe to lift/set_position and lift/cmd from mqtt_op
        self.create_subscription(Int32, 'lift/set_position', self._on_set_position, 10)
        self.create_subscription(String, 'lift/cmd', self._on_lift_cmd, 10)

        # Service server
        self.service = self.create_service(
            LiftControl,
            'lift_control',
            self.handle_lift_control
        )

        # Timer: publish state and check health at 1 Hz
        self.health_timer = self.create_timer(1.0, self.publish_state_and_check_health)

        self.get_logger().info('Lift service started')

    # =========================================================================
    # MQTT Callbacks
    # =========================================================================

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.get_logger().info("Connected to MQTT broker")
            self.mqtt_connected = True
            # Subscribe to ESP32 sensor data
            self.mqttclient.subscribe("ads")
            self.get_logger().info("Subscribed to ESP32 sensor data (ads)")
            # Clear disconnect condition
            self.report_ok(self.COND_MQTT_DISCONNECTED, "Lift MQTT connection restored")
        else:
            self.get_logger().error(f"Failed to connect to MQTT: {reason_code}")
            self.mqtt_connected = False

    def on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        self.mqtt_connected = False

        if self._shutdown_requested:
            return

        # Report MEDIUM severity - auto-resume when reconnected
        self.report_condition(
            self.COND_MQTT_DISCONNECTED,
            self.SEVERITY_MEDIUM,
            f"Lift MQTT disconnected: {reason_code}",
            active=True,
            auto_clearable=True
        )

        print(f"[lift_service] Disconnected from MQTT broker: {reason_code}")

        # Attempt reconnection
        reconnect_count = 0
        reconnect_delay = 1
        max_reconnect = 12

        while reconnect_count < max_reconnect and not self._shutdown_requested:
            print(f"[lift_service] Reconnecting in {reconnect_delay} seconds...")
            time.sleep(reconnect_delay)

            if self._shutdown_requested:
                break

            try:
                client.reconnect()
                print("[lift_service] Reconnected to MQTT broker")
                self.mqtt_connected = True
                return
            except Exception as err:
                print(f"[lift_service] Reconnect failed: {err}")

            reconnect_delay = min(reconnect_delay * 2, 60)
            reconnect_count += 1

        if not self._shutdown_requested:
            print(f"[lift_service] Failed to reconnect after {max_reconnect} attempts")

    def on_mqtt_message(self, client, userdata, msg):
        """Receive sensor data from ESP32 (topic: ads)."""
        if msg.topic != "ads":
            return
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            las = data.get('LAS')
            if las is None:
                return

            self.actual_position = int(las)
            self.last_sensor_time = time.time()

            # Clear sensor timeout if it was active
            self.report_ok(self.COND_SENSOR_TIMEOUT, "Receiving sensor data from ESP32")

        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f'Invalid sensor data on ads: {e}')

    # =========================================================================
    # State Publishing & Health Monitoring
    # =========================================================================

    def publish_state_and_check_health(self):
        """Publish lift state and run failure detection checks."""
        now = time.time()

        # Publish full state message
        msg = LiftStateMsg()
        msg.stamp = self.get_clock().now().to_msg()
        msg.current_position = self.actual_position
        msg.target_position = self.commanded_position
        msg.up_position = self.up_position
        msg.down_position = self.down_position
        msg.min_position = self.min_position
        msg.max_position = self.max_position
        msg.mqtt_connected = self.mqtt_connected

        # Determine if in motion (commanded position not yet reached)
        position_error = abs(self.commanded_position - self.actual_position)
        msg.in_motion = position_error > self.JAM_POSITION_TOLERANCE

        # Named position from actual sensor
        if self.actual_position <= 850:
            msg.position = "up"
        elif self.actual_position >= 1050:
            msg.position = "down"
        else:
            msg.position = "middle"

        self.state_pub.publish(msg)

        # Run failure detection
        self._check_sensor_timeout(now)
        self._check_sensor_out_of_range()
        self._check_actuator_jammed(now, position_error)
        self._check_stuck_sensor(now)

    def _check_sensor_timeout(self, now: float):
        """Detect: no sensor data from ESP32."""
        if self.last_sensor_time == 0.0:
            return  # Never received - normal at startup, don't report yet

        age = now - self.last_sensor_time
        if age > self.SENSOR_TIMEOUT_SEC:
            self.report_condition(
                self.COND_SENSOR_TIMEOUT,
                self.SEVERITY_HIGH,
                f"No sensor data from ESP32 for {age:.1f}s",
                active=True,
                auto_clearable=True
            )

    def _check_sensor_out_of_range(self):
        """Detect: sensor reading outside physical limits."""
        if self.last_sensor_time == 0.0:
            return

        if self.actual_position < self.min_position or self.actual_position > self.max_position:
            self.report_condition(
                self.COND_SENSOR_OUT_OF_RANGE,
                self.SEVERITY_HIGH,
                f"Sensor reading {self.actual_position} outside limits "
                f"[{self.min_position}, {self.max_position}]",
                active=True,
                auto_clearable=False
            )
        else:
            self.report_ok(self.COND_SENSOR_OUT_OF_RANGE, "Sensor reading within limits")

    def _check_actuator_jammed(self, now: float, position_error: int):
        """Detect: commanded to move but position hasn't changed enough."""
        if self.position_commanded_time == 0.0:
            return

        time_since_command = now - self.position_commanded_time

        if time_since_command > self.JAM_DETECTION_SEC and position_error > self.JAM_POSITION_TOLERANCE:
            self.report_condition(
                self.COND_ACTUATOR_JAMMED,
                self.SEVERITY_HIGH,
                f"Lift not reaching target: commanded={self.commanded_position}, "
                f"actual={self.actual_position} (error={position_error}) "
                f"after {time_since_command:.1f}s",
                active=True,
                auto_clearable=False
            )
        elif position_error <= self.JAM_POSITION_TOLERANCE:
            # Position reached - clear jammed condition
            self.report_ok(self.COND_ACTUATOR_JAMMED, "Lift reached commanded position")

    def _check_stuck_sensor(self, now: float):
        """Detect: sensor reading frozen while movement is commanded."""
        if self.last_sensor_time == 0.0:
            return

        # Track position stability
        if abs(self.actual_position - self.last_stable_position) > self.STUCK_SENSOR_TOLERANCE:
            # Position changed - reset stability timer
            self.last_stable_position = self.actual_position
            self.position_stable_since = now
            return

        # Position has been stable
        stable_duration = now - self.position_stable_since
        position_error = abs(self.commanded_position - self.actual_position)

        if stable_duration > self.STUCK_SENSOR_SEC and position_error > self.JAM_POSITION_TOLERANCE:
            self.report_condition(
                self.COND_ACTUATOR_JAMMED,
                self.SEVERITY_HIGH,
                f"Sensor reading stuck at {self.actual_position} for {stable_duration:.1f}s "
                f"while movement commanded (target={self.commanded_position})",
                active=True,
                auto_clearable=False
            )

    # =========================================================================
    # Service Handler
    # =========================================================================

    def publish_lift_position(self, position):
        """Publish lift position command to ESP32 via MQTT."""
        if not self.mqtt_connected:
            self.get_logger().warn("MQTT not connected, cannot publish lift position")
            return False

        msg = '{"data": ' + str(position) + '}'
        self.mqttclient.publish('lift/set_position', msg)
        self.commanded_position = position
        self.position_commanded_time = time.time()
        # Reset stuck sensor tracking when a new command is issued
        self.last_stable_position = self.actual_position
        self.position_stable_since = time.time()
        return True

    def _on_lift_cmd(self, msg: String):
        """Handle lift_up/lift_down commands from mqtt_op via ROS topic."""
        cmd = msg.data.lower()
        self.get_logger().info(f"lift/cmd: {cmd}")
        if cmd == 'lift_up':
            self.publish_lift_position(self.up_position)
        elif cmd == 'lift_down':
            self.publish_lift_position(self.down_position)
        else:
            self.get_logger().warn(f"Unknown lift/cmd: {msg.data}")

    def _on_set_position(self, msg: Int32):
        """Handle lift position commands from mqtt_op via ROS topic."""
        self.get_logger().info(f"lift/set_position: {msg.data}")
        self.publish_lift_position(msg.data)

    def handle_lift_control(self, request, response):
        """Handle lift control service requests."""
        command = request.command.lower()

        if command != 'get_status':
            self.get_logger().info(f"Received lift command: {command}")

        if command == 'lift_up':
            if self.publish_lift_position(self.up_position):
                response.success = True
                response.message = f"Lift moving up to position {self.up_position}"
            else:
                response.success = False
                response.message = "Failed to send lift command - MQTT not connected"

        elif command == 'lift_down':
            if self.publish_lift_position(self.down_position):
                response.success = True
                response.message = f"Lift moving down to position {self.down_position}"
            else:
                response.success = False
                response.message = "Failed to send lift command - MQTT not connected"

        elif command == 'set_position':
            position = request.position

            if position < self.min_position or position > self.max_position:
                self.report_condition(
                    self.COND_POSITION_OUT_OF_RANGE,
                    self.SEVERITY_HIGH,
                    f"Requested lift position {position} outside safe range "
                    f"[{self.min_position}, {self.max_position}]",
                    active=True,
                    auto_clearable=False
                )
                response.success = False
                response.message = (
                    f"Position {position} out of safe range "
                    f"[{self.min_position}, {self.max_position}] - operator review required"
                )
            elif position > 0:
                if self.publish_lift_position(position):
                    response.success = True
                    response.message = f"Lift moving to position {position}"
                    self.report_ok(self.COND_POSITION_OUT_OF_RANGE, "Valid position command received")
                else:
                    response.success = False
                    response.message = "Failed to send lift command - MQTT not connected"
            else:
                response.success = False
                response.message = "Invalid position value (must be > 0)"

        elif command == 'get_status':
            response.success = True
            response.message = "Status retrieved"

        else:
            response.success = False
            response.message = f"Unknown command: {command}"
            self.get_logger().warn(f"Unknown lift command: {command}")

        response.current_position = self.actual_position
        return response

    def destroy_node(self):
        """Cleanup on shutdown."""
        self._shutdown_requested = True
        self.mqttclient.loop_stop()
        self.mqttclient.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LiftServiceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
