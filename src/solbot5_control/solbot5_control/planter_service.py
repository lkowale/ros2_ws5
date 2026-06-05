#!/usr/bin/env python3
"""
Planter Control Service Node with Self-Diagnostics

Provides a ROS2 service interface for controlling multiple planters with MQTT backend.
Reports diagnostic conditions to PauseManager.

Diagnostic conditions reported:
- mqtt_disconnected (HIGH): MQTT broker connection lost - crop damage risk
- rpm_calculation_error (LOW): RPM calculation issue (report only)
- planter_jammed/<name> (HIGH): actual RPM < 20% of target for >3s while active

Parameters:
  - broker_ip_address: MQTT broker address (default: t630)
  - planter_names: List of planter names (default: ["PLANTER"])

Service: /planter_control (solbot4_msgs/srv/PlanterControl)
Commands: start, stop, rotate_once, set_density, set_aospr, get_status

If planter_name in request is empty, command applies to ALL planters.
"""

import math

import rclpy
from rclpy.node import Node
import rclpy.executors
import yaml
import os
import time
import paho.mqtt.client as mqtt
from nav_msgs.msg import Odometry
from solbot4_msgs.srv import PlanterControl
from ament_index_python.packages import get_package_share_directory

from solbot_telemetry.diagnostic_mixin import DiagnosticMixin


class PlanterState:
    """State for a single planter."""
    def __init__(self, name, desired_density=10, aospr=12):
        self.name = name
        self.is_active = False
        self.desired_density = desired_density
        self.aospr = aospr
        self.current_rpm = 0.0       # target RPM commanded to motor
        self.actual_rpm = 0.0        # measured RPM from Hall sensors (MQTT feedback)
        self.actual_density = 0.0
        self.low_rpm_since = None    # monotonic timestamp when RPM drop started


class PlanterServiceNode(Node, DiagnosticMixin):
    # Diagnostic condition IDs
    COND_MQTT_DISCONNECTED = "mqtt_disconnected"
    COND_RPM_CALC_ERROR = "rpm_calculation_error"

    # Jam watchdog parameters
    JAM_RPM_RATIO = 0.20    # actual/target ratio below which jam is suspected
    JAM_TIMEOUT_S = 3.0     # seconds of low RPM before triggering pause
    JAM_MIN_TARGET_RPM = 5.0  # only watch when target RPM is above this

    def __init__(self):
        super().__init__('planter_service')
        self._shutdown_requested = False

        # Setup diagnostics
        self.setup_diagnostics('planter_service')

        # Load configuration
        try:
            pkg_dir = get_package_share_directory('solbot5_control')
            config_path = os.path.join(pkg_dir, 'config', 'config.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')

        with open(config_path, 'r') as file:
            self.yaml_params = yaml.safe_load(file)

        # MQTT setup
        self.broker_address = self.declare_parameter("broker_ip_address", 't630').value
        self.get_logger().info(f"MQTT broker address: {self.broker_address}")

        # Planter names parameter - list of planter names
        default_names = ["PLANTER"]
        self.planter_names = self.declare_parameter("planter_names", default_names).value
        self.get_logger().info(f"Planter names: {self.planter_names}")

        # Default settings from config
        default_density = self.yaml_params.get('planter', {}).get('desired_density', 10)
        default_aospr = self.yaml_params.get('planter', {}).get('aospr', 12)

        # Create state for each planter
        self.planters = {}
        for name in self.planter_names:
            self.planters[name] = PlanterState(name, default_density, default_aospr)
            self.get_logger().info(f"Registered planter: {name}")

        # MQTT client
        self.mqttclient = mqtt.Client(client_id="planter_service", userdata=None, protocol=mqtt.MQTTv5)
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
                self.SEVERITY_HIGH,
                f"Planter MQTT connection failed - crop damage risk: {e}",
                active=True,
                auto_clearable=False  # Operator must verify planter state
            )

        # Robot speed from odometry
        self.robot_speed = 0.0

        # Odometry topic parameter (configurable)
        odom_topic = self.declare_parameter("odom_topic", '/odometry/gps_vel').value
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        self.get_logger().info(f"Subscribed to {odom_topic} for robot speed")

        # Service server
        self.service = self.create_service(
            PlanterControl,
            'planter_control',
            self.handle_planter_control
        )

        # Timer for continuous planter speed updates when active
        self.update_timer = self.create_timer(0.1, self.update_planter_speeds)

        # Jam watchdog timer — checks at 2 Hz
        self.jam_timer = self.create_timer(0.5, self.check_jam_watchdog)

        self.get_logger().info(f'Planter service started with {len(self.planters)} planter(s)')

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.get_logger().info("Connected to MQTT broker")
            self.mqtt_connected = True
            # Subscribe to actual RPM feedback from each planter
            for name in self.planter_names:
                topic = f"{name}/rpm"
                client.subscribe(topic)
                self.get_logger().info(f"Subscribed to {topic} for jam watchdog")
            self.get_logger().info("MQTT reconnected - operator verification required")
        else:
            self.get_logger().error(f"Failed to connect to MQTT: {reason_code}")
            self.mqtt_connected = False

    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages — update actual RPM for jam watchdog."""
        topic = msg.topic
        for name, planter in self.planters.items():
            if topic == f"{name}/rpm":
                try:
                    planter.actual_rpm = float(msg.payload.decode())
                except (ValueError, UnicodeDecodeError):
                    pass
                break

    def on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        self.mqtt_connected = False

        # Don't attempt reconnection if shutdown was requested
        if self._shutdown_requested:
            return

        # Report HIGH severity - planter disconnect could damage crops
        self.report_condition(
            self.COND_MQTT_DISCONNECTED,
            self.SEVERITY_HIGH,
            f"Planter MQTT disconnected - crop damage risk: {reason_code}",
            active=True,
            auto_clearable=False  # Operator must verify
        )

        print(f"[planter_service] Disconnected from MQTT broker: {reason_code}")

        # Attempt reconnection
        reconnect_count = 0
        reconnect_delay = 1
        max_reconnect = 12

        while reconnect_count < max_reconnect and not self._shutdown_requested:
            print(f"[planter_service] Reconnecting in {reconnect_delay} seconds...")
            time.sleep(reconnect_delay)

            if self._shutdown_requested:
                break

            try:
                client.reconnect()
                print("[planter_service] Reconnected to MQTT broker")
                self.mqtt_connected = True
                return
            except Exception as err:
                print(f"[planter_service] Reconnect failed: {err}")

            reconnect_delay = min(reconnect_delay * 2, 60)
            reconnect_count += 1

        if not self._shutdown_requested:
            print(f"[planter_service] Failed to reconnect after {max_reconnect} attempts")

    def check_jam_watchdog(self):
        """Detect planter stall: actual RPM << target RPM while active and robot moving."""
        if abs(self.robot_speed) < 0.1:
            # Robot not moving — reset all jam timers, no jam possible
            for planter in self.planters.values():
                planter.low_rpm_since = None
            return

        now = time.monotonic()
        for planter in self.planters.values():
            cond_id = f"planter_jammed/{planter.name}"
            if not planter.is_active or planter.current_rpm < self.JAM_MIN_TARGET_RPM:
                planter.low_rpm_since = None
                continue

            ratio = planter.actual_rpm / planter.current_rpm if planter.current_rpm > 0 else 1.0
            if ratio < self.JAM_RPM_RATIO:
                if planter.low_rpm_since is None:
                    planter.low_rpm_since = now
                elif now - planter.low_rpm_since >= self.JAM_TIMEOUT_S:
                    self.report_condition(
                        cond_id,
                        self.SEVERITY_HIGH,
                        f"{planter.name} jammed — actual {planter.actual_rpm:.0f} RPM vs "
                        f"target {planter.current_rpm:.0f} RPM ({ratio*100:.0f}%)",
                        active=True,
                        auto_clearable=False,
                    )
            else:
                planter.low_rpm_since = None
                if cond_id in self._active_conditions:
                    self.report_ok(cond_id, f"{planter.name} RPM recovered")

    def odom_callback(self, msg):
        """Update robot speed from GPS velocity odometry (ENU components, take magnitude)."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_speed = math.hypot(vx, vy)

    def update_planter_speeds(self):
        """Periodically update all active planters' speeds based on robot speed."""
        active_count = sum(1 for p in self.planters.values() if p.is_active)
        if active_count > 0 and abs(self.robot_speed) > 0.01:
            self.get_logger().debug(f"Robot speed: {self.robot_speed:.3f} m/s, active planters: {active_count}")

        for planter in self.planters.values():
            if not planter.is_active:
                continue

            # Use absolute value of speed (robot may move backward)
            speed = abs(self.robot_speed)
            if speed > 0.1:
                try:
                    # Check for division by zero
                    if planter.aospr <= 0:
                        self.report_condition(
                            self.COND_RPM_CALC_ERROR,
                            self.SEVERITY_LOW,  # Report only
                            f"AOSPR is zero or negative for {planter.name}",
                            active=True,
                            auto_clearable=True
                        )
                        planter.current_rpm = 0.0
                        continue

                    # Calculate planter RPM based on robot speed and desired density
                    seeds_per_second = speed * planter.desired_density
                    rps = seeds_per_second / planter.aospr
                    planter.current_rpm = rps * 60

                    # Actual density = (rpm / 60) * aospr / speed
                    planter.actual_density = planter.desired_density

                    # Clear RPM error if previously set
                    if self.COND_RPM_CALC_ERROR in self._active_conditions:
                        self.report_ok(self.COND_RPM_CALC_ERROR, "RPM calculation OK")

                except Exception as e:
                    self.report_condition(
                        self.COND_RPM_CALC_ERROR,
                        self.SEVERITY_LOW,
                        f"RPM calculation error: {e}",
                        active=True,
                        auto_clearable=True
                    )
                    planter.current_rpm = 0.0
                    planter.actual_density = 0.0
            else:
                planter.current_rpm = 0.0
                planter.actual_density = 0.0

            self.publish_planter_speed(planter.name, planter.current_rpm)

    def publish_planter_speed(self, planter_name, rpm):
        """Publish planter speed to MQTT."""
        if not self.mqtt_connected:
            self.get_logger().warn("MQTT not connected, cannot publish planter speed")
            return

        msg = '{"data": ' + str(rpm) + '}'
        topic = f'{planter_name}/set_speed'
        self.mqttclient.publish(topic, msg)

    def rotate_once(self, planter_name):
        """Command a planter to rotate once."""
        if not self.mqtt_connected:
            return False, "MQTT not connected"

        msg = '{"data": 1}'
        topic = f'{planter_name}/revolution'
        self.mqttclient.publish(topic, msg)
        return True, f"Rotate once command sent to {planter_name}"

    def get_target_planters(self, planter_name):
        """Get list of planters to target based on request."""
        if not planter_name:
            # Empty name = all planters
            return list(self.planters.values())
        elif planter_name in self.planters:
            return [self.planters[planter_name]]
        else:
            return []

    def handle_planter_control(self, request, response):
        """Handle planter control service requests."""
        command = request.command.lower()
        target_name = request.planter_name

        # Get target planters
        targets = self.get_target_planters(target_name)

        if not targets:
            response.success = False
            response.message = f"Unknown planter: {target_name}"
            response.planter_names = []
            response.is_active = []
            response.current_density = []
            response.current_aospr = []
            response.current_rpm = []
            response.actual_density = []
            return response

        target_names = [p.name for p in targets]
        if command != 'get_status':
            self.get_logger().info(f"Received command '{command}' for planters: {target_names}")

        success = True
        messages = []

        if command == 'start':
            for planter in targets:
                planter.is_active = True
                messages.append(f"{planter.name} started")
            self.get_logger().info(f"Activated planters: {target_names}")

        elif command == 'stop':
            for planter in targets:
                planter.is_active = False
                planter.current_rpm = 0.0
                self.publish_planter_speed(planter.name, 0.0)
                messages.append(f"{planter.name} stopped")
            self.get_logger().info(f"Deactivated planters: {target_names}")

        elif command == 'rotate_once':
            for planter in targets:
                ok, msg = self.rotate_once(planter.name)
                if not ok:
                    success = False
                messages.append(msg)

        elif command == 'set_density':
            if request.desired_density > 0:
                for planter in targets:
                    planter.desired_density = request.desired_density
                    messages.append(f"{planter.name} density set to {request.desired_density}")
                self.get_logger().info(f"Set density to {request.desired_density} for: {target_names}")
            else:
                success = False
                messages.append("Invalid density value (must be > 0)")

        elif command == 'set_aospr':
            if request.aospr > 0:
                for planter in targets:
                    planter.aospr = request.aospr
                    messages.append(f"{planter.name} AOSPR set to {request.aospr}")
                self.get_logger().info(f"Set AOSPR to {request.aospr} for: {target_names}")
            else:
                success = False
                messages.append("Invalid AOSPR value (must be > 0)")

        elif command == 'get_status':
            messages.append("Status retrieved")

        else:
            success = False
            messages.append(f"Unknown command: {command}")
            self.get_logger().warn(f"Unknown planter command: {command}")

        # Build response with state for all targeted planters
        response.success = success
        response.message = "; ".join(messages)
        response.planter_names = [p.name for p in targets]
        response.is_active = [p.is_active for p in targets]
        response.current_density = [p.desired_density for p in targets]
        response.current_aospr = [p.aospr for p in targets]
        response.current_rpm = [p.current_rpm for p in targets]
        response.actual_density = [p.actual_density for p in targets]

        return response

    def destroy_node(self):
        """Cleanup on shutdown."""
        self._shutdown_requested = True

        # Stop all planters
        for planter in self.planters.values():
            planter.is_active = False
            self.publish_planter_speed(planter.name, 0.0)

        self.mqttclient.loop_stop()
        self.mqttclient.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlanterServiceNode()
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
