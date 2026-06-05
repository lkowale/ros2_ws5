#!/usr/bin/env python3
"""
Drive Control Node with Self-Diagnostics

Controls wheel motors via MQTT and reports diagnostic conditions to PauseManager.

Diagnostic conditions reported:
- mqtt_disconnected (MEDIUM): MQTT broker connection lost
- cmd_timeout (MEDIUM): No cmd_vel received during navigation
- wheel_error_{wheel} (HIGH): ODrive axis error on a wheel motor
- wheel_no_current_{wheel} (HIGH): Wheel commanded but drawing no current (stall/disconnect)
- gps_accuracy (MEDIUM): GPS horizontal accuracy exceeds threshold
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
import paho.mqtt.client as mqtt
import math
import time
import json
from ament_index_python.packages import get_package_share_directory

from solbot_telemetry.diagnostic_mixin import DiagnosticMixin


class DriveControl(Node, DiagnosticMixin):
    # Diagnostic condition IDs
    COND_MQTT_DISCONNECTED = "mqtt_disconnected"
    COND_CMD_TIMEOUT = "cmd_timeout"
    COND_WHEEL_ERROR = "wheel_error"        # suffixed per wheel: wheel_error_FL etc.
    COND_WHEEL_NO_CURRENT = "wheel_no_current"  # suffixed per wheel
    COND_GPS_ACCURACY = "gps_accuracy"

    def __init__(self):
        super().__init__('drive_control')

        # Setup diagnostics
        self.setup_diagnostics('drive')

        # Load configuration
        try:
            pkg_dir = get_package_share_directory('solbot5_control')
            config_path = os.path.join(pkg_dir, 'config', 'config.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(config_path, 'r') as file:
            self.yaml_params = yaml.safe_load(file)

        # Iso-current config: equalise Iq across wheels during turns
        iso_cfg = self.yaml_params.get('iso_current', {})
        self.iso_current_enabled    = iso_cfg.get('enabled', True)
        self.iso_current_gain       = iso_cfg.get('gain', 0.3)
        self.iso_current_max_rpm    = iso_cfg.get('max_correction_rpm', 8.0)
        self.iso_current_deadband_a = iso_cfg.get('iq_deadband_a', 2.0)
        self.iso_current_min_turn   = iso_cfg.get('min_turn_angle', 0.1)

        # Wheel topology
        self.active_wheels    = self.yaml_params['wheels'].get('active', ['FL', 'FR'])
        self.reference_wheel  = self.yaml_params['wheels'].get('reference_wheel', 'FL')
        # Direction per wheel: 0=CW, 1=CCW — used to normalize Iq sign
        dir_cfg = self.yaml_params['wheels'].get('direction', {})
        self._iq_sign = {w: (-1 if dir_cfg.get(w, 0) == 1 else 1)
                         for w in ('FL', 'FR', 'RL', 'RR')}

        # Latest readings from each wheel, updated via MQTT
        self._iq = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self._motor_enabled = {'FL': False, 'FR': False, 'RL': False, 'RR': False}
        self._wheel_error = {'FL': 0, 'FR': 0, 'RL': 0, 'RR': 0}

        # No-current detection: time when each wheel first became commanded-but-no-current
        self._no_current_since: dict[str, float | None] = {w: None for w in ('FL', 'FR', 'RL', 'RR')}
        self.no_current_iq_threshold = self.declare_parameter('no_current_iq_threshold', 0.5).value
        self.no_current_delay = self.declare_parameter('no_current_delay', 3.0).value
        self.GPS_HACC_THRESHOLD_MM = self.declare_parameter('gps_hacc_threshold_mm', 15.0).value
        self._hacc_ok_since: float | None = None  # monotonic time when hAcc first came back OK
        self._hacc_resume_confirm_s: float = self.declare_parameter(
            'hacc_resume_confirm_s', 5.0).value

        # Timeout settings
        self.cmd_timeout_seconds = self.declare_parameter(
            'cmd_timeout_seconds', 2.0).value
        self.navigation_active = False
        self.last_cmd_time = self.get_clock().now()
        self._shutdown_requested = False

        self._wheel_speeds = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # Handbrake
        self.handbrake_active = False
        self.create_subscription(Bool, '/handbrake', self._cb_handbrake, 10)
        self._handbrake_state_pub = self.create_publisher(Bool, '/handbrake_state', 10)
        self.create_timer(1.0, self._pub_handbrake_state)

        # ODrive error clear
        self.create_subscription(Bool, '/clear_odrive_errors', self._cb_clear_odrive_errors, 10)

        # GPS accuracy monitor
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            UBXNavHPPosLLH, '/ubx_nav_hp_pos_llh',
            self._cb_hp_pos_llh, best_effort_qos)

        # cmd_vel subscription
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        # MQTT setup
        self.broker_address = self.declare_parameter(
            "broker_ip_address", 't630').value
        self.mqttclient = mqtt.Client(
            client_id="drive_outer_bridge",
            userdata=None,
            protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username="mark", password="pass")
        self.mqttclient.on_disconnect = self.on_disconnect
        self.mqttclient.on_connect = self.on_connect
        self.mqtt_connected = False

        self.mqttclient.on_message = self.on_mqtt_message

        self.get_logger().info(f"Connecting to MQTT broker: {self.broker_address}")
        try:
            self.mqttclient.connect(self.broker_address, keepalive=60)
            self.mqttclient.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            self.report_condition(
                self.COND_MQTT_DISCONNECTED,
                self.SEVERITY_MEDIUM,
                f"Failed to connect to MQTT broker: {e}",
                active=True,
                auto_clearable=True
            )

        # Publish wheel speed feedback at 10 Hz for odometry
        self._wheel_speed_pub = self.create_publisher(String, '/wheel_speeds', 10)
        self.create_timer(0.1, self._publish_wheel_speeds)

        # Timer for cmd_vel timeout check
        self.timeout_timer = self.create_timer(0.5, self.check_cmd_timeout)
        self.create_timer(0.2, self._check_no_current)

        # Iso-current correction timer (5 Hz), independent of cmd_vel
        self._last_speeds = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self._last_twist_cmd = 0.0
        self._last_gen_speed = 0.0
        self.iso_timer = self.create_timer(0.2, self.iso_current_update)


    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        """Handle MQTT connection."""
        if reason_code == 0:
            self.get_logger().info("Connected to MQTT broker")
            self.mqtt_connected = True
            self.report_ok(self.COND_MQTT_DISCONNECTED, "MQTT connection restored")
            # Subscribe to Iq, speed, motor_enabled, error, and handbrake echo topics
            for wheel in ('FL', 'FR', 'RL', 'RR'):
                client.subscribe(f"{wheel}/iq")
                client.subscribe(f"{wheel}/speed")
                client.subscribe(f"{wheel}/motor_enabled")
                client.subscribe(f"{wheel}/error")
                client.subscribe(f"{wheel}/handbrake_state")
        else:
            self.get_logger().error(f"Failed to connect to MQTT: {reason_code}")

    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages (wheel Iq and speed telemetry)."""
        parts = msg.topic.split('/')
        if len(parts) == 2 and parts[0] in self._iq:
            try:
                val = float(msg.payload.decode())
            except ValueError:
                return
            if parts[1] == 'iq':
                self._iq[parts[0]] = val
            elif parts[1] == 'speed':
                self._wheel_speeds[parts[0]] = val
            elif parts[1] == 'motor_enabled':
                self._motor_enabled[parts[0]] = val == 1.0
            elif parts[1] == 'error':
                self._handle_wheel_error(parts[0], int(val))

    def _handle_wheel_error(self, wheel: str, error_code: int):
        """Handle ODrive axis error code from MQTT. Reports HIGH condition on rising edge, clears on zero."""
        cond_id = f"{self.COND_WHEEL_ERROR}_{wheel}"
        prev = self._wheel_error.get(wheel, 0)
        self._wheel_error[wheel] = error_code

        if error_code != 0 and prev == 0:
            self.get_logger().error(
                f"Wheel {wheel} motor error: code={error_code:#010x}")
            self.report_condition(
                cond_id,
                self.SEVERITY_HIGH,
                f"Wheel {wheel} motor error: code={error_code:#010x}",
                active=True,
                auto_clearable=False,
            )
        elif error_code == 0 and prev != 0:
            self.get_logger().info(f"Wheel {wheel} motor error cleared")
            self.report_ok(cond_id, f"Wheel {wheel} error cleared")

    def _cb_hp_pos_llh(self, msg: UBXNavHPPosLLH):
        """Check GPS horizontal accuracy. Pause if hAcc > threshold, auto-resume when it improves."""
        import time as _time
        h_acc_mm = msg.h_acc * 0.1  # h_acc is in 0.1mm units
        if h_acc_mm > self.GPS_HACC_THRESHOLD_MM:
            self._hacc_ok_since = None  # reset confirmation timer on any bad reading
            if self.COND_GPS_ACCURACY not in self._active_conditions:
                self.get_logger().warn(
                    f"GPS hAcc {h_acc_mm:.1f}mm exceeds {self.GPS_HACC_THRESHOLD_MM}mm threshold")
                self.report_condition(
                    self.COND_GPS_ACCURACY,
                    self.SEVERITY_MEDIUM,
                    f"GPS hAcc {h_acc_mm:.1f}mm > {self.GPS_HACC_THRESHOLD_MM}mm",
                    active=True,
                    auto_clearable=True,
                )
        else:
            if self.COND_GPS_ACCURACY in self._active_conditions:
                # Require hAcc to stay within threshold for hacc_resume_confirm_s before clearing
                if self._hacc_ok_since is None:
                    self._hacc_ok_since = _time.monotonic()
                elapsed = _time.monotonic() - self._hacc_ok_since
                if elapsed >= self._hacc_resume_confirm_s:
                    self.get_logger().info(
                        f"GPS hAcc {h_acc_mm:.1f}mm OK for {elapsed:.1f}s — resuming")
                    self._hacc_ok_since = None
                    self.report_ok(self.COND_GPS_ACCURACY,
                                   f"GPS hAcc {h_acc_mm:.1f}mm OK")
            else:
                self._hacc_ok_since = None  # not paused, no timer needed

    def on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        """Handle MQTT disconnection with diagnostic reporting."""
        self.mqtt_connected = False

        if self._shutdown_requested:
            return

        # Report disconnect condition
        self.report_condition(
            self.COND_MQTT_DISCONNECTED,
            self.SEVERITY_MEDIUM,
            f"MQTT broker disconnected: {reason_code}",
            active=True,
            auto_clearable=True
        )

        self.get_logger().info(f"Disconnected with result code: {reason_code}")

        # Attempt reconnection
        FIRST_RECONNECT_DELAY = 1
        RECONNECT_RATE = 2
        MAX_RECONNECT_COUNT = 12
        MAX_RECONNECT_DELAY = 60

        reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
        while reconnect_count < MAX_RECONNECT_COUNT and not self._shutdown_requested:
            self.get_logger().info(f"Reconnecting in {reconnect_delay} seconds...")
            time.sleep(reconnect_delay)

            if self._shutdown_requested:
                break

            try:
                client.reconnect()
                self.get_logger().info("Reconnected successfully!")
                self.mqtt_connected = True
                # Clear condition - on_connect will handle this
                return
            except Exception as err:
                self.get_logger().error(f"Reconnect failed: {err}")

            reconnect_delay *= RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
            reconnect_count += 1

        if not self._shutdown_requested:
            self.get_logger().error(
                f"Reconnect failed after {reconnect_count} attempts")

    def check_cmd_timeout(self):
        """Check for cmd_vel timeout during navigation."""
        # Only check if navigation is active (we've received at least one cmd)
        if not self.navigation_active:
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_timeout_seconds:
            if self.COND_CMD_TIMEOUT not in self._active_conditions:
                self.report_condition(
                    self.COND_CMD_TIMEOUT,
                    self.SEVERITY_MEDIUM,
                    f"No cmd_vel received for {elapsed:.1f}s during navigation",
                    active=True,
                    auto_clearable=True,
                    details=[f"timeout_seconds={self.cmd_timeout_seconds}"]
                )
        else:
            # Clear timeout if we're receiving commands again
            if self.COND_CMD_TIMEOUT in self._active_conditions:
                self.report_ok(
                    self.COND_CMD_TIMEOUT,
                    "cmd_vel commands resumed")

    def _check_no_current(self):
        """Detect wheels commanded to move but drawing no current (stall or disconnect)."""
        now = time.monotonic()
        for w in self.active_wheels:
            commanded = abs(self._last_speeds.get(w, 0.0)) > 3.0  # rpm threshold
            has_current = abs(self._iq[w]) >= self.no_current_iq_threshold
            enabled = self._motor_enabled[w]
            cond_id = f"{self.COND_WHEEL_NO_CURRENT}_{w}"

            if commanded and enabled and not has_current:
                if self._no_current_since[w] is None:
                    self._no_current_since[w] = now
                elif now - self._no_current_since[w] >= self.no_current_delay:
                    if cond_id not in self._active_conditions:
                        self.get_logger().error(
                            f"Wheel {w}: commanded {self._last_speeds[w]:.1f} RPM "
                            f"but Iq={self._iq[w]:.2f}A — stall or disconnect")
                        self.report_condition(
                            cond_id,
                            self.SEVERITY_HIGH,
                            f"Wheel {w} commanded but drawing no current",
                            active=True,
                            auto_clearable=True,
                            details=[
                                f"commanded_rpm={self._last_speeds[w]:.1f}",
                                f"iq={self._iq[w]:.2f}",
                                f"threshold={self.no_current_iq_threshold}",
                            ]
                        )
            else:
                self._no_current_since[w] = None
                if cond_id in self._active_conditions:
                    self.report_ok(cond_id, f"Wheel {w} current restored")

    def _publish_wheel_speeds(self):
        """Publish wheel speed feedback (motor RPM from ODrive) as JSON."""
        msg = String()
        msg.data = json.dumps(self._wheel_speeds)
        self._wheel_speed_pub.publish(msg)

    def listener_callback(self, msg):
        """Process cmd_vel messages."""
        # A zero-velocity command means the controller has stopped the robot intentionally.
        # Reset navigation_active so the watchdog doesn't fire during stationary BT steps
        # (e.g. LiftUp/LiftDown) that follow the end of a path.
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.navigation_active = False
            if self.COND_CMD_TIMEOUT in self._active_conditions:
                self.report_ok(self.COND_CMD_TIMEOUT, "cmd_vel commands resumed")
            # Still update timestamp and forward the zero command to wheels
            self.last_cmd_time = self.get_clock().now()
        else:
            # Update timestamp for timeout detection
            self.last_cmd_time = self.get_clock().now()
            self.navigation_active = True

            # Clear timeout condition if active
            if self.COND_CMD_TIMEOUT in self._active_conditions:
                self.report_ok(self.COND_CMD_TIMEOUT, "cmd_vel commands resumed")

        drive_cmd = msg.linear.x
        twist_cmd = msg.angular.z

        # Handbrake interlock
        if self.handbrake_active:
            if abs(drive_cmd) >= 0.001 or abs(twist_cmd) >= 0.001:
                self._release_handbrake()
            else:
                return  # locked, discard zero vel

        # Get wheel parameters
        wheels_W = self.yaml_params['wheels']['width']
        wheels_L = self.yaml_params['wheels']['length']
        wheel_d = self.yaml_params['wheels']['diameter']
        wheel_gear_ratio = self.yaml_params['wheels']['gear_ratio']

        gen_speed = drive_cmd
        # Calculate wheel rpm speed knowing its diameter and the desired speed
        rps = gen_speed / (wheel_d * math.pi)  # in m/s
        rpm_w = rps * 60
        rpm_m = rpm_w / wheel_gear_ratio
        gen_speed = rpm_m

        abs_turn_angle_rd = abs(twist_cmd)
        speeds = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        if abs_turn_angle_rd > 0.02:
            # Use differential steering
            X = wheels_L / math.tan(abs_turn_angle_rd)
            R_inner_speed = gen_speed
            R_outer_speed = gen_speed * ((X + wheels_W) / X)
            F_inner_speed = gen_speed * (math.sqrt((X * X) + (wheels_L * wheels_L)) / X)
            F_outer_speed = gen_speed * (
                math.sqrt(((X + wheels_W) * (X + wheels_W)) + (wheels_L * wheels_L)) / X)

            if twist_cmd > 0:
                # Turn left: FL/RL inner, FR/RR outer
                speeds['FL'] = F_inner_speed
                speeds['RL'] = R_inner_speed
                speeds['FR'] = F_outer_speed
                speeds['RR'] = R_outer_speed
            else:
                # Turn right: FR/RR inner, FL/RL outer
                speeds['FL'] = F_outer_speed
                speeds['RL'] = R_outer_speed
                speeds['FR'] = F_inner_speed
                speeds['RR'] = R_inner_speed
        else:
            # Same speed for every wheel
            for w in speeds:
                speeds[w] = gen_speed

        # Store base speeds and twist for the iso_current timer
        self._last_speeds = dict(speeds)
        self._last_twist_cmd = twist_cmd
        self._last_gen_speed = gen_speed

        for w in ('FL', 'FR', 'RL', 'RR'):
            self.send_wheel_speed(w, speeds[w])

    def _apply_handbrake(self):
        self.handbrake_active = True
        self._last_gen_speed = 0.0
        self._last_speeds = {w: 0.0 for w in self._last_speeds}
        if self.mqtt_connected:
            for w in ('FL', 'FR', 'RL', 'RR'):
                self.send_wheel_speed(w, 0.0)
            for w in ('FL', 'FR', 'RL', 'RR'):
                self.mqttclient.publish(f'{w}/handbrake', '1')
        self._pub_handbrake_state()
        self.get_logger().info('Handbrake ENGAGED')

    def _release_handbrake(self):
        self.handbrake_active = False
        if self.mqtt_connected:
            for w in ('FL', 'FR', 'RL', 'RR'):
                self.mqttclient.publish(f'{w}/handbrake', '0')

        self._pub_handbrake_state()
        self.get_logger().info('Handbrake RELEASED')

    def _cb_handbrake(self, msg: Bool):
        if msg.data and not self.handbrake_active:
            self._apply_handbrake()
        elif not msg.data and self.handbrake_active:
            self._release_handbrake()

    def _pub_handbrake_state(self):
        msg = Bool()
        msg.data = self.handbrake_active
        self._handbrake_state_pub.publish(msg)

    def _cb_clear_odrive_errors(self, msg: Bool):
        if not msg.data:
            return
        if not self.mqtt_connected:
            self.get_logger().warn('Cannot clear ODrive errors: MQTT not connected')
            return
        for w in ('FL', 'FR', 'RL', 'RR'):
            self.mqttclient.publish(f'{w}/clear_errors', '1')
        self.get_logger().info('ODrive errors cleared on all wheels')

    def iso_current_update(self):
        """Apply iso-current correction at 5 Hz, independent of cmd_vel."""
        if not self.iso_current_enabled:
            return
        if self.handbrake_active:
            return

        if self._last_gen_speed == 0.0:
            return

        iq_ref = self._iq[self.reference_wheel] * self._iq_sign[self.reference_wheel]
        for w in self.active_wheels:
            if w == self.reference_wheel:
                continue
            iq_w = self._iq[w] * self._iq_sign[w]
            iq_diff = iq_ref - iq_w
            if abs(iq_diff) > self.iso_current_deadband_a:
                correction = self.iso_current_gain * iq_diff
                correction = max(-self.iso_current_max_rpm,
                                 min(self.iso_current_max_rpm, correction))
            else:
                correction = 0.0
            self.send_wheel_speed(w, self._last_speeds[w] + correction)

    def send_wheel_speed(self, wheel_name, speed):
        """Send wheel speed command via MQTT."""
        if not self.mqtt_connected:
            return

        topic = wheel_name + "/set_speed"
        msg = '{"data": ' + str(speed) + '}'
        self.mqttclient.publish(topic, msg)

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
        node = DriveControl()
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
