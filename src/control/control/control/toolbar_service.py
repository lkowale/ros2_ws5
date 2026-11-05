#!/usr/bin/env python3
"""
Toolbar (tool slider) controller node, built the same way as lift_service.

Executes toolbar movement commands, enforces safety limits and monitors the
actuator. The position loop itself runs on the ESP32. This node decides what
goal the ESP32 gets and when it gets none. Everything on the ROS side is in
metres; the `firmware` parameter selects the ESP32 protocol:

firmware: solbot_main   (solbot5 board, actuator #2 "slider", calibrated on the ESP32)
  sub  ads                   {"TBS": m, "TBS_goal": m, "estop": bool, ...}  10 Hz
  sub  <dev>/act/state       {"act":[{"name":"slider","duty","amps","fault"}]}  5 Hz
  pub  slider/set_position   {"data": m}

firmware: main_esp32_pio (solbot4 main ESP32, toolbar IBT-2 + pot on ADS1015 A2)
  sub  ads                   {"TPS": counts, "TPS_pwm": 0..255, "TPS_pos_goal": counts}  ~7 Hz
  pub  toolbar/set_position  {"data": counts}
  Raw counts <-> metres: linear through (counts_at_min, min_position) and
  (counts_at_max, max_position). This firmware has NO goal clamp, stall,
  overcurrent or comm-timeout protection, so this node is the only guard.

Same file in solbot5 (ros2_ws5, package `control`, solbot5_msgs) and solbot4
(ros2_ws4, package `solbot_control`, solbot4_msgs); the master copy is ws5.

ROS interface
  sub  tool_bar_position_cmd (Float32, m)  streaming target from tool_slider_controller
                                           (followed only in "auto" mode)
  sub  toolbar/cmd           (String)      same commands as the service, no position
  pub  tool_bar_position     (Float32, m)  measured position, on every sensor sample
  pub  /toolbar/state        (solbot5_msgs/ToolbarState, 10 Hz)
  srv  toolbar_control       (solbot5_msgs/ToolbarControl)
       set_position | center | hold | auto | enable | disable | reset_fault | get_status

Safety
  - Targets are checked against [min_position, max_position]. Service requests
    outside the range are rejected; the auto stream is clamped.
  - Nothing is sent while disabled, with no sensor data, with the e-stop pressed,
    or with a latched fault.
  - Disabled (monitor only) never pauses the robot: every condition is
    downgraded to LOW and nothing latches.
  - Auto mode: if the command stream stops for cmd_timeout_s, the toolbar holds
    where it is.
  - Jam: error above jam_tolerance with no progress for jam_timeout_s latches a
    fault and holds. A firmware stall or overcurrent latches too. Latched faults
    stay until reset_fault, because any new goal clears the firmware's own latch.

Diagnostic conditions (to PauseManager via /diagnostic_report)
  mqtt_disconnected      MEDIUM  auto-clear
  sensor_timeout         HIGH    no TBS from ESP32 for sensor_timeout_s
  sensor_out_of_range    HIGH    TBS outside limits +/- sensor_margin
  sensor_fault           HIGH    firmware reports pot fault (wiper <1% / >99%)
  actuator_jammed        HIGH    not reaching target (node side detection)
  actuator_fault         HIGH    firmware stall / overcurrent
  position_out_of_range  HIGH    service asked for a position outside the limits
  command_timeout        LOW     auto stream stopped, holding
  estop                  LOW     e-stop pressed (the stop itself is handled elsewhere)
"""

import json
import math
import os
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:  # solbot5
    from solbot5_msgs.msg import ToolbarState
    from solbot5_msgs.srv import ToolbarControl
except ImportError:  # solbot4
    from solbot4_msgs.msg import ToolbarState
    from solbot4_msgs.srv import ToolbarControl
from solbot_telemetry.diagnostic_mixin import DiagnosticMixin

# firmware profiles: ads keys, goal topic, whether ESP32 reports in metres
FIRMWARE = {
    'solbot_main': dict(pos_key='TBS', goal_key='TBS_goal', pwm_key=None,
                        goal_topic='slider/set_position', metres=True),
    'main_esp32_pio': dict(pos_key='TPS', goal_key='TPS_pos_goal', pwm_key='TPS_pwm',
                           goal_topic='toolbar/set_position', metres=False),
}

NAN = float('nan')


class ToolbarServiceNode(Node, DiagnosticMixin):
    COND_MQTT_DISCONNECTED = 'mqtt_disconnected'
    COND_SENSOR_TIMEOUT = 'sensor_timeout'
    COND_SENSOR_OUT_OF_RANGE = 'sensor_out_of_range'
    COND_SENSOR_FAULT = 'sensor_fault'
    COND_ACTUATOR_JAMMED = 'actuator_jammed'
    COND_ACTUATOR_FAULT = 'actuator_fault'
    COND_POSITION_OUT_OF_RANGE = 'position_out_of_range'
    COND_COMMAND_TIMEOUT = 'command_timeout'
    COND_ESTOP = 'estop'

    def __init__(self):
        super().__init__('toolbar_service')
        self._shutdown_requested = False
        self._lock = threading.Lock()
        self.setup_diagnostics('toolbar_service')

        cfg = self._load_yaml_section('toolbar')

        def p(name, default):
            return self.declare_parameter(name, cfg.get(name, default)).value

        self.firmware = p('firmware', 'solbot_main')
        if self.firmware not in FIRMWARE:
            raise ValueError(f'firmware must be one of {list(FIRMWARE)}, got {self.firmware}')
        self.fw = FIRMWARE[self.firmware]
        self.broker_address = p('broker_ip_address', 't630')
        self.broker_port = int(p('broker_port', 1883))
        self.device_name = p('device_name', 'ESP32_solbot_main')
        self.actuator_name = p('actuator_name', 'slider')
        self.min_position = float(p('min_position', -0.10))
        self.max_position = float(p('max_position', 0.10))
        self.center_position = float(p('center_position', 0.0))
        self.sensor_margin = float(p('sensor_margin', 0.02))
        self.position_tolerance = float(p('position_tolerance', 0.005))
        self.send_deadband = float(p('send_deadband', 0.0005))
        self.resend_period_s = float(p('resend_period_s', 1.0))
        self.sensor_timeout_s = float(p('sensor_timeout_s', 1.0))
        self.cmd_timeout_s = float(p('cmd_timeout_s', 1.0))
        self.jam_tolerance = float(p('jam_tolerance', 0.01))
        self.jam_progress = float(p('jam_progress', 0.003))
        self.jam_timeout_s = float(p('jam_timeout_s', 3.0))
        self.enabled = bool(p('actuator_enabled', False))
        # main_esp32_pio only: raw counts at min_position / max_position (CALIBRATE)
        self.counts_at_min = float(p('counts_at_min', 300.0))
        self.counts_at_max = float(p('counts_at_max', 1300.0))
        self.raw_valid_min = float(p('raw_valid_min', 10.0))     # pot open / shorted
        self.raw_valid_max = float(p('raw_valid_max', 1650.0))
        if not self.fw['metres'] and self.counts_at_min == self.counts_at_max:
            raise ValueError('counts_at_min == counts_at_max')

        if self.min_position >= self.max_position:
            raise ValueError(f'min_position {self.min_position} >= max_position {self.max_position}')
        if not self.min_position <= self.center_position <= self.max_position:
            raise ValueError(f'center_position {self.center_position} outside limits')

        # --- measured state (written by the MQTT thread under _lock)
        self.pos = NAN
        self.raw = NAN
        self.fw_goal = NAN
        self.estop = False
        self.last_sensor_t = 0.0   # 0 = never received
        self.duty = NAN
        self.amps = NAN
        self.fw_fault = ''
        self.mqtt_connected = False

        # --- command state
        self.mode = 'auto'
        self.target = NAN            # what we want the actuator to do
        self.sent_target = NAN       # what we last published
        self.last_send_t = 0.0
        self.last_stream_t = 0.0     # last tool_bar_position_cmd
        self.stream_holding = False
        self.fault_latched = False
        self.fault_reason = ''
        self.inhibit_reason = ''     # why we are not sending right now
        self._clamp_warn_t = 0.0
        self._last_report = {}

        # jam tracker
        self.jam_since = 0.0
        self.jam_ref_pos = NAN

        # --- ROS
        self.state_pub = self.create_publisher(ToolbarState, '/toolbar/state', 10)
        self.pos_pub = self.create_publisher(Float32, 'tool_bar_position', 10)
        self.create_subscription(Float32, 'tool_bar_position_cmd', self._on_stream_cmd, 10)
        self.create_subscription(String, 'toolbar/cmd', self._on_cmd_topic, 10)
        self.create_service(ToolbarControl, 'toolbar_control', self._handle_service)

        # --- MQTT
        self.act_state_topic = f'{self.device_name}/act/state'
        self.mqttclient = mqtt.Client(client_id='toolbar_service', userdata=None,
                                      protocol=mqtt.MQTTv5)
        self.mqttclient.username_pw_set(username='mark', password='pass')
        self.mqttclient.on_connect = self._on_connect
        self.mqttclient.on_disconnect = self._on_disconnect
        self.mqttclient.on_message = self._on_mqtt_message
        self.mqttclient.reconnect_delay_set(min_delay=1, max_delay=30)
        try:
            self.mqttclient.connect_async(self.broker_address, self.broker_port, keepalive=30)
            self.mqttclient.loop_start()
        except Exception as e:
            self.get_logger().error(f'MQTT connect failed: {e}')
            self._report_mqtt_down(str(e))

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'toolbar_service ({self.firmware}): '
            f'limits [{self.min_position:+.3f}, {self.max_position:+.3f}] m, '
            f'centre {self.center_position:+.3f}, broker {self.broker_address}, '
            f'device {self.device_name}, actuator {"ENABLED" if self.enabled else "DISABLED (monitor only)"}'
            + ('' if self.fw['metres'] else
               f', calib {self.counts_at_min:.0f}..{self.counts_at_max:.0f} counts'))

    # ======================================================================
    # Units
    # ======================================================================

    def _raw_to_m(self, raw):
        if self.fw['metres']:
            return raw
        return self.min_position + (raw - self.counts_at_min) / (
            self.counts_at_max - self.counts_at_min) * (self.max_position - self.min_position)

    def _m_to_raw(self, m):
        if self.fw['metres']:
            return round(m, 4)
        return int(round(self.counts_at_min + (m - self.min_position) / (
            self.max_position - self.min_position) * (self.counts_at_max - self.counts_at_min)))

    # ======================================================================
    # Config
    # ======================================================================

    def _load_yaml_section(self, section):
        err = None
        for pkg in ('control', 'solbot_control'):   # solbot5, solbot4
            try:
                path = os.path.join(get_package_share_directory(pkg), 'config', 'config.yaml')
                with open(path, 'r') as f:
                    return (yaml.safe_load(f) or {}).get(section, {}) or {}
            except Exception as e:
                err = e
        self.get_logger().warn(f'config.yaml [{section}] not loaded ({err}), using defaults')
        return {}

    # ======================================================================
    # MQTT
    # ======================================================================

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.mqtt_connected = True
            client.subscribe('ads')
            if self.fw['metres']:
                client.subscribe(self.act_state_topic)
            self.get_logger().info('MQTT connected')
            self.report_ok(self.COND_MQTT_DISCONNECTED, 'Toolbar MQTT connection restored')
            # force a fresh goal so the ESP32 is in sync after reconnect
            self.sent_target = NAN
        else:
            self.mqtt_connected = False
            self.get_logger().error(f'MQTT connect refused: {reason_code}')

    def _on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        self.mqtt_connected = False
        if not self._shutdown_requested:
            # paho's loop reconnects by itself (reconnect_delay_set). Flapping every
            # few seconds = another client with the same client_id (a second instance).
            self.get_logger().warn(f'MQTT disconnected: {reason_code}', throttle_duration_sec=10.0)
            self._report_mqtt_down(str(reason_code))

    def _report_mqtt_down(self, why):
        self._report(self.COND_MQTT_DISCONNECTED, self.SEVERITY_MEDIUM,
                     f'Toolbar MQTT disconnected: {why}', auto_clearable=True)

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'bad JSON on {msg.topic}: {e}', throttle_duration_sec=5.0)
            return
        if msg.topic == 'ads':
            self._on_ads(data)
        elif msg.topic == self.act_state_topic:
            self._on_act_state(data)

    def _on_ads(self, data):
        raw = data.get(self.fw['pos_key'])
        if raw is None:
            return
        try:
            raw = float(raw)
        except (TypeError, ValueError):
            return
        if not math.isfinite(raw):
            return
        pos = self._raw_to_m(raw)
        with self._lock:
            self.raw = raw
            self.pos = pos
            g = data.get(self.fw['goal_key'])
            self.fw_goal = self._raw_to_m(float(g)) if isinstance(g, (int, float)) else NAN
            if self.fw['pwm_key'] and isinstance(data.get(self.fw['pwm_key']), (int, float)):
                self.duty = data[self.fw['pwm_key']] / 255.0   # unsigned, and stale when stopped
            self.estop = bool(data.get('estop', False))
            self.last_sensor_t = time.monotonic()
        try:
            self.pos_pub.publish(Float32(data=pos))
        except Exception:   # rclpy already shut down, MQTT thread still delivering
            pass

    def _on_act_state(self, data):
        for a in data.get('act', []):
            if a.get('name') == self.actuator_name:
                with self._lock:
                    self.duty = float(a.get('duty', NAN))
                    self.amps = float(a.get('amps', NAN))
                    self.fw_fault = str(a.get('fault', ''))
                return

    def _send(self, target):
        """Publish goal to the ESP32. Caller has checked that sending is allowed."""
        if not self.mqtt_connected:
            return False
        info = self.mqttclient.publish(self.fw['goal_topic'],
                                       json.dumps({'data': self._m_to_raw(target)}))
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            return False
        self.sent_target = target
        self.last_send_t = time.monotonic()
        return True

    # ======================================================================
    # Command handling
    # ======================================================================

    def _sensor_fresh(self, now):
        return self.last_sensor_t > 0.0 and now - self.last_sensor_t <= self.sensor_timeout_s

    def _can_command(self, now):
        """Return '' if commands may reach the actuator, else the reason they cannot."""
        if not self.enabled:
            return 'disabled'
        if self.fault_latched:
            return f'fault latched: {self.fault_reason}'
        if not self._sensor_fresh(now):
            return 'no sensor data'
        if self.estop:
            return 'e-stop pressed'
        if not self.mqtt_connected:
            return 'MQTT not connected'
        return ''

    def _set_target(self, target):
        """Accept a new target (already limit-checked) and reset the jam tracker."""
        if math.isnan(self.target) or abs(target - self.target) > self.position_tolerance:
            self.jam_since = 0.0
        self.target = target

    def _hold_here(self):
        """Target := current position (if known) and push it to the ESP32 if allowed."""
        if math.isfinite(self.pos):
            self.target = min(max(self.pos, self.min_position), self.max_position)
            self.jam_since = 0.0
            if not self._can_command(time.monotonic()):
                self._send(self.target)

    def _on_stream_cmd(self, msg: Float32):
        now = time.monotonic()
        self.last_stream_t = now
        if self.mode != 'auto' or not math.isfinite(msg.data):
            return
        target = msg.data
        clamped = min(max(target, self.min_position), self.max_position)
        if clamped != target and now - self._clamp_warn_t > 5.0:
            self._clamp_warn_t = now
            self.get_logger().warn(
                f'stream target {target:+.3f} clamped to {clamped:+.3f} '
                f'(limits [{self.min_position:+.3f}, {self.max_position:+.3f}])')
        if self.stream_holding:
            self.stream_holding = False
            self.report_ok(self.COND_COMMAND_TIMEOUT, 'Toolbar command stream resumed')
            self.get_logger().info('command stream resumed')
        self._set_target(clamped)

    def _execute(self, command, position=NAN):
        """Run a command. Returns (success, message)."""
        command = command.strip().lower()
        now = time.monotonic()

        if command == 'get_status':
            return True, 'ok'

        if command == 'enable':
            self.enabled = True
            self._hold_here()
            return True, 'Actuator enabled, holding current position'

        if command == 'disable':
            if self.enabled and self._can_command(now) == '':
                self._hold_here()   # last goal = where it is
            self.enabled = False
            return True, 'Actuator disabled (monitor only)'

        if command == 'reset_fault':
            if not self._sensor_fresh(now):
                return False, 'Cannot reset: no sensor data'
            self.fault_latched = False
            self.fault_reason = ''
            self.mode = 'manual'
            self._hold_here()      # a fresh goal also clears the firmware stall/OC latch
            for c in (self.COND_ACTUATOR_JAMMED, self.COND_ACTUATOR_FAULT,
                      self.COND_SENSOR_OUT_OF_RANGE, self.COND_POSITION_OUT_OF_RANGE):
                self.report_ok(c, 'Fault reset by operator')
            return True, 'Fault reset, holding current position (manual mode, send "auto" to resume)'

        if command == 'auto':
            self.mode = 'auto'
            self.last_stream_t = now   # timeout counts from here
            return True, 'Following tool_bar_position_cmd'

        if command == 'hold':
            self.mode = 'manual'
            self._hold_here()
            return True, f'Holding at {self.target:+.3f} m'

        if command in ('center', 'set_position'):
            target = self.center_position if command == 'center' else float(position)
            if not math.isfinite(target):
                return False, 'Invalid position'
            why = self._can_command(now)
            if why:
                return False, f'Rejected: {why}'
            if not self.min_position <= target <= self.max_position:
                self._report(
                    self.COND_POSITION_OUT_OF_RANGE, self.SEVERITY_HIGH,
                    f'Requested toolbar position {target:+.3f} outside safe range '
                    f'[{self.min_position:+.3f}, {self.max_position:+.3f}]',
                    auto_clearable=False)
                return False, (f'Position {target:+.3f} outside safe range '
                               f'[{self.min_position:+.3f}, {self.max_position:+.3f}]')
            self.mode = 'manual'
            self._set_target(target)
            self.report_ok(self.COND_POSITION_OUT_OF_RANGE, 'Valid position command received')
            return True, f'Moving to {target:+.3f} m (manual mode)'

        return False, f'Unknown command: {command}'

    def _on_cmd_topic(self, msg: String):
        with self._lock:
            ok, text = self._execute(msg.data)
        self._log_result(ok, f'toolbar/cmd {msg.data}: {text}')

    def _handle_service(self, request, response):
        with self._lock:
            ok, text = self._execute(request.command, request.position)
            response.success = ok
            response.message = text
            response.state = self._state_name(time.monotonic())
            response.current_position = self.pos
            response.target_position = self.target
        if request.command.lower() != 'get_status':
            self._log_result(ok, f'toolbar_control {request.command} {request.position:+.3f}: {text}')
        return response

    def _log_result(self, ok, text):
        # rclpy forbids one call site switching severity, so keep them separate
        if ok:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    # ======================================================================
    # Periodic: monitor, decide, send, publish
    # ======================================================================

    def _report(self, cond, severity, message, auto_clearable):
        """report_condition, repeated at most once per second while active.
        Monitor-only (disabled) toolbar never pauses the robot: severity -> LOW."""
        if not self.enabled:
            severity = self.SEVERITY_LOW
        now = time.monotonic()
        if cond in self._active_conditions and now - self._last_report.get(cond, 0.0) < 1.0:
            return
        self._last_report[cond] = now
        self.report_condition(cond, severity, message, active=True, auto_clearable=auto_clearable)

    def _latch(self, cond, reason):
        if not self.enabled:   # monitor only: report, don't latch
            self.get_logger().warn(f'(monitor only) {reason}', throttle_duration_sec=10.0)
            self._report(cond, self.SEVERITY_LOW, reason, auto_clearable=True)
            return
        if not self.fault_latched:
            self.get_logger().error(f'FAULT: {reason}')
            self.fault_latched = True
            self.fault_reason = reason
            # stop it once: goal := current position (sent directly, bypasses the latch).
            # Not repeated: every new goal also clears the firmware stall/OC latch.
            if math.isfinite(self.pos) and self.enabled and self.mqtt_connected:
                self.target = self.pos
                self._send(self.pos)
        self._report(cond, self.SEVERITY_HIGH, reason, auto_clearable=False)

    def _tick(self):
        with self._lock:
            now = time.monotonic()
            self._check_sensor(now)
            self._check_firmware_fault()
            self._check_estop()
            self._check_stream_timeout(now)
            self._check_jam(now)
            self._maybe_send(now)
            self._publish_state(now)

    def _check_sensor(self, now):
        if self.last_sensor_t == 0.0:
            return
        age = now - self.last_sensor_t
        if age > self.sensor_timeout_s:
            self._report(self.COND_SENSOR_TIMEOUT, self.SEVERITY_HIGH,
                         f'No toolbar sensor data from ESP32 for {age:.1f}s', auto_clearable=True)
            self.jam_since = 0.0
            return
        self.report_ok(self.COND_SENSOR_TIMEOUT, 'Receiving toolbar sensor data')

        if not self.fw['metres'] and not self.raw_valid_min <= self.raw <= self.raw_valid_max:
            self._latch(self.COND_SENSOR_FAULT,
                        f'Toolbar pot reading {self.raw:.0f} counts outside '
                        f'[{self.raw_valid_min:.0f}, {self.raw_valid_max:.0f}] (open / shorted?)')
            return

        lo = self.min_position - self.sensor_margin
        hi = self.max_position + self.sensor_margin
        if not lo <= self.pos <= hi:
            self._latch(self.COND_SENSOR_OUT_OF_RANGE,
                        f'Toolbar sensor {self.pos:+.3f} outside [{lo:+.3f}, {hi:+.3f}] '
                        f'(check pot / calibration)')

    def _check_firmware_fault(self):
        f = self.fw_fault
        if f == 'sensor':
            self._latch(self.COND_SENSOR_FAULT, 'ESP32 reports toolbar pot fault (wiper at rail)')
        elif f in ('stall', 'overcurrent'):
            self._latch(self.COND_ACTUATOR_FAULT, f'ESP32 reports toolbar actuator {f}')

    def _check_estop(self):
        if self.estop:
            if self.COND_ESTOP not in self._active_conditions:
                self.get_logger().warn('e-stop pressed, toolbar commands suspended')
            self._report(self.COND_ESTOP, self.SEVERITY_LOW, 'E-stop pressed', auto_clearable=True)
            # firmware holds in place during e-stop; do not resume an old target afterwards
            if math.isfinite(self.pos):
                self.target = self.pos
                self.jam_since = 0.0
        elif self.COND_ESTOP in self._active_conditions:
            self.report_ok(self.COND_ESTOP, 'E-stop released')
            self.sent_target = NAN   # resync goal

    def _check_stream_timeout(self, now):
        if self.mode != 'auto' or self.last_stream_t == 0.0 or self.stream_holding:
            return
        age = now - self.last_stream_t
        if age > self.cmd_timeout_s:
            self.stream_holding = True
            self.get_logger().warn(f'tool_bar_position_cmd stopped ({age:.1f}s), holding position')
            self.report_condition(self.COND_COMMAND_TIMEOUT, self.SEVERITY_LOW,
                                  f'Toolbar command stream timed out ({age:.1f}s), holding',
                                  active=True, auto_clearable=True)
            if math.isfinite(self.pos):
                self.target = min(max(self.pos, self.min_position), self.max_position)
                self.jam_since = 0.0

    def _check_jam(self, now):
        """Error above jam_tolerance with less than jam_progress movement for jam_timeout_s."""
        if (self._can_command(now) or math.isnan(self.target)
                or math.isnan(self.sent_target)):
            self.jam_since = 0.0
            return
        err = abs(self.target - self.pos)
        if err <= self.jam_tolerance:
            self.jam_since = 0.0
            self.report_ok(self.COND_ACTUATOR_JAMMED, 'Toolbar reached target')
            return
        if self.jam_since == 0.0 or abs(self.pos - self.jam_ref_pos) > self.jam_progress:
            self.jam_since = now
            self.jam_ref_pos = self.pos
            return
        if now - self.jam_since > self.jam_timeout_s:
            self._latch(self.COND_ACTUATOR_JAMMED,
                        f'Toolbar not reaching target {self.target:+.3f}: at {self.pos:+.3f} '
                        f'(error {err * 1000:.0f} mm), no movement for {now - self.jam_since:.1f}s')

    def _maybe_send(self, now):
        why = self._can_command(now)
        self.inhibit_reason = why
        if why or math.isnan(self.target):
            return
        changed = (math.isnan(self.sent_target)
                   or abs(self.target - self.sent_target) > self.send_deadband)
        if changed or now - self.last_send_t > self.resend_period_s:
            self._send(self.target)

    def _state_name(self, now):
        if not self.enabled:
            return 'disabled'
        if not self._sensor_fresh(now):
            return 'no_sensor'
        if self.estop:
            return 'estop'
        if self.fault_latched:
            return 'fault'
        if self.mode == 'auto' and self.stream_holding:
            return 'holding'
        if self._in_motion():
            return 'moving'
        return 'at_target'

    def _in_motion(self):
        # main_esp32_pio TPS_pwm is not reset when the error hits exactly 0: ignore it
        if self.fw['metres'] and math.isfinite(self.duty) and abs(self.duty) > 0.01:
            return True
        return (math.isfinite(self.target) and math.isfinite(self.pos)
                and abs(self.target - self.pos) > self.position_tolerance)

    def _publish_state(self, now):
        m = ToolbarState()
        m.stamp = self.get_clock().now().to_msg()
        m.state = self._state_name(now)
        m.mode = self.mode
        m.fault_reason = self.fault_reason
        m.current_position = self.pos
        m.raw_position = self.raw
        m.target_position = self.target
        m.firmware_goal = self.fw_goal
        m.min_position = self.min_position
        m.max_position = self.max_position
        m.center_position = self.center_position
        m.duty = self.duty
        m.current_a = self.amps
        m.firmware_fault = self.fw_fault
        m.enabled = self.enabled
        m.sensor_ok = self._sensor_fresh(now)
        m.mqtt_connected = self.mqtt_connected
        m.estop = self.estop
        m.in_motion = self._in_motion()
        m.fault_latched = self.fault_latched
        self.state_pub.publish(m)

    def destroy_node(self):
        self._shutdown_requested = True
        try:
            self.mqttclient.loop_stop()
            self.mqttclient.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ToolbarServiceNode()
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
