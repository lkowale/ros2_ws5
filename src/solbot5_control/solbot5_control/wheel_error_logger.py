#!/usr/bin/env python3
"""
Wheel Error Logger — ROS2 node

Subscribes to all wheel MQTT telemetry and keeps a per-wheel rolling buffer.
When any wheel reports a non-zero error code (rising edge), the last
buffer_seconds of all wheel data are saved to timestamped CSV files for
post-analysis.

Wheel names and reference_wheel are read from config.yaml:
    wheels:
      active: [FL, FR, RL, RR]
      reference_wheel: FL

Output directory: ~/wheel_errors/<YYYYMMDD_HHMMSS>_err_<wheel>/
    <wheel>.csv  — one CSV per active wheel, same format as odrive_logger.py

MQTT topics per wheel (device prefix = wheel name):
    {wheel}/speed          RPM
    {wheel}/speed_rps      rev/s
    {wheel}/speed_goal     target rev/s
    {wheel}/iq             torque current (A)
    {wheel}/pi_output      PI controller output / current command (A)
    {wheel}/pi_integral    PI integrator accumulator
    {wheel}/current        DC bus current (A)
    {wheel}/vbus           DC bus voltage (V)
    {wheel}/fet_temp       FET temperature (°C)
    {wheel}/motor_enabled  0=IDLE, 1=CLOSED_LOOP
    {wheel}/handbrake      handbrake state
    {wheel}/error          axis error code  ← triggers dump on rising edge
    {wheel}/errors         error string
    {wheel}/connected      ODrive UART connected
    {wheel}/pwm            PWM value

Usage:
    ros2 run solbot5_control wheel_error_logger \
        --ros-args -p broker_ip:=pd600 -p buffer_seconds:=10.0
"""

import csv
import os
import threading
import time
from collections import deque
from datetime import datetime

import rclpy
import yaml
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

try:
    import paho.mqtt.client as mqtt
except ImportError:
    raise ImportError("paho-mqtt not installed. Run: sudo apt install python3-paho-mqtt")


TOPICS = [
    "speed",
    "speed_rps",
    "speed_goal",
    "iq",
    "pi_output",
    "pi_integral",
    "current",
    "vbus",
    "fet_temp",
    "motor_enabled",
    "handbrake",
    "error",
    "errors",
    "connected",
    "pwm",
]

CSV_FIELDS = ["timestamp", "elapsed_s"] + TOPICS


def _load_config():
    try:
        pkg_dir = get_package_share_directory("solbot5_control")
        path = os.path.join(pkg_dir, "config", "config.yaml")
    except Exception:
        path = os.path.join(os.path.dirname(__file__), "..", "config", "config.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


class _WheelBuffer:
    """Thread-safe rolling buffer for one wheel's telemetry rows."""

    def __init__(self, buffer_seconds: float):
        self.buffer_seconds = buffer_seconds
        self._rows: deque = deque()  # each entry: (monotonic_time, row_dict)
        self._lock = threading.Lock()
        self.state = {t: "" for t in TOPICS}
        self.start_time: float | None = None

    def update(self, field: str, value: str):
        """Update field and append a snapshot row to the buffer."""
        self.state[field] = value
        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time
        wall_ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        row = {"timestamp": wall_ts, "elapsed_s": f"{elapsed:.3f}"}
        row.update(self.state)
        with self._lock:
            self._rows.append((now, row))
            self._prune(now)

    def _prune(self, now: float):
        cutoff = now - self.buffer_seconds
        while self._rows and self._rows[0][0] < cutoff:
            self._rows.popleft()

    def snapshot(self) -> list[dict]:
        """Return a copy of current buffer rows (already within window)."""
        now = time.monotonic()
        with self._lock:
            self._prune(now)
            return [row for _, row in self._rows]


class WheelErrorLogger(Node):
    def __init__(self):
        super().__init__("wheel_error_logger")

        broker_ip = self.declare_parameter("broker_ip", "pd600").value
        output_dir = self.declare_parameter(
            "output_dir", os.path.expanduser("~/wheel_errors")
        ).value
        self.output_dir = os.path.expanduser(output_dir)
        self.buffer_seconds = self.declare_parameter("buffer_seconds", 10.0).value
        self.cooldown_seconds = self.declare_parameter("cooldown_seconds", 30.0).value

        # Load wheel names from config.yaml
        cfg = _load_config()
        wheels_cfg = cfg.get("wheels", {})
        self.active_wheels: list[str] = wheels_cfg.get("active", ["FL", "FR", "RL", "RR"])
        self.reference_wheel: str = wheels_cfg.get("reference_wheel", self.active_wheels[0])

        os.makedirs(self.output_dir, exist_ok=True)

        self._buffers = {w: _WheelBuffer(self.buffer_seconds) for w in self.active_wheels}
        self._last_error_state: dict[str, bool] = {w: False for w in self.active_wheels}
        self._last_dump_wall: dict[str, float] = {w: 0.0 for w in self.active_wheels}
        self._dump_lock = threading.Lock()

        self.get_logger().info(
            f"Wheel error logger starting\n"
            f"  Wheels       : {self.active_wheels}\n"
            f"  Reference    : {self.reference_wheel}\n"
            f"  Buffer       : {self.buffer_seconds}s\n"
            f"  Cooldown     : {self.cooldown_seconds}s\n"
            f"  Output dir   : {self.output_dir}\n"
            f"  MQTT broker  : {broker_ip}"
        )

        self._mqtt = mqtt.Client(client_id="wheel_error_logger")
        self._mqtt.username_pw_set("mark", "pass")
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        self._mqtt.on_disconnect = self._on_disconnect

        try:
            self._mqtt.connect(broker_ip, 1883, keepalive=60)
            self._mqtt.loop_start()
        except Exception as exc:
            self.get_logger().error(f"MQTT connect failed: {exc}")

    # ──────────────────────────────────────────────────────────────────────────
    # MQTT callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc != 0:
            self.get_logger().error(f"MQTT connection failed: rc={rc}")
            return
        self.get_logger().info("MQTT connected — subscribing to wheel topics")
        for wheel in self.active_wheels:
            for topic in TOPICS:
                client.subscribe(f"{wheel}/{topic}")

    def _on_disconnect(self, client, userdata, flags, rc, properties=None):
        self.get_logger().warning(f"MQTT disconnected: rc={rc}")

    def _on_message(self, client, userdata, msg):
        parts = msg.topic.split("/", 1)
        if len(parts) != 2:
            return
        wheel, field = parts
        if wheel not in self._buffers or field not in self.state_fields:
            return

        payload = msg.payload.decode("utf-8", errors="replace").strip()
        self._buffers[wheel].update(field, payload)

        # Detect rising edge on error field
        if field == "error":
            is_err = payload not in ("", "0", "0.0", "0x0", "None")
            was_err = self._last_error_state[wheel]
            self._last_error_state[wheel] = is_err
            if is_err and not was_err:
                self.get_logger().warning(
                    f"Wheel {wheel} error detected: {payload!r} — saving buffer"
                )
                self._trigger_dump(wheel, payload)

    @property
    def state_fields(self):
        return set(TOPICS)

    # ──────────────────────────────────────────────────────────────────────────
    # Dump logic
    # ──────────────────────────────────────────────────────────────────────────

    def _trigger_dump(self, trigger_wheel: str, error_value: str):
        now = time.time()
        with self._dump_lock:
            if now - self._last_dump_wall[trigger_wheel] < self.cooldown_seconds:
                self.get_logger().info(
                    f"Dump for {trigger_wheel} skipped (cooldown active)"
                )
                return
            self._last_dump_wall[trigger_wheel] = now

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        dump_dir = os.path.join(self.output_dir, f"{ts}_err_{trigger_wheel}")
        os.makedirs(dump_dir, exist_ok=True)

        for wheel in self.active_wheels:
            rows = self._buffers[wheel].snapshot()
            path = os.path.join(dump_dir, f"{wheel}.csv")
            self._write_csv(path, wheel, rows, trigger_wheel, error_value)
            self.get_logger().info(f"  Saved {len(rows)} rows → {path}")

        self.get_logger().info(f"Dump complete: {dump_dir}")

    def _write_csv(
        self,
        path: str,
        wheel: str,
        rows: list[dict],
        trigger_wheel: str,
        error_value: str,
    ):
        with open(path, "w", newline="") as f:
            f.write(f"# wheel: {wheel}\n")
            f.write(f"# saved: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# trigger_wheel: {trigger_wheel}\n")
            f.write(f"# trigger_error: {error_value}\n")
            f.write(f"# buffer_seconds: {self.buffer_seconds}\n")
            f.write(f"# reference_wheel: {self.reference_wheel}\n")
            f.write(f"# active_wheels: {', '.join(self.active_wheels)}\n")
            f.write(f"# rows_in_buffer: {len(rows)}\n")
            f.write("# ---\n")
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

    # ──────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelErrorLogger()
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


if __name__ == "__main__":
    main()
