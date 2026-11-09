#!/usr/bin/env python3
"""HiveMQ cloud ("outer") -> toolbar_service bridge.

Same pattern as ws4 solbot_control/mqtt_op (outer/steer, outer/speed): the phone MQTT
app publishes JSON {"data": ...} to the HiveMQ cloud broker, this node forwards it to
the toolbar_control service. Lives in ws5 so toolbar code stays out of ws4.

  outer/toolbar_pos  {"data": mm}       target in mm from centre, +right, -100..100
                                        (toolbar_service limits; rejected outside them)
  outer/toolbar_cmd  {"data": "center"} any toolbar_control command without a position:
                                        enable | disable | center | hold | auto |
                                        reset_fault | get_status

  pub to HiveMQ  outer/toolbar_state  {"pos_mm", "target_mm", "state", "message"}
                 after every command (for a text/gauge widget in the app)

Slider apps send a burst of values while dragging: only the newest one is sent,
at most one service call in flight.
"""

import json
import math
import uuid

import paho.mqtt.client as mqtt
import rclpy
import rclpy.executors
from rclpy.node import Node

from toolbar_msgs.srv import ToolbarControl

POS_TOPIC = 'outer/toolbar_pos'
CMD_TOPIC = 'outer/toolbar_cmd'
STATE_TOPIC = 'outer/toolbar_state'


class ToolbarOuterBridge(Node):
    def __init__(self):
        super().__init__('toolbar_outer_bridge')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        host = p('outer_host', 'ae4cb1b10ad84e53af8887dd32476b04.s2.eu.hivemq.cloud')
        port = int(p('outer_port', 8883))
        user = p('outer_user', 'aargideon')
        password = p('outer_password', 'para!234')

        self.client = self.create_client(ToolbarControl, 'toolbar_control')
        self._pending = None      # (command, position_m) waiting to be sent
        self._busy = False
        self.create_timer(0.1, self._pump)

        self.mqtt = mqtt.Client(client_id=f'toolbar_outer_bridge_{uuid.uuid4().hex[:8]}',
                                protocol=mqtt.MQTTv5)
        self.mqtt.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
        self.mqtt.username_pw_set(username=user, password=password)
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_message
        self.mqtt.reconnect_delay_set(min_delay=1, max_delay=30)
        self.mqtt.connect_async(host, port, keepalive=30)
        self.mqtt.loop_start()
        self.get_logger().info(f'toolbar_outer_bridge: {host}:{port} {POS_TOPIC} {CMD_TOPIC}')

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            client.subscribe([(POS_TOPIC, 0), (CMD_TOPIC, 0)])
            self.get_logger().info('Connected to HiveMQ cloud broker')
        else:
            self.get_logger().error(f'HiveMQ connect failed: {reason_code}')

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))['data']
            if msg.topic == POS_TOPIC:
                mm = float(data)
                if not math.isfinite(mm):
                    raise ValueError('not finite')
                self._pending = ('set_position', mm / 1000.0)
            elif msg.topic == CMD_TOPIC:
                self._pending = (str(data).strip().lower(), 0.0)
        except (ValueError, KeyError, TypeError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'bad payload on {msg.topic} {msg.payload!r}: {e} '
                                   '(expected {"data": ...})')

    def _pump(self):
        if self._busy or self._pending is None:
            return
        if not self.client.service_is_ready():
            self.get_logger().warn('toolbar_control service not available',
                                   throttle_duration_sec=5.0)
            return
        command, position = self._pending
        self._pending = None
        req = ToolbarControl.Request()
        req.command = command
        req.position = float(position)
        self._busy = True
        self.client.call_async(req).add_done_callback(
            lambda f, c=command, pos=position: self._done(f, c, pos))

    def _done(self, future, command, position):
        self._busy = False
        try:
            r = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'toolbar_control {command} failed: {e}')
            return
        text = f'{command} {position * 1000:+.0f} mm: {r.message}'
        if r.success:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        mm = lambda m: round(m * 1000, 1) if math.isfinite(m) else None  # noqa: E731
        self.mqtt.publish(STATE_TOPIC, json.dumps({
            'pos_mm': mm(r.current_position), 'target_mm': mm(r.target_position),
            'state': r.state, 'ok': r.success, 'message': r.message}))

    def destroy_node(self):
        self.mqtt.loop_stop()
        self.mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ToolbarOuterBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
