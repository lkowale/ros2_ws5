#!/usr/bin/env python3
"""Runs ON pd600, started over ssh by toolbar_gui.py (laptop). Bridges the robot's
loopback-only ROS graph to a JSON-lines pipe:

  stdout: {"type": "state", ...ToolbarState fields}   (every /toolbar/state message)
          {"type": "reply", "command": ..., "success": ..., "message": ...}
  stdin:  {"command": "set_position", "position": 0.05}   (any ToolbarControl command)

Needs: source ~/ros2_ws4/pd600_env.sh && source ~/ros2_ws5/install/setup.bash
"""

import json
import math
import queue
import sys
import threading

import rclpy
from rclpy.node import Node
from toolbar_msgs.msg import ToolbarState
from toolbar_msgs.srv import ToolbarControl

out_lock = threading.Lock()


def emit(obj):
    line = json.dumps(obj, allow_nan=False, default=str)
    with out_lock:
        sys.stdout.write(line + '\n')
        sys.stdout.flush()


def clean(v):
    return None if isinstance(v, float) and not math.isfinite(v) else v


class GuiBridge(Node):
    def __init__(self):
        super().__init__('toolbar_gui_bridge')
        self.requests = queue.Queue()
        self.create_subscription(ToolbarState, '/toolbar/state', self.on_state, 10)
        self.client = self.create_client(ToolbarControl, '/toolbar_control')
        self.create_timer(0.02, self.pump)
        threading.Thread(target=self.read_stdin, daemon=True).start()
        emit({'type': 'log', 'message': 'bridge up on pd600'})

    def on_state(self, m):
        d = {'type': 'state'}
        for f in m.get_fields_and_field_types():
            if f == 'stamp':
                continue
            d[f] = clean(getattr(m, f))
        emit(d)

    def read_stdin(self):
        for line in sys.stdin:
            line = line.strip()
            if line:
                try:
                    self.requests.put(json.loads(line))
                except ValueError:
                    emit({'type': 'log', 'message': f'bad request: {line!r}'})
        # laptop side closed the pipe
        rclpy.try_shutdown()

    def pump(self):
        while not self.requests.empty():
            r = self.requests.get()
            if not self.client.service_is_ready():
                emit({'type': 'reply', 'command': r.get('command'), 'success': False,
                      'message': '/toolbar_control not available'})
                continue
            req = ToolbarControl.Request()
            req.command = str(r.get('command', 'get_status'))
            req.position = float(r.get('position', 0.0))
            fut = self.client.call_async(req)
            fut.add_done_callback(lambda f, c=req.command, p=req.position: self.on_reply(c, p, f))

    def on_reply(self, command, position, fut):
        try:
            res = fut.result()
            emit({'type': 'reply', 'command': command, 'position': position,
                  'success': res.success, 'message': res.message})
        except Exception as e:  # noqa: BLE001
            emit({'type': 'reply', 'command': command, 'success': False, 'message': str(e)})


def main():
    rclpy.init()
    node = GuiBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
