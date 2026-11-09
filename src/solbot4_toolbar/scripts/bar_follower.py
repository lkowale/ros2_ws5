#!/usr/bin/env python3
"""bar_follower: keep the hoeing knife a fixed lateral distance from a red bar / cable (solbot4).

The OAK-D is fixed to the lift frame, so the bar's image position does not depend on the
toolbar. Each frame:
  1. Red mask -> per-row midpoints -> robust line fit -> bar column u_bar at row v_ref.
  2. Desired knife column  u_des = u_bar + offset_mm * ground_px_per_mm   (+ = knife right of bar)
  3. Knife column now (calibration, config/toolbar_cam.yaml):
       u_knife = u0 + px_per_m * p + last_dir * hysteresis_px / 2      p = /toolbar/state position
  4. If |u_des - u_knife| > deadband: dir = sign, target p = (u_des - u0 - dir*hysteresis_px/2) / px_per_m
     (accounts for the implement joint play when approaching from that side), else keep the target.
  5. Publish target on /tool_bar_position_cmd (toolbar_service follows it in "auto" mode).
The toolbar firmware closes the position loop; this node only computes the setpoint.

Bar lost for lost_timeout_s: stop publishing -> toolbar_service holds (command_timeout, LOW).
dry_run: true (default) computes and logs everything but publishes no command.

Topics
  sub  /oakd/rgb/image_raw/compressed, /toolbar/state
  pub  /tool_bar_position_cmd (Float32, m)   unless dry_run
       /bar_follower/status   (String JSON, every frame)
       /bar_follower/image/compressed (overlay JPEG, debug_image_hz)
"""

import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String
from toolbar_msgs.msg import ToolbarState


class BarFollower(Node):
    def __init__(self):
        super().__init__('bar_follower')
        d = self.declare_parameter
        # calibration (config/toolbar_cam.yaml)
        self.v_ref = d('v_ref', 580).value
        self.u0 = d('u0', 1071.7).value
        self.px_per_m = d('px_per_m', 2907.0).value
        self.hyst_px = d('hysteresis_px', -40.3).value
        self.ground_px_per_mm = d('ground_px_per_cm', 26.2).value / 10.0
        # follower
        d('offset_mm', 30.0)
        d('deadband_mm', 5.0)
        d('dry_run', True)
        self.limit = d('limit_m', 0.095).value
        self.lost_timeout = d('lost_timeout_s', 0.5).value
        self.smooth = d('smoothing', 0.5).value          # EMA weight of the new u_bar
        self.min_rows = d('min_rows', 80).value
        self.red_s_min = d('red_s_min', 60).value
        self.red_v_min = d('red_v_min', 40).value
        self.debug_hz = d('debug_image_hz', 2.0).value

        self.state = None
        self.u_bar = None
        self.last_seen = 0.0
        self.last_dir = 0
        self.target = None
        self.last_dbg = 0.0

        self.create_subscription(ToolbarState, '/toolbar/state', self.on_state, 10)
        self.create_subscription(CompressedImage, '/oakd/rgb/image_raw/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Float32, '/tool_bar_position_cmd', 10)
        self.pub_status = self.create_publisher(String, '/bar_follower/status', 10)
        self.pub_dbg = self.create_publisher(CompressedImage, '/bar_follower/image/compressed',
                                             qos_profile_sensor_data)
        self.get_logger().info(
            f'bar_follower: offset {self.p("offset_mm"):.0f} mm, deadband {self.p("deadband_mm"):.0f} mm, '
            f'dry_run={self.p("dry_run")}  (live: ros2 param set /bar_follower <name> <value>)')

    def p(self, name):
        return self.get_parameter(name).value

    def on_state(self, msg):
        self.state = msg

    # ---------------------------------------------------------------- detection
    def detect(self, img):
        """Return (u at v_ref, slope du/dv, mask rows used) of the red bar centreline, or None."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        m = (((h < 10) | (h > 160)) & (s > self.red_s_min) & (v > self.red_v_min)).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        n, lab, st, _ = cv2.connectedComponentsWithStats(m)
        keep = [j for j in range(1, n) if st[j, cv2.CC_STAT_AREA] > 800
                and st[j, cv2.CC_STAT_HEIGHT] > 2 * st[j, cv2.CC_STAT_WIDTH]]   # long, image-vertical
        if not keep:
            return None
        mask = np.isin(lab, keep)
        ys, xs = np.nonzero(mask)
        rows = np.unique(ys)
        if len(rows) < self.min_rows:
            return None
        # per row: min/max column (bar edges) -> midpoint
        lo = np.full(mask.shape[0], 1e9)
        hi = np.full(mask.shape[0], -1.0)
        np.minimum.at(lo, ys, xs)
        np.maximum.at(hi, ys, xs)
        lo, hi = lo[rows], hi[rows]
        w = hi - lo
        k = np.abs(w - np.median(w)) < 6        # drop rows where the bar is merged / clipped
        if k.sum() < self.min_rows:
            return None
        r, c = rows[k], (lo[k] + hi[k]) / 2
        a1, a0 = np.polyfit(r, c, 1)
        res = c - (a0 + a1 * r)
        k2 = np.abs(res) < 3 * max(res.std(), 0.5)
        a1, a0 = np.polyfit(r[k2], c[k2], 1)
        return float(a0 + a1 * self.v_ref), float(a1), int(k2.sum())

    # ---------------------------------------------------------------- control
    def on_image(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        det = self.detect(img)
        if det is not None:
            u, slope, nrows = det
            self.u_bar = u if self.u_bar is None or now - self.last_seen > self.lost_timeout \
                else self.smooth * u + (1 - self.smooth) * self.u_bar
            self.last_seen = now
        lost = det is None and now - self.last_seen > self.lost_timeout

        st = {'lost': lost, 'u_bar': None if self.u_bar is None else round(self.u_bar, 1)}
        if self.state is not None and not lost and self.u_bar is not None:
            pos = self.state.current_position
            u_des = self.u_bar + self.p('offset_mm') * self.ground_px_per_mm
            u_knife = self.u0 + self.px_per_m * pos + self.last_dir * self.hyst_px / 2
            err_px = u_des - u_knife
            if abs(err_px) > self.p('deadband_mm') * self.ground_px_per_mm or self.target is None:
                dir_ = 1 if err_px > 0 else -1
                tgt = (u_des - self.u0 - dir_ * self.hyst_px / 2) / self.px_per_m
                tgt = max(-self.limit, min(self.limit, tgt))
                if self.target is None or abs(tgt - self.target) > 1e-4:
                    self.last_dir = dir_
                self.target = tgt
            st.update(pos=round(pos, 4), u_des=round(u_des, 1), u_knife=round(u_knife, 1),
                      err_mm=round(err_px / self.ground_px_per_mm, 1), target=round(self.target, 4),
                      dir=int(self.last_dir), tb_state=self.state.state, mode=self.state.mode,
                      saturated=bool(abs(self.target) >= self.limit - 1e-6))
            if not self.p('dry_run'):
                self.pub_cmd.publish(Float32(data=float(self.target)))
        st['dry_run'] = self.p('dry_run')
        self.pub_status.publish(String(data=json.dumps(st)))

        if self.debug_hz > 0 and now - self.last_dbg >= 1.0 / self.debug_hz:
            self.last_dbg = now
            self.publish_debug(img, det, st, msg.header)

    def publish_debug(self, img, det, st, header):
        vis = img
        cv2.line(vis, (0, self.v_ref), (vis.shape[1] - 1, self.v_ref), (255, 255, 0), 1)
        if det is not None:
            u, slope, _ = det
            vb = vis.shape[0] - 1
            cv2.line(vis, (int(u - slope * self.v_ref), 0), (int(u + slope * (vb - self.v_ref)), vb),
                     (0, 255, 0), 2)
        if 'u_des' in st:
            for key, col in (('u_des', (0, 255, 255)), ('u_knife', (255, 0, 255))):
                x = int(st[key])
                cv2.line(vis, (x, self.v_ref - 40), (x, self.v_ref + 40), col, 2)
            txt = f"err {st['err_mm']:+.1f} mm  p {st['pos']:+.4f}  tgt {st['target']:+.4f}  {st['tb_state']}/{st['mode']}"
        else:
            txt = 'BAR LOST' if st['lost'] else 'no toolbar state'
        if st['dry_run']:
            txt += '  DRY RUN'
        cv2.putText(vis, txt, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        ok, jpg = cv2.imencode('.jpg', cv2.resize(vis, (640, 360)), [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            out = CompressedImage(header=header, format='jpeg', data=jpg.tobytes())
            self.pub_dbg.publish(out)


def main():
    rclpy.init()
    node = BarFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
