#!/usr/bin/env python3
"""Bar-follower accuracy check: measure knife-to-red-bar distance from the OAK-D image after each settle.

Runs on pd600 (robot graph) next to start_bar. A step is measured when the red bar and the toolbar
have both been still for `--settle` s after something moved. Measurement, independent of the
follower's own prediction: knife = bottom-left corner of the black blade (docs/toolbar_cam_calib/
corner.py), bar = red centreline fit (bar_follower.detect) evaluated at the knife's row.
distance_mm = (u_knife - u_bar) / ground px/mm, + = knife right of the bar (follower goal: offset_mm).

  python3 measure_knife_bar.py [--out DIR] [--settle 2.0]
Writes DIR/steps.csv and DIR/stepNN.jpg (annotated).
"""
import argparse, json, os, time
import cv2, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from toolbar_msgs.msg import ToolbarState

PX_PER_MM = 2.62          # toolbar_cam.yaml ground_px_per_cm / 10
TAPE_ROW = 635            # ignore the tape and below when looking for the blade


def red_line(img):
    """(a0, a1, half_width): bar centre column u = a0 + a1*v, or None."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    m = (((h < 10) | (h > 160)) & (s > 60) & (v > 40)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    keep = [j for j in range(1, n) if st[j, cv2.CC_STAT_AREA] > 800
            and st[j, cv2.CC_STAT_HEIGHT] > 2 * st[j, cv2.CC_STAT_WIDTH]]
    if not keep:
        return None
    ys, xs = np.nonzero(np.isin(lab, keep))
    rows = np.unique(ys)
    if len(rows) < 80:
        return None
    lo = np.full(img.shape[0], 1e9); hi = np.full(img.shape[0], -1.0)
    np.minimum.at(lo, ys, xs); np.maximum.at(hi, ys, xs)
    lo, hi = lo[rows], hi[rows]
    w = hi - lo
    k = np.abs(w - np.median(w)) < 6
    if k.sum() < 80:
        return None
    a1, a0 = np.polyfit(rows[k], (lo[k] + hi[k]) / 2, 1)
    return float(a0), float(a1), float(np.median(w[k]) / 2)


def knife_corner(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # blue steel (H~103, S 45-170). The 11-07 "V<95" dark mask misses the lit left wing under
    # the shop light; floor is S 15-35, shadow H~20.
    h = hsv[..., 0]
    m = ((h >= 90) & (h <= 125) & (hsv[..., 1] > 45) & (hsv[..., 2] > 30)).astype(np.uint8)
    m[TAPE_ROW:] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    j = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[j, cv2.CC_STAT_AREA] < 3000:
        return None
    ys, xs = np.nonzero(lab == j)
    c = int(np.argmax(ys - xs))                     # bottom-left extreme
    if xs[c] >= img.shape[1] - 3:
        return None                                 # clipped at the right edge
    return int(xs[c]), int(ys[c])


class Meas:
    def __init__(self, node, out, settle):
        self.n, self.out, self.settle = node, out, settle
        self.state = None; self.status = {}; self.frames = []
        self.hist = []                                  # (t, pos, u_bar)
        self.last_meas = None                           # (pos, u_bar)
        self.step = 0
        node.create_subscription(ToolbarState, '/toolbar/state', lambda m: setattr(self, 'state', m), 10)
        node.create_subscription(String, '/bar_follower/status', self.on_status, 10)
        node.create_subscription(CompressedImage, '/oakd/rgb/image_raw/compressed', self.on_img,
                                 qos_profile_sensor_data)
        node.create_timer(0.2, self.tick)
        self.csv = open(os.path.join(out, 'steps.csv'), 'a')
        if self.csv.tell() == 0:
            self.csv.write('step,time,toolbar_m,u_bar,u_knife,v_knife,dist_mm,dist_edge_mm,'
                           'follower_err_mm,offset_mm,n_ok\n')

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def on_img(self, m):
        self.frames.append(bytes(m.data))
        self.frames = self.frames[-3:]

    def tick(self):
        if self.state is None or self.status.get('u_bar') is None or self.status.get('lost'):
            return
        t = time.time()
        self.hist.append((t, self.state.current_position, self.status['u_bar']))
        self.hist = [h for h in self.hist if t - h[0] <= self.settle]
        if t - self.hist[0][0] < self.settle - 0.3:
            return
        ps = [h[1] for h in self.hist]; us = [h[2] for h in self.hist]
        still = max(ps) - min(ps) < 0.0015 and max(us) - min(us) < 4 and not self.state.in_motion
        if not still:
            return
        pos, ub = float(np.mean(ps)), float(np.mean(us))
        if self.last_meas is not None and abs(pos - self.last_meas[0]) < 0.002 \
                and abs(ub - self.last_meas[1]) < 6:
            return                                       # nothing moved since the last step
        self.measure(pos)
        self.last_meas = (pos, ub)

    def measure(self, pos):
        res = []
        for f in self.frames:
            img = cv2.imdecode(np.frombuffer(f, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            k, line = knife_corner(img), red_line(img)
            if k and line:
                ub = line[0] + line[1] * k[1]
                res.append((k[0], k[1], ub, line[2], img))
        self.step += 1
        ts = time.strftime('%H:%M:%S')
        ferr = self.status.get('err_mm')
        if not res:
            print(f'step {self.step:2d} {ts}  toolbar {pos*1000:+6.1f} mm  knife or bar NOT FOUND '
                  f'(knife leaves the image right of ~+30 mm)', flush=True)
            return
        uk = float(np.mean([r[0] for r in res])); vk = float(np.mean([r[1] for r in res]))
        ub = float(np.mean([r[2] for r in res])); hw = float(np.mean([r[3] for r in res]))
        d = (uk - ub) / PX_PER_MM
        de = (uk - (ub + hw)) / PX_PER_MM
        img = res[-1][4].copy()
        cv2.circle(img, (int(uk), int(vk)), 8, (0, 255, 255), 2)
        cv2.line(img, (int(ub), int(vk)), (int(uk), int(vk)), (0, 255, 255), 2)
        cv2.putText(img, f'step {self.step}: knife-bar {d:+.1f} mm (edge {de:+.1f})  toolbar {pos*1000:+.1f} mm',
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imwrite(os.path.join(self.out, f'step{self.step:02d}.jpg'), img)
        self.csv.write(f'{self.step},{ts},{pos:.4f},{ub:.1f},{uk:.1f},{vk:.0f},{d:.1f},{de:.1f},'
                       f'{ferr},{30},{len(res)}\n'); self.csv.flush()
        print(f'step {self.step:2d} {ts}  toolbar {pos*1000:+6.1f} mm  knife-bar centre {d:+6.1f} mm  '
              f'(bar edge {de:+6.1f})  follower err {ferr} mm  [{len(res)}/3 frames]', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/knife_bar_meas'))
    ap.add_argument('--settle', type=float, default=2.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('measure_knife_bar')
    Meas(node, a.out, a.settle)
    print(f'measuring after each settle -> {a.out}', flush=True)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
