#!/usr/bin/env python3
"""Sweep the toolbar and record TPS (raw_position) vs the two knives' inner ground tips in the OAK-D image.

No bar needed: the two knives are split at the widest empty column gap between the blue-steel blobs.
Writes OUT/sweep.csv and OUT/stepNN.jpg.  python3 knife_tps_sweep.py OUT
"""
import sys, os, time, json
import cv2, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from toolbar_msgs.msg import ToolbarState
from toolbar_msgs.srv import ToolbarControl

H, S_MIN, V_MIN, MIN_BLOB, TIP_N = (90, 125), 45, 30, 1500, 30


def tape_top(hsv):
    h = hsv[..., 0]
    y = ((h >= 18) & (h <= 45) & (hsv[..., 1] > 50) & (hsv[..., 2] > 130)).astype(np.uint8)
    y = cv2.morphologyEx(y, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, _, st, _ = cv2.connectedComponentsWithStats(y)
    t = [st[j, 1] for j in range(1, n) if st[j, 2] > 150 and st[j, 2] > 3 * st[j, 3]]
    return min(t) if t else None


def tips(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = hsv[..., 0]
    m = ((h >= H[0]) & (h <= H[1]) & (hsv[..., 1] > S_MIN) & (hsv[..., 2] > V_MIN)).astype(np.uint8)
    tt = tape_top(hsv)
    if tt is not None:
        m[max(0, tt - 8):] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    big = [j for j in range(1, n) if st[j, cv2.CC_STAT_AREA] >= MIN_BLOB]
    keep = np.isin(lab, big)
    ys, xs = np.nonzero(keep)
    if len(xs) < 2000:
        return None
    col = keep.sum(axis=0) > 3
    lo, hi = np.percentile(xs, 15), np.percentile(xs, 85)
    # widest empty run of columns between the knives
    best, run, start = (0, 0), 0, 0
    for u in range(int(lo), int(hi)):
        if not col[u]:
            if run == 0:
                start = u
            run += 1
            if run > best[0]:
                best = (run, start)
        else:
            run = 0
    if best[0] < 15:
        return None
    split = best[1] + best[0] / 2
    L, R = xs < split, xs > split
    if L.sum() < 300 or R.sum() < 300:
        return None
    xl, yl, xr, yr = xs[L], ys[L], xs[R], ys[R]
    i = np.argsort(xl + yl)[-TIP_N:]
    j = np.argsort(yr - xr)[-TIP_N:]
    return float(xl[i].mean()), float(yl[i].mean()), float(xr[j].mean()), float(yr[j].mean()), tt


class Sweep:
    def __init__(self, node, out):
        self.n, self.out = node, out
        self.state, self.frames = None, []
        node.create_subscription(ToolbarState, '/toolbar/state', lambda m: setattr(self, 'state', m), 10)
        node.create_subscription(CompressedImage, '/oakd/rgb/image_raw/compressed',
                                 lambda m: self.frames.append((time.time(), bytes(m.data))),
                                 qos_profile_sensor_data)
        self.cli = node.create_client(ToolbarControl, '/toolbar_control')

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)
        self.frames = self.frames[-30:]

    def call(self, cmd, pos=0.0):
        self.cli.wait_for_service(timeout_sec=5)
        f = self.cli.call_async(ToolbarControl.Request(command=cmd, position=float(pos)))
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=5)
        r = f.result()
        print(f'  {cmd} {pos:+.4f}: {getattr(r, "success", None)} {getattr(r, "message", "")}', flush=True)

    def settle(self, timeout=12.0):
        t0, hist = time.time(), []
        while time.time() - t0 < timeout:
            self.spin(0.1)
            if self.state is None:
                continue
            hist = (hist + [(time.time(), self.state.raw_position)])[-15:]
            if (not self.state.in_motion and len(hist) == 15 and hist[-1][0] - hist[0][0] > 1.2
                    and max(h[1] for h in hist) - min(h[1] for h in hist) <= 1):
                return True
        return False

    def measure(self, k, tgt, d):
        t_settled = time.time()
        self.spin(1.0)                                   # frames taken after the settle
        res, imgs = [], []
        for t, b in self.frames:
            if t < t_settled:
                continue
            img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
            r = tips(img) if img is not None else None
            if r:
                res.append(r[:4]); imgs.append(img)
        st = self.state
        if not res:
            print(f'{k:2d} tgt {tgt*1000:+6.1f} TPS {st.raw_position:.0f}  knives NOT FOUND', flush=True)
            row = [k, d, tgt, st.raw_position, st.current_position] + [''] * 6 + [0]
        else:
            a = np.median(np.array(res), axis=0)
            mid = (a[0] + a[2]) / 2
            img = imgs[-1].copy()
            for u, v in ((a[0], a[1]), (a[2], a[3])):
                cv2.circle(img, (int(u), int(v)), 8, (0, 255, 255), 2)
            cv2.line(img, (int(mid), 0), (int(mid), 719), (0, 255, 255), 1)
            cv2.putText(img, f'{k}: TPS {st.raw_position:.0f} p {st.current_position*1000:+.1f}mm mid {mid:.1f}px',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.imwrite(os.path.join(self.out, f'step{k:02d}.jpg'), img)
            row = [k, d, tgt, st.raw_position, st.current_position, *[round(x, 1) for x in a],
                   round(mid, 1), round(a[2] - a[0], 1), len(res)]
            print(f'{k:2d} dir {d:+d} tgt {tgt*1000:+6.1f} TPS {st.raw_position:5.0f} p {st.current_position*1000:+6.1f} '
                  f'uL {a[0]:6.1f} uR {a[2]:6.1f} mid {mid:6.1f} gap {a[2]-a[0]:5.1f} [{len(res)}]', flush=True)
        self.csv.write(','.join(str(x) for x in row) + '\n'); self.csv.flush()

    def run(self, targets):
        self.csv = open(os.path.join(self.out, 'sweep.csv'), 'w')
        self.csv.write('step,dir,target_m,tps,pos_m,uL,vL,uR,vR,u_mid,gap_px,n\n')
        self.spin(2.0)
        self.call('enable')
        prev = self.state.current_position if self.state else 0.0
        for k, t in enumerate(targets):
            d = 1 if t > prev else -1
            prev = t
            self.call('set_position', t)
            ok = self.settle()
            st = self.state
            if not ok or st.fault_latched or st.state in ('fault', 'estop', 'no_sensor') \
                    or abs(st.current_position - t) > 0.004:
                print(f'ABORT step {k}: settled={ok} state={st.state} fault={st.fault_reason}/{st.firmware_fault} '
                      f'pos {st.current_position*1000:+.1f} mm vs target {t*1000:+.1f}', flush=True)
                self.call('hold')
                break
            self.measure(k, t, d)
        self.csv.close()


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    lo, hi, step = -0.075, 0.080, 0.010
    up = list(np.round(np.arange(lo, hi - 1e-9, step), 4)) + [hi]
    targets = [lo] + up[1:] + up[::-1][1:] + [0.0]
    rclpy.init()
    n = rclpy.create_node('knife_tps_sweep')
    Sweep(n, out).run(targets)
    print('done', flush=True)


if __name__ == '__main__':
    main()
