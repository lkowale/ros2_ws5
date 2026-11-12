#!/usr/bin/env python3
"""bar_follower: keep the hoeing knives centred on a red bar / cable (solbot4).

The OAK-D is fixed to the lift frame, so the bar's image position does not depend on the
toolbar; the knives move with the toolbar (+p = image right).

reference: knives (default)
  Same loop as markers, but the reference points are the knives themselves at ground level: the inner
  wing tip of each knife = extreme of the blue-steel mask on its side of the bar centreline (left knife
  max u+v, right knife max v-u), rows from the measuring tape down ignored. No marker parallax.

reference: markers
  Two knives, one on each side of the crop row, each carrying a yellow marker. Visual servo:
    1. Yellow blobs (compact, not on the tape) -> left / right marker, mid point (u_mid, v_mid).
       One marker hidden: the other +- the remembered marker spacing.
    2. Red bar centreline (tallest red component + pieces on its line) -> u_bar at v_mid.
    3. err = u_bar + center_offset_mm*px/mm - u_mid   (+ = bar right of the knives' middle)
    4. Only when the toolbar has settled (reached its target, still for settle_s) and
       |err| > deadband: target = p + err / gain, gain = d(u)/dp at p from the calibration
       (px_per_m + 2*px_per_m2*p). The gain only sets how fast it converges; the end point is
       measured on the image, so joint play and scale errors are corrected by the next step.
  The firmware closes the position loop; this node only computes the setpoint.

reference: model (the 2026-11-07 single-knife mode)
  u_knife = u0 + px_per_m*p + last_dir*hysteresis_px/2 (config/toolbar_cam.yaml); keeps the knife's
  left edge offset_mm right of the bar, open loop.

Bar (or both markers) lost for lost_timeout_s: stop publishing -> toolbar_service holds.
dry_run: true (default) computes and logs everything but publishes no command.

Topics
  sub  /oakd/rgb/image_raw/compressed, /toolbar/state
  pub  /tool_bar_position_cmd (Float32, m)   unless dry_run
       /bar_follower/status   (String JSON, every frame)
       /bar_follower/image/compressed (overlay JPEG, debug_image_hz)
"""

import json

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
        d('reference', 'knives')                         # knives | markers | model
        # calibration (config/toolbar_cam.yaml)
        self.v_ref = d('v_ref', 580).value
        self.u0 = d('u0', 1011.7).value
        self.px_per_m = d('px_per_m', 2907.0).value
        self.px_per_m2 = d('px_per_m2', 0.0).value      # quadratic term, markers-mode gain only
        self.hyst_px = d('hysteresis_px', -40.3).value
        self.ground_px_per_mm = d('ground_px_per_cm', 26.2).value / 10.0
        # follower
        d('offset_mm', 30.0)                             # model mode
        d('center_offset_mm', 0.0)                       # markers mode: bar this far right of the middle
        d('deadband_mm', 5.0)
        d('dry_run', True)
        self.limit = d('limit_m', 0.095).value
        self.lost_timeout = d('lost_timeout_s', 0.5).value
        self.smooth = d('smoothing', 0.5).value          # EMA weight of the new u_bar
        self.min_rows = d('min_rows', 80).value
        self.red_s_min = d('red_s_min', 60).value
        self.red_v_min = d('red_v_min', 75).value       # rusty frame V 43-62, painted bar V 88-140
        # markers mode
        d('servo', 'continuous')                         # continuous | stepped
        d('servo_gain', 0.8)                             # continuous: fraction of the error per update
        d('release_frac', 0.5)                           # continuous: stop correcting below deadband*this
        self.latency = d('image_latency_s', 0.10).value  # used when the image stamp is missing
        self.retarget = d('retarget_m', 0.0005).value    # continuous: min target change to resend
        self.settle_s = d('settle_s', 0.6).value         # still this long before measuring
        self.settle_frames = d('settle_frames', 3).value  # median of this many errors per decision
        self.arrive_tol = d('arrive_tol_m', 0.004).value
        self.arrive_timeout = d('arrive_timeout_s', 4.0).value
        self.yel_h = d('marker_hue', [18, 45]).value
        self.yel_s_min = d('marker_s_min', 50).value
        self.yel_v_min = d('marker_v_min', 130).value
        self.mk_area = d('marker_area', [400, 4000]).value
        # knives mode: blue-steel mask, inner wing tip of each knife on its side of the bar
        self.kn_h = d('knife_hue', [90, 125]).value
        self.kn_s_min = d('knife_s_min', 45).value
        self.kn_v_min = d('knife_v_min', 30).value
        self.kn_margin = d('knife_margin_px', 25).value  # ignore this close to the bar centreline
        self.kn_tip_n = d('knife_tip_px', 30).value      # tip = mean of the N most extreme pixels
        self.kn_min_px = d('knife_min_px', 300).value    # blue pixels needed on each side
        self.kn_min_blob = d('knife_min_blob_px', 1500).value  # smaller blue blobs (dirt) ignored
        self.debug_hz = d('debug_image_hz', 2.0).value

        self.state = None
        self.u_bar = None              # smoothed bar column at v_ref
        self.slope = 0.0               # du/dv of the bar
        self.last_seen = 0.0
        self.last_dir = 0
        self.target = None
        self.last_dbg = 0.0
        # markers mode
        self.spacing = None            # u_right - u_left, px
        self.mk_last = None            # (uL, vL, uR, vR, p) last good pair
        self.mk_seen = 0.0
        self.moved_t = 0.0             # last time the toolbar moved
        self.prev_pos = None
        self.issued_t = 0.0            # when the current target was issued
        self.errs = []
        self.pos_hist = []             # (t, position) from /toolbar/state, last 2 s
        self.img_t = None              # capture time of the frame being processed
        self.servo_active = False      # continuous: correcting (between deadband and release)
        self.tip_hist = []             # knives: last 3 raw tip detections

        self.create_subscription(ToolbarState, '/toolbar/state', self.on_state, 10)
        self.create_subscription(CompressedImage, '/oakd/rgb/image_raw/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Float32, '/tool_bar_position_cmd', 10)
        self.pub_status = self.create_publisher(String, '/bar_follower/status', 10)
        self.pub_dbg = self.create_publisher(CompressedImage, '/bar_follower/image/compressed',
                                             qos_profile_sensor_data)
        self.get_logger().info(
            f'bar_follower: reference {self.p("reference")}, deadband {self.p("deadband_mm"):.0f} mm, '
            f'dry_run={self.p("dry_run")}  (live: ros2 param set /bar_follower <name> <value>)')

    def p(self, name):
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pos_at(self, t):
        """Toolbar position at time t (linear interpolation in the recent state history)."""
        h = self.pos_hist
        if not h:
            return self.state.current_position
        if t <= h[0][0]:
            return h[0][1]
        for (t0, p0), (t1, p1) in zip(h, h[1:]):
            if t0 <= t <= t1:
                return p0 + (p1 - p0) * (t - t0) / max(t1 - t0, 1e-6)
        return h[-1][1]

    def on_state(self, msg):
        self.state = msg
        now = self.now()
        self.pos_hist.append((now, msg.current_position))
        self.pos_hist = [x for x in self.pos_hist if now - x[0] < 2.0]
        pos = msg.current_position
        if msg.in_motion or self.prev_pos is None or abs(pos - self.prev_pos) > 0.0007:
            self.moved_t = self.now()
            self.prev_pos = pos

    # ---------------------------------------------------------------- detection
    def detect(self, img, cols=None):
        """Red bar centreline: (u at v_ref, slope du/dv, rows used), or None.

        Tallest image-vertical red component, plus other red pieces lying on its line (the bar
        is split by nuts / joints); rusty frame parts off the line are ignored.
        cols = (u0, u1): search only these image columns."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        m = (((h < 10) | (h > 160)) & (s > self.red_s_min) & (v > self.red_v_min)).astype(np.uint8)
        if cols is not None:
            m[:, :max(0, int(cols[0]))] = 0
            m[:, max(0, int(cols[1])):] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        cand = [j for j in range(1, n) if st[j, cv2.CC_STAT_AREA] > 800
                and st[j, cv2.CC_STAT_HEIGHT] > 2 * st[j, cv2.CC_STAT_WIDTH]]   # long, image-vertical
        if not cand:
            return None
        main = max(cand, key=lambda j: st[j, cv2.CC_STAT_HEIGHT])
        line = self._fit(lab == main)
        if line is None:
            return None
        a0, a1 = line
        keep = [main] + [j for j in range(1, n) if j != main and st[j, cv2.CC_STAT_AREA] > 300
                         and abs(cen[j][0] - (a0 + a1 * cen[j][1])) < 25]
        if len(keep) > 1:
            line = self._fit(np.isin(lab, keep)) or line
            a0, a1 = line
        return float(a0 + a1 * self.v_ref), float(a1), 0

    def _fit(self, mask):
        """Per-row edge midpoints -> robust line u = a0 + a1*v, or None."""
        ys, xs = np.nonzero(mask)
        rows = np.unique(ys)
        if len(rows) < self.min_rows:
            return None
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
        return float(a0), float(a1)

    def detect_markers(self, img):
        """Yellow knife markers: list of (u, v, area), compact blobs not on the measuring tape."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h = hsv[..., 0]
        m = ((h >= self.yel_h[0]) & (h <= self.yel_h[1]) & (hsv[..., 1] > self.yel_s_min)
             & (hsv[..., 2] > self.yel_v_min)).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        # tape: long horizontal yellow strips; reject anything on those rows
        tape = [(st[j, 1] - 15, st[j, 1] + st[j, 3] + 15) for j in range(1, n)
                if st[j, cv2.CC_STAT_WIDTH] > 150 and st[j, 2] > 3 * st[j, 3]]
        out = []
        for j in range(1, n):
            x, y, w, hh, a = st[j]
            if not self.mk_area[0] <= a <= self.mk_area[1]:
                continue
            if max(w, hh) > 1.8 * min(w, hh) or a < 0.55 * w * hh:
                continue
            if any(t0 <= y + hh and y <= t1 for t0, t1 in tape):
                continue
            out.append((float(cen[j][0]), float(cen[j][1]), int(a)))
        return out

    @staticmethod
    def tape_top(hsv):
        """Top row of the measuring tape (long horizontal yellow strip), or None."""
        h = hsv[..., 0]
        y = ((h >= 18) & (h <= 45) & (hsv[..., 1] > 50) & (hsv[..., 2] > 130)).astype(np.uint8)
        y = cv2.morphologyEx(y, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n, _, st, _ = cv2.connectedComponentsWithStats(y)
        t = [st[j, 1] for j in range(1, n) if st[j, 2] > 150 and st[j, 2] > 3 * st[j, 3]]
        return min(t) if t else None

    def detect_knife_tips(self, img):
        """Inner wing tips of the two knives, at ground level: (uL, vL, uR, vR, 2) or None.

        Blue-steel mask, split by the bar centreline: left knife tip = bottom-right extreme of the
        blue left of the bar (max u+v), right knife tip = bottom-left extreme right of it (max v-u).
        Rows from the measuring tape down are ignored (tape and the shank bolt look bluish)."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h = hsv[..., 0]
        m = ((h >= self.kn_h[0]) & (h <= self.kn_h[1]) & (hsv[..., 1] > self.kn_s_min)
             & (hsv[..., 2] > self.kn_v_min)).astype(np.uint8)
        tt = self.tape_top(hsv)
        if tt is not None:
            m[max(0, tt - 8):] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        # knife = big blobs only: a bluish dirt spot next to the left wing tip was taken as the tip
        # now and then (2026-11-11), error jumping by ~6 mm and the loop oscillating
        n, lab, st, _ = cv2.connectedComponentsWithStats(m)
        big = [j for j in range(1, n) if st[j, cv2.CC_STAT_AREA] >= self.kn_min_blob]
        ys, xs = np.nonzero(np.isin(lab, big))
        ub = self.u_bar + self.slope * (ys - self.v_ref)
        L = xs < ub - self.kn_margin
        R = xs > ub + self.kn_margin
        if L.sum() < self.kn_min_px or R.sum() < self.kn_min_px:
            return None
        n = self.kn_tip_n
        xl, yl = xs[L], ys[L]
        i = np.argsort(xl + yl)[-n:]
        xr, yr = xs[R], ys[R]
        j = np.argsort(yr - xr)[-n:]
        tip = (float(xl[i].mean()), float(yl[i].mean()), float(xr[j].mean()), float(yr[j].mean()))
        # median of the last 3 frames per coordinate: one bad frame can't move the toolbar
        self.tip_hist = (self.tip_hist + [tip])[-3:]
        med = np.median(np.array(self.tip_hist), axis=0)
        return float(med[0]), float(med[1]), float(med[2]), float(med[3]), 2

    def pair_markers(self, mk, pos):
        """(uL, vL, uR, vR, n_seen) or None, using the last pair to resolve one-marker views."""
        if len(mk) >= 2:
            if self.mk_last is not None and len(mk) > 2:
                # keep the two nearest to where the last pair should be now
                du = self.gain(pos) * (pos - self.mk_last[4])
                pl = (self.mk_last[0] + du, self.mk_last[1])
                pr = (self.mk_last[2] + du, self.mk_last[3])
                a = min(mk, key=lambda m: (m[0] - pl[0]) ** 2 + (m[1] - pl[1]) ** 2)
                b = min((m for m in mk if m is not a),
                        key=lambda m: (m[0] - pr[0]) ** 2 + (m[1] - pr[1]) ** 2)
                mk = [a, b]
            else:
                mk = sorted(mk, key=lambda m: -m[2])[:2]
            left, right = sorted(mk, key=lambda m: m[0])
            return left[0], left[1], right[0], right[1], 2
        if len(mk) == 1 and self.mk_last is not None and self.spacing is not None:
            u, v, _ = mk[0]
            du = self.gain(pos) * (pos - self.mk_last[4])
            if abs(u - (self.mk_last[0] + du)) < abs(u - (self.mk_last[2] + du)):
                return u, v, u + self.spacing, self.mk_last[3], 1
            return u - self.spacing, self.mk_last[1], u, v, 1
        return None

    def gain(self, pos):
        """Knife image motion per toolbar metre at pos (px/m)."""
        return max(500.0, self.px_per_m + 2 * self.px_per_m2 * pos)

    # ---------------------------------------------------------------- control
    def on_image(self, msg):
        now = self.now()
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.img_t = stamp if 0 < now - stamp < 1.0 else now - self.latency
        cols = None
        if self.p('reference') in ('markers', 'knives') and self.mk_last is not None and self.spacing is not None \
                and now - self.mk_seen < 2.0:
            cols = (self.mk_last[0] - self.spacing, self.mk_last[2] + self.spacing)   # around the knives
        det = self.detect(img, cols)
        if det is not None:
            u, slope, _ = det
            self.u_bar = u if self.u_bar is None or now - self.last_seen > self.lost_timeout \
                else self.smooth * u + (1 - self.smooth) * self.u_bar
            self.slope = slope
            self.last_seen = now
        lost = det is None and now - self.last_seen > self.lost_timeout

        st = {'lost': lost, 'u_bar': None if self.u_bar is None else round(self.u_bar, 1)}
        extra = {}
        if self.state is not None and not lost and self.u_bar is not None:
            if self.p('reference') in ('markers', 'knives'):
                extra = self.step_markers(img, now, st)
            else:
                self.step_model(st)
        st['dry_run'] = self.p('dry_run')
        self.pub_status.publish(String(data=json.dumps(st)))

        if self.debug_hz > 0 and now - self.last_dbg >= 1.0 / self.debug_hz:
            self.last_dbg = now
            self.publish_debug(img, det, st, extra, msg.header)

    def publish(self):
        if self.target is not None and not self.p('dry_run'):
            self.pub_cmd.publish(Float32(data=float(self.target)))

    def clamp(self, t):
        lo, hi = -self.limit, self.limit
        if self.state is not None:
            # 5 mm inside: the firmware clamps goals to TPS 370..1280 (~4.2 mm inside the ends), and a
            # target beyond that is never reached (stays "moving" until arrive_timeout)
            lo = max(lo, self.state.min_position + 0.005)
            hi = min(hi, self.state.max_position - 0.005)
        return max(lo, min(hi, t))

    def step_markers(self, img, now, st):
        pos = self.state.current_position
        if self.p('reference') == 'knives':
            mk = []
            pair = self.detect_knife_tips(img)
        else:
            mk = self.detect_markers(img)
            pair = self.pair_markers(mk, pos)
        if pair is not None:
            uL, vL, uR, vR, nseen = pair
            self.mk_seen = now
            if nseen == 2:
                sp = uR - uL
                self.spacing = sp if self.spacing is None else 0.8 * self.spacing + 0.2 * sp
            self.mk_last = (uL, vL, uR, vR, pos)
        if self.target is None:
            self.target = pos                      # start by holding where we are
            self.issued_t = now
        st.update(pos=round(pos, 4), target=round(self.target, 4), markers=len(mk),
                  tb_state=self.state.state, mode=self.state.mode)
        if pair is None:
            st['markers_lost'] = True
            if now - self.mk_seen > self.lost_timeout:
                return {'mk': mk}                  # stop commanding -> service holds
            self.publish()
            return {'mk': mk}
        v_mid = (vL + vR) / 2
        u_mid = (uL + uR) / 2
        u_bar = self.u_bar + self.slope * (v_mid - self.v_ref)
        err_px = u_bar + self.p('center_offset_mm') * self.ground_px_per_mm - u_mid
        err_mm = err_px / self.ground_px_per_mm
        arrived = abs(pos - self.target) < self.arrive_tol or now - self.issued_t > self.arrive_timeout
        settled = arrived and not self.state.in_motion and now - self.moved_t > self.settle_s
        decided = False
        if self.p('servo') == 'continuous':
            # every frame: the markers show the knives where the toolbar was when the image was
            # taken, so correct from that position, not from the current one (camera latency)
            # hysteresis: start correcting above deadband_mm, keep streaming targets while correcting,
            # stop updating (let the toolbar finish the last target) below deadband_mm * release_frac.
            # Without it, 0.5 mm detection noise kept re-targeting a stationary bar (hunting).
            p_img = self.pos_at(self.img_t)
            db = self.p('deadband_mm')
            if not self.servo_active and abs(err_mm) > db:
                self.servo_active = True
            elif self.servo_active and (abs(err_mm) < db * self.p('release_frac') or (
                    abs(err_mm) <= db and not self.state.in_motion and now - self.moved_t > 0.3)):
                # also release when at rest inside the deadband: near the target small actuator moves
                # are absorbed by the implement joint play, the error never gets below the release
                # level and the loop would dither back and forth through the play
                self.servo_active = False
            if self.servo_active:
                tgt = self.clamp(p_img + self.p('servo_gain') * err_px / self.gain(p_img))
                if abs(tgt - self.target) > self.retarget:
                    self.last_dir = 1 if tgt > self.target else -1
                    self.target = tgt
                    self.issued_t = now
                    decided = True
            self.errs = []
        elif settled:
            self.errs.append(err_px)
        else:
            self.errs = []
        if len(self.errs) >= self.settle_frames:
            e = float(np.median(self.errs))
            self.errs = []
            if abs(e) > self.p('deadband_mm') * self.ground_px_per_mm:
                tgt = self.clamp(pos + e / self.gain(pos))
                if abs(tgt - self.target) > 5e-4:
                    self.last_dir = 1 if tgt > self.target else -1
                    self.target = tgt
                    self.issued_t = now
                    decided = True
        self.publish()
        st.update(target=round(self.target, 4), err_mm=round(err_mm, 1), u_mid=round(u_mid, 1),
                  u_bar_mid=round(u_bar, 1), v_mid=round(v_mid), seen=nseen,
                  spacing=None if self.spacing is None else round(self.spacing, 1),
                  settled=settled, new_target=decided,
                  saturated=bool(abs(self.clamp(self.target + np.sign(err_px) * 1e-3) - self.target) < 1e-6
                                 and abs(err_mm) > self.p('deadband_mm')))
        return {'mk': mk, 'pair': pair, 'u_bar': u_bar, 'v_mid': v_mid}

    def step_model(self, st):
        pos = self.state.current_position
        u_des = self.u_bar + self.p('offset_mm') * self.ground_px_per_mm
        u_knife = self.u0 + self.px_per_m * pos + self.last_dir * self.hyst_px / 2
        err_px = u_des - u_knife
        if abs(err_px) > self.p('deadband_mm') * self.ground_px_per_mm or self.target is None:
            dir_ = 1 if err_px > 0 else -1
            tgt = self.clamp((u_des - self.u0 - dir_ * self.hyst_px / 2) / self.px_per_m)
            if self.target is None or abs(tgt - self.target) > 1e-4:
                self.last_dir = dir_
            self.target = tgt
        st.update(pos=round(pos, 4), u_des=round(u_des, 1), u_knife=round(u_knife, 1),
                  err_mm=round(err_px / self.ground_px_per_mm, 1), target=round(self.target, 4),
                  dir=int(self.last_dir), tb_state=self.state.state, mode=self.state.mode,
                  saturated=bool(abs(self.target) >= self.limit - 1e-6))
        self.publish()

    def publish_debug(self, img, det, st, extra, header):
        vis = img
        if det is not None:
            u, slope, _ = det
            vb = vis.shape[0] - 1
            cv2.line(vis, (int(u - slope * self.v_ref), 0), (int(u + slope * (vb - self.v_ref)), vb),
                     (0, 255, 0), 2)
        for (mu, mv, _) in extra.get('mk', []):
            cv2.circle(vis, (int(mu), int(mv)), 22, (255, 0, 255), 2)
        if 'pair' in extra:
            uL, vL, uR, vR, nseen = extra['pair']
            cv2.line(vis, (int(uL), int(vL)), (int(uR), int(vR)), (255, 0, 255), 2)
            um, vm = int((uL + uR) / 2), int(extra['v_mid'])
            cv2.line(vis, (um, vm - 50), (um, vm + 50), (255, 0, 255), 3)
            cv2.line(vis, (int(extra['u_bar']), vm), (um, vm), (0, 255, 255), 3)
        if 'u_des' in st:
            cv2.line(vis, (0, self.v_ref), (vis.shape[1] - 1, self.v_ref), (255, 255, 0), 1)
            for key, col in (('u_des', (0, 255, 255)), ('u_knife', (255, 0, 255))):
                x = int(st[key])
                cv2.line(vis, (x, self.v_ref - 40), (x, self.v_ref + 40), col, 2)
        if 'err_mm' in st:
            txt = f"err {st['err_mm']:+.1f} mm  p {st['pos']:+.4f}  tgt {st['target']:+.4f}  {st['tb_state']}/{st['mode']}"
        elif st.get('markers_lost'):
            txt = 'MARKERS LOST'
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
