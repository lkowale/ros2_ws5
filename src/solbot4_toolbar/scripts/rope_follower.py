#!/usr/bin/env python3
"""rope_follower: keep the two knives centred on a rope / cable lying along the crop row (solbot4).

Model-based, the knives need not be visible (in the field they are in the soil):
  1. Rope, per horizontal band of rows (rope_detect):
     color (default, outdoors): tan rope on grey pavers / soil: Lab b* rope_b (yellowish), S < rope_s_max
     (leaves are far more saturated), V > rope_v_min; the column of maximum mask fill (box-smoothed to the
     rope width), accepted when the fill >= rope_min_fill. 2026-11-12 courtyard: fill 0.9-1.0, pavers 0.
     texture (shop floor, where the rope has the floor's colour): the column of maximum texture (|Sobel| summed down the band, box-smoothed to the rope width)
     with knife-blue and tape-yellow pixels masked out; robust line fit through the band peaks ->
     u_rope at v_ref (the knife-tip row). Tracking window around the last rope position.
  2. Knives: knife_source 'auto' (default): measured from the pink markers on the knives' inner tips when both
     are found (knife_markers), else the model u_knife = knife_tps_cal.knife_u(TPS, last_dir). With markers
     the target is T_img + err / (du/dT), T_img = TPS when the frame was taken (TPS history), so the joint
     play and the knives' swing while driving are measured, not modelled (2026-11-14 0.4 m/s lap: the knives
     sat ~35 px / 12 mm AHEAD in the direction of travel, the static model assumes 7 px behind -> +-12 mm).
  3. err = u_rope + center_offset_px - u_knife. Hysteresis: start correcting above deadband_px, while
     correcting target = tps_for(u_rope + offset, sign(err)) every frame (includes the joint play on a
     reversal), stop below deadband_px * release_frac or at rest inside the deadband.
  4. TPS -> metres with the toolbar.yaml linear map, published on /tool_bar_position_cmd.

Rope lost for lost_timeout_s: stop publishing -> toolbar_service holds. dry_run: true by default.
require_lift_down (default true): command only while /lift/state reports position 'down' and not in_motion
(field nav lowers the lift at each swath start); otherwise no commands -> toolbar_service holds.

Topics
  sub  /oakd/rgb/image_raw/compressed, /toolbar/state
  pub  /tool_bar_position_cmd (Float32, m)   unless dry_run
       /rope_follower/status  (String JSON, every frame)
       /rope_follower/image/compressed (overlay JPEG, debug_image_hz)
"""

import json
import warnings

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String
from toolbar_msgs.msg import ToolbarState
try:
    from solbot4_msgs.msg import LiftStateMsg       # ros2_ws4 (pd600 underlay)
except ImportError:                                 # laptop without ws4: no lift gating available
    LiftStateMsg = None

import knife_tps_cal as cal


def rope_bands(img, u_lo, u_hi, v_top, v_bot, n_bands, width_px, kn_mask_cfg, min_ratio):
    """[(v, u_peak, ratio)] per row band where the texture peak stands out of the window."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    (h0, h1), s_min, v_min = kn_mask_cfg
    blue = (h >= h0) & (h <= h1) & (s > s_min) & (v > v_min)
    tape = (h >= 18) & (h <= 45) & (s > 50) & (v > 130)
    bad = cv2.dilate((blue | tape).astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    tex = np.abs(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)) + np.abs(cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))
    tex = cv2.GaussianBlur(tex, (0, 0), 2)
    tex[bad] = np.nan
    u_lo, u_hi = max(0, int(u_lo)), min(img.shape[1], int(u_hi))
    if u_hi - u_lo < 3 * width_px:
        return []
    out = []
    edges = np.linspace(v_top, v_bot, n_bands + 1).astype(int)
    k = np.ones(width_px) / width_px
    for a, b in zip(edges[:-1], edges[1:]):
        blk = tex[a:b, u_lo:u_hi]
        valid = np.isfinite(blk).mean(axis=0)
        if not valid.any():
            continue
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)   # all-masked columns -> NaN
            prof = np.nanmean(blk, axis=0)
        prof = np.where(valid > 0.5, prof, np.nan)          # columns mostly masked: no evidence
        fill = np.nan_to_num(prof, nan=float(np.nanmedian(prof)) if np.isfinite(prof).any() else 0.0)
        sm = np.convolve(fill, k, 'same')
        j = int(np.argmax(sm))
        med = float(np.median(sm))
        if med <= 0 or not np.isfinite(prof[j]):
            continue
        r = sm[j] / med
        if r >= min_ratio:
            out.append(((a + b) / 2.0, float(u_lo + j), float(r)))
    return out


def rope_bands_color(img, u_lo, u_hi, v_top, v_bot, n_bands, width_px, b_rng, s_max, v_min, min_fill,
                     b_rel=0.0, b_floor=0.0):
    """[(v, u_peak, fill)] per row band from the rope colour mask.
    b_rel > 0: adaptive yellowness, b* >= max(b_floor, median b* of the searched area + b_rel) instead of
    b_rng[0] (2026-11-13 afternoon: rope b* 130-131 < 133 on pavers b* 122 -> lost while clearly visible)."""
    u_lo, u_hi = max(0, int(u_lo)), min(img.shape[1], int(u_hi))
    if u_hi - u_lo < 3 * width_px:
        return []
    sub = img[v_top:v_bot, u_lo:u_hi]
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    b = cv2.cvtColor(sub, cv2.COLOR_BGR2LAB)[..., 2]
    b_lo = max(b_floor, float(np.median(b)) + b_rel) if b_rel > 0 else b_rng[0]
    m = ((b >= b_lo) & (b <= b_rng[1]) & (hsv[..., 1] < s_max) & (hsv[..., 2] > v_min)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(np.float32)
    out = []
    edges = np.linspace(0, v_bot - v_top, n_bands + 1).astype(int)
    k = np.ones(width_px) / width_px
    for a, c in zip(edges[:-1], edges[1:]):
        prof = np.convolve(m[a:c].mean(axis=0), k, 'same')
        j = int(np.argmax(prof))
        if prof[j] >= min_fill:
            out.append((v_top + (a + c) / 2.0, float(u_lo + j), float(prof[j])))
    return out


MARKER_H, MARKER_S_MIN, MARKER_V_MIN = (150, 180), 40, 200


def knife_markers(img, v_lo, v_hi, v_min=MARKER_V_MIN, gap_rng=(110, 240), h_rng=MARKER_H, s_min=MARKER_S_MIN):
    """Pink markers on the knives' inner tips: (u_mid, v_mid, gap, (ul, vl), (ur, vr)) or None.
    The two largest compact pink blobs in rows v_lo..v_hi, gap (px) within gap_rng.
    Thresholds 2026-11-14: the markers are bright (V 243-255) but an overexposed one fades to S 43-49,
    H 154-158 (was S > 60, H >= 158 -> lost); the red frame stays dark (V 120-180)."""
    sub = img[v_lo:v_hi]
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    m = ((h >= h_rng[0]) & (h <= h_rng[1]) & (s > s_min) & (v > v_min)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(m)
    b = sorted([j for j in range(1, n) if 150 <= st[j, 4] <= 6000
                and max(st[j, 2], st[j, 3]) < 2.5 * min(st[j, 2], st[j, 3])], key=lambda j: -st[j, 4])[:2]
    if len(b) < 2:
        return None
    (ul, vl), (ur, vr) = sorted((float(cen[j][0]), float(cen[j][1]) + v_lo) for j in b)
    gap = ur - ul
    if not gap_rng[0] <= gap <= gap_rng[1] or abs(vr - vl) > 60:
        return None
    return (ul + ur) / 2.0, (vl + vr) / 2.0, gap, (ul, vl), (ur, vr)


def fit_line(pts, tol):
    """Robust u = a + b*v through band peaks: (a, b, inliers) or None.
    RANSAC over all point pairs (most inliers, then most fill), then weighted refit on the inliers.
    The old fit started from least squares over all points: 2 bands on the sandy knife (rope-coloured
    in midday sun, 2026-11-17 12:55) pulled the line off the rope -> no inliers -> lost on 89 % of frames."""
    if len(pts) < 3:
        return None
    v = np.array([p[0] for p in pts], float); u = np.array([p[1] for p in pts], float)
    w = np.array([p[2] for p in pts], float)
    best = None
    for i in range(len(v)):
        for j in range(i + 1, len(v)):
            if v[i] == v[j]:
                continue
            b = (u[j] - u[i]) / (v[j] - v[i])
            keep = np.abs(u - (u[i] + b * (v - v[i]))) < tol
            score = (int(keep.sum()), float(w[keep].sum()))
            if best is None or score > best[0]:
                best = (score, keep)
    if best is None or best[0][0] < 3:
        return None
    keep = best[1]
    for _ in range(3):
        b, a = np.polyfit(v[keep], u[keep], 1, w=w[keep])
        new = np.abs(u - (a + b * v)) < tol
        if new.sum() < 3 or (new == keep).all():
            break
        keep = new
    return float(a), float(b), keep


class RopeFollower(Node):
    def __init__(self):
        super().__init__('rope_follower')
        d = self.declare_parameter
        self.v_ref = d('v_ref', 480).value                 # knife-tip row (calibration rows ~430..500)
        self.v_top = d('rope_v_top', 40).value             # rows searched for the rope
        self.v_bot = d('rope_v_bot', 530).value            # above the measuring tape
        self.n_bands = d('rope_bands', 8).value
        self.width = d('rope_width_px', 31).value          # box filter ~ rope width
        self.detect_mode = d('rope_detect', 'color').value  # color | texture
        self.min_ratio = d('rope_min_ratio', 1.8).value    # texture: band peak / window median
        self.rope_b = d('rope_b', [133, 146]).value        # color: Lab b* (128 = neutral grey)
        self.rope_s_max = d('rope_s_max', 75).value
        self.rope_v_min = d('rope_v_min', 80).value    # was 140; 1.5 ms AE cap: rope V 98-115 (2026-11-17)
        self.min_fill = d('rope_min_fill', 0.3).value      # color: band peak fill of the mask
        d('rope_b_rel', 6.0)                               # color: b* lower bound = background median + this
        d('rope_b_floor', 126.0)                           #   (0 = fixed rope_b[0]); never below the floor
        self.fit_tol = d('rope_fit_tol_px', 20.0).value
        self.win = d('track_window_px', 120).value
        self.min_inliers = d('rope_min_bands', 5).value   # bands on the fitted line (of rope_bands)
        self.max_jump = d('max_jump_px', 40.0).value       # per frame at v_ref while tracking (rope end,
                                                           # 2026-11-12: 642 -> 313 px in one step)         # +- around the last rope column
        self.search = d('search_cols', [300, 1000]).value  # columns searched when not tracking
        self.kn_cfg = (tuple(d('knife_hue', [90, 125]).value), d('knife_s_min', 45).value,
                       d('knife_v_min', 30).value)
        self.cal_name = d('calibration', 'field2').value   # knife_tps_cal set: field2 (default, on-field fit
                                                           # 2026-11-14) | outdoor | shop
        cal.use(self.cal_name)
        d('center_offset_px', 0.0)                         # rope this far right of the knives' middle
        d('deadband_px', 5.0)                              # ~2 mm on the ground (2.5 px/mm); was 8 (0.4 s
                                                           # of lag at 20 px/s rope drift, 2026-11-13 laps)
        # Rope motion prediction (2026-11-13: knives lagged the rope ~0.6 s -> +-15-20 mm swinging at ~0.4 Hz):
        # aim at u_rope + v*lead, v = rope column speed [px/s] from a line fit over the last vel_window_s.
        d('predict_lead_s', 0.3)                           # 0 = off; offline on the 2-lap log: 0.3 best, 0.5+ worse
        d('predict_max_px', 40.0)                          # cap of the lead offset
        d('vel_window_s', 0.5)
        d('release_frac', 0.5)
        d('dry_run', True)
        d('require_lift_down', True)
        self.tps_lim = d('tps_limits', [560.0, 1270.0]).value   # inside the calibrated 542..1278
        self.retarget = d('retarget_counts', 2.0).value
        self.lost_timeout = d('lost_timeout_s', 0.5).value
        self.smooth = d('smoothing', 0.7).value      # weight of the newest rope column (was 0.5)
        # toolbar.yaml linear TPS <-> metres map
        self.c_min = d('counts_at_min', 1300.0).value
        self.c_max = d('counts_at_max', 350.0).value
        self.p_min = d('min_position', -0.0794).value
        self.p_max = d('max_position', 0.1206).value
        self.debug_hz = d('debug_image_hz', 2.0).value
        d('knife_source', 'auto')                          # auto (markers, else model) | markers | model
        self.mk_rows = d('marker_rows', [330, 620]).value  # rows searched for the pink markers
        d('marker_v_min', MARKER_V_MIN)                    # HSV thresholds of the pink markers
        d('marker_s_min', MARKER_S_MIN)
        d('marker_h', list(MARKER_H))
        d('marker_max_dev_px', 120.0)                      # reject markers this far from the model
        d('px_per_mm', 2.8)                                # ground scale at the tip rows (outdoor 2.8, shop 2.5)
        # knives in a frame = TPS reading at image time + tps_delay_s (the TPS reading lags the camera;
        # knife_tps_fit.py 2026-11-14: ~0.28 s), extrapolated with the TPS rate when that is still ahead
        d('tps_delay_s', 0.2)                              # measured 0.15-0.2 s (cal4 laps)
        # marker steering: observer (default, 2026-11-17) | direct (T_img + err/du_dT with the calibration slope)
        d('marker_control', 'observer')
        d('knife_gain_px', 1.6)                            # knives px per TPS count on short moves (logs 1.3-1.9)
        d('observer_tau_s', 1.0)                           # filter of the knives' offset c (sim: 0.5-2 similar)
        d('observer_hold_s', 1.0)                          # keep steering on c this long without markers
        d('observer_step_counts', 3.0)                     # min target change sent

        self.state = None
        self.lift = None
        self.u_rope = None
        self.slope = 0.0
        self.last_seen = 0.0
        self.last_dir = 0
        self.tps_ref = None            # TPS where the last direction change was registered
        self.target_tps = None
        self.active = False
        self.moved_t = 0.0
        self.last_dbg = 0.0
        self.hist = []                 # (t, raw u) of recent detections for the speed estimate
        self.vel = 0.0
        self.u_pred = None
        self.tps_hist = []             # (t, TPS) for the TPS at image time
        self.mk = None                 # last knife_markers() result of this frame
        self.img_t = 0.0
        self.c_hat = None              # observer: knives' offset u + g*TPS (markers, filtered)
        self.c_t = 0.0
        self.mk_v = self.v_ref         # row of the last markers

        self.create_subscription(ToolbarState, '/toolbar/state', self.on_state, 10)
        self.create_subscription(CompressedImage, '/oakd/rgb/image_raw/compressed', self.on_image,
                                 qos_profile_sensor_data)
        if LiftStateMsg is not None:
            self.create_subscription(LiftStateMsg, '/lift/state', lambda m: setattr(self, 'lift', m), 10)
        self.pub_cmd = self.create_publisher(Float32, '/tool_bar_position_cmd', 10)
        self.pub_status = self.create_publisher(String, '/rope_follower/status', 10)
        self.pub_dbg = self.create_publisher(CompressedImage, '/rope_follower/image/compressed',
                                             qos_profile_sensor_data)
        self.get_logger().info(f'rope_follower: calibration {self.cal_name}, deadband {self.p("deadband_px"):.0f} px, '
                               f'dry_run={self.p("dry_run")}  (live: ros2 param set /rope_follower <name> <value>)')

    def p(self, name):
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tps_to_m(self, t):
        return self.p_min + (t - self.c_min) * (self.p_max - self.p_min) / (self.c_max - self.c_min)

    def on_state(self, msg):
        self.state = msg
        t = msg.raw_position
        now = self.now()
        self.tps_hist = [h for h in self.tps_hist if now - h[0] < 2.0] + [(now, float(t))]
        if self.tps_ref is None:
            self.tps_ref = t
        elif abs(t - self.tps_ref) > 3:                    # knives' direction = sign of the slider motion
            self.last_dir = 1 if t < self.tps_ref else -1  # TPS falling = knives toward image right
            self.tps_ref = t
            self.moved_t = self.now()
        if msg.in_motion:
            self.moved_t = self.now()

    def on_image(self, msg):
        now = self.now()
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        tracking = self.u_rope is not None and now - self.last_seen < 2.0
        if tracking:
            # window around the last rope line, over the whole searched height
            us = [self.u_rope + self.slope * (v - self.v_ref) for v in (self.v_top, self.v_bot)]
            lo, hi = min(us) - self.win, max(us) + self.win
        else:
            lo, hi = self.search
        if self.p('rope_detect') == 'texture':
            pts = rope_bands(img, lo, hi, self.v_top, self.v_bot, self.n_bands, self.width, self.kn_cfg,
                             self.min_ratio)
        else:
            pts = rope_bands_color(img, lo, hi, self.v_top, self.v_bot, self.n_bands, self.width,
                                   self.p('rope_b'), self.p('rope_s_max'), self.p('rope_v_min'),
                                   self.p('rope_min_fill'), self.p('rope_b_rel'), self.p('rope_b_floor'))
        fit = fit_line(pts, self.fit_tol)
        if fit is not None and int(fit[2].sum()) < self.min_inliers:
            fit = None
        if fit is not None and tracking and now - self.last_seen <= self.lost_timeout \
                and abs(fit[0] + fit[1] * self.v_ref - self.u_rope) > self.max_jump:
            fit = None                                     # implausible jump: treat as not seen
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or now
        self.img_t = stamp if abs(now - stamp) < 1.0 else now
        self.mk = None
        if self.p('knife_source') != 'model':
            self.mk = knife_markers(img, int(self.mk_rows[0]), int(self.mk_rows[1]), self.p('marker_v_min'),
                                    h_rng=self.p('marker_h'), s_min=self.p('marker_s_min'))
        if fit is not None:
            a, b, _ = fit
            u = a + b * self.v_ref
            if not tracking or now - self.last_seen > self.lost_timeout:
                self.hist = []                             # reacquired: no speed history across a gap
            self.hist = [h for h in self.hist if stamp - h[0] <= self.p('vel_window_s')] + [(stamp, u)]
            if len(self.hist) >= 3 and self.hist[-1][0] - self.hist[0][0] > 0.15:
                th = np.array([h[0] for h in self.hist]); uh = np.array([h[1] for h in self.hist])
                self.vel = float(np.polyfit(th - th[-1], uh, 1)[0])
            else:
                self.vel = 0.0
            self.u_rope = u if not tracking or now - self.last_seen > self.lost_timeout \
                else self.smooth * u + (1 - self.smooth) * self.u_rope
            self.slope = b
            self.last_seen = now
        lost = fit is None and now - self.last_seen > self.lost_timeout
        st = {'img_t': round(self.img_t, 3), 'lost': lost, 'u_rope': None if self.u_rope is None else round(self.u_rope, 1),
              'bands': len(pts), 'dry_run': self.p('dry_run')}
        if not lost and self.u_rope is not None:
            lead = max(-self.p('predict_max_px'), min(self.p('predict_max_px'), self.vel * self.p('predict_lead_s')))
            self.u_pred = self.u_rope + lead
            st.update(vel_px_s=round(self.vel, 1), u_pred=round(self.u_pred, 1))
        if self.state is not None and not lost and self.u_rope is not None:
            self.control(now, st)
        self.pub_status.publish(String(data=json.dumps(st)))
        if self.debug_hz > 0 and now - self.last_dbg >= 1.0 / self.debug_hz:
            self.last_dbg = now
            self.publish_debug(img, pts, fit, st, msg.header)

    def tps_at(self, t):
        """TPS at time t from the /toolbar/state history: interpolated, extrapolated with the rate of the
        last 0.3 s beyond the newest sample."""
        h = self.tps_hist
        if not h:
            return self.state.raw_position
        if t <= h[0][0]:
            return h[0][1]
        if t >= h[-1][0]:
            old = [x for x in h if h[-1][0] - x[0] <= 0.3]
            rate = (h[-1][1] - old[0][1]) / (h[-1][0] - old[0][0]) if h[-1][0] - old[0][0] > 0.05 else 0.0
            return h[-1][1] + rate * min(t - h[-1][0], 0.5)
        ts = [x[0] for x in h]
        return float(np.interp(t, ts, [x[1] for x in h]))

    def lift_ok(self):
        if not self.p('require_lift_down'):
            return True
        return self.lift is not None and self.lift.position == 'down' and not self.lift.in_motion

    def control(self, now, st):
        T = self.state.raw_position
        st['lift'] = None if self.lift is None else self.lift.position
        if not self.lift_ok():
            # lift up / moving: no commands (toolbar_service holds); forget the target so the next
            # swath starts from wherever the toolbar is
            self.active = False
            self.target_tps = None
            self.c_hat = None
            st.update(tps=round(T), gated=True, tb_state=self.state.state, mode=self.state.mode)
            return
        st['gated'] = False
        if self.target_tps is None:
            self.target_tps = T                            # start by holding where we are
        T_img = self.tps_at(self.img_t + self.p('tps_delay_s'))   # TPS matching the knives in this frame
        u_model = cal.knife_u(T_img, self.last_dir)
        mk = self.mk
        if mk is not None and abs(mk[0] - u_model) > self.p('marker_max_dev_px'):
            mk = None                                      # implausible (not the markers)
        use_mk = mk is not None and self.p('knife_source') in ('auto', 'markers')
        if use_mk:
            # rope column at the markers' row; knives measured in the same frame
            u_goal = self.u_pred + self.slope * (mk[1] - self.v_ref) + self.p('center_offset_px')
            u_knife = mk[0]
        else:
            u_goal = self.u_pred + self.p('center_offset_px')   # predicted rope column (lead), see on_image
            u_knife = u_model
        # Observer (marker_control 'observer', 2026-11-17): knives u = c - g*TPS with the short-move gain
        # g = knife_gain_px (measured 1.3-1.9 px/count, the calibration's 0.6 made every correction 2-3x
        # too big -> limit cycle). c (knives' offset, loose joints) is estimated from the markers, filtered
        # over observer_tau_s; target = (c - u_goal) / g, no deadband/hysteresis (firmware stops < 5 counts).
        # Markers missing for <= observer_hold_s: keep steering with the last c; longer: model fallback.
        g = self.p('knife_gain_px')
        obs = self.p('marker_control') == 'observer' and self.p('knife_source') in ('auto', 'markers')
        if obs and use_mk:
            c_meas = mk[0] + g * T_img
            if self.c_hat is None or self.img_t - self.c_t > self.p('observer_hold_s'):
                self.c_hat = c_meas
            else:
                tau = self.p('observer_tau_s')
                a = min(1.0, (self.img_t - self.c_t) / tau) if tau > 0 else 1.0
                self.c_hat += a * (c_meas - self.c_hat)
            self.c_t = self.img_t
            self.mk_v = mk[1]
        obs_ok = obs and self.c_hat is not None and self.img_t - self.c_t <= self.p('observer_hold_s')
        if obs_ok and not use_mk:
            u_goal = self.u_pred + self.slope * (self.mk_v - self.v_ref) + self.p('center_offset_px')
            u_knife = self.c_hat - g * T_img
        err = u_goal - u_knife
        db = self.p('deadband_px')
        if self.last_dir == 0 and not use_mk:
            db += cal.H / 2.0      # no move seen yet: the knives may sit anywhere in the joint play
        if obs_ok:
            self.active = True
        elif not self.active and abs(err) > db:
            self.active = True
        elif self.active and (abs(err) < db * self.p('release_frac') or (
                abs(err) <= db and not self.state.in_motion and now - self.moved_t > 0.3)):
            self.active = False
        new = False
        if obs_ok:
            t = min(max((self.c_hat - u_goal) / g, self.tps_lim[0]), self.tps_lim[1])
            if abs(t - self.target_tps) > self.p('observer_step_counts'):
                self.target_tps = t
                new = True
        elif self.active:
            if use_mk:
                x = (T_img - cal.T0) / 100.0
                du_dT = (cal.U1 + 2.0 * cal.U2 * x) / 100.0    # px per TPS count (negative)
                t = T_img + err / du_dT
            else:
                t = cal.tps_for(u_goal, cal.direction(u_goal, u_knife), clamp=False)
            t = min(max(t, self.tps_lim[0]), self.tps_lim[1])
            if abs(t - self.target_tps) > self.retarget:
                self.target_tps = t
                new = True
        if not self.p('dry_run'):
            self.pub_cmd.publish(Float32(data=float(self.tps_to_m(self.target_tps))))
        st.update(ctl='observer' if obs_ok else 'direct', c_hat=None if self.c_hat is None else round(self.c_hat, 1),
                  knife_gain=g)
        st.update(knife_src='markers' if use_mk else ('observer' if obs_ok else 'model'), u_model=round(u_model, 1),
                  u_knife_meas=None if mk is None else round(mk[0], 1),
                  marker_v=None if mk is None else round(mk[1]), marker_gap=None if mk is None else round(mk[2]))
        st.update(tps=round(T), last_dir=self.last_dir, u_knife=round(u_knife, 1),
                  err_px=round(err, 1), err_mm=round(err / self.p('px_per_mm'), 1),
                  err_rope_px=round(self.u_rope - u_knife, 1),     # to the current (not predicted) rope
                  active=self.active, new_target=new,
                  target_tps=round(self.target_tps, 1), target_m=round(self.tps_to_m(self.target_tps), 4),
                  saturated=bool(self.target_tps in self.tps_lim and abs(err) > db),
                  tb_state=self.state.state, mode=self.state.mode)

    def publish_debug(self, img, pts, fit, st, header):
        vis = img
        for v, u, r in pts:
            cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), 2)
        if fit is not None:
            a, b, _ = fit
            h = vis.shape[0] - 1
            cv2.line(vis, (int(a), 0), (int(a + b * h), h), (0, 255, 0), 2)
        cv2.line(vis, (0, self.v_ref), (vis.shape[1] - 1, self.v_ref), (255, 255, 0), 1)
        if st.get('gated'):
            txt = f"LIFT {st.get('lift')}: not commanding  TPS {st['tps']}"
        elif 'u_knife' in st:
            uk = st['u_knife']
            half = st['marker_gap'] / 2.0 if st.get('marker_gap') else 82   # ~half the knife-tip gap
            for x in (uk - half, uk, uk + half):
                cv2.line(vis, (int(x), self.v_ref - 40), (int(x), self.v_ref + 40), (255, 0, 255), 3)
            cv2.line(vis, (int(st['u_rope']), self.v_ref), (int(uk), self.v_ref), (0, 255, 255), 3)
            if st.get('u_pred') is not None:                   # predicted rope column (orange)
                cv2.line(vis, (int(st['u_pred']), self.v_ref - 60), (int(st['u_pred']), self.v_ref + 60),
                         (0, 140, 255), 3)
            if self.mk is not None:                            # measured markers (white rings + centre tick)
                for (u, v) in (self.mk[3], self.mk[4]):
                    cv2.circle(vis, (int(u), int(v)), 16, (255, 255, 255), 3)
                cv2.line(vis, (int(self.mk[0]), int(self.mk[1]) - 50), (int(self.mk[0]), int(self.mk[1]) + 50),
                         (255, 255, 255), 3)
            txt = (f"[{st.get('knife_src', '?')}] "
                   f"err {st['err_px']:+.0f}px ({st['err_mm']:+.1f}mm) v {st.get('vel_px_s', 0):+.0f}px/s "
                   f"TPS {st['tps']} -> {st['target_tps']:.0f}"
                   f"  {st['tb_state']}/{st['mode']}")
        else:
            txt = 'ROPE LOST' if st['lost'] else 'no toolbar state'
        if st['dry_run']:
            txt += '  DRY RUN'
        cv2.putText(vis, txt, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        ok, jpg = cv2.imencode('.jpg', cv2.resize(vis, (640, 360)), [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            self.pub_dbg.publish(CompressedImage(header=header, format='jpeg', data=jpg.tobytes()))


def main():
    rclpy.init()
    node = RopeFollower()
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
