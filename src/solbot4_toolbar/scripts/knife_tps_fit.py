#!/usr/bin/env python3
"""Fit the knife-position <-> TPS calibration (knife_tps_cal model) from recorded on-field runs.

    knife_tps_fit.py BAG_DIR [BAG_DIR ...] [--since YYYYMMDD_HHMM] [--csv OUT.csv] [--base outdoor]
                     [--delay S] [--name field2]
    knife_tps_fit.py BAG_DIR ... --eval field2 [--delay S]    check a calibration set against new runs

Data per bag (pd600.sh ropelog records these):
  /toolbar/state                      stamp + raw_position (TPS), the TPS history
  /lift/state                         samples only while the lift is 'down' and not moving
  /oakd/rgb/image_raw/compressed      if recorded (ropelog <tag> raw): markers re-detected with
                                      rope_follower.knife_markers (preferred, current thresholds)
  /rope_follower/knife_markers        else: marker midpoints logged live by rope_precision.py
Each sample: u = knife midpoint column from the pink markers, T = TPS at the image time (shifted by the
fitted camera delay), dir = last motion direction (+1 knives toward image right = TPS falling, the
knife_tps_cal convention), vel = dT/dt over the last 0.3 s [counts/s].

Models (x = (T - 925) / 100), least squares with 3*MAD outlier rejection:
  A  u = U0 + U1 x + U2 x^2 - dir H/2                  (knife_tps_cal, static calibration form)
  B  A + kv * vel                                       (swing of the knives while the slider moves)
Prints rms per model, the fitted camera delay, the base calibration's error on the same samples and a
CALS entry to paste into knife_tps_cal.py. Positive H: knives lag the slider (shop/static); negative H:
the knives run ahead in the direction of travel (seen while driving, 2026-11-14).
"""
import argparse
import bisect
import glob
import json
import math
import os
import sys
import time

import numpy as np

import knife_tps_cal as cal

T0 = cal.T0


def read_bag(path, want_raw=True):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from toolbar_msgs.msg import ToolbarState
    try:
        from solbot4_msgs.msg import LiftStateMsg
    except ImportError:
        LiftStateMsg = None
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    topics = {t.name for t in r.get_all_topics_and_types()}
    raw = want_raw and '/oakd/rgb/image_raw/compressed' in topics
    sel = ['/toolbar/state', '/lift/state', '/rope_follower/knife_markers']
    if raw:
        sel.append('/oakd/rgb/image_raw/compressed')
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in sel if t in topics]))
    tb, lift, mk = [], [], []
    if raw:
        import cv2
        from rope_follower import knife_markers
    while r.has_next():
        topic, data, t_rec = r.read_next()
        if topic == '/toolbar/state':
            m = deserialize_message(data, ToolbarState)
            tb.append((m.stamp.sec + m.stamp.nanosec * 1e-9, float(m.raw_position)))
        elif topic == '/lift/state' and LiftStateMsg is not None:
            m = deserialize_message(data, LiftStateMsg)           # recording time (no stamp in the msg)
            lift.append((t_rec * 1e-9, m.position == 'down' and not m.in_motion))
        elif topic == '/rope_follower/knife_markers' and not raw:
            d = json.loads(deserialize_message(data, String).data)
            if d.get('u_knife_meas') is not None and d.get('stamp'):
                mk.append((d['stamp'], d['u_knife_meas'], d.get('gap_px'), d.get('lift')))
        elif topic == '/oakd/rgb/image_raw/compressed':
            m = deserialize_message(data, CompressedImage)
            img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            res = knife_markers(img, 330, 620) if img is not None else None
            if res is not None:
                mk.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, res[0], res[2], None))
    return tb, lift, mk, raw


def motion_series(tb):
    """Per toolbar sample: last motion direction (follower convention: TPS falling -> +1), velocity."""
    ts = np.array([t for t, _ in tb]); T = np.array([v for _, v in tb])
    dirs = np.zeros(len(T)); ref = T[0]; d = 0
    for i, v in enumerate(T):
        if abs(v - ref) > 3:
            d = 1 if v < ref else -1
            ref = v
        dirs[i] = d
    return ts, T, dirs


def samples(tb, lift, mk, delay):
    ts, T, dirs = motion_series(tb)
    lt = [x for x in lift if x[0] is not None]
    lt_t = [x[0] for x in lt]
    out = []
    for (t_img, u, gap, lift_txt) in mk:
        t = t_img + delay
        if t < ts[0] or t > ts[-1]:
            continue
        if lt:
            k = bisect.bisect_right(lt_t, t_img) - 1         # lift state at the image time
            if k < 0 or not lt[k][1]:
                continue
        elif lift_txt not in (None, 'down'):
            continue
        Ti = float(np.interp(t, ts, T))
        k = max(bisect.bisect_right(ts, t) - 1, 0)
        j = bisect.bisect_left(ts, t - 0.3)
        vel = (T[k] - T[j]) / max(ts[k] - ts[j], 1e-3) if k > j else 0.0
        out.append((t_img, Ti, dirs[k], vel, u, gap))
    return out


def fit(S, with_vel):
    x = (S[:, 1] - T0) / 100.0
    cols = [np.ones_like(x), x, x * x, S[:, 2]] + ([S[:, 3]] if with_vel else [])
    A = np.stack(cols, 1); y = S[:, 4]
    keep = np.ones(len(y), bool)
    for _ in range(5):
        c, *_ = np.linalg.lstsq(A[keep], y[keep], rcond=None)
        res = y - A @ c
        mad = np.median(np.abs(res[keep] - np.median(res[keep]))) + 1e-6
        new = np.abs(res) < 3 * 1.4826 * mad + 2.0
        if (new == keep).all():
            break
        keep = new
    rms = float(np.sqrt(np.mean(res[keep] ** 2)))
    return c, rms, keep, res


def evaluate(a, data):
    """Residual u_meas - knife_u(T at image time + delay, dir) of a calibration set, per run and overall."""
    cal.use(a.eval)
    delay = a.delay if a.delay is not None else 0.0
    allr = []
    print(f'\nset "{a.eval}" (U0 {cal.U0:.1f} U1 {cal.U1:.2f} U2 {cal.U2:.3f} H {cal.H:.1f}), delay {delay:+.2f} s, '
          f'{a.px_per_mm} px/mm')
    for (name, tb, lift, mk, _) in data:
        S = np.array(samples(tb, lift, mk, delay))
        if not len(S):
            continue
        r = S[:, 4] - np.array([cal.knife_u(t, d) for t, d in zip(S[:, 1], S[:, 2])])
        allr.append(r)
        print(f'  {name}: n {len(r)}  mean {r.mean():+.1f} px  ' + stats(r, a.px_per_mm))
    r = np.concatenate(allr)
    print(f'  ALL: n {len(r)}  mean {r.mean():+.1f} px ({r.mean() / a.px_per_mm:+.1f} mm)  ' + stats(r, a.px_per_mm))


def stats(r, ppm):
    e = np.abs(r) / ppm
    return (f'|err| median {np.median(e):.1f} mm, p90 {np.percentile(e, 90):.1f}, rms {np.sqrt(np.mean(e ** 2)):.1f}; '
            f'within 5 mm {np.mean(e <= 5) * 100:.0f}%, 10 mm {np.mean(e <= 10) * 100:.0f}%')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='*', default=sorted(glob.glob(os.path.expanduser('~/rope_logs/rope_*'))))
    ap.add_argument('--since', help='only bags whose name stamp is >= YYYYMMDD_HHMM')
    ap.add_argument('--csv', help='write the samples here')
    ap.add_argument('--base', default='outdoor', help='calibration set to compare against')
    ap.add_argument('--no-raw', action='store_true', help='use logged markers even when raw images exist')
    ap.add_argument('--delay', type=float, help='image->TPS delay [s] (default: fitted)')
    ap.add_argument('--name', default='field', help='name of the printed CALS entry')
    ap.add_argument('--eval', help='only evaluate this knife_tps_cal set on the samples (no fit)')
    ap.add_argument('--px-per-mm', type=float, default=2.8)
    a = ap.parse_args()
    bags = [b for b in a.bags if os.path.isdir(b)]
    if a.since:
        bags = [b for b in bags if b.rstrip('/').split('_', 2)[-1] >= a.since]
    data = []
    for b in bags:
        try:
            tb, lift, mk, raw = read_bag(b, not a.no_raw)
        except Exception as e:                              # noqa: BLE001 (unreadable / still recording)
            print(f'skip {os.path.basename(b)}: {e}', file=sys.stderr); continue
        if len(tb) < 10 or not mk:
            print(f'skip {os.path.basename(b)}: {len(tb)} toolbar, {len(mk)} marker samples', file=sys.stderr)
            continue
        data.append((os.path.basename(b), tb, lift, mk, raw))
        print(f'{os.path.basename(b)}: {len(mk)} marker samples ({"raw re-detected" if raw else "logged"}), '
              f'{len(tb)} TPS samples')
    if not data:
        sys.exit('no usable data')

    if a.eval:
        evaluate(a, data); return
    # camera delay: shift of the image stamps against the TPS history that fits model A best
    best = (a.delay, None) if a.delay is not None else None
    for delay in ([] if a.delay is not None else np.arange(-0.4, 0.61, 0.02)):
        S = np.array([s for (_, tb, lift, mk, _) in data for s in samples(tb, lift, mk, delay)])
        if len(S) < 30:
            continue
        _, rms, _, _ = fit(S, False)
        if best is None or rms < best[1]:
            best = (delay, rms)
    delay = best[0]
    runs = [(name, np.array(samples(tb, lift, mk, delay))) for (name, tb, lift, mk, _) in data]
    runs = [(n, s) for n, s in runs if len(s)]
    S = np.vstack([s for _, s in runs])
    print(f'\n{len(S)} samples (lift down), TPS {S[:, 1].min():.0f}..{S[:, 1].max():.0f}, '
          f'best image->TPS delay {delay:+.2f} s')

    cal.use(a.base)
    base_res = S[:, 4] - np.array([cal.knife_u(t, d) for t, d in zip(S[:, 1], S[:, 2])])
    print(f'base "{a.base}": mean {base_res.mean():+.1f} px, rms {np.sqrt(np.mean(base_res ** 2)):.1f} px')
    for d in (1, -1):
        m = S[:, 2] == d
        if m.any():
            print(f'   dir {d:+d} ({"toward image right" if d > 0 else "toward image left"}): '
                  f'mean {base_res[m].mean():+.1f} px  n={m.sum()}')

    cA, rmsA, keepA, _ = fit(S, False)
    cB, rmsB, keepB, resB = fit(S, True)
    H = -2.0 * cA[3]
    print(f'\nA (static form): U0 {cA[0]:.3f} U1 {cA[1]:.3f} U2 {cA[2]:.3f} H {H:.3f} px   '
          f'rms {rmsA:.2f} px ({keepA.sum()}/{len(S)} kept)')
    print(f'B (+velocity):   U0 {cB[0]:.3f} U1 {cB[1]:.3f} U2 {cB[2]:.3f} H {-2 * cB[3]:.3f} px '
          f'kv {cB[4]:.4f} px per count/s   rms {rmsB:.2f} px ({keepB.sum()}/{len(S)} kept)')
    print('residual by run (model B): ' + ', '.join(
        f'{n}: {np.mean(resB[i:i + len(s)][keepB[i:i + len(s)]]):+.1f}' for (n, s), i in
        zip(runs, np.cumsum([0] + [len(s) for _, s in runs])[:-1])))
    print(f"\nknife_tps_cal.CALS entry (model A):\n    '{a.name}': dict(U0={cA[0]:.3f}, U1={cA[1]:.3f}, "
          f"U2={cA[2]:.3f}, H={H:.3f}),   # {time.strftime('%Y-%m-%d')} on-field fit, {len(S)} samples, "
          f"rms {rmsA:.2f} px, delay {delay:+.2f} s")

    if a.csv:
        with open(a.csv, 'w') as f:
            f.write('run,t_img,tps,dir,vel,u_meas,gap,res_B\n')
            i = 0
            for n, s in runs:
                for row in s:
                    f.write(f'{n},{row[0]:.3f},{row[1]:.1f},{row[2]:+.0f},{row[3]:.1f},{row[4]:.1f},'
                            f'{row[5]},{resB[i]:.2f}\n'); i += 1
        print(f'samples -> {a.csv}')


if __name__ == '__main__':
    main()
