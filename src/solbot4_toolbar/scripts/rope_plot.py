#!/usr/bin/env python3
"""Time chart of a rope-follower bag (pd600.sh ropelog): rope / knife columns, error, toolbar PWM, TPS.

    rope_plot.py BAG_DIR [-o out.png] [--from HH:MM:SS] [--to HH:MM:SS] [--down-only]

Panels (shared time axis, local time):
  1 image column px: rope (detected), rope predicted, knives camera-measured (pink markers), knives model
  2 error mm: rope - knives (markers) and rope - knives (model)
  3 toolbar PWM duty (/toolbar/state.duty, 0..1, unsigned)
  4 TPS counts: measured, target sent by the follower, firmware goal
Grey bands = lift down (working).
"""
import argparse
import datetime as dt
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

PX_MM = 2.8
C = dict(rope='#2a78d6', pred='#86b6ef', meas='#eb6834', model='#1baf7a', pwm='#eda100',
         tps='#2a78d6', target='#eb6834', goal='#1baf7a', ink='#3a3a38', grid='#e4e3de')
TOPICS = ['/rope_follower/status', '/rope_follower/knife_markers', '/toolbar/state', '/lift/state']


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    cls = {t: get_message(types[t]) for t in TOPICS if t in types}
    out = {t: [] for t in TOPICS}
    while r.has_next():
        topic, data, t_ns = r.read_next()
        m = deserialize_message(data, cls[topic])
        out[topic].append((t_ns * 1e-9, m))
    return out


def col(rows, f):
    t = np.array([r[0] for r in rows])
    v = np.array([np.nan if (x := f(r[1])) is None else float(x) for r in rows], float)
    return t, v


def lift_down_spans(rows):
    spans, start = [], None
    for t, m in rows:
        down = m.position == 'down'
        if down and start is None:
            start = t
        elif not down and start is not None:
            spans.append((start, t))
            start = None
    if start is not None and rows:
        spans.append((start, rows[-1][0]))
    return spans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('-o', '--out')
    ap.add_argument('--from', dest='t0')
    ap.add_argument('--to', dest='t1')
    a = ap.parse_args()
    B = read(a.bag)

    st = [(t, json.loads(m.data)) for t, m in B['/rope_follower/status']]
    mk = [(t, json.loads(m.data)) for t, m in B['/rope_follower/knife_markers']]
    tb = B['/toolbar/state']

    def g(k, lost_ok=False):
        return lambda d: None if (d.get('lost') and not lost_ok) else d.get(k)

    t_s, rope = col(st, g('u_rope'))
    _, pred = col(st, g('u_pred'))
    _, kmodel = col(st, g('u_model', True))
    if np.all(np.isnan(kmodel)):
        _, kmodel = col(st, g('u_knife', True))
    _, kmeas_st = col(st, g('u_knife_meas', True))
    _, tgt = col(st, g('target_tps', True))
    t_m, kmeas = col(mk, g('u_knife_meas', True))
    _, em_meas = col(mk, g('err_meas_mm'))
    err_model = (rope - kmodel) / PX_MM
    t_tb, tps = col(tb, lambda m: m.raw_position)
    _, duty = col(tb, lambda m: m.duty)
    _, goal = col(tb, lambda m: m.firmware_goal)
    if np.all(np.isnan(kmeas)) and not np.all(np.isnan(kmeas_st)):     # older bags: use the follower's own
        t_m, kmeas = t_s, kmeas_st
        em_meas = (rope - kmeas_st) / PX_MM

    spans = lift_down_spans(B['/lift/state'])
    if spans:                                   # markers seen with the lift up are grass noise
        down = np.zeros_like(t_m, bool)
        for s0, s1 in spans:
            down |= (t_m >= s0) & (t_m <= s1)
        kmeas = np.where(down, kmeas, np.nan)
        em_meas = np.where(down, em_meas, np.nan)

    tz = dt.datetime.now().astimezone().tzinfo
    T = lambda t: [dt.datetime.fromtimestamp(x, tz) for x in t]
    lo, hi = min(t_s.min(), t_tb.min()), max(t_s.max(), t_tb.max())
    day = dt.datetime.fromtimestamp(lo, tz).date()
    hm = lambda s: dt.datetime.combine(day, dt.time.fromisoformat(s), tz).timestamp()
    if a.t0:
        lo = hm(a.t0)
    if a.t1:
        hi = hm(a.t1)

    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': C['grid'], 'axes.labelcolor': C['ink'],
                         'xtick.color': C['ink'], 'ytick.color': C['ink'], 'axes.titlecolor': C['ink']})
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(15, 11),
                           gridspec_kw=dict(height_ratios=[3, 2, 1.2, 2]))
    ln = dict(lw=1.6)

    ax[0].plot(T(t_s), rope, color=C['rope'], label='rope (detected)', **ln)
    ax[0].plot(T(t_s), pred, color=C['pred'], label='rope predicted', lw=1.2, ls='--')
    ax[0].plot(T(t_m), kmeas, color=C['meas'], marker='o', ms=3, lw=1.2, label='knives (pink markers)')
    ax[0].plot(T(t_s), kmodel, color=C['model'], label='knives (TPS model)', lw=1.2)
    ax[0].set_ylabel('image column [px]')

    ax[1].axhline(0, color=C['ink'], lw=0.8)
    for y in (-5, 5):
        ax[1].axhline(y, color=C['ink'], lw=0.6, ls=':')
    ax[1].plot(T(t_m), em_meas, color=C['meas'], marker='o', ms=3, lw=1.2, label='rope − knives (markers)')
    ax[1].plot(T(t_s), err_model, color=C['model'], lw=1.2, label='rope − knives (model)')
    ax[1].set_ylabel('error [mm]')

    ax[2].plot(T(t_tb), duty * 255, color=C['pwm'], label='PWM', **ln, drawstyle='steps-post')
    ax[2].set_ylabel('PWM [0..255]')
    ax[2].set_ylim(-5, 265)

    ax[3].plot(T(t_tb), tps, color=C['tps'], label='TPS measured', **ln)
    ax[3].plot(T(t_s), tgt, color=C['target'], label='target (follower)', lw=1.2, drawstyle='steps-post')
    ax[3].plot(T(t_tb), goal, color=C['goal'], label='firmware goal', lw=1.0, ls='--', drawstyle='steps-post')
    ax[3].set_ylabel('TPS [counts]')

    for x in ax:
        for s, e in spans:
            x.axvspan(dt.datetime.fromtimestamp(s, tz), dt.datetime.fromtimestamp(e, tz),
                      color='#00000010', lw=0)
        x.grid(True, color=C['grid'], lw=0.6)
        for side in ('top', 'right'):
            x.spines[side].set_visible(False)
        x.legend(loc='upper left', fontsize=8, frameon=False, ncol=4)
    ax[3].set_xlim(dt.datetime.fromtimestamp(lo, tz), dt.datetime.fromtimestamp(hi, tz))
    ax[3].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S', tz=tz))
    # y-limits from the visible window
    sel = lambda t: (t >= lo) & (t <= hi)
    for x, series in ((ax[0], [(t_s, rope), (t_m, kmeas), (t_s, kmodel)]),
                      (ax[1], [(t_m, em_meas), (t_s, err_model)]),
                      (ax[3], [(t_tb, tps), (t_s, tgt)])):
        v = np.concatenate([s[sel(t)] for t, s in series])
        v = v[np.isfinite(v)]
        if v.size:
            p = (v.max() - v.min()) * 0.08 + 1
            x.set_ylim(v.min() - p, v.max() + p)
    name = a.bag.rstrip('/').split('/')[-1]
    fig.suptitle(f'Rope following — {name}   (grey = lift down)', x=0.01, ha='left', color=C['ink'])
    fig.tight_layout()
    out = a.out or a.bag.rstrip('/') + '_timechart.png'
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == '__main__':
    main()
