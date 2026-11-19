#!/usr/bin/env python3
"""Knives position <-> toolbar sensor (TPS) calibration, solbot4 OAK-D (lift down, 1280x720), 2026-11-12.

Measured by sweeping the toolbar -75..+80..-75 mm in 10 mm steps (docs/toolbar_cam_calib/knife_tps_2026-11-12):
u_mid = image column [px] of the midpoint between the two knives' inner wing tips at ground level
(rows v ~430..500, same detection as bar_follower reference=knives).

    u_mid(T, dir) = U0 + U1*x + U2*x^2 - dir*H/2,    x = (T - T0) / 100

    T    TPS raw counts (/toolbar/state raw_position), valid 542..1278 (knives u_mid ~462..845)
    dir  +1 if the knives last moved toward image RIGHT (TPS decreasing, toolbar +p),
         -1 if toward image left (TPS increasing)
    H    implement joint play: the knives lag the slider by H/2 in the direction of travel.
         After a reversal the knives don't move until ~H/|du/dT| ~ 45 counts (~9.5 mm) are taken up.

Fit: rms 0.98 px, max 2.5 px (0.4 / 1 mm) over 30 settled steps (reversal steps inside the play excluded).
Ground scale at the tip rows ~2.5 px/mm (measuring tape) -> 1 TPS count ~ 0.55 px ~ 0.22 mm of knife motion.

Use to keep a cable between the knives: detect the cable column u_cable at the tip rows, then
    tps_goal = tps_for(u_cable, direction(u_cable, u_now))
where u_now is the current knife midpoint (measured, or knife_u(T, last_dir)).

CLI:  knife_tps_cal.py u <u_mid> [dir] [shop|outdoor|field]   -> TPS
      knife_tps_cal.py tps <T> [dir] [shop|outdoor|field]     -> u_mid
"""
import math
import sys

T0 = 925.0
# Calibration sets (same model, lift DOWN). use(name) selects one; default 'shop'.
CALS = {
    # 2026-11-12 shop floor, knives' inner blue-steel wing tips at ground level (docs/toolbar_cam_calib/
    # knife_tps_2026-11-12): rms 0.98 px, max 2.5 px
    'shop': dict(U0=661.753, U1=-55.435, U2=-1.125, H=24.833),
    # 2026-11-13 outdoors (courtyard, sunlit), pink markers on the knives' inner tips
    # (docs/toolbar_cam_calib/knife_tps_2026-11-13_outdoor): rms 1.00 px, max 3.1 px.
    # Same slope as the shop, +15 px offset (knives look ~12% larger: 157-182 px gap vs 143-153),
    # less joint play (14.6 vs 24.8 px). The marker gap changes with direction (173 vs 165 px at TPS 898):
    # the knives yaw slightly in the play.
    'outdoor': dict(U0=676.995, U1=-56.140, U2=-1.156, H=14.609),
    # 2026-11-14 on-field fit while driving (knife_tps_fit.py over 6 rope runs 11-13/14, 1157 samples, lift
    # down, pavers): TPS at image time + 0.28 s (the TPS reading lags the camera), rms 17 px (~6 mm) - the
    # knives wobble in the joint play while dragging, which TPS cannot see. H < 0: knives slightly ahead.
    'field': dict(U0=674.955, U1=-61.699, U2=-1.783, H=-7.074),
    # 2026-11-14 14:04 4 laps 0.4 m/s, markers re-detected from raw frames (rope_cal4_20261114_140419),
    # 2792 lift-down samples, TPS 695..1131, image->TPS delay 0.2 s (use tps_delay_s 0.2): rms 20.5 px (~7 mm)
    # over all swaths, 12.8 px within one swath (the knives' rest offset shifts between swaths/laps).
    'field2': dict(U0=672.471, U1=-60.634, U2=-2.937, H=6.080),
}
TPS_MIN, TPS_MAX = 542.0, 1278.0    # calibrated range (TPS 400 ~ right end binds/stalls: knives tilt
                                    # instead of sliding beyond ~+85 mm, 2026-11-12 first sweep)
NAME = None


def use(name):
    """Select a calibration set by name (KeyError if unknown)."""
    global U0, U1, U2, H, NAME
    c = CALS[name]
    U0, U1, U2, H, NAME = c['U0'], c['U1'], c['U2'], c['H'], name


use('shop')


def knife_u(tps, dir=0):
    """Knife midpoint image column [px] at TPS `tps`; dir +1/-1 last move direction (0 = mid-play)."""
    x = (tps - T0) / 100.0
    return U0 + U1 * x + U2 * x * x - dir * H / 2.0


def direction(u_goal, u_now):
    """+1 if the knives must move right in the image to reach u_goal, else -1."""
    return 1 if u_goal > u_now else -1


def tps_for(u_goal, dir=0, clamp=True):
    """TPS that puts the knife midpoint at image column u_goal, approaching in direction dir."""
    c = U0 - dir * H / 2.0 - u_goal
    # U2*x^2 + U1*x + c = 0, root near the linear solution (numerically stable form)
    disc = U1 * U1 - 4.0 * U2 * c
    x = -2.0 * c / (U1 - math.sqrt(max(disc, 0.0))) if U1 < 0 else -2.0 * c / (U1 + math.sqrt(max(disc, 0.0)))
    t = T0 + 100.0 * x
    return min(max(t, TPS_MIN), TPS_MAX) if clamp else t


if __name__ == '__main__':
    if len(sys.argv) < 3 or sys.argv[1] not in ('u', 'tps'):
        print(__doc__.split('CLI:')[1]); sys.exit(1)
    v = float(sys.argv[2]); d = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    if len(sys.argv) > 4:
        use(sys.argv[4])
    if sys.argv[1] == 'u':
        print(f'{tps_for(v, d):.1f}')
    else:
        print(f'{knife_u(v, d):.1f}')
