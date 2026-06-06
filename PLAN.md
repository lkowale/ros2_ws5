# solbot5 incremental stack plan

solbot5 = solbot4 + dual-GPS-antenna heading. Built self-contained in `ros2_ws5`
with copied + renamed `solbot5_*` packages. Reuse solbot4 code, optimize where
possible.

## What changes vs solbot4

Only **heading** changes fundamentally. Everything downstream is reusable.

| | solbot4 | solbot5 |
|---|---|---|
| Heading source | GPS velocity vector (`gps_vel_odom`) | dual-antenna RTK baseline (`UBXNavRelPosNED.rel_pos_heading`) |
| Valid when | speed > 0.3 m/s, forward only, low yaw rate | **always** — standstill, reverse, turning |
| EKF yaw input | `imu0` differential + `imu1=/imu/gps_heading` (intermittent) | absolute heading available continuously |
| BT consequence | heading-acquisition wiggles, no reversing, careful turns | simpler BTs, reversing allowed |

Hardware path already scaffolded in `ublox_dgnss`:
`ublox_mb+r_base.launch.py` (moving base, RTCM 4072/107x/108x… on UART2) +
rover with `CFG_MSGOUT_UBX_NAV_RELPOSNED_USB`.

## Launch architecture — independently-relaunchable layers

Generalizes the existing `run_localization.sh restart` precedent. One DDS domain
shared by all layers; each layer is its own `run_*.sh` that pkills only its own
nodes and relaunches. Bring up `run_hw.sh` once, iterate freely above it.

| Layer | run script | nodes | restart cadence |
|---|---|---|---|
| Hardware | `run_hw.sh` | drive, steering, imu_bridge, ublox MB+R containers, ackermann_odom, mqtt_op | rare |
| Localization | `run_localization.sh` | **relposned_heading** (new), navsat_transform, EKF, navsat_init | freely while driving |
| Navigation | `run_nav.sh` | Nav2, controllers, planners | to retune |
| Mission | `run_mission.sh` | field / one_line navigators, action servers, BTs | per mission |
| Telemetry | `run_telemetry.sh` | loggers, rosbridge, supervisors, domain_bridge | anytime |

Sim mirror: `run_sim.sh` brings up Gazebo + the same localization/nav layers,
with hardware nodes replaced by Gazebo adapters.

## Milestones (each testable in sim before hardware)

- **M1 — minimal localization** ← start here, in sim
  - hw layer (sim) + new `relposned_heading` node + EKF only. No Nav2.
  - New sim node `sim_relposned_publisher.py`: emit `UBXNavRelPosNED` from Gazebo
    ground-truth yaw (with optional noise) so the localization path is identical
    sim↔real.
  - Validate: heading correct at standstill, forward, reverse, turning.
- **M2 — Nav2 drive** — add Nav2, send goals manually. ← IN PROGRESS
  - KNOWN ISSUE deferred from M1: EKF localizes the GPS antenna (gps_link, 0.95m
    fwd of base_footprint), so base_footprint estimate is biased 0.95m forward.
    Fix = antenna lever-arm correction in navsat_transform. Address during M2
    since Nav2 accuracy depends on it.
- **M3 — one_line navigator** — port + simplify BT (heading always available).
- **M4 — field navigator** — port + simplify BT.

## M1 package copy/rename map

| solbot4 source | solbot5 package | M1 contents |
|---|---|---|
| `src/control/solbot_control` | `solbot5_control` | drive, steering, imu_bridge, ackermann_odom, navsat_init (drop gps_vel_odom, heading_fuser) |
| `src/navigation/localization` | `solbot5_localization` | EKF launch+config, **new** `relposned_heading` node |
| `src/simulation/gazebo_spawn` | `solbot5_gazebo_spawn` | sim adapters + **new** `sim_relposned_publisher.py` |
| `src/ublox_dgnss` | copied as-is | provides `ublox_ubx_msgs` (UBXNavRelPosNED) + MB+R launches |
| `solbot5_description` | exists | already in ws5 |

`solbot5_msgs` only if M1 needs custom msgs — RELPOSNED comes from
`ublox_ubx_msgs`, so likely not needed until M3/M4.

## New code for M1

1. `solbot5_localization/.../relposned_heading.py` — subscribe `UBXNavRelPosNED`,
   convert `rel_pos_heading` (deg×1e-5, baseline direction) → `Imu` on
   `/imu/gps_heading` with absolute yaw + covariance from `rel_pos_length` /
   carrier-solution flags. Apply antenna-mounting offset (front antenna ahead of
   rear → baseline yaw = robot heading; verify sign).
2. `solbot5_gazebo_spawn/.../sim_relposned_publisher.py` — Gazebo ground-truth
   yaw → `UBXNavRelPosNED` (noise optional), so #1 is exercised in sim.
3. EKF config: absolute yaw now continuous — drop `imu0` differential reliance,
   feed heading as a well-trusted absolute measurement.
4. `run_hw.sh` (sim variant) + `run_localization.sh` + `run_sim.sh`.

## Open questions for later milestones
- Antenna baseline sign / mounting offset calibration (front=gps_link +0.95,
  rear=gps_rear_link −0.95 → baseline points forward).
- Whether IMU is still fused at all, or heading is purely dual-GPS + yaw-rate.
- BT simplifications once reversing is allowed.
