# ros2_ws5

## Sliding toolbar — software

All toolbar code lives here (ros2_ws5). It runs in the Gazebo sim and, for testing,
on the physical **solbot4** robot (pd600) as an overlay on `~/ros2_ws4`. Do not copy
it into ros2_ws4; change it here and redeploy.

| Package | What |
|---|---|
| `toolbar` | `toolbar_service`: executes toolbar moves, enforces soft limits, monitors the actuator, reports to PauseManager. `firmware` param: `solbot_main` (solbot5 board, metres) or `main_esp32_pio` (solbot4 board, ADC counts). Config: `config/toolbar_solbot5.yaml` |
| `toolbar_msgs` | `ToolbarState.msg` (`/toolbar/state`, 10 Hz), `ToolbarControl.srv` (`/toolbar_control`) |
| `solbot4_toolbar` | solbot4 launch (`core_toolbar.launch.py` = ws4 `pd600_core.launch.py` + `toolbar_service`) and calibration (`config/toolbar.yaml`) |
| `tool_slider` | `tool_slider_controller` (row offset → `/tool_bar_position_cmd`, PID), sim actuator + plant-row sim |
| `crop_row_vision` | camera crop-row detection → `/plant_row_offset` |

Interface (metres from centre, +right): `toolbar_service` follows `/tool_bar_position_cmd`
in `auto` mode and publishes the measured `/tool_bar_position`, the same topics
`tool_actuator_sim` uses in the sim. Commands: `set_position`, `center`, `hold`, `auto`,
`enable`, `disable`, `reset_fault`, `get_status` (service `/toolbar_control` or
topic `/toolbar/cmd`). Starts **monitor only** (`actuator_enabled: false`). A disabled
toolbar never pauses the robot.

### Running on solbot4 from the laptop

```bash
bash pd600.sh deploy                # rsync toolbar packages to pd600:~/ros2_ws5, build there on ws4
bash pd600.sh start_bar             # all-in-one: core + toolbar + OAK-D + bar_follower (one tmux)
bash pd600.sh start                 # core stack (run_core_um982.sh, field_nav_hw params) + toolbar, tmux "solbot4"
bash pd600.sh attach                # watch (Ctrl-b d detach, Ctrl-c stop)
bash pd600.sh state                 # one /toolbar/state (raw_position = TPS counts)
bash pd600.sh toolbar enable        # set_position 0.02 | center | hold | auto | reset_fault | disable
bash pd600.sh stop | status
bash pd600.sh ros2 <args...>        # any ros2 command on the robot
bash pd600.sh camera                # OAK-D (solbot4_toolbar/oakd.launch.py, RGB 1280x720 @10 fps) + JPEG
                                    #   domain_bridge 0→1 to hp-zbook, tmux "oakd"; start AFTER `start`
bash pd600.sh view                  # laptop: oakd_view.py (OpenCV window, Hz/kB overlay) of /oakd/rgb/image_raw/compressed, domain 1
bash pd600.sh camera_stop
bash pd600.sh gui                   # laptop: toolbar_gui.py — slider (target sent on release), Enable/Disable/
                                    #   Centre/Auto/Reset fault/HOLD, live state; ssh pipe to toolbar_gui_remote.py, no DDS
```

Requirements on pd600: `~/ros2_ws4` on branch `feature/toolbar-service` (adds the
`CORE_PKG` / `CORE_LAUNCH` / `OVERLAY_SETUP` hook to `run_core_um982.sh`).
`pd600:~/ros2_ws5` is build-only: `deploy` overwrites it, and the deployed version is in `DEPLOYED`.

Gotchas:
- ws4's `run_core*.sh` begin with `pkill -9 -f "ros2|..."`, which kills **any** process with "ros2" in its
  command line, `ros2_ws5` paths included. `pd600.sh start` avoids this. If you start by hand, use
  `cd ~/ros2_ws5 && bash run_solbot4_toolbar.sh` from an interactive shell.
- After removing messages from a rosidl package, rebuild it clean (`rm -rf build/<pkg> install/<pkg>`),
  otherwise typesupport fails to import.
- solbot4 firmware (`main_esp32_pio`) has no goal clamp and no stall, overcurrent or comm-timeout
  protection. The node's limits and jam check are the only guard.

### Status / TODO
- [x] Knives <-> TPS calibration (`scripts/knife_tps_cal.py`, 2026-11-12), lift down, shop floor.
  - [ ] cable follower for field nav on top of it (orange cable mask, cable column at the tip rows -> `tps_for`)
  - [ ] check the right end mechanically: knives stop sliding beyond ~+85 mm, stall at TPS 400
  - [ ] re-check outdoors on the courtyard field (ws4 `src/fields/`, which one: house_short?)
- [ ] OAK-D needs a replug after every pd600 reboot: switchable USB power (uhubctl hub or ESP32 load switch)
- [x] pd600 CMOS battery replaced 2026-11-14, BIOS settings unchanged (`docs/pd600_bios_2026-11-14.txt`)
  - [x] After Power Loss = **Power On** (set in F10 setup, 2026-11-14)
- [ ] solbot_main v2 rev C (5 V sensors, ADS1115 module on 5 V): review the new routing in KiCad, check
  Q5/Q6/C24/R35/R40 in JLC's placement preview, order. Confirm the module pin order matches the board labels.

- [x] pd600 ESP32 (`ESP32_solbot_main`, DHCP on `solap`: was .101/.104/.102, **192.168.0.106** on 2026-11-11) runs
  `main_esp32_pio` ws4 (uncommitted: PV hold + per-target drive timer + toolbar min PWM 100), OTA-flashed 2026-11-11.
  The laptop can't reach it; build here, relay through pd600 and resolve the IP there:
  `pio run -e esp32dev_ota` (in `~/ros2_ws4/embedded/main_esp32_pio`), `scp .pio/build/esp32dev_ota/firmware.bin
  ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py pd600:/tmp/`, then
  `ssh pd600 'python3 /tmp/espota.py -i $(getent hosts ESP32_solbot_main.local | cut -d" " -f1) -f /tmp/firmware.bin'`
- [x] Firmware safeguards in `main_esp32_pio` (ws4): toolbar idle until the first `toolbar/set_position`, stall
  (1.5 s, ≤3 counts) and max-drive-time (20 s) trips, `TPS_fault` / `TPS_active` in `ads`. Built, host unit-tested.
  - [x] goal clamp 370..1280 (~4 mm inside the ends), max drive 10 s. OTA-flashed 2026-11-05
- [x] Actuator moves (2nd test 2026-11-05; 1st test had no movement and tripped stall)
- [x] PV SSR 60 s min on/off hold + 42 V hard cut-off in `main_esp32_pio`: OTA-flashed 2026-11-07
  - [ ] commit in ws4; compare the next day's `~/ros2_ws4/logs/pv_ssr/` with 2026-11-07 (up to 58 switches/h)
- [x] Toolbar max-drive timeout (10 s) now counts per target: a goal change > 3 counts restarts it (stall check
  unchanged). OTA-flashed 2026-11-07 together with the PV hold
- [ ] Node: on a firmware-reported fault, `_latch` sends a hold goal, which re-arms the firmware (`TPS_active` true).
  It should leave the firmware idle
- [x] Direction correct, no change needed (`CW`): +right → TPS falls, −left → TPS rises. Both IBT-2 inputs work
- [x] Calibrated: TPS **1300 far left, 350 far right**, toolbar travel **±0.10 m** (200 mm; actuator stroke
  100 mm at 15 mm/s, full stroke ~6.7 s) → 4.75 counts/mm, counts fall going right. Centre 825 counts
- [x] Wire the actuator, `toolbar enable` (holds at the current position, clamped into 370..1280), small `set_position` moves
- [x] IBT-2 pins: GPIO26 RPWM / GPIO27 LPWM (confirmed 2026-11-05)
- [ ] **IMPORTANT — motion interlock:** lift **down** → toolbar may only move while the robot is moving
  (tool in the soil, sliding sideways while stationary would gouge/strain it); lift **up** → always allowed.
  Needs lift state + robot speed in the node (and ideally a firmware-side guard too)
- [x] Laptop slider GUI (`bash pd600.sh gui`)
- [ ] Phone slider via HiveMQ (`outer/toolbar_pos`, `toolbar_outer_bridge`) — running on pd600, get_status round trip via HiveMQ OK; slider move not yet tested
- [ ] Vision loop on solbot4: deploy `tool_slider` + `crop_row_vision` (`PKGS=... bash pd600.sh deploy`), camera topic on solbot4

## Tool slider actuator — IBT-2 on the old main ESP32 board

The sliding toolbar actuator is driven by an IBT-2 (BTS7960) plugged into one of
the spare IBT-2 ports on the old classic-ESP32 main board.

Sources:
- Old wheel firmware (defines the IBT-2 ports): `~/tmp/solbot_arduino/solbot2-2x12vwipersmotors/esp32_master____wheels_diff*`
- Firmware currently on the board: `~/ros2_ws4/embedded/main_esp32_pio` (earlier `main_esp32`)

### Old IBT-2 ports vs current pin usage

| Port (solbot2)   | RPWM | LPWM | Current use (main_esp32_pio)                         |
|------------------|------|------|------------------------------------------------------|
| Front-left wheel | 32   | 33   | taken: steering_act (LEDC ch 0/1)                    |
| Front-right wheel| 25   | 26   | 25 taken: `PV_SWITCH_PIN`; **26 = toolbar RPWM**     |
| Rear-left wheel  | 23   | 22   | 22 taken: I2C SCL (BNO08x + ADS1115, default Wire)   |
| Rear-right wheel | 27   | 13   | **27 = toolbar LPWM**; 13 free                       |
| Lifter           | 19   | 21   | 21 taken: I2C SDA                                    |
| Planter          | 18   | 17   | taken: lift_act (LEDC ch 2/3)                        |
| LAS analog in    | 35   | –    | free (input-only ADC)                                |

Also in use: GPIO34 = steering pot ADC.

### Assignment for the slider

Used (confirmed 2026-11-05), as in ws4 `main_esp32_pio` since `ee2e8b7`:

- RPWM = **GPIO26**, LPWM = **GPIO27**. Neither is a strapping pin.
- LEDC channels **4 / 5** (5 kHz, 8-bit, same as `IBT_2.cpp`)
- Position sensor (TPS): **ADS1115 AIN2** (`TOOLBAR_ADS_CH 2`), published as `TPS` in the `ads` topic

```cpp
#define TOOLBAR_RPWM_PIN 26
#define TOOLBAR_LPWM_PIN 27
#define TOOLBAR_ADS_CH   2
toolbar_act.begin(0, "toolbar_act", TOOLBAR_RPWM_PIN, TOOLBAR_LPWM_PIN, 4, 5, Lin_act_pid::CW);
```

(An earlier plan used RPWM 27 / LPWM 13, the old rear-right wheel port. Not used.)

## WAS (wheel angle sensor) — salvaged BMW position sensor

Under test (2026-10-30) on an ESP32 DevKit V1. Range is about 90°.

| Sensor pin | Function |
|------------|----------|
| 1          | GND      |
| 4          | Signal   |
| 5          | +5 V     |

Pins 2 and 3: unknown / not used.

The sensor runs on 5 V, so its output may go above 3.3 V. Measure the signal
range with a meter before connecting it to an ESP32 ADC pin, and use a divider
(e.g. 10k/20k) if it exceeds ~3.1 V.

## Changelog

Newest first. Toolbar and solbot4-test work. Add an entry with every change.

### 2026-11-17
- **rope_follower `fit_line` is now RANSAC** (best pair by inliers, then fill; weighted refit on inliers).
  Lap 12:55 (observer, tau 0.3) was "ROPE LOST" almost all swath with 6-8 bands found: 2 bands on the sandy
  left knife (rope-coloured in midday sun) pulled the all-points least-squares start off the rope -> no
  inliers. Offline on 200 raw frames 12:55:20-12:56:00: old fit 22 ok, RANSAC 153 ok (rest: <=5 bands while
  lowering, rope end at y ~ -15). Deployed; that lap gives no tracking data (only ~5 s tracked).
- Lap 13:17 (RANSAC, tau 0.3, sunny with hard shadows): rope found on ~1/3 of swath frames, and once
  (13:18:29) RANSAC took 6 bands on the sandy left knife as the rope (u 190) -> TPS 1270 (limit). Cause is
  the colour detector, not the fit: with the 1.5 ms cap the rope is V 98-115 < `rope_v_min` 140; sunlit
  pavers are b* 133-138 (yellower than the rope, 132) and shadows pass a lower V. Offline on 172 rope
  frames: v_min 140 -> 69 fits, v_min 80 + b_rel 8 -> ~112, texture mode 25-48. No colour threshold
  separates the rope in sun + shadow. TODO: shade the camera area (hood, LED light) or a width-selective
  rope detector; reject rope fits on the knives.
- **rope_follower marker observer** (`marker_control: observer`, default; `direct` = old law): knives
  u = c - g*TPS with the short-move gain `knife_gain_px` 1.6 (was the calibration slope 0.6 -> 2-3x
  over-correction, ~1.6-1.9 s limit cycle). c = knives' offset from the markers, filtered over
  `observer_tau_s` 1.0; target = (c - u_goal)/g, no deadband, sent when it moves > `observer_step_counts` 3;
  steers on the last c for `observer_hold_s` 1.0 without markers, then the model. Status: `ctl`, `c_hat`,
  `knife_gain`, knife_src 'observer'. Offline replay of the 10:19 swath: no errors, c drifts 2264->2130->2190
  over the swath (loose joints). Also fixed: status `active`/`new_target` were inside a comment.
- Lap 10:19 (old follower, 1.5 ms exposure cap, 0.56 m/s): frames sharp (rope twist / paver grain visible),
  markers 210/217 frames; swath 6 marker error median 6.0 mm, p90 10.8, 80% within 10 mm (same limit cycle,
  ~1.6 s); acquisition ~6 s with rope dropouts (TPS started at 687).
- **OAK-D auto exposure capped at 1.5 ms** (`config/oakd.yaml` rgb `r_set_auto_exposure_limit: true`,
  `r_auto_exposure_limit: 1500`; also set live). Dry test standing, bright morning (pavers mean / fine noise):
  auto 118 / 11.1, manual 2 ms ISO800 88 / 8.1, 2 ms ISO1600 132 / 8.5, 1 ms ISO1600 87 / 6.4,
  AE limit 1.5 ms 95 / 7.9, AE limit 1 ms 71 / 6.5 - all usable, markers fine. Smear at 0.53 m/s ~0.8 mm
  (was ~20 mm with ~35-40 ms auto exposure on 2026-11-14 afternoon). TODO: verify sharpness while driving
  (raw-frame lap), check brightness in dull light; 1000 us for 1 m/s.

### 2026-11-15
- `rope_plot.py BAG [-o png] [--from HH:MM:SS --to HH:MM:SS]`: time chart of a `ropelog` bag, 4 panels on
  one time axis - rope / predicted rope / knives (pink markers) / knives (TPS model) image column, error mm
  (markers and model), toolbar PWM (`/toolbar/state.duty`·255), TPS measured / follower target / firmware
  goal; grey = lift down, markers seen with the lift up are dropped. Runs on the laptop or pd600.
- Charts of 2026-11-14 runs (`~/rope_logs/*_timechart.png`, `*_swath*.png` on the laptop):
  - `rope_fastfw` (new firmware): 14:54:22-14:55:00 the robot **stood still** with the lift down (stale
    lift_service pause after the flash), the rope fixed at ~690 px - the toolbar still oscillated at ~1.9 s
    period, knives 660..710 px, PWM ~240 the whole time, follower target 880..960 leading TPS by ~0.5 s:
    a pure ROS-side control limit cycle (marker retarget with full gain on 0.5 s-old knife positions).
    Swath driven 14:55:01-14:55:27: markers error ±10..17 mm, same ~2 s swing; markers lost from 14:55:08
    because the **left marker washed out white in sun** (fails S > 40), model fallback then within ~±5 mm.
  - `rope_fast` (old firmware, 0.53 m/s): marker error mostly ±10 mm, irregular, PWM 110..180.
  - `rope_check2` (0.4 m/s, model control): markers show ±18 mm while the model claims ±5 mm.
- System identification from those logs (scratchpad scripts sysid / preview / plant_id / closed_sim):
  - Knives move **1.3-1.9 px per TPS count on short corrections** (standstill 1.3, driving 1.5-1.9) vs
    0.6 in the calibration (= the slider geometry, 0.21 mm/count). Not explained by a play term or by
    slider speed - loose joints along actuator / slider / implement. The follower converted px -> counts
    with 0.6, i.e. every correction 2-3x too big = the limit cycle.
  - Effective target -> TPS dead time ~0.5 s in the loop (fit to the standstill limit cycle; open-loop
    fit 0.3 s new fw, 0.6 s old fw), slider ~60-80 counts/s, ramping down in the last ~45 counts.
  - Rope prediction is not the bottleneck: the rope column changes only 6.5 px rms (2.3 mm) in 0.4 s;
    look-ahead along the fitted rope line 6.0, rope-speed extrapolation (current) 7.2, line slope x speed
    + yaw rate 5.5 px.
  - Closed-loop sim (plant fitted to the real limit cycle, 4 recorded swaths, rms/p90 mm): current 7.3/11.5
    (15/22 if the knife gain is 2.2); gain 1.6 3.8/5.6; offset observer (markers filtered 0.5-1 s, target =
    (c - u_rope)/g) + 0.3 s lead 4.0/5.9 with ~1.5 instead of 2.3 reversals/s.
- **Camera motion blur:** at 0.53 m/s the raw frames are smeared ~50 px (~20 mm) along travel (auto
  exposure ~35-40 ms) - too much for locating individual plants. Need exposure <= ~2 ms at 0.5 m/s, <= 1 ms
  at 1 m/s (1 mm smear). depthai_ros_driver runtime params `rgb.r_set_man_exposure`, `rgb.r_exposure` (us),
  `rgb.r_iso`. TODO: field test of manual exposure 1-2 ms / ISO 800-1600; else LED illumination under a hood.

### 2026-11-14
- pd600 (HP ProDesk 600 G4 DM, BIOS Q22 02.32.00): CR2032 replaced. All 219 BIOS settings dumped before and
  after via `sudo modprobe hp_bioscfg` (`/sys/class/firmware-attributes/hp-bioscfg/`): identical, saved in
  `docs/pd600_bios_2026-11-14.txt`. RTC in UTC, NTP-synced. **After Power Loss = Power Off**: pd600 only
  auto-started on robot power-up because the dead cell let the chipset's RTC-well power-loss bit reset to
  its "power on" default; with a good cell the BIOS setting applies -> set to **Power On** in F10 setup
  (writing it via hp_bioscfg sysfs fails on this BIOS: WMI error 0x4 "Invalid command type"). Saved dump now
  holds the final settings.
- ws4 field BT `field_nav_sim.xml`: PARK line. A field's last line with turn `park` is only turned into: after the
  turn the swath stage ends the field (ClearFieldProgress, field_complete) before LiftDown -> robot waits in front
  of that swath, lift up. `yard_wide_6_3/make_field.py A B PAIRS park` appends swath A as the park line; field
  `yard_wide_6_3_x1p` = 6 S, 3 N, park 6 (loop that always ends ready for the next run).
  bt_navigator reloads XML only on a new filename: `run_field_nav_arc.sh` takes `BT_XML=<path>` (pass
  `.../behavior_trees/./field_nav_sim.xml` once after editing; a core restart also reloads).
- `rope_precision.py` (knife marker logger) now in solbot4_toolbar (was a /tmp script, lost on reboot);
  `pd600.sh ropelog [tag]` / `ropelog_stop`: marker logger (tmux ropemark) + bag (tmux ropelog) ->
  pd600 `~/rope_logs/`.
- Slow test: nav2 `/speed_limit` 0.4 m/s (`ros2 topic pub -t 3 -w 1 /speed_limit nav2_msgs/msg/SpeedLimit
  "{percentage: false, speed_limit: 0.4}"`, lost on core restart; floor = min_linear_vel 0.4).
- Slow lap (0.4 m/s, model knives): swath 6 median |err| 11.3 mm. The knives sat ~35 px (12 mm) AHEAD in the
  direction of travel while driving; the static calibration assumes H/2 = 7 px behind -> +-12 mm by direction.
- rope_follower `knife_source` auto (default): knife midpoint measured from the pink markers on the inner tips
  (rows marker_rows 330..620, V > marker_v_min 215, gap 110..240 px, within marker_max_dev_px 120 of the model),
  target = TPS at image time + err / (du/dT); model fallback when the markers are not found (shade). Rope column
  taken at the markers' row. `px_per_mm` 2.8 (err_mm). Overlay: white rings = markers, `[markers]`/`[model]`.
  Slow lap 13:39 with markers: swath 6 median |err| 2.4 mm, p90 5.5 mm, max 14 mm after the 3 s acquisition,
  86% within 5 mm, 98% within 10 mm (was 11.3 / 15 / 37 mm). Markers lost in the shaded last third -> model
  fallback, TPS targets swing +-50 counts there.
- Starting a run when the robot is already in front of the swath (after a park): write
  `{"current_line_index": 0, "stage": "swath"}` and send start_line -1 (skips the approach; a fresh start_line 0
  approach from that spot tripped the cross-track monitor, 204 -> 319 cm).
- Marker detection (rope_follower.knife_markers, shared with rope_precision.py): H 150-180, S > 40, V > 200
  (params marker_h / marker_s_min / marker_v_min). The "shade" misses of the 13:39 lap were an overexposed
  marker fading to S 43-49, H 154-158 (old S > 60, H >= 158), not shade; red frame stays V 120-180.
  Overlay test: 5/6 found vs 0/6. Status now carries img_t (image stamp).
- On-field calibration: `ros2 run solbot4_toolbar knife_tps_fit.py [BAG_DIR...] [--since YYYYMMDD_HHMM]
  [--csv out.csv]` (default all pd600 ~/rope_logs/rope_*): samples = marker midpoint (re-detected from raw
  camera frames when the bag has them, else the logged /rope_follower/knife_markers) vs TPS at image time
  (fitted camera delay), lift down; fits knife_tps_cal model A (dir term) and B (+velocity), prints a CALS
  entry. `pd600.sh ropelog <tag> raw` also records the camera (~1.5 MB/s) for that.
  First fit (6 runs 11-13/14, 1157 samples): TPS lags the image ~0.28 s (broad minimum 0.2-0.36 s),
  H -7 px, velocity term ~0, rms 17 px (~6 mm; last marker lap alone 14 px) - knife wobble in the joint play
  while driving; the old outdoor set: rms 57 px incl. outliers, +7 / -10 px by direction. Added as
  knife_tps_cal CALS 'field' (not default).
- On-field calibration runs 14:04: 4 laps 0.4 m/s with marker control, bag with raw camera
  (rope_cal4_20261114_140419, 1.1 GB). TPS lag measured by cross-correlation on the rope swaths: 0.15-0.2 s
  (the global fit's 0.6 s was an artefact: the knives' rest offset shifts +-10 px between swaths). Fit with
  `--delay 0.2` -> CALS 'field2' (U0 672.5, U1 -60.63, U2 -2.94, H 6.1; TPS 695..1131), rms 20 px over all
  swaths, 12.8 px within a swath. knife_tps_fit.py: `--eval SET --delay S` checks a set on new bags.
  rope_follower `tps_delay_s` (TPS at image time + delay, extrapolated).
- Check, 2 laps 14:26 with knife_source model, calibration field2, tps_delay_s 0.2 (rope_check2_..._142601):
  calibrated TPS vs the markers (unseen data): field2 median 6.8 mm, rms 8.0, mean +1.0, 77% within 10 mm;
  old outdoor set median 8.0, rms 8.8, mean -1.8, 68%. Rope following on swath 6 (model only): median 5.1 /
  5.7 mm, p90 13.5 / 9.8 (old model at 0.4 m/s: 11.3 / 15; marker control: 2.4 / 5.5).
  Defaults now: knife_source auto (markers, model fallback), calibration field2, tps_delay_s 0.2.
- Toolbar actuator vs rope following (cal4 bag 14:04): firmware main_esp32_pio Lin_act_pid is P-only,
  20 Hz loop: full PWM above 100 counts error (~22 mm), map(5..100 -> min_pwm 100..250), stop < 5 counts;
  TPS sent over MQTT every 4th cycle (~5 Hz). Measured with the knives dragging: duty 0.3-0.5 -> median
  12 counts/s (~3 mm/s), 0.9-1.0 -> 45 (p90 108, max 204); only 8% of the moving time at full duty; ROS
  command step -> TPS moving (seen in ROS) median 0.5 s, p90 0.9 s; distinct TPS values ~2/s.
  -> small corrections crawl; candidate firmware changes: steeper PWM map (full at ~30 counts, floor ~160),
  TPS every cycle (20 Hz), read the ADS before update() (position is one cycle old).
- Faster lap 14:41 (no speed limit, swath 6 median 0.53 m/s, markers control): median 4.2 mm, p90 10.9,
  max 13, 85% within 10 mm (0.4 m/s: 2.4 / 5.5 / 14 mm, 98%).
- FLASHED 2026-11-14 14:52 (ws4 embedded/main_esp32_pio, OTA via pd600, ESP32 at .104; rollback image built
  from the previous source): toolbar full PWM above 30 counts (Lin_act_pid.full_pwm_err, steering/lift keep
  100), toolbar min_pwm 100 -> 160, `ads` published every cycle (ADS_MQTT_SEND_LEAP 3 -> 0: ~17 Hz measured,
  was ~5), sensors read before update() for all three actuators. MQTT needs `-u mark -P pass`.
  Stand step test, lift down (scratch step_test.py): 5 mm step moving after 0.21-0.35 s, within 1 mm
  (5 counts) after 0.57-0.92 s (PWM 230-245; the old map gave ~140); 10 mm 0.49-1.29 s; 20 mm 0.75-1.64 s;
  50 mm 1.54-2.53 s (toward the centre slower); overshoot <= 5 counts, final error <= 4, no hunting.

### 2026-11-13
- GPS hAcc pause threshold 20 -> 25 mm (ws4 `pd600_core.launch.py` default `GPS_HACC_THRESHOLD_MM`; live drive node
  restarted with 25). Reason: a 20.2 mm hAcc pause during the reverse segment of an arc turn cancelled the
  MoveSequence and the BT then finished the field as SUCCEEDED without driving the next swath (ws4 BT bug, open).
- Rope follower logging: `ros2 bag record` in tmux `ropelog` -> pd600 `~/rope_logs/rope_<stamp>/` (status, toolbar
  state, lift, cmd, odom, overlay images, `/rope_follower/knife_markers`). Knife markers (camera-measured knife
  midpoint from the pink tip markers vs rope, err mm @ 2.8 px/mm) from tmux `ropemark` (`rope_precision.py`, scratch),
  also CSV in `~/rope_logs/markers/`. Live lap 14:33: settled err mostly 0.2-1.5 mm, 3-4 mm corrections, one 26 mm jump.
- **Outdoor knife <-> TPS calibration** (courtyard, sunlit, lift down), reference = pink markers stuck on the
- 2 laps on ws4 field `yard_wide_6_3_x2` (6S,3N,6S,3N; make_field.py 6 3 2), 15:09-15:14, no pauses, 4/4 swaths.
  Logs: pd600 `~/rope_logs/rope_2laps_20261113_150917/` (bag, 84 MB) + `markers/precision_20261113_150919.csv`.
  Camera-measured knife-vs-rope err while the rope was seen: lap 1 swath 6 median 7 mm, max 19 mm (61% within
  10 mm); lap 2 swath 6 median 17 mm incl. the acquisition (85 mm -> 0 in 2 s). Tracking oscillates +-15-20 mm
  at ~0.4 Hz: the rope sweeps +-40 px as the robot yaws and the toolbar lags. Markers lost in shade ~45% of frames.
  False rope detections at the rope end / on swath 3 grass drive the toolbar to the limits.
- rope_follower: rope motion prediction (aim at u_rope + v*predict_lead_s, v from a line fit over vel_window_s 0.5 s,
  capped predict_max_px 40; lead 0.3 s, offline best on the 2-lap log - small gain, median 10 -> 9 px), deadband
  8 -> 5 px, smoothing 0.5 -> 0.7 (newest weight). Status adds vel_px_s, u_pred, err_rope_px; overlay orange =
  predicted rope. Measured lag rope -> knives ~0.6 s; dominant part is the toolbar itself: 0.3-0.5 s from a new
  target to motion start, ~120 TPS/s (~26 mm/s) vs rope drift up to 70 px/s.
- Lap with prediction (15:29, 0.55 m/s): swath 6 median |err| 8.5 mm, p90 18 mm - no measurable gain. Rerun at
  0.4 m/s (nav2 /speed_limit, floor = GpsLineFollower min_linear_vel 0.4): rope lost on sunlit pavers although
  clearly visible: afternoon rope b* 130-131 < rope_b[0] 133 (pavers 122). rope_follower: adaptive yellowness
  rope_b_rel 6 (b* >= median of the searched area + 6, floor rope_b_floor 126; 0 = old fixed range). Offline on
  the lost frames: 7-8/8 bands vs 0-6, grass frames still no fit. Field resume trick: write
  `{"current_line_index": 1, "stage": "turn"}` to the field's field_progress.json and send start_line -1.
  knives' inner tips (blue-steel detection fails here: tape above the tips, overexposed ground). Sweep
  -75..+80..-75 mm, 10/10 frames every step, `docs/toolbar_cam_calib/knife_tps_2026-11-13_outdoor/` (steps,
  csv, `knife_tps_sweep.py ... pink`). Fit rms 1.00 px, max 3.1 px: same slope as the shop (-56.1 vs -55.4
  px/100 counts), +15 px offset, joint play 14.6 px (shop 24.8). `knife_tps_cal.use('outdoor')`,
  rope_follower `calibration` default `outdoor` (`calibration:=shop` for the shop floor).

### 2026-11-12
- **`scripts/rope_follower.py` (new): knives on a rope / cable during field nav, model-based (knife_tps_cal,
  knives need not be visible).** Rope per row band: `rope_detect: color` (default; Lab b* 133-146, S<75, V>140,
  courtyard pavers fill 0.9-1.0 vs 0) or `texture` (shop floor, rope same colour as the floor). Robust line fit,
  >= 5 of 8 bands, jumps > 40 px/frame rejected (rope end on grass sent the toolbar to the limit once).
  Commands only while `/lift/state` is down and still (`require_lift_down`); deadband 8 px (+H/2 until the
  first move). `pd600.sh rope | rope_stop | rope_view`, topics bridged to the laptop.
  Shop: settled knife-rope error checked in the image 0.5 mm. Courtyard, field nav `yard_wide_6_3_x1`
  (ws4 `src/fields/`, swath 6 S / swath 3 N, lift down/up per swath): knives within ~1.5 mm of the rope,
  corrections 3-14 mm settle in 1-2 s; over plain grass (swath 3) stays lost, no commands.
  Outdoors the knives look ~9% larger (tip gap 161 vs 148 px): TODO re-run the calibration sweep on the pavers.
- ws4 fields: `yard_wide_6_3` (40 swaths, shuttle 6 S <-> 3 N, 4.62 m apart) and `yard_wide_6_3_x1` (one pair),
  generated by their `make_field.py` (yard_wide indices as in `directed_lines.json`). Navigator loads only
  `*_directed_turns.geojson`.
- Field-nav pauses seen: lift_service "sensor stuck" on a manual lift_up (lift was moving, false trip);
  swath_cross_track_monitor 121 cm > 50 cm on an approach that started next to the swath (turn-in).
- **Knives position <-> toolbar sensor (TPS) calibration, `scripts/knife_tps_cal.py`** (for keeping the orange
  cable between the knives during field nav). Toolbar swept -75..+80..-75 mm in 10 mm steps, two knives'
  inner ground tips detected per settled step (blue-steel mask, split at the column gap between the knives,
  no bar needed); data, sweep script and frames in `docs/toolbar_cam_calib/knife_tps_2026-11-12/`.
  `u_mid(T, dir) = 661.753 - 55.435 x - 1.125 x^2 - dir*24.833/2`, `x = (T - 925)/100`, u_mid = image column of
  the knives' midpoint, dir +1 = last moved toward image right (TPS decreasing). Fit rms 0.98 px, max 2.5 px
  (~0.4 / 1 mm); inverse `tps_for(u, dir)` within 4 counts of every step. Joint play 24.8 px ~ 45 counts
  ~ 9.5 mm: after a reversal the knives stand still until it is taken up. Valid TPS 542..1278.
- **Right end binds:** first sweep went to +115 mm. Beyond ~+85 mm TPS keeps changing but the knives barely
  slide (tilt/rise in the image instead), and the toolbar stalled at TPS 400 (+110 mm), fault latched; sweep
  script now aborts + holds on any fault. Keep the right limit at <= +85 mm until checked mechanically.
- pd600: OAK-D not enumerating after reboot is the camera, not the port: dmesg `device descriptor read/64,
  error -110/-71`, kernel port power cycle ineffective (`port power management may be unreliable`), xhci
  unbind/bind of 0000:00:14.0 does not recover it. Needs a real VBUS cut (uhubctl-capable hub or ESP32-switched
  load switch). pd600 RTC/CMOS battery is dead ("System clock time unset"); boot-loops after a power loss.

### 2026-11-11
- **bar_follower `reference: markers` (new default): two knives, one each side of the crop row, each with a
  yellow marker; keep the red bar midway between the markers.** Visual servo on the image, no knife model:
  markers (yellow H 18-45, S>50, V>130, compact 400-4000 px blobs, rows of the measuring tape rejected) ->
  midpoint; bar centreline at the markers' row; `err = u_bar + center_offset_mm·2.62 - u_mid`. Only after the
  toolbar reached its target and was still `settle_s` 0.6 s: median of `settle_frames` 3 errors, if over
  `deadband_mm` (now 3) -> `target = p + err / gain(p)`, gain = `px_per_m + 2·px_per_m2·p` (new `px_per_m2`
  −2867 from the 11-10 sweep). Joint play / scale errors are taken out by the next iteration. One marker
  hidden: the other ± the remembered spacing. Markers or bar lost 0.5 s -> stop commanding (service holds).
  Clamped to the service limits (asymmetric since the zero move). `reference: model` keeps the 11-07 mode.
- Bar detection: tallest red component + pieces on its line (was: all red pieces merged), `red_v_min` 40 -> 75,
  and in markers mode only columns within one marker spacing of the knives. The rusty frame at the left image
  edge (V 43-62) was passing as the bar (u ≈ 18 px).
- Live test markers mode: bar 9 mm off -> −0.8 mm in 2 moves; chasing the bar to the left end the target hit
  −77.4 mm, but the firmware clamps goals at TPS 1280 (−75.2 mm), so the toolbar stayed "moving" until
  `arrive_timeout_s`. Follower clamp now 5 mm inside the service limits (was 2). After the fix: reversal from
  the left end +27 -> +17.6 (joint play ate half the first step) -> +4.1 -> +0.6 mm.
- **bar_follower `reference: knives` (new default): the knives themselves at ground level, not the markers.**
  The yellow markers sit 5-10 cm above the ground, and their parallax put the "centred" bar 22 mm off the
  knives' real midpoint. Inner wing tip of each knife = extreme of the blue-steel mask on its side of the bar
  centreline (left knife max u+v, right knife max v-u, mean of the 30 most extreme px), rows from the tape
  top down ignored (tape and the shank bolt look bluish), blobs < `knife_min_blob_px` 1500 ignored (a bluish
  dirt spot next to the left tip was taken as the tip now and then, error jumping ~6 mm), tips = median of
  the last 3 frames. Stationary noise: sd 1.8 mm with outliers to −6 -> 0.28 mm, range 0.3-1.7 mm.
- Continuous servo hysteresis: start correcting above `deadband_mm` (3), stop updating the target below
  `release_frac`·deadband (1.5 mm) or when at rest inside the deadband. Without it the loop dithered back and
  forth through the implement joint play (near the target small actuator moves don't move the knives).
  After both fixes: target unchanged for 18 s with a still bar, error 0.1-0.3 mm.
- **bar_follower `servo: continuous` (new default for markers mode):** the stepped loop (move, wait for the
  target + 0.6 s + 3 frames, measure, move) made the toolbar visibly stop-and-go behind a moving bar. Now every
  frame: `target = p(t_image) + servo_gain(0.8) * err / gain(p)`, `p(t_image)` interpolated from the
  /toolbar/state history at the image header stamp (pd600 clock), resent only when it changes > 0.5 mm; held
  when still and |err| <= deadband. `servo: stepped` keeps the old loop. Live: 44-50 mm bar moves followed in one
  smooth motion, converged −0.4..+0.8 mm in 2.7-9.3 s, no stalls.
- **Firmware `TOOLBAR_MIN_PWM` 100 (ws4 `main_esp32_pio`, uncommitted, OTA-flashed 2026-11-11 to .106):**
  `Lin_act_pid::min_pwm` (default 40, steering/lift unchanged) is the bottom of `calculate_pwm`'s 5..100-count
  ramp. A 3.5 mm (17-count) follower step got PWM ~75/255, didn't move with the knives on concrete, and tripped
  `stall` after 1.5 s (latched until reset_fault + auto). After the flash: four 3 mm (12-count) steps both ways
  reached the target, no stall.
- pd600 rebooted again 2026-11-11 and the OAK-D was missing from `lsusb` until replugged (same as 11-10).

### 2026-11-10
- **Toolbar zero re-referenced to the bar's working centre (TPS 923, was 825 = mid-stroke).**
  `config/toolbar.yaml`: same counts at the ends (1300 left, 350 right, 4.75 counts/mm), limits now
  `min_position -0.0794` / `max_position +0.1206`; `center` command and 0 m go to TPS 923. Verified on
  pd600 after deploy + restart: raw 923 reads 0.000 m. toolbar_gui picks the new limits up from `/toolbar/state`.
- **`toolbar_cam.yaml` u0 1071.7 -> 1011.7** for the new zero (u0 − 2907·0.0206). Camera check of the sweep:
  max left (raw 1277) knife ≈800 px vs 815 predicted, back to centre ≈1035 vs 1032. Against the tape the knife moves
  ~1.1–1.18 mm per configured TPS mm (−88 mm at −74.6, +66 at +60.8), consistent with the 11-07 px_per_m (1.11).
- Bar-follower test re-run with `start_bar`: bar at u 788 px, follower live (enable, auto, dry_run false) settled
  at −0.056 m with err −0.7 mm.
- **Follower accuracy sweep** (`measure_knife_bar.py` on pd600: after each settle, knife bottom-left corner
  (blue-steel mask; the 11-07 V<95 mask misses the lit wing) vs red-bar centreline at that row, 2.62 px/mm;
  data + annotated frames in `docs/toolbar_cam_calib/follow_check_2026-11-10/`). Goal offset 30 mm, 13 steps:
  arriving from the right (−p) 29.3–31.7 mm; arriving from the left (+p) 27.3 mm at −46 mm, 24–26 mm around
  0, 21.5 at +43, 14.5 at +90 (+94.5 saturated at `limit_m`). The follower's own err stayed within ±2 mm, so the
  error is the camera model. Fit of knife u vs p (new frame): linear `999 + 2644·p ± 39/2` leaves 6.4 px rms
  (10 px max, curved residuals); quadratic `1009.6 + 2788·p − 2867·p² − 47.8/2·dir` leaves 2.3 px rms (3.8 max,
  < 1.5 mm). Local scale falls from 3130 px/m at −0.06 to 2270 at +0.09 (lens edge + linkage).
  The 11-07 linear model (2907 px/m) was fitted on −0.10…−0.01 m (new frame) only. TODO: quadratic in bar_follower.
- OAK-D on pd600 enumerated at 5 Gbps (USB SPEED: SUPER) on bus 2 port 10. pd600 rebooted once
  unexpectedly with the camera attached: watch for a USB power brown-out.

### 2026-11-09
- **solbot_main v2 rev C: 5 V position sensors and an optional ADS1115 module on 5 V, as on the ws4 toolbar.**
  Hall sensors: POS terminal pin 1 = +5V through a 0 R link (direct, as ws4). The output goes to the ESP ADC through 100k/150k (×0.6, 5 V → 3.0 V) + 100 n +
  BAV99. The ADS1115 breakout module sits in a 1x10 socket (J15), lying flat below the SPARE IDC. It is powered from
  +5V and reads the raw 0..5 V wipers (PGA ±6.144 V, I2C 0x48); SDA/SCL pass 2× 2N7002 level shifters (Q5/Q6) so its
  5 V pull-ups don't reach the ESP. R35/R40 moved above the TURN terminal (their old spots were boxed in).
  Routing: rev B copper locked; segments on re-netted pads dropped, dead stubs pruned (`gen_pcb.py`), new connections
  by FreeRouting (pass4) + `gen/route_net.py` (pass5, 2 nets). Only the nets this change touches differ from rev B.
  DRC: 0 unconnected, no clearance errors. Production: BOM 27 lines (new Basic 0 R C21189, 150k C22807; 1M gone),
  99 placed parts. Firmware: `pot_mv` = sensor supply (default 5000, NVS key `pot_mv5`), ESP reading ÷ `pins::POS_DIV`,
  ADS gain ±6.144 V, ADS auto-detect with fallback to the ESP ADC (`settings {"pos_adc":"auto|esp|ads"}`), channel map
  `pins::ADS_CH`. Builds and the host tests pass; not run on hardware.
- **solbot_main v2 firmware (solbot5): ported the ws4 toolbar and PV changes.** The slider (act 3) now starts idle
  until the first `slider/set_position`. It trips `timeout` after 10 s driving towards one target; a goal change of more
  than `stall_delta` restarts that clock. Soft limits come from the solbot4 measurements (counts 370..1280 = ±0.0958 m;
  calibrate on solbot5). PV: 60 s minimum on and off hold, 42 V immediate cut-off in any mode. `ads` adds `TBS_pwm`,
  `TBS_fault`, `TBS_active`; `act/state` adds `active`. Payloads that aren't JSON are ignored (they used to read as goal 0).
  Host tests extended, all pass; firmware builds. Not run on hardware (board not built yet).
- **Removed `embedded/solbot_main` (v1 board)**; v2 is the only solbot5 controller. v1 is in git history before this commit.

### 2026-11-07
- **Firmware: toolbar timeout per target** (ws4 `Lin_act_pid::command()`, flashed with the PV hold change). The
  follower tripped `ESP32 reports toolbar actuator timeout` at 13:28: the target kept moving with the bar, the motor
  never stopped, and the 10 s continuous-drive limit ran out. Now a new goal (> `stall_delta` = 3 counts, so the
  node's periodic resends don't count) restarts the 10 s clock; the stall reference is not reset, so a jammed
  actuator still trips after 1.5 s. After the flash: reset_fault / enable / auto, follower at target (−0.7 mm).
  OTA: the ESP32 had moved to 192.168.0.102 (DHCP); resolve `ESP32_solbot_main.local` on pd600.
- **PV SSR hold (firmware, not flashed):** `main_esp32_pio` (ws4) PV switch keeps each state >= 60 s
  (`PV_MIN_HOLD_MS`) and still cuts PV at once above 42 V (`PV_HARD_OFF_VOLTAGE`). Reason: on 2026-11-07, with the
  41/40 V thresholds, PV switched up to 58 times an hour, with 1-3 s bounces after reconnects.
- **One-session startup for the bar-follower test:** `bash pd600.sh start_bar [launch args]` ->
  `run_solbot4_bar_follower.sh` -> ws4 `run_core_um982.sh` with `CORE_LAUNCH=core_bar_follower.launch.py`
  (`solbot4_toolbar/launch/`): core_toolbar (pd600_core + toolbar_service + toolbar_outer_bridge) + OAK-D +
  camera domain_bridge (`CAMERA_BRIDGE_CYCLONEDDS_URI`) + bar_follower. Args `use_camera`, `use_bar_follower`,
  `camera_model`, `bar_follower_params`. Replaces `start` + `camera` + `follow` (start_bar refuses if those
  sessions run); `stop` handles both launches. Toolbar still starts disabled, follower in dry_run.
- **`bar_follower`** (`solbot4_toolbar/scripts/bar_follower.py`, params in `config/toolbar_cam.yaml`): keeps the knife's
  left edge `offset_mm` (30) right of a red bar/cable. Red mask -> per-row edge midpoints -> line fit -> bar column at
  v_ref; target `p = (u_bar + offset·2.62 px/mm - u0 ∓ hysteresis/2) / px_per_m`, approach-direction aware, 5 mm
  deadband, published on `/tool_bar_position_cmd` (no PID: the camera is frame-fixed, the firmware closes the loop).
  Bar lost 0.5 s -> stops publishing, `toolbar_service` holds. Starts `dry_run: true`. Status JSON
  `/bar_follower/status`, overlay `/bar_follower/image/compressed` (bridged to the laptop).
  `pd600.sh follow | follow_stop | follow_view`, `view [topic]`. Static check: bar 795 px, toolbar −0.0236 ->
  target −0.075 m; the TPS prediction matched the detected knife corner within 7 px (~3 mm).
- **OAK-D <-> toolbar calibration** (`solbot4_toolbar/config/toolbar_cam.yaml`, data and frames in
  `docs/toolbar_cam_calib/`): lift down, tape on the floor, toolbar swept ±0.08 m, knife bottom-left corner
  auto-detected. `u_impl = 1071.7 + 2907·p ± 20 px` (px, p in m, +p = image right; knife visible only for p <= ~+0.01).
  Ground scale 26.2 px/cm, so the knife moves 1.11 mm per TPS mm (TPS travel scale or knife pivoting).
  **Hysteresis ~40 px = ~15 mm** between approach directions (repeatable to 1 px per direction): implement joint play.
- `toolbar_gui.py` + `toolbar_gui_remote.py`, `bash pd600.sh gui`: tkinter slider GUI on the laptop. The robot graph is
  loopback-only, so the GUI copies `toolbar_gui_remote.py` to pd600 and runs it over ssh (JSON lines: `/toolbar/state`
  out, `/toolbar_control` requests in). Slider sends `set_position` on release; ←/→ ±5 mm (Shift ±1 mm), `c` centre,
  space HOLD; closing the window sends hold if enabled. `deploy` also copies the remote helper.
- `toolbar_outer_bridge` (package `toolbar`, started by `core_toolbar.launch.py`): phone MQTT app via the HiveMQ cloud
  broker, same pattern as ws4 `mqtt_op` outer/steer + outer/speed. `outer/toolbar_pos` `{"data": mm}` (−100..100,
  +right) → `toolbar_control set_position`; `outer/toolbar_cmd` `{"data": "enable"|"center"|"hold"|…}`; replies on
  `outer/toolbar_state` `{pos_mm, target_mm, state, ok, message}`. Coalesces slider bursts (newest value, one call in flight).

### 2026-11-06
- OAK-D on solbot4: `solbot4_toolbar/launch/oakd.launch.py` + `config/oakd.yaml` (depthai_ros_driver, RGB only,
  720p 10 fps, `CAMERA_MODEL` default OAK-D-LITE), `config/camera_bridge.yaml`, `run_solbot4_camera.sh`;
  `pd600.sh camera | camera_stop | view`. `pd600.sh stop` now waits for `core_toolbar.launch.py` only.
  First run: OAK-D-LITE (MXID 19443010A18FDC7D00) on the pd600 front USB-C, USB3 SUPER_PLUS after a replug
  (first plug-in: no Type-C partner detected). 10 Hz on pd600, only ~3 Hz arriving on hp-zbook over Tailscale.
  Replugging re-enumerated the UM982 CH340 (`/dev/um982` → ttyUSB1); GPS kept running.
  JPEG q95 (426 kB frames, ~4 MB/s) saturated the solap WiFi once the laptop joined it (LAN ping 460 ms, viewer
  stalled): `oakd.yaml` sets jpeg_quality 50 (~140 kB, ping 1.5 ms, 8 Hz on the laptop), bridge image QoS best effort.
  `view` runs `oakd_view.py` (rqt_image_view / image_view never subscribed to the bridged compressed topic), under `env -i` (rqt crashed in the VS Code snap shell: GLib-GIO settings schema).
- OAK-D ground footprint (1280x720, fx 912), tape measures on flat concrete:

  | Lift | Camera height (measured) | Ground window | Scale | Optical-equivalent height |
  |---|---|---|---|---|
  | down | 44 cm | 55 × 31 cm, rectangular | 23.2 px/cm (0.43 mm/px), uniform | ~39 cm |
  | up | 64 cm | ~79 × 50 cm, trapezoid | 17.8 (top) → 15.5 (mid) → 12.5 (bottom) px/cm | ~59 cm |

  Lift up the camera pitches (bottom of the image looks farther away) and the lift frame hides the right ~15 %.
  The measured heights are ~5 cm above the optical-equivalent height (measured to the mount, not the lens).
  Lift down is a clean nadir view; lift up needs a tilt/homography correction before metric use.
  Frames: `docs/oakd_ground/lift_down_44cm.jpg`, `docs/oakd_ground/lift_up_64cm.jpg`.
- TODO added: toolbar motion interlock — lift down ⇒ move only while driving; lift up ⇒ always allowed.

### 2026-11-05
- **Toolbar moves, direction correct** (2nd powered test, standalone node, no robot stack):
  `set_position −0.023` (10 mm right): TPS 983 → 937 in ~0.9 s, goal 934 (3 counts ≈ 0.6 mm off, inside the
  firmware deadband), `at_target`. Back to `−0.033` (10 mm left): TPS 937 → 979 in ~1 s, goal 982. No faults.
  `CW` in firmware is correct.
- **First powered toolbar test** (standalone `toolbar_service` on pd600, no robot stack): enable held at TPS 983
  (−0.033 m). `set_position −0.023` (goal 934) drove PWM ~142 for 1.5 s, but TPS stayed at 982–983. The firmware
  tripped `stall`, the node latched `actuator_fault`. **No movement, so the direction is still unknown.**
  The safeguards worked as designed.
- **Toolbar travel corrected to ±0.10 m** (200 mm toolbar travel from the 100 mm actuator stroke): node limits
  ±0.10 m, 4.75 counts/mm. Firmware clamp and timers are unchanged (counts/seconds), comments only, no reflash.
- **Toolbar calibrated and firmware flashed:** TPS 1300 left / 350 right, actuator 100 mm stroke, 15 mm/s.
  `solbot4_toolbar/config/toolbar.yaml`: ±0.050 m (corrected to ±0.10 m, see above), counts 1300/350,
  tolerances 2 mm and jam 5 mm.
  Firmware: clamp 370..1280, max drive 10 s. OTA-flashed: boots idle (`TPS_active` false, PWM 0). Before the flash,
  the old firmware was again driving PWM 250 (goal 1313, TPS 495, after the toolbar was moved by hand to measure).
  Deployed to pd600.
- **Firmware toolbar safeguards** (ws4 `main_esp32_pio`, `Lin_act_pid::enable_safeguards`, opt-in, so steering and lift
  are unchanged): idle at boot until the first command, goal clamp, stall trip (driving, ≤3 counts of movement in
  1.5 s) and timeout trip (driving > 20 s). Trips stop the motor and set goal := position, and a new
  `toolbar/set_position` re-arms. Null or invalid `data` is ignored, where before it read as goal 0. Motor is stopped
  when exactly on target (the old code kept the previous PWM). `ads` gains `TPS_fault` and `TPS_active`.
  `toolbar_service` latches `TPS_fault` stall/timeout as `actuator_fault`. Not flashed yet: waiting for the
  end-of-travel TPS values for the clamp.
- **ESP32 OTA flash** (`ESP32_solbot_main` @ 192.168.0.104): ws4 `main_esp32_pio` `ee2e8b7`. Before the flash it was already
  publishing `TPS`, but was driving the toolbar output at PWM 250 (goal 931, TPS 1313). After the reboot it holds:
  goal = TPS = 1313, PWM 0.
- **Toolbar IBT-2 pins confirmed:** GPIO26 RPWM / GPIO27 LPWM (matches `main_esp32_pio`). README hardware
  section corrected (it had said 27/13).
- **Toolbar runs on solbot4 from ws5** (`bfe66d9`): `toolbar_service` moved from `control` into its own
  `toolbar` package. Interfaces moved from `solbot5_msgs` into `toolbar_msgs`. New `solbot4_toolbar` package
  (launch + solbot4 calibration), `pd600.sh` (laptop: deploy/start/attach/stop/status/state/toolbar/ros2),
  `run_solbot4_toolbar.sh` (pd600). Node takes ROS params only (dropped the `config.yaml` `toolbar:` section).
  ws4 `feature/toolbar-service` reduced to the `run_core_um982.sh` overlay hook (ws4 `27420a8`).
  Deployed and built on pd600; not yet started on the robot.
- **`firmware` parameter** (in `bfe66d9`, first in the ws4 port `4a126bf`): adds the solbot4 `main_esp32_pio` protocol (`TPS` counts,
  `toolbar/set_position`, counts↔metres calibration, pot-rail check). Monitor-only mode downgrades every
  condition to LOW and never latches. MQTT disconnects are logged.
- **`toolbar_service` node** (`6ad17d5`): lift_service-style toolbar controller for the solbot5 `solbot_main` board.
  Soft limits, auto stream with 1 s hold-on-timeout, jam detection, firmware stall/OC/pot faults latched until
  `reset_fault`, e-stop handling, `/toolbar/state`. Tested against fake ESP32s for both firmwares.

### Earlier
- `8fa20e3` crop_row_vision detection, tool slider control loop fixed (PID instead of feedforward)
- `0ff59b8` toolbar re-enabled in the sim, green crop rows
