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
- [ ] solbot_main v2 rev C (5 V sensors, ADS1115 module on 5 V): review the new routing in KiCad, check
  Q5/Q6/C24/R35/R40 in JLC's placement preview, order. Confirm the module pin order matches the board labels.

- [x] pd600 ESP32 (`ESP32_solbot_main`, DHCP on `solap`: was .101/.104, **192.168.0.102** on 2026-11-07) runs
  `main_esp32_pio` ws4 (uncommitted: PV hold + per-target drive timer), OTA-flashed 2026-11-07.
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
