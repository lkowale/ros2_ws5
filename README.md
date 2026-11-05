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
bash pd600.sh start                 # core stack (run_core_um982.sh, field_nav_hw params) + toolbar, tmux "solbot4"
bash pd600.sh attach                # watch (Ctrl-b d detach, Ctrl-c stop)
bash pd600.sh state                 # one /toolbar/state (raw_position = TPS counts)
bash pd600.sh toolbar enable        # set_position 0.02 | center | hold | auto | reset_fault | disable
bash pd600.sh stop | status
bash pd600.sh ros2 <args...>        # any ros2 command on the robot
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

- [x] pd600 ESP32 (`ESP32_solbot_main`, 192.168.0.104 on `solap`) runs `main_esp32_pio` ws4 `ee2e8b7`, OTA-flashed 2026-11-05.
  Re-flash: `cd ~/ros2_ws4/embedded/main_esp32_pio && OTA_HOST=192.168.0.104 ~/.platformio/penv/bin/pio run -e esp32dev_ota -t upload`
- [x] Firmware safeguards in `main_esp32_pio` (ws4): toolbar idle until the first `toolbar/set_position`, stall
  (1.5 s, ≤3 counts) and max-drive-time (20 s) trips, `TPS_fault` / `TPS_active` in `ads`. Built, host unit-tested.
  - [x] goal clamp 370..1280 (~4 mm inside the ends), max drive 10 s. OTA-flashed 2026-11-05
- [x] Actuator moves (2nd test 2026-11-05; 1st test had no movement and tripped stall)
- [ ] Node: on a firmware-reported fault, `_latch` sends a hold goal, which re-arms the firmware (`TPS_active` true).
  It should leave the firmware idle
- [x] Direction correct, no change needed (`CW`): +right → TPS falls, −left → TPS rises. Both IBT-2 inputs work
- [x] Calibrated: TPS **1300 far left, 350 far right**, toolbar travel **±0.10 m** (200 mm; actuator stroke
  100 mm at 15 mm/s, full stroke ~6.7 s) → 4.75 counts/mm, counts fall going right. Centre 825 counts
- [x] Wire the actuator, `toolbar enable` (holds at the current position, clamped into 370..1280), small `set_position` moves
- [x] IBT-2 pins: GPIO26 RPWM / GPIO27 LPWM (confirmed 2026-11-05)
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
