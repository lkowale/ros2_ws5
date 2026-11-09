# solbot_main v2 (rev B / rev C) — compact ESP32-S3 main controller, JLCPCB Basic parts

> **rev C (2026-11-09): production files regenerated and checked by script. Two manual checks
> are left before ordering** (see the end of this list).
> * **Hall position sensors on 5 V** (as on the ws4 toolbar): POS terminal pin 1 = +5V through a 0 R
>   link (R35/R40/R45/R50; fit 10–47 R there if you want a shorted cable not to pull +5V down, at the
>   cost of supply volts). The sensor output goes to the ESP ADC through a 100k/150k divider (5 V reads
>   as 3.0 V; an open wiper reads 0) + 100 n + BAV99 clamp. The rev B 1 k from 3V3A and the 1M
>   wiper pull-down are gone. R35/R40 (A1/A2 supply) moved to the free strip above the TURN
>   terminal: their old spots could only be reached through the 3V3A pull-up row.
> * A 1x10 socket (J15, hand-solder) for an optional **ADS1115 breakout module** plugged in flat,
>   **powered from +5V like on ws4** and reading the **raw 0..5 V wipers** (PGA ±6.144 V). SDA/SCL
>   reach it through two 2N7002 level shifters (Q5/Q6, gate 3V3A), so the module's own 10k pull-ups
>   to 5 V never reach the ESP pins. C24 = 100 n on its VDD.
> * Routing: the rev B copper is kept apart from the segments that ended on re-netted pads (dropped)
>   and the dead stubs this left (pruned at the end). Only the nets this change touches differ from
>   rev B. New connections: FreeRouting (`solbot_main_v2_pass4.ses`) + `gen/route_net.py` for the two
>   it couldn't close (`solbot_main_v2_pass5_*.ses`).
> * DRC: 0 unconnected, 0 clearance errors, 0 shorts. The other warnings are the same as rev B, plus
>   headless-only `lib_footprint_issues` for J15/C24/Q5/Q6.
> * `hardware/production/` was regenerated with the Fabrication Toolkit plugin (your saved
>   options). BOM: 27 lines, all with LCSC numbers. New: 0 R (C21189) and 150k (C22807), both Basic;
>   1M is gone; Q5/Q6 joined the 2N7002 line (C8545). Positions: 99 SMD parts. J15 is in neither file.
> * **Still to do by hand:** look over the new routing in KiCad (resistor block, around R35/R40
>   above the TURN terminal, under the ADS socket), and check Q5/Q6/C24/R35/R40 in JLC's placement
>   preview. Order as before: 2-layer, 1 oz. Upload
>   `production/solbot_main_v2.zip`, `production/bom.csv` and `production/positions.csv`.

The ESP32-S3 main controller for solbot5. It replaced v1 (a 190 × 155 mm board with a full
on-board 4-pack power path, removed from the repo 2026-11-09; see git history before that date).
This board has **no battery switching on board**. It

* runs from the robot's **12 V rail**
* **monitors the main battery bus voltage** (10S…20S, up to ~95 V) and its own 12 V rail
* switches **PV on/off through external DC solid-state relays** (2 driver outputs)
* drives **4 IBT-2 (BTS7960)** modules on **2×5 IDC sockets with the v3 pinout**, plus
  current sensing on the previously unused IDC pins 5/6
* reads the **4 position pots directly on the ESP32-S3 ADC1**, plus the **BNO085** on I2C

Board: **120 × 85 mm**, 2 layers, 1 oz is fine.

## JLCPCB assembly (rev B)

Every SMD part was checked on jlcpcb.com on 2026-10-16: **25 Basic lines and 1 Preferred
Extended (SMBJ18A TVS), 0 Extended**, so there are no extended-part fees with Economic
assembly.

**Production files: KiCad → Fabrication Toolkit (JLC plugin)** writes `hardware/production/`
(Gerber/drill zip, `bom.csv`, `positions.csv`). It's been checked against this board:
* Every connector is labelled on the silkscreen: function (TURN / LIFT / SLIDE / SPARE,
  12V IN, BUS SENSE, SSR1/2 PV, E-STOP, IMU) and pins (`+ W -`, `+ -`, `NC`, IMU pin names),
  plus an IBT-2 IDC pin legend at the top right.
* Every SMD footprint has an `LCSC` property (copied from the schematic field), so the
  plugin's BOM has all 26 lines with the right part numbers.
* Hand-soldered through-hole parts (12 V, BUS, SSR, E-stop and pot terminals, the IDC
  sockets, JST-XH, and the DevKit sockets) are "not in BOM" in the schematic and "exclude
  from BOM / position files" on the PCB, so they don't appear in either file.
* Keep the default options ("auto translate" on: rotation fixes for SOT-23/SOT-223/SOIC).
* In JLC's placement preview, check the polarity of D1/D3 (SS34), D2 (TVS) and the LEDs.
  The plugin has no rotation entry for D_SMA/D_SMB/LED footprints. If one is wrong, add an
  `FT Rotation Offset` field (e.g. 180) to that symbol.
* Don't rename the `LCSC` field or add a field called `JLC`/`MPN`: the plugin reads those as
  part numbers.

`hardware/bom_jlc.csv` (from `build.py`) is only a cross-check. Always order with the plugin's
`production/` files.

Changes from rev A that this required (none of the replaced parts is Basic):

| rev A (Extended) | rev B (Basic) |
|---|---|
| AP63205 buck + 4.7 µH inductor | 2× AMS1117-5.0 (logic / ESP32, like v3's 2× 7805), 2×22 Ω pre-drop on the ESP rail |
| AP2112K-3.3 | XC6206P332MR-G |
| ADS1115 | ESP32-S3 ADC1 (GPIO1–10 all used), 16× oversampled; rev B: pot top from 3.3 V through 1 k. Rev C: 5 V sensors, wiper ×0.6 divider, optional ADS1115 module back on a socket |
| 74HCT244 + /OE gate | 2× 74HC14 (inverting → PWM driven active-low, 10 k pull-ups keep it off at boot); E-stop now gates the IBT-2 **R_EN/L_EN** (IDC 3/4) through 2× 2N7002 |
| BAT54S | BAV99 |
| PTC fuse, 47 µF electrolytic | removed (fuse the 12 V feed externally, ≤ 2 A) / 2× 10 µF 1206 |
| 0603 green/yellow LEDs | KT-0805G / KT-0805Y |

Thermal: at about 150 mA the ESP-rail AMS1117 dissipates about 0.7 W (the 22 Ω resistors take
about 0.25 W). Both regulator tabs have copper areas. If the robot has a 5 V rail, the board
could be fed from that instead.

```
solbot_main_v2/
├── hardware/      KiCad 7 project, bom.csv, schematic PDF (production files: JLC plugin)
│   └── gen/       Python generators (design.py = single source of truth)
└── firmware/      PlatformIO, ESP32-S3 (env solbot_main_v2), test/host/run.sh
```

## PV switch with the DELIXI CDG1-1DD/25A SSRs (proposal)

DC SSRs like the CDG1-1DD (input 3–32 V DC, load 12–220 V DC 25 A) have a MOSFET output.
A MOSFET **conducts backwards through its body diode when off**. A single SSR between PV
and the battery therefore either can't stop charging (wired one way), or lets the battery
back-feed the panel at night (wired the other way). With your **two SSRs, wire the outputs
in series back-to-back**:

```
            SSR1 (output)          SSR2 (output)
 PV+ ──fuse──[+ ····· −]────────────[− ····· +]──fuse── BUS+ (battery)
 PV− ─────────────────────────────────────────────────── GND

 board J "SSR1": pin1 +12V → SSR1 input +, pin2 → SSR1 input −
 board J "SSR2": pin1 +12V → SSR2 input +, pin2 → SSR2 input −
 (firmware default: SSR2 mirrors SSR1)
```

* Off: each body diode is blocked by the other SSR, so there's no charging and no night
  back-feed.
* On: two MOSFET channels in series, with no diode drop.
* **Check first:** with an SSR unpowered, use the multimeter diode test across its output.
  If it reads about 0.5 V from − to +, it has the body diode and the back-to-back pair is
  required. If it blocks both ways, one SSR is enough (set SSR2 to manual and use it for
  something else).
* About 10 A (400 W @ 40 V) is fine for 25 A parts, but give them a heatsink plate.
* If you only want to use one SSR: one SSR plus a blocking Schottky
  (e.g. 45 V / 40 A, on a heatsink) also works, at the cost of ~0.5 V of drop.

Charge control is the v3 behaviour, scaled by cell count. At 10S, PV goes **off at
≥ 41.0 V** and **back on below 40.0 V**, with a 30 s minimum off-time. PV is forced off
whenever the bus reading is implausible (sense wire disconnected).

## Hardware

| Block | Notes |
|---|---|
| 12 V in | MKDS 5.08 terminal → SS34 reverse-polarity diode → SMBJ18A TVS (fuse externally) |
| 5 V | 2× AMS1117-5.0: `5V_ESP` (DevKit, via SS34 USB back-feed block) and `+5V` (IBT-2 logic, 74HC14, IMU) |
| 3.3 V | XC6206P332MR → I2C pull-ups, clamps, level-shifter gates (rev B: also pot excitation) |
| Bus monitor | `BUS SENSE` terminal → 470k+470k / 30k (2.94 V @ 95 V) → GPIO9, BAV99 clamp. **Bus GND must be common with the 12 V GND.** |
| 12 V monitor | 100k / 10k → GPIO10 |
| SSR drivers ×2 | 2N7002 low-side sink for the SSR input (powered from +12 V); yellow LED = SSR on |
| Actuators | 2× 74HC14 (3.3→5 V, inverting) → 4× IDC 2×5 RPWM/LPWM |
| E-stop | GPIO39 → NC loop → 2N7002 → 2N7002 → `ACT_EN5` = IBT-2 R_EN/L_EN (fit a wire bridge if unused) |
| IMU | JST-XH 6: 5V GND SCL SDA INT RST, BNO085 @0x4A |

### IBT-2 IDC 2×5 pinout (same as v3; a flat cable maps 1:1 onto the IBT-2 header)

| IDC pin | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| IBT-2 | RPWM | LPWM | R_EN (**EN gate**) | L_EN (**EN gate**) | **R_IS** | **L_IS** | VCC (+5V) | GND | n.c. | n.c. |

v3 left 5/6 unconnected. v2 joins them into a 1 k sense resistor (1.18 V at 10 A),
read on ESP ADC1. Don't fit that 1 k resistor if your IBT-2 modules already have one.

### GPIO map (ESP32-S3-DevKitC-1)

| Function | GPIO |
|---|---|
| POS A1…A4 (ADC1) | 1, 2, 3, 4 |
| IS A1…A4 (ADC1) | 5, 6, 7, 8 |
| BUS_VS / V12_SNS (ADC1) | 9 / 10 |
| RPWM/LPWM A1, A2, A3, A4 (active-low) | 11/12, 13/14, 15/16, 17/18 |
| SSR1 (PV) / SSR2 | 21 / 47 |
| I2C SDA / SCL | 41 / 42 |
| ACT_EN / ESTOP_SENSE | 39 / 40 |
| IMU_RST / IMU_INT / LED_ERR | 19 / 20 / 46 |

GPIO19/20 are used, so program the board through the DevKit's **UART** USB port.

## Firmware

```sh
cd firmware
pio run -e solbot_main_v2 -t upload
OTA_HOST=<ip> pio run -e solbot_main_v2_ota -t upload
test/host/run.sh       # actuator + PV logic tests (after one `pio run`)
```

Tasks: **control** at 50 Hz (BNO085, positions, IS currents, PWM), **pv** at 10 Hz (bus and
12 V voltage, SSR control), **loop()** for WiFi/MQTT/OTA/telemetry. The control loops never
depend on the network.

### MQTT API

v3 topics are unchanged (ROS `steering.py`, `imu_bridge.py`, `esp32_power_monitor.py` keep working):

| Topic | Dir | Payload |
|---|---|---|
| `steer_position` | sub | `{"data": legacy}`. Legacy counts are the old ADS1015 scale (ratio × 1666.7), so `steering.yaml` stays valid |
| `lift/set_position` | sub | `{"data": legacy}` |
| `motor_direction` | sub | toggles the steering direction |
| `ads` | pub 10 Hz | `WAS, WAS_pwm, WAS_pos_goal, LAS, LAS_pwm, LAS_pos_goal` (+ `TBS`, `TBS_goal` in metres, `TBS_pwm` 0..255, `TBS_fault`, `TBS_active`, `estop`) |
| `imu` | pub 5 Hz | `orientation[4], heading_accuracy, angular_velocity[3], linear_acceleration[3]` |
| `esp32/power` | pub 1 Hz | `voltage, pv_on, v12, low, mode, reason, ssr2_on, ssr2_mode` |
| `esp32/diag` | pub | loop gaps, E-stop events, `pv_switch` on/off |
| `<dev>/config`, `get_config`, `enter_ap` | sub | as v3 (`config_status` also contains actuator and PV configs) |

New:

| Topic | Payload |
|---|---|
| `slider/set_position` / `slider/position` | `{"data": metres}` (toolbar slider, left end = −0.10 m) |
| `<dev>/act/<1-4\|name>/set` | `{"pos": units}` or `{"raw": legacy}` or `{"ratio": 0..1}` |
| `<dev>/act/<1-4\|name>/config` | partial update of `name, enabled, invert, r_min, r_max, u_min, u_max, deadband, slow_zone, pwm_min, pwm_max, pwm_slew, i_limit, oc_ms, stall_delta, stall_ms, cmd_timeout_ms, max_drive_ms, idle_at_boot, sensor_check, pot_mv` (saved to NVS) |
| `<dev>/act/reset`, `<dev>/act/enable {"data":0/1}` | clear faults / software enable |
| `<dev>/act/state` (pub 5 Hz) | per actuator `pos, goal, ratio, duty, amps, fault, active`; `pos_src` (`esp`/`ads`) |
| `<dev>/power/cmd` | `{"pv":"auto\|on\|off"}`, `{"ssr2":"mirror\|on\|off"}` |
| `<dev>/power/config` | `cells, v_chg, v_chg_hyst, v_hard, v_low, v_min_valid, min_on_s, min_off_s, v_scale, v12_scale` (NVS) |
| `<dev>/settings` | `imu_hz, ads_hz, pot_mv, pos_adc` (`pot_mv`: default pot-top voltage, see below; `pos_adc`: `auto` \| `esp` \| `ads`, see "Pot ADC") |

`<dev>` defaults to `ESP32_solbot_main`. Payloads that aren't valid JSON are ignored.

### Pot ADC (ESP32 ADC1 or optional ADS1115)

Each (Hall) position sensor is fed from **+5V through a 0 R link** (POS terminal: + W −). Its output goes through
a 100k/150k divider (×0.6, so 5 V → 3.0 V) with 100 n and a BAV99 clamp to the ESP32-S3 ADC1; the
firmware divides by `pins::POS_DIV` to get the wiper voltage back. It works in a ratio
`r = wiper mV / pot_mv`, so `pot_mv` must be the **sensor supply voltage** at the terminal (default
5000; measure it). Set the global `settings {"pot_mv":…}` or per
actuator `act/<n>/config {"pot_mv":…}` (0 = use the global one). Range 3000–5500. The NVS key is
`pot_mv5`, so an old 3 V value isn't loaded. The legacy WAS/LAS counts match v3 only when this is right.
Hall outputs are low-impedance, so the divider stays linear. Many only swing about 0.5–4.5 V:
calibrate `r_min`/`r_max` on the machine. The 150k to GND makes a disconnected output read 0.

Rev C adds a 1x10 socket **J15** for an optional **ADS1115 breakout module** (the common blue one,
about 28 × 17.6 mm, header `VDD GND SCL SDA ADDR ALRT A0 A1 A2 A3`), set up like the ws4 toolbar:
**VDD = +5V, inputs = the raw 0..5 V wipers**, PGA ±6.144 V (0.1875 mV/LSB). The module's 10k pull-ups
go to its VDD, so SDA/SCL pass two 2N7002 level shifters (Q5/Q6, gate on 3V3A; ESP side keeps its
4.7 k pull-ups to 3V3A). It lies **flat**, with the header nearest the SPARE IDC and the body over the
area above the pot terminals (Q5/Q6 sit under it). The module's print is then upside down: **match the
pin names on the board (A3 … VDD, left to right) with the names on the module**. ADDR is tied to GND,
so it sits at 0x48. Module input A0 = A4 wiper, A1 = A2, A2 = A3, A3 = A1 (PCB routing order; the
firmware maps it in `pins::ADS_CH`). `pos_adc`:
* `auto` (default): ADS if it answers at 0x48, else ESP ADC1. Re-probes every 5 s; falls back to
  the ESP ADC after 10 failed reads in a row.
* `esp` / `ads`: force one (`ads` still falls back to the ESP ADC if the module doesn't answer).
`config_status` shows `pos_adc`, `pos_src`, `ads_found`, `ads_err`; `act/state` shows `pos_src`.
To compare the two on the bench, switch `pos_adc` and watch `ratio` in `act/state`.
Neither path is ratiometric: a drifting +5V shows up as position error (`pot_mv` is fixed).

### Actuator safeguards

Every actuator has soft limits (goals are clamped to `r_min..r_max`), over-current and stall
trips (driving but moved less than `stall_delta` for `stall_ms`) and a pot sanity check.
A new goal clears a stall, over-current or timeout fault. Two more are on for the slider only by default
(ported from the solbot4 toolbar firmware, ws4 `main_esp32_pio`):

* `idle_at_boot`: no drive, not even holding, until the first command.
* `max_drive_ms` (10 s, about 1.5× a full stroke): driving longer than that towards **one**
  target trips with fault `timeout`. A goal change larger than `stall_delta` restarts the
  clock, so a streamed, slowly moving target (vision follower) doesn't trip. Resending the same
  goal doesn't restart it, and the stall reference is never reset by new goals.

Slider defaults come from the solbot4 measurements (2026-11-05): legacy counts 1300 = left
end, 350 = right end, 4.75 counts/mm. The soft limits are counts 370..1280 (about 4 mm off
the end stops), which is `r_min=0.222` (+0.0958 m) and `r_max=0.768` (−0.0958 m).
**Calibrate on this robot.**

### PV control (auto mode)

Bang-bang CV like v3: PV off at `cells × v_chg` (41.0 V at 10S), back on below
`cells × (v_chg − v_chg_hyst)` (40.0 V). Each state is held for at least `min_on_s` /
`min_off_s` (60 s / 60 s), because v3 switched up to 58 times an hour without a hold
(2026-11-07). Above `cells × v_hard` (42.0 V) PV is cut immediately in any mode, ignoring the
hold. Manual `on` ignores the hold times but still stops at `v_chg`. PV is never switched on
when the bus sense reads open (`v_min_valid`).

### Calibration (once per robot)

1. Voltage: compare `voltage` with a multimeter and set `power/config {"v_scale": …}`.
2. Actuators: jog to each mechanical end with `act/<n>/set {"ratio":…}`, read `ratio`
   from `act/state`, then set `r_min/r_max` a little inside the ends, plus `u_min/u_max`
   in your units. Steering and lift default to legacy counts; the slider to metres.
3. Direction: if an actuator runs away from its goal, set `{"invert":true}`.

## Regenerate

```sh
cd hardware/gen
python3 build.py
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py place
(cd .. && env -i PATH=/usr/bin:/bin HOME=$HOME timeout 1800 \
   java -jar freerouting-2.0.1.jar -de solbot_main_v2.dsn -do solbot_main_v2.ses -mt 4 --gui.enabled=false)
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py finish
```

Adding parts to a routed board, or changing a pad's net (rev C): `place` (existing footprints don't
move), then route only the open connections with all other copper locked. `build_copper` imports the
base session, **drops the segments that end on (or touch) a pad whose net changed**, then imports the
sessions in `SES_INCREMENTAL` in order. At the end of `finish` it **prunes dead stubs** (segments with a
free end are trimmed back to their last T-junction, or removed; single-connection vias go too).

```sh
cd hardware/gen
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py place
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py export_incremental pass4   # -> solbot_main_v2_pass4.dsn (stubs kept)
(cd .. && env -i PATH=/usr/bin:/bin HOME=$HOME timeout 1800 \
   java -jar freerouting-2.0.1.jar -de solbot_main_v2_pass4.dsn -do solbot_main_v2_pass4.ses -mt 4 -mp 40 --gui.enabled=false)
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py finish                     # imports pass4 (+ pass5_*)
```

FreeRouting doesn't always finish (or sometimes writes an empty session). For each connection it
leaves open, `gen/route_net.py` does a grid search over the real copper (Default class rules) from
one pad to **another island** of its net, and writes a session. `finish` picks up
`solbot_main_v2_pass5_*.ses` in name order. Run it on a board finished with `NO_PRUNE=1`, so the
stubs are still there to connect to, and finish again after each net (rev C: +5V at R50, A2_POS at R41):

```sh
env -i NO_PRUNE=1 PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py finish
env -i PATH=/usr/bin:/bin HOME=$HOME python3 route_net.py ../solbot_main_v2.kicad_pcb +5V R50 1 \
   ../solbot_main_v2_pass5_1__5V.ses 35 20 90 55
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py finish
```

## Status / check before ordering

* Schematic: KiCad's netlist matches `design.py` exactly (rev C: 72 nets, adds SDA5/SCL5). The firmware pin map was
  checked against the board netlist (27/27 match).
* PCB: fully routed. DRC finds 0 unconnected items, 0 clearance errors and 0 shorts. What's
  left are silkscreen warnings, plus `lib_footprint_issues`, which only appear in the
  headless run. The signal routing is autorouted, so look it over in KiCad before ordering.
  Rev C: 5 V sensors + ADS module routed incrementally (rev B copper locked); same DRC result.
* Firmware builds (RAM 17 %, flash 26 %). The host tests pass (and were mutation-checked).
  2026-11-09: ported the ws4 toolbar safeguards (per-target max-drive timeout, idle at boot,
  slider soft limits) and the PV 60 s on/off hold + 42 V hard cut-off; tests extended.
  **It hasn't run on hardware yet.**
* **Verify** the DevKitC-1 header row spacing on your module (the footprint uses 25.4 mm;
  Espressif's own footprint says 22.86 mm).
* **Verify** the SSR body-diode behaviour (see above) before wiring PV.
