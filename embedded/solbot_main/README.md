# solbot_main v4 — ESP32-S3 main controller

> **⚠️ Superseded — do not order this board.** The board to order is `../solbot_main_v2`
> (compact, JLC Basic parts, ready to order). This v1 board (190 × 155 mm, on-board 4-pack
> power path) is kept as a reference design only. Its parts were never mapped to JLC numbers.

Successor of the v3 board (`ros2_ws4/embedded/main_esp32_pio`, EasyEDA schematic
`Schematic_solbo3_2026-10-15/esp32.svg`). One board that

* manages power: up to **4 battery packs (10S … 20S)** + **PV input**, each with
  voltage & bidirectional current measurement and its own switch; PV charging control
* drives **6 linear actuators** through external **IBT-2 (BTS7960)** modules, closed loop
  on potentiometer feedback, with per-actuator current sensing
  (A1 steering, A2 tool lift, A3 tool-bar slider, A4–A6 spare)
* reads the **BNO085** IMU
* talks MQTT over WiFi, with the same topics and payloads as v3

```
embedded/solbot_main/
├── hardware/               KiCad 7 project (generated, then editable in KiCad)
│   ├── solbot_main.kicad_pro / .kicad_sch (+4 sub-sheets) / .kicad_pcb
│   ├── lib/                ESP32-S3-DevKitC-1 symbol + socket footprint
│   ├── bom.csv
│   └── gen/                Python generators (design.py = single source of truth)
└── firmware/               PlatformIO (ESP32-S3, Arduino)
    └── test/host/          host unit tests for actuator + power logic
```

## Hardware

### Block diagram

```
 BAT1+ ─[1mΩ]─┬─[Q◄►Q]─┐            BUS+ ──► loads (drive, 12 V DC-DC, lasers…)
 BAT2+ ─[1mΩ]─┼─[Q◄►Q]─┤
 BAT3+ ─[1mΩ]─┼─[Q◄►Q]─┼── BUS+ ── 2×100 µF, TVS, divider
 BAT4+ ─[1mΩ]─┼─[Q◄►Q]─┤
 PV+   ─[1mΩ]─┼─[Q◄►Q]─┘            each channel: INA241A2 (I) + 1031k/33k divider (V)
              │
              └─ES1D─► VAUX (diode-OR of all inputs) ─► LM5164 ─► +5V ─► AP2112 ─► +3V3A
                                                               └► AP2112 ─► +3V3_POT (pots)
 ESP32-S3-DevKitC-1
   I2C-A: BNO085, ADS1115 0x48/0x49 (6 pot positions, VREF, 5V)
   I2C-B: ADS1115 0x48/0x49/0x4A (pack/PV/bus V+I, temp), TCA9535 (pack switches)
   MCPWM 12 ch ─► 74HCT244 (5 V, /OE = E-stop gate) ─► 6× IBT-2 header
   ADC1 ×6 ◄── IBT-2 R_IS‖L_IS (1 kΩ)
```

### Design decisions

| Topic | Choice | Why |
|---|---|---|
| Pack/PV switch | Back-to-back P-FETs, SQM120P10_10M1L (−100 V, 10 mΩ, D2PAK), high side | blocks current in both directions, pack negatives stay on common GND, simple gate drive |
| Gate drive | MMBTA42 current sink (~2 mA) + 12 V zener + 10 k | Vgs = −12 V **independent of bus voltage**, so the same circuit covers 10S and 20S |
| Current sense | 1 mΩ Kelvin shunt (WSK2512) + **INA241A2** (20 V/V, −5…110 V CM, REF = Vs/2) | bidirectional (charge/discharge), ±82 A range, 6 mA/LSB at the ADS1115 |
| Voltage sense | 2×499 k (1206) + 33 k 0.1 % → ADS1115 | 3.2 V at 100 V; measured on the pack side, so the pack voltage is visible with the switch open |
| Logic supply | diode-OR of all inputs → **LM5164** (6–100 V) → 5 V | the controller never browns out, whichever packs are switched off; RON/FB/Type-3 ripple values computed from the LM5164 datasheet (SNVSAU4D eq. 15, 24–26) |
| Switch latch | **TCA9535** (no internal pull-ups) drives pack switches | outputs survive ESP32 reset/OTA (firmware adopts the latched state); power-on default = all packs off |
| PV switch | direct GPIO40, same switch circuit | fast cut-off on reverse current / over-voltage |
| Actuator outputs | MCPWM (2 units × 3 operators = 12 outputs) → 74HCT244 | ESP32-S3 has only 8 LEDC channels; 3.3 V is out of spec for the 74HC244 on the IBT-2 module |
| E-stop | GPIO39 → NC loop → 2N7002 → 74HCT244 /OE, outputs pulled down | an open loop, a missing jumper or an un-booted ESP32 all leave the IBT-2s disabled, in hardware |
| Pot excitation | own AP2112 (+3V3_POT), 47 Ω per connector, ratiometric (VREF measured) | a shorted sensor cable can't take down the ADC/I2C rail |
| Wiper inputs | 4.7 k + 100 n + BAT54S clamp, 1 M pull-down | survive miswiring; an open wiper reads 0, so the firmware raises a SENSOR fault |
| Two I2C buses | IMU + actuator ADCs on A, power on B | BNO085 clock-stretch problems can't stall power management; resolves the BNO085/ADS 0x4A address clash |

### GPIO map (ESP32-S3-DevKitC-1)

| Function | GPIO |
|---|---|
| IBT-2 IS (ADC1) A1…A6 | 1, 2, 4, 5, 6, 7 |
| RPWM/LPWM A1 / A2 / A3 | 9/10, 11/12, 13/14 |
| RPWM/LPWM A4 / A5 / A6 | 15/16, 17/18, 21/47 |
| I2C-A SDA/SCL | 41 / 42 |
| I2C-B SDA/SCL | 19 / 20 (native USB port unusable; program via the **UART** USB port) |
| ACT_EN / ESTOP_SENSE | 39 / 8 |
| PV_EN | 40 |
| IMU_INT / LED_ERR | 3 / 46 |
| left free | 0, 35–37 (PSRAM on R8 modules), 38/48 (on-board RGB LED), 43/44 (UART0), 45 |

TCA9535 (0x20): P00–P03 = BAT1–4_EN, P04 = IMU_RST, P05 = PWR LED, P06–P17 → `EXP IO` header.

### Connectors

| Ref | What | Pinout |
|---|---|---|
| BAT1–4+, PV+, BUS+ ×2, GND ×3 | M4 ring-terminal lugs | — |
| IBT-2 #1–#6 | 1×8 pin header, 1:1 with the IBT-2 header (straight Dupont cable) | RPWM LPWM R_EN L_EN R_IS L_IS VCC(5 V) GND |
| POS 1–6 | 3-pin 3.5 mm screw terminal | +3V3_POT, wiper, GND |
| IMU BNO085 | JST-XH 6 | 5V GND SCL SDA INT RST |
| E-STOP | 2-pin 3.5 mm screw terminal | NC contact loop (fit a wire bridge if no E-stop is used) |
| I2C-B ext | JST-XH 4 | 3V3 GND SDA SCL |
| EXP IO | 1×12 header | 3V3 GND IO0…IO9 |

### PCB

190 × 155 mm, 2 layers, **2 oz copper** (required). The high-current path per channel is a
straight column (lug → shunt → FET → FET → bus bar) of locked wide tracks on both layers.
The bus bar is 8 mm wide on both layers, stitched with vias. Both layers carry GND pours.
Signals were autorouted with FreeRouting; see "Status" below.

Regenerate:
```sh
cd hardware/gen
python3 build.py                                   # schematic, project, BOM
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py place     # board + DSN
(cd .. && env -i PATH=/usr/bin:/bin HOME=$HOME timeout 2700 \
   java -jar freerouting-2.0.1.jar -de solbot_main.dsn -do solbot_main.ses -mt 4 --gui.enabled=false)
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py finish    # import SES, pours, DRC
```
FreeRouting 2.0.1 is the newest release that runs on Java 21. It ignores its pass limit
and only stops when everything is routed, so run it under `timeout`. The committed
`solbot_main.ses` converged after 45 passes (about 30 min).
(`env -i` is needed inside the VS Code snap, whose libraries break KiCad/Java.)
Once you start editing the PCB by hand in KiCad, stop re-running `gen_pcb.py place`,
because it rebuilds the board from scratch. Schematic edits can still go through
`design.py` + `build.py`, followed by *Update PCB from Schematic* (UUIDs are stable).

## Firmware

```sh
cd firmware
pio run -e solbot_main -t upload          # via the UART USB port
OTA_HOST=<ip> pio run -e solbot_main_ota -t upload
test/host/run.sh                          # host unit tests (after one `pio run`)
```

Tasks: **control** at 50 Hz (bus A: BNO085 drained between ADC conversions, positions,
IS currents, PWM), **power** at 10 Hz (bus B: 12 ADC channels, policy,
break-before-make switch writes), **loop()** for WiFi/MQTT/OTA/telemetry. The control
loops never depend on the network.

### MQTT API

Unchanged from v3 (ROS `steering.py`, `imu_bridge.py`, `esp32_power_monitor.py` keep working):

| Topic | Dir | Payload |
|---|---|---|
| `steer_position` | sub | `{"data": legacy}`. Legacy counts are the old ADS1015 scale, so `steering.yaml` stays valid |
| `lift/set_position` | sub | `{"data": legacy}` |
| `motor_direction` | sub | toggles the steering direction |
| `ads` | pub 10 Hz | `WAS, WAS_pwm, WAS_pos_goal, LAS, LAS_pwm, LAS_pos_goal` (+ new `TBS`, `TBS_goal`, `estop`) |
| `imu` | pub 5 Hz | `orientation[4], heading_accuracy, angular_velocity[3], linear_acceleration[3]` |
| `esp32/power` | pub 1 Hz | `voltage, pv_on` (+ `bus_v, load_i, temp_c, pv{…}, packs[{id,v,i,on,mode,soc,ah_out,ah_in,fault}]`) |
| `esp32/diag` | pub | loop gaps, E-stop events |
| `<dev>/config`, `get_config`, `enter_ap` | sub | as before (`config_status` now also contains actuator and power configs) |

New:

| Topic | Payload |
|---|---|
| `slider/set_position` / `slider/position` | `{"data": metres}` (tool-bar slider, ±0.10 m after calibration) |
| `<dev>/act/<1-6\|name>/set` | `{"pos": units}` or `{"raw": legacy}` or `{"ratio": 0..1}` |
| `<dev>/act/<1-6\|name>/config` | partial update of `name, enabled, invert, r_min, r_max, u_min, u_max, deadband, slow_zone, pwm_min, pwm_max, pwm_slew, i_limit, oc_ms, stall_delta, stall_ms, cmd_timeout_ms, sensor_check` (saved to NVS) |
| `<dev>/act/reset`, `<dev>/act/enable {"data":0/1}` | clear faults / software enable |
| `<dev>/act/state` (pub 5 Hz) | per-actuator `pos, goal, ratio, duty, amps, fault` |
| `<dev>/power/cmd` | `{"pack":2,"mode":"auto\|on\|off"}`, `{"pv":"auto\|on\|off"}`, `{"reset_faults":true}` |
| `<dev>/power/config` | partial update of `cells[4], cap_ah[4], v_chg, v_chg_hyst, v_ov, v_uv, v_uv_reconnect, v_crit, dv_par, i_dis_max, i_chg_max, i_pv_max, smart_swap, v_scale[6], …` |
| `<dev>/settings` | `{"imu_hz": 5, "ads_hz": 10}` |

`<dev>` defaults to `ESP32_solbot_main`.

### Power policy (auto mode)

* **Discharge:** the highest pack anchors the bus. Other packs join only when their
  open-circuit voltage (V + I·R_int) is within `dv_par` × cells (0.3 V at 10S) of the
  bus, so packs are never paralleled across a large voltage step.
* **Charging:** the PV switch closes when the PV voltage exceeds the bus voltage and every
  connected pack is below `cells × (v_chg − v_chg_hyst)`. It opens at `cells × v_chg`, on
  reverse current (sun gone), or on charge over-current. Defaults reproduce v3 at 10S:
  off at 41.0 V, back on below 40.0 V, with a 30 s minimum off-time.
* **Protection:** discharge over-current (30 A) and charge over-current (15 A) per pack,
  pack over-voltage (stops PV), under-voltage disconnect after 5 s (the last pack stays
  connected until `v_crit`), and board over-temperature (stops PV).
* **Smart swap** (off by default): when PV has had a surplus of more than 2 A for 60 s,
  the bus is moved onto a lower pack that couldn't join, so PV energy goes into the
  emptiest pack.
* Every switch change is **break-before-make** (3 ms).

### Calibration (once per robot)

1. Voltage: compare `packs[].v` with a multimeter and set `power/config {"v_scale":[…]}`
   (BAT1–4, PV, BUS).
2. Actuators: jog to each mechanical end with `act/<n>/set {"ratio":…}`, read `ratio`
   from `act/state`, then set `r_min/r_max` a little inside the ends, plus `u_min/u_max`
   in your units. Steering and lift default to legacy counts; the slider defaults to
   −0.10…+0.10 m.
3. Direction: if an actuator runs away from its goal, set `{"invert":true}`.
4. Current-sense zero needs nothing: the firmware auto-zeroes each channel while its switch
   is open.

## Status / check before ordering

* Schematic: generated; KiCad's netlist matches `design.py` node-for-node (166 nets).
* Firmware: builds (RAM 17 %, flash 26 %), and the host logic tests pass
  (they were mutation-checked). **It hasn't run on hardware yet.**
* PCB: fully routed. DRC finds 0 unconnected items, 0 clearance errors and 0 shorts.
  The remaining entries are silkscreen warnings, plus `lib_footprint_issues`, which only
  appear in the headless run (`KICAD7_FOOTPRINT_DIR` unset). Gerbers, drill and
  placement files are in `hardware/fab/`, and the schematic PDF is
  `hardware/solbot_main_schematic.pdf`. The signal routing is autorouted, so look it over
  in KiCad before ordering. Copper-to-edge clearance is 0.3 mm (within JLCPCB's limits).
* **Verify:** the ESP32-S3-DevKitC-1 header row spacing on your actual module. The
  footprint uses 25.4 mm (third-party CAD models), but Espressif's own footprint says
  22.86 mm. Measure it.
* **Verify:** whether your IBT-2 modules already have an IS resistor. If they do, don't
  fit the 1 k R_IS resistors (A*_IS to GND).
* **For 20S:** the FETs are −100 V parts, so keep PV Voc ≤ 90 V and change the TVS from
  SMCJ58A to SMCJ85A. Set `cells` to 20.
* Pack fuses are **not** on the board. Fit one per pack, close to the pack.
* Inrush: the first pack switches on in about 0.5 ms (slow gate charge), which acts as a
  soft start for the board's 200 µF. If the loads on BUS+ add several mF (motor
  controllers), give them their own precharge, or check the SQM120P10 SOA for ½·C·V².
* LM5164 component values follow the datasheet equations. Cross-check them in TI
  WEBENCH (VIN 18–100 V, 5 V / 1 A).
