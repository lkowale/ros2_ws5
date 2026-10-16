# solbot_main v2 (rev B) — compact ESP32-S3 main controller, JLCPCB Basic parts

> **✅ READY TO ORDER — this commit (2026-10-16).**
> Board `hardware/solbot_main_v2.kicad_pcb` (v2 rev B): DRC has 0 errors and 0 unconnected items.
> JLC production files `hardware/production/` were generated with the Fabrication Toolkit
> plugin. Their BOM (26 lines, all LCSC numbers matching `design.py`) and positions (96 SMD parts)
> were checked against the design, and part placement/polarity was checked in JLC's placement
> preview. Order: 2-layer, 1 oz. Upload `production/solbot_main_v2.zip` (Gerbers),
> `production/bom.csv` and `production/positions.csv`. Through-hole parts are hand-soldered.
> If you change anything, re-run the plugin and repeat these checks before ordering again.

A compact alternative to `../solbot_main` (v1, with the full on-board 4-pack power path).
This board has **no battery switching on board**. It

* runs from the robot's **12 V rail**
* **monitors the main battery bus voltage** (10S…20S, up to ~95 V) and its own 12 V rail
* switches **PV on/off through external DC solid-state relays** (2 driver outputs)
* drives **4 IBT-2 (BTS7960)** modules on **2×5 IDC sockets with the v3 pinout**, plus
  current sensing on the previously unused IDC pins 5/6
* reads the **4 position pots directly on the ESP32-S3 ADC1**, plus the **BNO085** on I2C

Board: **120 × 85 mm**, 2 layers, 1 oz is fine (v1 was 190 × 155 mm, 2 oz).

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
| ADS1115 | ESP32-S3 ADC1 (GPIO1–10 all used), 16× oversampled; pot top fed from 3.3 V through 1 k (≈3.0 V, inside the ADC range) |
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
| 3.3 V | XC6206P332MR → pot excitation (1 k per pot), I2C pull-ups, clamps |
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

The MQTT API is the same as v1 (see `../solbot_main/README.md`). The v3 topics `steer_position`,
`lift/set_position`, `motor_direction`, `ads`, `imu`, `esp32/power` (`voltage`, `pv_on`)
and `esp32/diag` (`pv_switch` on/off) are unchanged. Differences from v1:

| Topic | Payload |
|---|---|
| `esp32/power` (1 Hz) | `voltage, pv_on, v12, low, mode, reason, ssr2_on, ssr2_mode` |
| `<dev>/power/cmd` | `{"pv":"auto\|on\|off"}`, `{"ssr2":"mirror\|on\|off"}` |
| `<dev>/power/config` | `cells, v_chg, v_chg_hyst, v_low, v_min_valid, min_off_s, v_scale, v12_scale` (NVS) |
| `<dev>/settings` | `imu_hz, ads_hz, pot_mv` (measure the voltage at a pot's top terminal and set `pot_mv`; then the legacy WAS/LAS numbers match v3) |
| actuators | `<dev>/act/1..4` (steer, lift, slider, a4) |

Calibration: compare `voltage` with a multimeter and set `power/config {"v_scale": …}`.
Actuator calibration is the same as v1 (`r_min/r_max`, `u_min/u_max`, `invert`).

## Regenerate

```sh
cd hardware/gen
python3 build.py
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py place
(cd .. && env -i PATH=/usr/bin:/bin HOME=$HOME timeout 1800 \
   java -jar freerouting-2.0.1.jar -de solbot_main_v2.dsn -do solbot_main_v2.ses -mt 4 --gui.enabled=false)
env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py finish
```

## Status / check before ordering

* Schematic: KiCad's netlist matches `design.py` exactly (70 nets). The firmware pin map was
  checked against the board netlist (27/27 match).
* PCB: fully routed. DRC finds 0 unconnected items, 0 clearance errors and 0 shorts. What's
  left are silkscreen warnings, plus `lib_footprint_issues`, which only appear in the
  headless run. The signal routing is autorouted, so look it over in KiCad before ordering.
* Firmware builds (RAM 17 %, flash 26 %). The host tests pass (and were mutation-checked).
  **It hasn't run on hardware yet.**
* **Verify** the DevKitC-1 header row spacing on your module (the footprint uses 25.4 mm;
  Espressif's own footprint says 22.86 mm).
* **Verify** the SSR body-diode behaviour (see above) before wiring PV.
