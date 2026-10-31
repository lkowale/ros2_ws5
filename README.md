# ros2_ws5

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
| Front-right wheel| 25   | 26   | 25 taken: `PV_SWITCH_PIN`; 26 free                   |
| Rear-left wheel  | 23   | 22   | 22 taken: I2C SCL (BNO08x + ADS1115, default Wire)   |
| **Rear-right wheel** | **27** | **13** | **free**                                   |
| Lifter           | 19   | 21   | 21 taken: I2C SDA                                    |
| Planter          | 18   | 17   | taken: lift_act (LEDC ch 2/3)                        |
| LAS analog in    | 35   | –    | free (input-only ADC)                                |

Also in use: GPIO34 = steering pot ADC.

### Assignment for the slider

- RPWM = **GPIO27**, LPWM = **GPIO13** (old rear-right wheel port). Neither is a
  strapping pin. GPIO13 is a JTAG pin, which only matters if you debug over JTAG.
- LEDC channels **4 / 5** (5 kHz, 8-bit, same as `IBT_2.cpp`)
- Optional position feedback: GPIO35 (or 36/39, input-only ADC), or a spare ADS1115 channel

```cpp
Lin_act_pid slider_act;
slider_act.begin(0, "slider_act", 27, 13, 4, 5, Lin_act_pid::CW);
```

Not yet checked on the physical board: confirm the rear-right header actually
routes to GPIO27/13 and provides 5V/GND and R_EN/L_EN for the IBT-2.

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
