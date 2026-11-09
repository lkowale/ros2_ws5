// Pin map for solbot_main v2 rev B PCB (see hardware/solbot_main_v2.kicad_sch, root sheet).
#pragma once
#include <stdint.h>

namespace pins {

constexpr int I2C_SDA = 41;      // BNO085 (0x4A)
constexpr int I2C_SCL = 42;

constexpr int ACT_EN = 39;       // high = actuators allowed (through E-stop loop -> IBT-2 R_EN/L_EN)
constexpr int ESTOP_SENSE = 40;  // high = E-stop loop closed
constexpr int BUS_VS = 9;        // ADC1: main bus / 32.33
constexpr int V12_SNS = 10;      // ADC1: 12 V rail / 11
constexpr int SSR1 = 21;         // PV SSR, high = on
constexpr int SSR2 = 47;         // second SSR (back-to-back PV pair or spare)
constexpr int IMU_INT = 20;
constexpr int IMU_RST = 19;      // active low
constexpr int LED_ERR = 46;      // red LED, active high

constexpr int NACT = 4;
struct ActPins {
  int rpwm, lpwm, is_adc, pos_adc;
};
// A1..A4. PWM pins are ACTIVE-LOW (74HC14 inverter in front of the IBT-2).
// is_adc: IBT-2 IS voltage (1k -> 8.5 A/V), pos_adc: sensor wiper x POS_DIV (5 V sensor -> 3.0 V)
constexpr ActPins ACT[NACT] = {{11, 12, 5, 1}, {13, 14, 6, 2}, {15, 16, 7, 3}, {17, 18, 8, 4}};
// rev C optional ADS1115 module (J15): module input A0..A3 for actuator A1..A4, in PCB routing order
// (socket: A0 = A4_WIP, A1 = A2_WIP, A2 = A3_WIP, A3 = A1_WIP: raw 0..5 V, module on +5V)
constexpr uint8_t ADS_CH[NACT] = {3, 1, 2, 0};
// ESP ADC sees the wiper through a 100k/150k divider (5 V sensors -> 3.0 V); the ADS1115 sees it raw
constexpr float POS_DIV = 150.0f / 250.0f;

}  // namespace pins

namespace i2caddr {
constexpr uint8_t BNO085 = 0x4A;
}  // namespace i2caddr
