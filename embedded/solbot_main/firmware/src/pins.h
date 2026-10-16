// Pin map for solbot_main v4 PCB (see hardware/solbot_main.kicad_sch, root sheet).
#pragma once
#include <stdint.h>

namespace pins {

// I2C bus A: BNO085 (0x4A), actuator ADS1115 0x48 (A1..A4) / 0x49 (A5, A6, VREF/2, 5V/2)
constexpr int I2CA_SDA = 41;
constexpr int I2CA_SCL = 42;
// I2C bus B: power ADS1115 0x48/0x49/0x4A, TCA9535 0x20, expansion connector
constexpr int I2CB_SDA = 19;
constexpr int I2CB_SCL = 20;

constexpr int ACT_EN = 39;       // high = actuators allowed (goes through E-stop loop)
constexpr int ESTOP_SENSE = 8;   // high = E-stop loop closed
constexpr int PV_EN = 40;        // high = PV switch on
constexpr int IMU_INT = 3;
constexpr int LED_ERR = 46;      // red LED, active high

struct ActPins {
  int rpwm, lpwm, is_adc;
};
// index 0..5 = A1..A6 ; is_adc = ADC1 pin with IBT-2 IS voltage (1k -> 8.5 A/V)
constexpr ActPins ACT[6] = {
    {9, 10, 1}, {11, 12, 2}, {13, 14, 4}, {15, 16, 5}, {17, 18, 6}, {21, 47, 7}};

}  // namespace pins

namespace i2caddr {
constexpr uint8_t BNO085 = 0x4A;      // bus A
constexpr uint8_t ADS_ACT0 = 0x48;    // bus A
constexpr uint8_t ADS_ACT1 = 0x49;    // bus A
constexpr uint8_t ADS_PWR0 = 0x48;    // bus B: BAT1_V, BAT1_I, BAT2_V, BAT2_I
constexpr uint8_t ADS_PWR1 = 0x49;    // bus B: BAT3_V, BAT3_I, BAT4_V, BAT4_I
constexpr uint8_t ADS_PWR2 = 0x4A;    // bus B: PV_V, PV_I, BUS_V, TEMP
constexpr uint8_t TCA9535 = 0x20;     // bus B
}  // namespace i2caddr

// TCA9535 port bits (P00..P07 = bit 0..7, P10..P17 = bit 8..15)
namespace expbit {
constexpr int BAT_EN0 = 0;   // BAT1..BAT4 = bits 0..3
constexpr int IMU_RST = 4;   // active low
constexpr int LED_PWR = 5;   // active low (LED to 3V3)
}  // namespace expbit
