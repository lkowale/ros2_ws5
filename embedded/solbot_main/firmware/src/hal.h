// Small hardware helpers: TCA9535 IO expander, async ADS1115 access, MCPWM H-bridge outputs.
#pragma once
#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------- TCA9535
// The expander keeps its output latch across ESP32 resets (no reset pin), so the
// battery switch state survives reboots/OTA. begin() adopts the existing state.
class Tca9535 {
 public:
  bool begin(TwoWire *w, uint8_t addr);
  bool set(int bit, bool level);          // queue + write if changed
  bool get(int bit) const { return (out_ >> bit) & 1; }
  bool verify();                          // re-read latch, rewrite on mismatch
  bool adopted() const { return adopted_; }
  bool ok() const { return ok_; }

 private:
  bool writeReg16(uint8_t reg, uint16_t v);
  bool readReg16(uint8_t reg, uint16_t &v);
  TwoWire *w_ = nullptr;
  uint8_t addr_ = 0x20;
  uint16_t out_ = 0;
  uint16_t outMask_ = 0x003F;  // bits driven as outputs (BAT1-4, IMU_RST, LED)
  bool adopted_ = false;
  bool ok_ = false;
};

// ---------------------------------------------------------------- ADS1115
class AdsAsync {
 public:
  bool begin(TwoWire *w, uint8_t addr);
  void start(uint8_t ch);                 // single-ended single-shot
  bool ready();
  int16_t result();
  bool ok() const { return ok_; }
  static constexpr float LSB_V = 4.096f / 32768.0f;  // GAIN_ONE

 private:
  Adafruit_ADS1115 ads_;
  bool ok_ = false;
};

// ---------------------------------------------------------------- MCPWM
// 6 H-bridges on 2 MCPWM units x 3 operators; A = RPWM, B = LPWM.
namespace hbridge {
void begin(uint32_t freq_hz);
// duty -1..1 : >0 drives RPWM, <0 drives LPWM
void write(int idx, float duty);
}  // namespace hbridge
