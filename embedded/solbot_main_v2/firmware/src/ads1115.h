// Minimal ADS1115 driver for the optional rev C pot ADC (breakout module on socket J15, I2C 0x48,
// VDD 5 V, reads the raw 0..5 V wipers). Single-shot, +-6.144 V range (187.5 uV/LSB), 860 SPS (~1.2 ms/ch).
// Non-blocking: start(ch) -> do other work -> ready() -> result(). Not thread-safe: use it only
// from the task that owns the I2C bus (the control task, which also services the BNO085).
#pragma once
#include <stdint.h>

class Ads1115 {
 public:
  static constexpr float MV_PER_LSB = 0.1875f;

  bool probe(uint8_t addr = 0x48);  // true if a device answers and its config register reads back
  bool ok() const { return ok_; }
  void lost() { ok_ = false; }
  bool start(uint8_t ch);            // single-ended AIN<ch>, ch 0..3
  bool ready(bool &rdy);             // false on I2C error
  bool result(int16_t &raw);         // false on I2C error

 private:
  uint8_t addr_ = 0x48;
  bool ok_ = false;
  bool readReg(uint8_t reg, uint16_t &v);
  bool writeReg(uint8_t reg, uint16_t v);
};
