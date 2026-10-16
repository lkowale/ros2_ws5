#include "hal.h"

#include <driver/mcpwm.h>

#include "pins.h"

// ---------------------------------------------------------------- TCA9535
namespace {
constexpr uint8_t REG_IN = 0x00, REG_OUT = 0x02, REG_CFG = 0x06;
}

bool Tca9535::writeReg16(uint8_t reg, uint16_t v) {
  w_->beginTransmission(addr_);
  w_->write(reg);
  w_->write(v & 0xFF);
  w_->write(v >> 8);
  return w_->endTransmission() == 0;
}

bool Tca9535::readReg16(uint8_t reg, uint16_t &v) {
  w_->beginTransmission(addr_);
  w_->write(reg);
  if (w_->endTransmission(false) != 0) return false;
  if (w_->requestFrom((int)addr_, 2) != 2) return false;
  v = w_->read();
  v |= (uint16_t)w_->read() << 8;
  return true;
}

bool Tca9535::begin(TwoWire *w, uint8_t addr) {
  w_ = w;
  addr_ = addr;
  uint16_t cfg, out;
  if (!readReg16(REG_CFG, cfg) || !readReg16(REG_OUT, out)) return ok_ = false;
  // Power-on default: CFG=0xFFFF (all inputs). If our bits are already outputs,
  // the ESP32 was merely reset -> keep the latched output state.
  adopted_ = (cfg & outMask_) == 0;
  out_ = adopted_ ? out : (uint16_t)((1 << expbit::IMU_RST) | (1 << expbit::LED_PWR));
  ok_ = writeReg16(REG_OUT, out_) && writeReg16(REG_CFG, (uint16_t)~outMask_);
  return ok_;
}

bool Tca9535::set(int bit, bool level) {
  uint16_t n = level ? (out_ | (1u << bit)) : (out_ & ~(1u << bit));
  if (n == out_ && ok_) return true;
  out_ = n;
  ok_ = writeReg16(REG_OUT, out_);
  return ok_;
}

bool Tca9535::verify() {
  uint16_t cfg, out;
  if (!readReg16(REG_CFG, cfg) || !readReg16(REG_OUT, out)) return ok_ = false;
  if ((cfg & outMask_) != 0 || (out & outMask_) != (out_ & outMask_)) {
    ok_ = writeReg16(REG_OUT, out_) && writeReg16(REG_CFG, (uint16_t)~outMask_);
    return false;
  }
  return ok_ = true;
}

// ---------------------------------------------------------------- ADS1115
bool AdsAsync::begin(TwoWire *w, uint8_t addr) {
  ok_ = ads_.begin(addr, w);
  if (ok_) {
    ads_.setGain(GAIN_ONE);
    ads_.setDataRate(RATE_ADS1115_860SPS);
  }
  return ok_;
}

void AdsAsync::start(uint8_t ch) {
  static const uint16_t mux[4] = {ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
                                  ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3};
  if (ok_) ads_.startADCReading(mux[ch & 3], false);
}

bool AdsAsync::ready() { return ok_ && ads_.conversionComplete(); }

int16_t AdsAsync::result() { return ads_.getLastConversionResults(); }

// ---------------------------------------------------------------- MCPWM
namespace hbridge {
namespace {
struct Map {
  mcpwm_unit_t unit;
  mcpwm_timer_t timer;
  mcpwm_io_signals_t sa, sb;
};
const Map MAP[6] = {
    {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B}, {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B},
    {MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2A, MCPWM2B}, {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM0B},
    {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MCPWM1B}, {MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2A, MCPWM2B}};
}  // namespace

void begin(uint32_t freq_hz) {
  for (int i = 0; i < 6; i++) {
    const Map &m = MAP[i];
    mcpwm_gpio_init(m.unit, m.sa, pins::ACT[i].rpwm);
    mcpwm_gpio_init(m.unit, m.sb, pins::ACT[i].lpwm);
    mcpwm_config_t cfg = {};
    cfg.frequency = freq_hz;
    cfg.cmpr_a = 0;
    cfg.cmpr_b = 0;
    cfg.counter_mode = MCPWM_UP_COUNTER;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(m.unit, m.timer, &cfg);
    mcpwm_set_signal_low(m.unit, m.timer, MCPWM_GEN_A);
    mcpwm_set_signal_low(m.unit, m.timer, MCPWM_GEN_B);
  }
}

void write(int idx, float duty) {
  const Map &m = MAP[idx];
  float d = fabsf(duty);
  if (d > 1.0f) d = 1.0f;
  if (d < 0.005f) {
    mcpwm_set_signal_low(m.unit, m.timer, MCPWM_GEN_A);
    mcpwm_set_signal_low(m.unit, m.timer, MCPWM_GEN_B);
    return;
  }
  mcpwm_generator_t on = duty > 0 ? MCPWM_GEN_A : MCPWM_GEN_B;
  mcpwm_generator_t off = duty > 0 ? MCPWM_GEN_B : MCPWM_GEN_A;
  // never drive both sides: force the idle side low first
  mcpwm_set_signal_low(m.unit, m.timer, off);
  mcpwm_set_duty(m.unit, m.timer, on, d * 100.0f);
  mcpwm_set_duty_type(m.unit, m.timer, on, MCPWM_DUTY_MODE_0);
}
}  // namespace hbridge
