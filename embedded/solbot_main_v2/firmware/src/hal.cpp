#include "hal.h"

#include <driver/mcpwm.h>

#include "pins.h"

// ---------------------------------------------------------------- MCPWM
namespace hbridge {
namespace {
struct Map {
  mcpwm_unit_t unit;
  mcpwm_timer_t timer;
  mcpwm_io_signals_t sa, sb;
};
const Map MAP[pins::NACT] = {
    {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B}, {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B},
    {MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2A, MCPWM2B}, {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM0B}};
}  // namespace

void begin(uint32_t freq_hz) {
  for (int i = 0; i < pins::NACT; i++) {
    const Map &m = MAP[i];
    mcpwm_gpio_init(m.unit, m.sa, pins::ACT[i].rpwm);
    mcpwm_gpio_init(m.unit, m.sb, pins::ACT[i].lpwm);
    mcpwm_config_t cfg = {};
    cfg.frequency = freq_hz;
    cfg.cmpr_a = 0;
    cfg.cmpr_b = 0;
    cfg.counter_mode = MCPWM_UP_COUNTER;
    cfg.duty_mode = MCPWM_DUTY_MODE_1;  // active low: duty 0 -> pin high -> IBT-2 input low
    mcpwm_init(m.unit, m.timer, &cfg);
    mcpwm_set_signal_high(m.unit, m.timer, MCPWM_GEN_A);
    mcpwm_set_signal_high(m.unit, m.timer, MCPWM_GEN_B);
  }
}

void write(int idx, float duty) {
  const Map &m = MAP[idx];
  float d = fabsf(duty);
  if (d > 1.0f) d = 1.0f;
  if (d < 0.005f) {  // both IBT-2 inputs low (pins high, inverter)
    mcpwm_set_signal_high(m.unit, m.timer, MCPWM_GEN_A);
    mcpwm_set_signal_high(m.unit, m.timer, MCPWM_GEN_B);
    return;
  }
  mcpwm_generator_t on = duty > 0 ? MCPWM_GEN_A : MCPWM_GEN_B;
  mcpwm_generator_t off = duty > 0 ? MCPWM_GEN_B : MCPWM_GEN_A;
  // never drive both sides: force the idle side inactive (pin high) first
  mcpwm_set_signal_high(m.unit, m.timer, off);
  mcpwm_set_duty(m.unit, m.timer, on, d * 100.0f);
  mcpwm_set_duty_type(m.unit, m.timer, on, MCPWM_DUTY_MODE_1);
}
}  // namespace hbridge
