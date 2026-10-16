// IBT-2 H-bridges on MCPWM (A = RPWM, B = LPWM), one operator per actuator.
// Rev B drives the IBT-2 through a 74HC14 inverter, so the GPIO outputs are active-low.
#pragma once
#include <Arduino.h>

namespace hbridge {
void begin(uint32_t freq_hz);
// duty -1..1 : >0 drives RPWM, <0 drives LPWM
void write(int idx, float duty);
}  // namespace hbridge
