// Minimal Arduino shim so actuator.cpp / power.cpp can be unit-tested on the host.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <algorithm>

extern uint32_t g_millis;
inline uint32_t millis() { return g_millis; }
template <class T, class A, class B>
inline T constrain(T x, A lo, B hi) { return x < lo ? lo : (x > hi ? hi : x); }
