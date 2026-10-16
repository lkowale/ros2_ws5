// Host-side tests for the actuator controller and power policy.
//   cd firmware/test/host && ./run.sh
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../../src/actuator.h"
#include "../../src/power.h"

uint32_t g_millis = 0;
static int fails = 0;
#define CHECK(c, ...)                         \
  do {                                        \
    if (!(c)) {                               \
      printf("FAIL %s:%d ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                    \
      printf("\n");                           \
      fails++;                                \
    }                                         \
  } while (0)

// ------------------------------------------------------------- actuator plant
// actuator speed proportional to duty above a 0.1 friction threshold
static float plant(float r, float duty, bool invert, float dt, bool jammed = false) {
  if (jammed) return r;
  float d = invert ? -duty : duty;
  float eff = fabsf(d) < 0.1f ? 0 : d;
  return r + eff * 0.15f * dt;  // full duty: 15% stroke per second
}

static void test_actuator_converges(bool invert) {
  Actuator a;
  a.cfg.enabled = true;
  a.cfg.invert = invert;
  g_millis = 1000;
  float r = 0.30f;
  a.updatePosition(r, g_millis);
  CHECK(fabsf(a.st.goal_r - 0.30f) < 1e-6, "boot goal should hold position");
  a.setGoalLegacy(0.70f * LEGACY_SCALE);
  int moves = 0;
  for (int i = 0; i < 1000; i++) {  // 20 s
    g_millis += 20;
    a.updatePosition(r, g_millis);
    a.updateCurrent(1.0f * fabsf(a.st.duty));
    float d = a.control(0.02f, true, g_millis);
    r = plant(r, d, invert, 0.02f);
    if (fabsf(d) > 0.01f) moves++;
  }
  CHECK(fabsf(r - 0.70f) < 2 * a.cfg.deadband + 0.002f, "invert=%d did not converge r=%.4f", invert, r);
  CHECK(a.st.fault == ActFault::NONE, "unexpected fault %s", faultName(a.st.fault));
  CHECK(fabsf(a.st.duty) < 0.01f, "still driving at goal duty=%.3f", a.st.duty);
  // no hunting: once settled, stays off
  int after = 0;
  for (int i = 0; i < 200; i++) {
    g_millis += 20;
    a.updatePosition(r, g_millis);
    if (fabsf(a.control(0.02f, true, g_millis)) > 0.01f) after++;
  }
  CHECK(after == 0, "hunting after settle (%d)", after);
}

static void test_actuator_limits_and_faults() {
  Actuator a;
  a.cfg.enabled = true;
  a.cfg.r_min = 0.2f;
  a.cfg.r_max = 0.8f;
  g_millis = 0;
  a.updatePosition(0.5f, 0);
  a.setGoalRatio(0.95f);
  CHECK(fabsf(a.st.goal_r - 0.8f) < 1e-6, "goal not clamped to r_max");

  // stall: motor driving, position frozen
  float r = 0.5f;
  bool stalled = false;
  for (int i = 0; i < 200 && !stalled; i++) {
    g_millis += 20;
    a.updatePosition(r, g_millis);
    a.control(0.02f, true, g_millis);
    stalled = a.st.fault == ActFault::STALL;
  }
  CHECK(stalled, "stall not detected");
  CHECK(a.st.duty == 0, "duty not zero after stall");
  a.setGoalRatio(0.6f);
  CHECK(a.st.fault == ActFault::NONE, "new goal should clear stall");

  // over-current
  Actuator b;
  b.cfg.enabled = true;
  b.updatePosition(0.5f, g_millis);
  b.setGoalRatio(0.9f);
  r = 0.5f;
  for (int i = 0; i < 50; i++) {
    g_millis += 20;
    b.updatePosition(r, g_millis);
    b.updateCurrent(20.0f * fabsf(b.st.duty));  // 20 A real current
    float d = b.control(0.02f, true, g_millis);
    r = plant(r, d, false, 0.02f);
  }
  CHECK(b.st.fault == ActFault::OVERCURRENT, "overcurrent not detected (%s, I=%.1f)", faultName(b.st.fault),
        b.st.current);

  // sensor disconnected
  Actuator c;
  c.cfg.enabled = true;
  c.updatePosition(0.0f, g_millis);
  CHECK(c.st.fault == ActFault::SENSOR, "sensor fault not detected");
  CHECK(c.control(0.02f, true, g_millis) == 0, "drives with sensor fault");

  // E-stop: not allowed -> holds current position, no drive
  Actuator e;
  e.cfg.enabled = true;
  e.updatePosition(0.4f, g_millis);
  e.setGoalRatio(0.6f);
  CHECK(e.control(0.02f, false, g_millis) == 0, "drives while not allowed");
  CHECK(fabsf(e.st.goal_r - 0.4f) < 1e-6, "goal not reset to position on estop");
}

static void test_units() {
  Actuator a;
  a.cfg.r_min = 0.1f, a.cfg.r_max = 0.9f, a.cfg.u_min = -0.1f, a.cfg.u_max = 0.1f;
  CHECK(fabsf(a.toUnits(0.5f)) < 1e-6, "center");
  CHECK(fabsf(a.fromUnits(0.1f) - 0.9f) < 1e-6, "u_max");
  CHECK(fabsf(a.toUnits(a.fromUnits(0.037f)) - 0.037f) < 1e-6, "roundtrip");
}

// ------------------------------------------------------------- power sim
// Simple circuit model: packs with OCV + R, PV current source, constant load.
struct Sim {
  float ocv[4] = {40.0f, 38.0f, 0, 0};
  float R = 0.08f;
  float pv_isc = 0, pv_voc = 48;
  float load = 8.0f;  // A
  PowerManager pm;
  PowerRaw raw;

  void step() {
    // solve bus voltage: sum((ocv-Vb)/R) + Ipv = load
    float vb = 0;
    int n = 0;
    for (int k = 0; k < 4; k++)
      if (pm.pack[k].on) vb += ocv[k], n++;
    float ipv = pm.pv.on ? pv_isc : 0;
    if (n) vb = (vb + (ipv - load) * R) / n;
    else vb = pm.pv.on ? pv_voc : 0;
    float i[4] = {};
    for (int k = 0; k < 4; k++) {
      i[k] = pm.pack[k].on ? (ocv[k] - vb) / R : 0;
      float vt = pm.pack[k].on ? vb : ocv[k];
      raw.v[k] = vt / pm.cfg.v_scale[k];
      raw.i[k] = 1.65f + i[k] * 0.02f;
      ocv[k] -= i[k] * 0.1f / 3600 * 0.5f;  // crude: 0.5 V per Ah
    }
    raw.v[4] = (pm.pv.on ? vb : (pv_isc > 0 ? pv_voc : 0)) / pm.cfg.v_scale[4];
    raw.i[4] = 1.65f + ipv * 0.02f;
    raw.v[5] = vb / pm.cfg.v_scale[5];
    raw.temp_v = 1.65f;
    raw.vref = 3.3f;
    g_millis += 100;
    pm.update(raw, 0.1f, g_millis);
  }
  void run(float s) {
    for (int t = 0; t < s * 10; t++) step();
  }
};

static void test_power_anchor_and_join() {
  g_millis = 0;
  Sim s;
  s.run(1);
  CHECK(s.pm.pack[0].on && !s.pm.pack[1].on, "should anchor on highest pack only (%d %d)", s.pm.pack[0].on,
        s.pm.pack[1].on);
  // drain pack 1 down to pack 2: pack 2 must join once within dv_par
  s.load = 0;
  s.ocv[0] = 38.2f;
  s.run(1);
  CHECK(s.pm.pack[1].on, "pack 2 should join at 0.2 V difference");
  float ia = s.pm.pack[0].i, ib = s.pm.pack[1].i;
  CHECK(fabsf(ia + ib) < 0.5f, "join cross-current sane (%.2f %.2f)", ia, ib);
}

static void test_power_pv_hysteresis() {
  g_millis = 0;
  Sim s;
  s.ocv[1] = 0;
  s.pm.cfg.cells[1] = 0;  // single pack
  s.ocv[0] = 39.0f;
  s.load = 1.0f;
  s.pv_isc = 6.0f;
  s.run(2);
  CHECK(s.pm.pv.on, "PV should charge at 39 V (%s)", s.pm.pv.reason);
  s.ocv[0] = 40.7f;  // 6 A * 0.08 = +0.4 V under charge -> 41.1 V > 41.0
  s.run(1);
  CHECK(!s.pm.pv.on, "PV should stop at cells*v_chg (bus %.2f, %s)", s.pm.bus_v, s.pm.pv.reason);
  s.ocv[0] = 39.95f;  // below 40.0 but min-off time not elapsed
  s.run(5);
  CHECK(!s.pm.pv.on, "min off-time not respected");
  s.run(30);
  CHECK(s.pm.pv.on, "PV should resume below hysteresis (%s)", s.pm.pv.reason);
  // sun gone: PV current reverses -> off immediately
  s.pv_isc = -2.0f;
  s.run(0.3f);
  CHECK(!s.pm.pv.on, "reverse PV current must switch off (%s)", s.pm.pv.reason);
}

static void test_power_protection() {
  g_millis = 0;
  Sim s;
  s.ocv[1] = 39.9f;
  s.ocv[0] = 40.0f;
  s.run(2);
  CHECK(s.pm.pack[0].on && s.pm.pack[1].on, "both packs should be parallel");
  s.load = 70;  // 35 A each > 30 A
  s.run(1);
  CHECK(!s.pm.pack[0].on && !s.pm.pack[1].on, "overcurrent should trip both");
  CHECK(s.pm.pack[0].fault == PackFault::OC_DIS, "fault code (%s)", packFaultName(s.pm.pack[0].fault));
  s.load = 5;
  s.run(2);
  CHECK(!s.pm.pack[0].on, "OC fault must latch");
  s.pm.resetFaults();
  s.run(1);
  CHECK(s.pm.pack[0].on, "reconnect after fault reset");

  // under-voltage: the second pack drops out, the last one is kept until critical
  g_millis = 0;
  Sim u;
  u.ocv[0] = 33.5f;
  u.ocv[1] = 33.4f;
  u.load = 4;
  u.run(2);
  u.ocv[0] = 32.0f;
  u.ocv[1] = 32.3f;
  u.run(7);
  int on = u.pm.pack[0].on + u.pm.pack[1].on;
  CHECK(on == 1, "UV: exactly one pack should remain (%d)", on);
  u.ocv[0] = u.ocv[1] = 30.0f;
  u.run(7);
  CHECK(!u.pm.pack[0].on && !u.pm.pack[1].on, "critical UV drops the last pack");
}

static void test_smart_swap() {
  g_millis = 0;
  Sim s;
  s.pm.cfg.smart_swap = true;
  s.ocv[0] = 39.0f;
  s.ocv[1] = 36.0f;
  s.load = 1;
  s.pv_isc = 8;
  s.run(65);
  CHECK(s.pm.pack[1].on && !s.pm.pack[0].on, "swap to the lower pack while PV has surplus");
  CHECK(s.pm.swaps == 1, "swaps=%u", (unsigned)s.pm.swaps);
}

int main() {
  test_actuator_converges(false);
  test_actuator_converges(true);
  test_actuator_limits_and_faults();
  test_units();
  test_power_anchor_and_join();
  test_power_pv_hysteresis();
  test_power_protection();
  test_smart_swap();
  printf(fails ? "%d FAILED\n" : "all tests passed\n", fails);
  return fails ? 1 : 0;
}
