// Host-side tests for the actuator controller and power policy.
//   cd firmware/test/host && ./run.sh
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../../src/actuator.h"
#include "../../src/pv.h"

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

// ------------------------------------------------------------- PV / SSR
static void pv_step(PvManager &p, float bus, float s = 0.1f) {
  for (int t = 0; t < s * 10; t++) {
    g_millis += 100;
    p.update(bus / p.cfg.v_scale, 12.0f / p.cfg.v12_scale, g_millis);
  }
}

static void test_pv_hysteresis() {
  g_millis = 0;
  PvManager p;
  pv_step(p, 39.0f);
  CHECK(p.pv_on && p.ssr2_on, "PV + mirrored SSR2 on at 39 V (%s)", p.reason);
  CHECK(fabsf(p.v12 - 12.0f) < 0.01f, "12 V monitor");
  pv_step(p, 41.05f);
  CHECK(!p.pv_on && !p.ssr2_on, "off at 41.0 V (v3 behaviour)");
  pv_step(p, 40.5f, 10);
  CHECK(!p.pv_on, "stays off inside hysteresis");
  pv_step(p, 39.9f, 5);
  CHECK(!p.pv_on, "min off-time respected");
  pv_step(p, 39.9f, 30);
  CHECK(p.pv_on, "back on below 40.0 V after min off-time (%s)", p.reason);
}

static void test_pv_safety_and_modes() {
  g_millis = 0;
  PvManager p;
  pv_step(p, 0.3f);  // sense wire open
  CHECK(!p.pv_on, "never charge with invalid bus sense (%s)", p.reason);
  pv_step(p, 33.0f);
  CHECK(p.low && p.pv_on, "low alarm + charging at 33 V");
  p.mode = PvMode::OFF;
  pv_step(p, 33.0f);
  CHECK(!p.pv_on, "mode off");
  p.mode = PvMode::ON;
  pv_step(p, 33.0f);
  CHECK(p.pv_on, "manual on");
  pv_step(p, 41.5f);
  CHECK(!p.pv_on, "manual on still stops at full");
  pv_step(p, 40.5f);
  CHECK(!p.pv_on, "manual: no chatter inside hysteresis");
  p.ssr2_mode = Ssr2Mode::ON;
  pv_step(p, 40.5f);
  CHECK(p.ssr2_on && !p.pv_on, "SSR2 manual independent of PV");
  // 20S scaling
  PvManager q;
  q.cfg.cells = 20;
  pv_step(q, 79.0f);
  CHECK(q.pv_on, "20S charging at 79 V");
  pv_step(q, 82.1f);
  CHECK(!q.pv_on, "20S off at 82 V");
}

int main() {
  test_actuator_converges(false);
  test_actuator_converges(true);
  test_actuator_limits_and_faults();
  test_units();
  test_pv_hysteresis();
  test_pv_safety_and_modes();
  printf(fails ? "%d FAILED\n" : "all tests passed\n", fails);
  return fails ? 1 : 0;
}
