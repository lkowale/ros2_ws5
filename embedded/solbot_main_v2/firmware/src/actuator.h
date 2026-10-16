// Closed-loop linear actuator (IBT-2 + potentiometer feedback).
//
// Position is handled internally as a ratio r = V_wiper / V_ref (0..1), so it is
// independent of the pot supply. Three external scales are supported:
//   ratio  : 0..1
//   legacy : r * 1666.7  -> same numbers the v3 board published/accepted
//            (ADS1015 @ +-6.144 V reading a 5 V pot), keeps steering.yaml valid
//   units  : linear calibration (r_min..r_max) -> (u_min..u_max), e.g. metres
#pragma once
#include <ArduinoJson.h>
#include <stdint.h>

constexpr float LEGACY_SCALE = 1666.7f;

struct ActConfig {
  char name[16] = "";
  bool enabled = false;
  bool invert = false;        // swap motor direction
  float r_min = 0.05f;        // soft limits (ratio); also calibration points
  float r_max = 0.95f;
  float u_min = 0.0f;         // engineering units at r_min / r_max
  float u_max = 1.0f;
  float deadband = 0.003f;    // ratio; ~5 legacy counts
  float slow_zone = 0.06f;    // ratio; full speed beyond this error (~100 legacy counts)
  float pwm_min = 0.16f;      // 0..1
  float pwm_max = 0.98f;
  float pwm_slew = 4.0f;      // duty units per second
  float i_limit = 12.0f;      // A, over-current trip
  uint16_t oc_ms = 300;
  float stall_delta = 0.002f; // ratio movement required ...
  uint16_t stall_ms = 1500;   // ... within this time while driving
  uint16_t cmd_timeout_ms = 0;  // 0 = hold goal forever
  bool sensor_check = true;   // fault if wiper reads <1% or >99%

  void toJson(JsonObject o) const;
  void fromJson(JsonObjectConst o);
};

enum class ActFault : uint8_t { NONE, SENSOR, OVERCURRENT, STALL };
const char *faultName(ActFault f);

struct ActState {
  float r = 0;           // measured ratio
  float goal_r = 0;      // goal ratio
  float duty = 0;        // applied -1..1
  float current = 0;     // A (estimated, PWM-compensated)
  ActFault fault = ActFault::NONE;
  bool moving = false;
  bool valid = false;    // position has been measured at least once
};

class Actuator {
 public:
  ActConfig cfg;
  ActState st;

  float toUnits(float r) const;
  float fromUnits(float u) const;
  float clampGoal(float r) const;

  // commands (called from network task)
  void setGoalRatio(float r);
  void setGoalUnits(float u) { setGoalRatio(fromUnits(u)); }
  void setGoalLegacy(float v) { setGoalRatio(v / LEGACY_SCALE); }
  void hold();                  // goal := current position
  void resetFault();

  // control task: new measurement, then compute duty
  void updatePosition(float r, uint32_t now_ms);
  void updateCurrent(float amps_raw);   // averaged IS reading (A, not PWM compensated)
  float control(float dt, bool allowed, uint32_t now_ms);

 private:
  bool inDeadband_ = true;
  uint32_t lastCmdMs_ = 0;
  uint32_t ocSinceMs_ = 0;
  uint32_t stallRefMs_ = 0;
  float stallRefR_ = 0;
  bool goalPending_ = false;
};
