// PV on/off control through external DC solid-state relays + bus voltage monitoring.
//
// SSR1 switches PV onto the battery bus. A DC SSR (MOSFET output) conducts backwards
// through its body diode when off, so a single SSR cannot block a battery that is above
// the panel voltage (night) or stop charging if wired the other way round. Recommended
// wiring: two SSRs with outputs in series, back-to-back ("-" to "-"), driven together
// (SSR2 in MIRROR mode). Alternatively one SSR + a blocking Schottky diode.
//
// Charging is a bang-bang CV like v3: PV off when the bus reaches cells*v_chg,
// back on below cells*(v_chg - v_chg_hyst) after a minimum off-time.
#pragma once
#include <ArduinoJson.h>
#include <math.h>
#include <stdint.h>

enum class PvMode : uint8_t { AUTO, ON, OFF };
enum class Ssr2Mode : uint8_t { MIRROR, ON, OFF };

const char *pvModeName(PvMode m);
bool parsePvMode(const char *s, PvMode &m);
const char *ssr2ModeName(Ssr2Mode m);
bool parseSsr2Mode(const char *s, Ssr2Mode &m);

struct PvConfig {
  uint8_t cells = 10;
  float v_chg = 4.10f;        // per cell: PV off at/above (v3: 41.0 V @10S)
  float v_chg_hyst = 0.10f;   // per cell: back on below v_chg - hyst (v3: 40.0 V)
  float v_low = 3.40f;        // per cell: low-battery alarm (v3 monitor: 34 V)
  float v_min_valid = 2.0f;   // per cell: below this the sense wire is assumed open
  uint16_t min_off_s = 30;
  float v_scale = 32.33f;     // bus divider (470k+470k+30k)/30k, calibrate
  float v12_scale = 11.0f;    // 12 V divider (100k+10k)/10k

  void toJson(JsonObject o) const;
  void fromJson(JsonObjectConst o);
};

class PvManager {
 public:
  PvConfig cfg;
  PvMode mode = PvMode::AUTO;
  Ssr2Mode ssr2_mode = Ssr2Mode::MIRROR;
  float bus_v = NAN, v12 = NAN;
  bool pv_on = false, ssr2_on = false;
  bool low = false;
  const char *reason = "boot";

  // adc volts at the ESP pins (already averaged)
  void update(float bus_adc_v, float v12_adc_v, uint32_t now_ms);
  void toJson(JsonObject o) const;

 private:
  uint32_t offSince_ = 0;
  bool everOn_ = false;
  void off(const char *why, uint32_t now);
};
