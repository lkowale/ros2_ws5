// Battery / PV power management for the 4+1 channel power path.
//
// Topology: every pack and the PV input reach the common BUS+ through their own
// back-to-back P-FET switch with a bidirectional shunt. Pack negatives, PV- and
// loads share GND. Logic power comes from a diode-OR of all inputs (VAUX), so the
// controller survives any switch state.
//
// Policy (auto mode)
//  * Discharge: the bus is anchored by the pack with the highest open-circuit
//    voltage; other packs join only when their OCV is within dv_par of the bus
//    OCV, so packs are never paralleled with a large voltage difference.
//  * Charge: PV is switched onto the bus (hence into the connected packs) when
//    PV is available and every connected pack is below cells*v_chg. It is
//    switched off at the limit and back on below cells*(v_chg - v_chg_hyst)
//    after a minimum off-time (bang-bang CV), or on charge over-current.
//  * Protection: per-pack over-current (discharge & charge), pack over-voltage,
//    under-voltage disconnect with reconnect hysteresis, board over-temperature.
//  * Optional smart swap: while PV covers the load, connect a lower pack that
//    could not join otherwise, so PV energy goes into the emptiest pack.
#pragma once
#include <ArduinoJson.h>
#include <math.h>
#include <stdint.h>

constexpr int NPACK = 4;

enum class Mode : uint8_t { AUTO, ON, OFF };
const char *modeName(Mode m);
bool parseMode(const char *s, Mode &m);

struct PowerConfig {
  uint8_t cells[NPACK] = {10, 10, 0, 0};  // 0 = not installed
  float cap_ah[NPACK] = {20, 20, 20, 20};
  float r_int = 0.08f;          // ohm per 10 cells (scaled with cell count)
  float v_chg = 4.10f;          // per cell, PV off above (v3: 41.0 V @10S)
  float v_chg_hyst = 0.10f;     // per cell, PV back on below v_chg - hyst (v3: 40.0 V)
  float v_ov = 4.22f;           // per cell, pack over-voltage fault
  float v_uv = 3.30f;           // per cell, disconnect below (5 s)
  float v_uv_reconnect = 3.50f; // per cell
  float v_crit = 3.10f;         // per cell, even the last pack is dropped
  float dv_par = 0.03f;         // per cell, max OCV difference to parallel packs
  float i_dis_max = 30.0f;      // A per pack
  float i_chg_max = 15.0f;      // A per pack
  float i_pv_max = 20.0f;       // A
  uint16_t oc_ms = 300;
  uint16_t pv_min_off_s = 30;
  float t_max = 75.0f;          // board temperature, C
  bool smart_swap = false;
  float v_scale[6] = {31.24f, 31.24f, 31.24f, 31.24f, 31.24f, 31.24f};  // BAT1-4, PV, BUS divider
  float shunt_mohm = 1.0f;
  float ina_gain = 20.0f;

  void toJson(JsonObject o) const;
  void fromJson(JsonObjectConst o);
};

enum class PackFault : uint8_t { NONE, OC_DIS, OC_CHG, OV, UV };
const char *packFaultName(PackFault f);

struct PackState {
  float v = 0, i = 0;           // i > 0: discharging into the bus
  float i_zero = 0;             // auto-zeroed offset (A)
  bool zeroed = false;          // i_zero measured on this channel at least once
  float ocv = 0;
  float soc = -1;               // 0..1, -1 unknown
  float ah_out = 0, ah_in = 0;  // since boot
  bool on = false;
  Mode mode = Mode::AUTO;
  PackFault fault = PackFault::NONE;
  uint32_t uv_since = 0, oc_since = 0;
  uint32_t off_since = 0;       // for OCV rest detection
};

struct PvState {
  float v = 0, i = 0, i_zero = 0;
  bool zeroed = false;
  bool on = false;
  Mode mode = Mode::AUTO;
  bool fault = false;
  uint32_t off_since = 0, oc_since = 0;
  const char *reason = "boot";
};

struct PowerRaw {  // ADC volts at the ADS inputs
  float v[6];      // BAT1-4, PV, BUS (divided)
  float i[5];      // INA outputs BAT1-4, PV
  float temp_v;
  float vref;      // INA reference supply (3V3A) estimate
  bool valid[3];   // which ADS responded
};

class PowerManager {
 public:
  PowerConfig cfg;
  PackState pack[NPACK];
  PvState pv;
  float bus_v = 0, load_i = 0, temp_c = NAN;
  bool overtemp = false;
  uint32_t swaps = 0;

  // called by the power task: raw measurements in, desired switch states out
  void update(const PowerRaw &raw, float dt, uint32_t now_ms);
  void adoptSwitchStates(const bool on[NPACK]);  // after ESP32 reset with latched expander
  void setPackMode(int idx, Mode m);
  void setPvMode(Mode m);
  void resetFaults();
  void toJson(JsonObject o) const;

 private:
  void measure(const PowerRaw &raw, float dt, uint32_t now_ms);
  void protect(uint32_t now_ms);
  void discharge(uint32_t now_ms);
  void charge(uint32_t now_ms);
  void smartSwap(uint32_t now_ms);
  bool installed(int k) const { return cfg.cells[k] > 0; }
  float rint(int k) const { return cfg.r_int * cfg.cells[k] / 10.0f; }
  bool canAuto(int k) const {
    return installed(k) && pack[k].mode == Mode::AUTO && pack[k].fault == PackFault::NONE;
  }
  float busOcv() const;
  int nConnected() const;
  uint32_t surplusSince_ = 0;
  bool first_ = true;
};
