#include "pv.h"

#include <string.h>

const char *pvModeName(PvMode m) { return m == PvMode::ON ? "on" : m == PvMode::OFF ? "off" : "auto"; }

bool parsePvMode(const char *s, PvMode &m) {
  if (!s) return false;
  if (!strcmp(s, "auto")) m = PvMode::AUTO;
  else if (!strcmp(s, "on")) m = PvMode::ON;
  else if (!strcmp(s, "off")) m = PvMode::OFF;
  else return false;
  return true;
}

const char *ssr2ModeName(Ssr2Mode m) { return m == Ssr2Mode::ON ? "on" : m == Ssr2Mode::OFF ? "off" : "mirror"; }

bool parseSsr2Mode(const char *s, Ssr2Mode &m) {
  if (!s) return false;
  if (!strcmp(s, "mirror")) m = Ssr2Mode::MIRROR;
  else if (!strcmp(s, "on")) m = Ssr2Mode::ON;
  else if (!strcmp(s, "off")) m = Ssr2Mode::OFF;
  else return false;
  return true;
}

void PvConfig::toJson(JsonObject o) const {
  o["cells"] = cells;
  o["v_chg"] = v_chg;
  o["v_chg_hyst"] = v_chg_hyst;
  o["v_low"] = v_low;
  o["v_min_valid"] = v_min_valid;
  o["min_off_s"] = min_off_s;
  o["v_scale"] = v_scale;
  o["v12_scale"] = v12_scale;
}

void PvConfig::fromJson(JsonObjectConst o) {
  cells = o["cells"] | cells;
  if (cells < 1) cells = 1;
  if (cells > 24) cells = 24;
  v_chg = o["v_chg"] | v_chg;
  v_chg_hyst = o["v_chg_hyst"] | v_chg_hyst;
  v_low = o["v_low"] | v_low;
  v_min_valid = o["v_min_valid"] | v_min_valid;
  min_off_s = o["min_off_s"] | min_off_s;
  v_scale = o["v_scale"] | v_scale;
  v12_scale = o["v12_scale"] | v12_scale;
}

void PvManager::off(const char *why, uint32_t now) {
  if (pv_on) offSince_ = now;
  pv_on = false;
  reason = why;
}

void PvManager::update(float bus_adc_v, float v12_adc_v, uint32_t now) {
  bus_v = bus_adc_v * cfg.v_scale;
  v12 = v12_adc_v * cfg.v12_scale;
  const float n = cfg.cells;
  bool valid = bus_v > n * cfg.v_min_valid;
  low = valid && bus_v < n * cfg.v_low;

  if (mode == PvMode::OFF) {
    off("mode_off", now);
  } else if (!valid) {
    off("bus_sense_invalid", now);  // can't see the battery: never charge blind
  } else if (bus_v >= n * cfg.v_chg) {
    off(mode == PvMode::ON ? "full_manual" : "full", now);
  } else if (mode == PvMode::ON) {  // manual: no min off-time, but keep the hysteresis
    if (pv_on || bus_v < n * (cfg.v_chg - cfg.v_chg_hyst)) {
      pv_on = true;
      reason = "manual";
    }
  } else if (!pv_on) {
    bool rested = !everOn_ || now - offSince_ >= cfg.min_off_s * 1000UL;
    if (bus_v < n * (cfg.v_chg - cfg.v_chg_hyst) && rested) {
      pv_on = true;
      everOn_ = true;
      reason = "charging";
    }
  }
  ssr2_on = ssr2_mode == Ssr2Mode::MIRROR ? pv_on : ssr2_mode == Ssr2Mode::ON;
}

void PvManager::toJson(JsonObject o) const {
  o["voltage"] = bus_v;  // v3 compatible (esp32_power_monitor.py)
  o["pv_on"] = pv_on;
  o["v12"] = v12;
  o["low"] = low;
  o["mode"] = pvModeName(mode);
  o["reason"] = reason;
  o["ssr2_on"] = ssr2_on;
  o["ssr2_mode"] = ssr2ModeName(ssr2_mode);
}
