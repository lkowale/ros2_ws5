#include "power.h"

#include <Arduino.h>
#include <math.h>

const char *modeName(Mode m) { return m == Mode::ON ? "on" : m == Mode::OFF ? "off" : "auto"; }

bool parseMode(const char *s, Mode &m) {
  if (!s) return false;
  if (!strcmp(s, "auto")) m = Mode::AUTO;
  else if (!strcmp(s, "on")) m = Mode::ON;
  else if (!strcmp(s, "off")) m = Mode::OFF;
  else return false;
  return true;
}

const char *packFaultName(PackFault f) {
  switch (f) {
    case PackFault::OC_DIS: return "oc_discharge";
    case PackFault::OC_CHG: return "oc_charge";
    case PackFault::OV: return "overvoltage";
    case PackFault::UV: return "undervoltage";
    default: return "none";
  }
}

// ------------------------------------------------------------------ config
void PowerConfig::toJson(JsonObject o) const {
  JsonArray c = o.createNestedArray("cells");
  JsonArray a = o.createNestedArray("cap_ah");
  for (int k = 0; k < NPACK; k++) {
    c.add(cells[k]);
    a.add(cap_ah[k]);
  }
  JsonArray vs = o.createNestedArray("v_scale");
  for (float v : v_scale) vs.add(v);
  o["r_int"] = r_int;
  o["v_chg"] = v_chg;
  o["v_chg_hyst"] = v_chg_hyst;
  o["v_ov"] = v_ov;
  o["v_uv"] = v_uv;
  o["v_uv_reconnect"] = v_uv_reconnect;
  o["v_crit"] = v_crit;
  o["dv_par"] = dv_par;
  o["i_dis_max"] = i_dis_max;
  o["i_chg_max"] = i_chg_max;
  o["i_pv_max"] = i_pv_max;
  o["oc_ms"] = oc_ms;
  o["pv_min_off_s"] = pv_min_off_s;
  o["t_max"] = t_max;
  o["smart_swap"] = smart_swap;
  o["shunt_mohm"] = shunt_mohm;
  o["ina_gain"] = ina_gain;
}

void PowerConfig::fromJson(JsonObjectConst o) {
  if (o["cells"].is<JsonArrayConst>())
    for (int k = 0; k < NPACK && k < (int)o["cells"].size(); k++) cells[k] = o["cells"][k] | cells[k];
  if (o["cap_ah"].is<JsonArrayConst>())
    for (int k = 0; k < NPACK && k < (int)o["cap_ah"].size(); k++) cap_ah[k] = o["cap_ah"][k] | cap_ah[k];
  if (o["v_scale"].is<JsonArrayConst>())
    for (int k = 0; k < 6 && k < (int)o["v_scale"].size(); k++) v_scale[k] = o["v_scale"][k] | v_scale[k];
  r_int = o["r_int"] | r_int;
  v_chg = o["v_chg"] | v_chg;
  v_chg_hyst = o["v_chg_hyst"] | v_chg_hyst;
  v_ov = o["v_ov"] | v_ov;
  v_uv = o["v_uv"] | v_uv;
  v_uv_reconnect = o["v_uv_reconnect"] | v_uv_reconnect;
  v_crit = o["v_crit"] | v_crit;
  dv_par = o["dv_par"] | dv_par;
  i_dis_max = o["i_dis_max"] | i_dis_max;
  i_chg_max = o["i_chg_max"] | i_chg_max;
  i_pv_max = o["i_pv_max"] | i_pv_max;
  oc_ms = o["oc_ms"] | oc_ms;
  pv_min_off_s = o["pv_min_off_s"] | pv_min_off_s;
  t_max = o["t_max"] | t_max;
  smart_swap = o["smart_swap"] | smart_swap;
  shunt_mohm = o["shunt_mohm"] | shunt_mohm;
  ina_gain = o["ina_gain"] | ina_gain;
  for (int k = 0; k < NPACK; k++)
    if (cells[k] > 24) cells[k] = 24;  // hardware is rated 100 V
}

// ------------------------------------------------------------------ helpers
static float ocvToSoc(float vcell) {
  static const float V[] = {3.00f, 3.30f, 3.50f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
  static const float S[] = {0.00f, 0.05f, 0.12f, 0.20f, 0.38f, 0.55f, 0.68f, 0.80f, 0.92f, 1.00f};
  if (vcell <= V[0]) return 0;
  for (int i = 1; i < 10; i++)
    if (vcell < V[i]) return S[i - 1] + (vcell - V[i - 1]) / (V[i] - V[i - 1]) * (S[i] - S[i - 1]);
  return 1;
}

int PowerManager::nConnected() const {
  int n = 0;
  for (int k = 0; k < NPACK; k++) n += pack[k].on;
  return n;
}

float PowerManager::busOcv() const {
  float s = 0;
  int n = 0;
  for (int k = 0; k < NPACK; k++)
    if (pack[k].on) s += pack[k].ocv, n++;
  return n ? s / n : bus_v;
}

void PowerManager::adoptSwitchStates(const bool on[NPACK]) {
  for (int k = 0; k < NPACK; k++) pack[k].on = on[k];
}

void PowerManager::setPackMode(int k, Mode m) {
  if (k >= 0 && k < NPACK) pack[k].mode = m;
}

void PowerManager::setPvMode(Mode m) { pv.mode = m; }

void PowerManager::resetFaults() {
  for (auto &p : pack) p.fault = PackFault::NONE, p.oc_since = p.uv_since = 0;
  pv.fault = false;
}

// ------------------------------------------------------------------ update
void PowerManager::measure(const PowerRaw &raw, float dt, uint32_t now) {
  const float shunt = cfg.shunt_mohm * 1e-3f * cfg.ina_gain;  // V per A at INA output
  const float mid = raw.vref * 0.5f;
  // PV first: its zero (taken whenever PV is off, e.g. every night) is the fallback
  // offset for packs that have never been switched off (all INAs share REF = 3V3A/2)
  float ipa = (raw.i[4] - mid) / shunt;
  if (!pv.on) pv.i_zero += (pv.zeroed ? 0.05f : 1.0f) * (ipa - pv.i_zero), pv.zeroed = true;
  for (int k = 0; k < NPACK; k++) {
    PackState &p = pack[k];
    p.v = raw.v[k] * cfg.v_scale[k];
    float ia = (raw.i[k] - mid) / shunt;
    if (!p.on) {  // switch open -> true zero
      p.i_zero += (p.zeroed ? 0.05f : 1.0f) * (ia - p.i_zero);
      p.zeroed = true;
    }
    float zero = p.zeroed ? p.i_zero : (pv.zeroed ? pv.i_zero : 0.0f);
    p.i = p.on ? ia - zero : 0.0f;
    p.ocv = p.on ? p.v + p.i * rint(k) : p.v;
    if (!installed(k)) continue;
    if (p.on) {
      float ah = p.i * dt / 3600.0f;
      if (ah > 0) p.ah_out += ah;
      else p.ah_in -= ah;
      if (p.soc >= 0) p.soc = constrain(p.soc - ah / cfg.cap_ah[k], 0.0f, 1.0f);
      p.off_since = 0;
    } else if (!p.off_since) {
      p.off_since = now;
    }
    bool rested = p.off_since && now - p.off_since > 600000;
    if (first_ || p.soc < 0 || rested) p.soc = ocvToSoc(p.v / cfg.cells[k]);
  }
  pv.v = raw.v[4] * cfg.v_scale[4];
  pv.i = pv.on ? ipa - pv.i_zero : 0.0f;
  bus_v = raw.v[5] * cfg.v_scale[5];
  load_i = pv.i;
  for (auto &p : pack) load_i += p.i;
  if (raw.temp_v > 0.05f && raw.temp_v < raw.vref - 0.05f) {
    float r = 10000.0f * raw.temp_v / (raw.vref - raw.temp_v);
    temp_c = 1.0f / (1.0f / 298.15f + logf(r / 10000.0f) / 3950.0f) - 273.15f;
  }
  overtemp = !isnan(temp_c) && temp_c > (overtemp ? cfg.t_max - 5 : cfg.t_max);
}

void PowerManager::protect(uint32_t now) {
  for (int k = 0; k < NPACK; k++) {
    PackState &p = pack[k];
    if (!installed(k)) {
      p.on = false;
      continue;
    }
    if (!p.on) {
      p.oc_since = 0;
      p.uv_since = 0;
      if (p.fault == PackFault::UV && p.v > cfg.cells[k] * cfg.v_uv_reconnect) p.fault = PackFault::NONE;
      if (p.fault == PackFault::OV && p.v < cfg.cells[k] * cfg.v_chg) p.fault = PackFault::NONE;
      continue;
    }
    // over-current; charge over-current is first handled by dropping PV
    bool oc_dis = p.i > cfg.i_dis_max;
    bool oc_chg = -p.i > cfg.i_chg_max;
    if (oc_dis || oc_chg) {
      if (!p.oc_since) p.oc_since = now;
      if (oc_chg && pv.on) {
        pv.on = false;
        pv.reason = "charge_oc";
        pv.off_since = now;
      } else if (now - p.oc_since > cfg.oc_ms) {
        p.on = false;
        p.fault = oc_dis ? PackFault::OC_DIS : PackFault::OC_CHG;
      }
    } else {
      p.oc_since = 0;
    }
    // over-voltage: stop charging, keep supplying
    if (p.v > cfg.cells[k] * cfg.v_ov) {
      p.fault = PackFault::OV;
      if (pv.on) pv.on = false, pv.reason = "pack_ov", pv.off_since = now;
    }
    // under-voltage
    if (p.v < cfg.cells[k] * cfg.v_uv) {
      if (!p.uv_since) p.uv_since = now;
      bool last = nConnected() <= 1;
      bool critical = p.v < cfg.cells[k] * cfg.v_crit;
      if (now - p.uv_since > 5000 && (!last || critical)) {
        p.on = false;
        p.fault = PackFault::UV;
      }
    } else {
      p.uv_since = 0;
    }
  }
  // PV reverse current (sun gone, bus > panel) or over-current
  if (pv.on && (pv.i < -0.3f || pv.i > cfg.i_pv_max)) {
    if (!pv.oc_since) pv.oc_since = now;
    if (pv.i < -1.0f || now - pv.oc_since > cfg.oc_ms) {
      pv.on = false;
      pv.reason = pv.i < 0 ? "reverse" : "pv_oc";
      pv.off_since = now;
    }
  } else {
    pv.oc_since = 0;
  }
}

void PowerManager::discharge(uint32_t now) {
  for (int k = 0; k < NPACK; k++) {
    if (!installed(k)) continue;
    if (pack[k].mode == Mode::OFF) pack[k].on = false;
    if (pack[k].mode == Mode::ON && pack[k].fault != PackFault::OC_DIS && pack[k].fault != PackFault::OC_CHG)
      pack[k].on = true;
  }
  auto usable = [&](int k) {
    return installed(k) && pack[k].mode == Mode::AUTO &&
           (pack[k].fault == PackFault::NONE || pack[k].fault == PackFault::OV);
  };
  if (nConnected() == 0) {
    // anchor: highest-voltage usable pack; as a last resort a UV pack above v_crit
    int best = -1;
    for (int k = 0; k < NPACK; k++)
      if (usable(k) && (best < 0 || pack[k].v > pack[best].v)) best = k;
    if (best < 0)
      for (int k = 0; k < NPACK; k++)
        if (installed(k) && pack[k].mode == Mode::AUTO && pack[k].fault == PackFault::UV &&
            pack[k].v > cfg.cells[k] * cfg.v_crit && (best < 0 || pack[k].v > pack[best].v))
          best = k;
    if (best >= 0 && pack[best].v > 5.0f) pack[best].on = true;
    return;
  }
  float bus = busOcv();
  bool charging = pv.on && pv.i > load_i + 0.5f;  // packs are net-charging
  for (int k = 0; k < NPACK; k++) {
    if (pack[k].on || !installed(k) || pack[k].mode != Mode::AUTO) continue;
    bool ok = usable(k) || (pack[k].fault == PackFault::UV && charging);
    if (ok && fabsf(pack[k].ocv - bus) < cfg.cells[k] * cfg.dv_par) {
      pack[k].on = true;
      if (pack[k].fault == PackFault::UV) pack[k].fault = PackFault::NONE;
    }
  }
}

void PowerManager::charge(uint32_t now) {
  if (pv.mode == Mode::OFF) {
    pv.on = false;
    pv.reason = "mode_off";
    return;
  }
  bool full = false, low = true;
  for (int k = 0; k < NPACK; k++) {
    if (!pack[k].on) continue;
    full |= pack[k].v >= cfg.cells[k] * cfg.v_chg || pack[k].fault == PackFault::OV;
    low &= pack[k].v < cfg.cells[k] * (cfg.v_chg - cfg.v_chg_hyst);
  }
  auto off = [&](const char *why) {
    if (pv.on) pv.off_since = now;
    pv.on = false;
    pv.reason = why;
  };
  if (pv.mode == Mode::ON) {  // manual: only hard protections apply
    if (full) return off("full");
    pv.on = true;
    pv.reason = "manual";
    return;
  }
  if (nConnected() == 0) return off("no_pack");
  if (overtemp) return off("overtemp");
  if (full) return off("full");
  if (!pv.on) {
    bool rested = !pv.off_since || now - pv.off_since > cfg.pv_min_off_s * 1000UL;
    bool sun = pv.v > bus_v + 1.0f;
    if (!sun) pv.reason = "no_sun";
    if (rested && low && sun) {
      pv.on = true;
      pv.reason = "charging";
    }
  }
}

void PowerManager::smartSwap(uint32_t now) {
  if (!cfg.smart_swap || !pv.on) {
    surplusSince_ = 0;
    return;
  }
  float surplus = pv.i - load_i;  // load_i = consumption = PV + net pack discharge
  if (surplus < 2.0f) {
    surplusSince_ = 0;
    return;
  }
  if (!surplusSince_) surplusSince_ = now;
  if (now - surplusSince_ < 60000) return;
  float bus = busOcv();
  int low = -1;
  for (int k = 0; k < NPACK; k++)
    if (!pack[k].on && canAuto(k) && pack[k].ocv < bus - cfg.cells[k] * cfg.dv_par &&
        pack[k].v > cfg.cells[k] * cfg.v_uv && (low < 0 || pack[k].ocv < pack[low].ocv))
      low = k;
  if (low < 0) return;
  // break-before-make is enforced by the switch driver; PV carries the load meanwhile
  for (int k = 0; k < NPACK; k++)
    if (pack[k].on && pack[k].mode == Mode::AUTO) pack[k].on = false;
  pack[low].on = true;
  swaps++;
  surplusSince_ = 0;
}

void PowerManager::update(const PowerRaw &raw, float dt, uint32_t now) {
  measure(raw, dt, now);
  if (first_) {
    first_ = false;
    return;  // one sample of settling before acting
  }
  protect(now);
  smartSwap(now);
  discharge(now);
  charge(now);
}

// ------------------------------------------------------------------ report
void PowerManager::toJson(JsonObject o) const {
  o["voltage"] = bus_v;  // v3 compatibility (esp32_power_monitor.py)
  o["pv_on"] = pv.on;
  o["bus_v"] = bus_v;
  o["load_i"] = load_i;
  o["load_w"] = bus_v * load_i;
  if (!isnan(temp_c)) o["temp_c"] = temp_c;
  JsonObject p = o.createNestedObject("pv");
  p["v"] = pv.v;
  p["i"] = pv.i;
  p["w"] = pv.v * pv.i;
  p["on"] = pv.on;
  p["mode"] = modeName(pv.mode);
  p["reason"] = pv.reason;
  JsonArray a = o.createNestedArray("packs");
  for (int k = 0; k < NPACK; k++) {
    if (!installed(k)) continue;
    JsonObject b = a.createNestedObject();
    b["id"] = k + 1;
    b["v"] = pack[k].v;
    b["i"] = pack[k].i;
    b["on"] = pack[k].on;
    b["mode"] = modeName(pack[k].mode);
    b["soc"] = pack[k].soc;
    b["ah_out"] = pack[k].ah_out;
    b["ah_in"] = pack[k].ah_in;
    b["fault"] = packFaultName(pack[k].fault);
  }
  o["swaps"] = swaps;
}
