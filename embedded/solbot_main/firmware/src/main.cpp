// solbot_main v4 — ESP32-S3 main controller firmware
//
// Tasks
//   control (50 Hz, I2C bus A): BNO085, actuator position ADCs, IS currents, H-bridge PWM
//   power   (10 Hz, I2C bus B): power ADCs, PowerManager, pack/PV switches, expander
//   loop()                    : WiFi/MQTT/OTA, command parsing, telemetry
//
// MQTT (v3-compatible topics marked *)
//   sub  steer_position* {"data":legacy}   lift/set_position* {"data":legacy}
//        motor_direction*                  slider/set_position {"data":metres}
//        <dev>/act/<n|name>/set {"pos":u}|{"raw":legacy}|{"ratio":r}
//        <dev>/act/<n|name>/config {...}   <dev>/act/reset   <dev>/act/enable {"data":0|1}
//        <dev>/power/cmd {"pack":n,"mode":"auto|on|off"}|{"pv":"auto|on|off"}|{"reset_faults":true}
//        <dev>/power/config {...}          <dev>/settings {"imu_hz":..,"ads_hz":..}
//        <dev>/get_config*  <dev>/config*  <dev>/enter_ap*
//   pub  imu*  ads*  esp32/power*  esp32/diag*  slider/position  <dev>/act/state  <dev>/config_status
#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>

#include "actuator.h"
#include "hal.h"
#include "netcfg.h"
#include "pins.h"
#include "power.h"

// ------------------------------------------------------------------ globals
static SemaphoreHandle_t gLock;
struct Lock {
  Lock() { xSemaphoreTake(gLock, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(gLock); }
};

static Actuator act[6];
static PowerManager pm;
static Tca9535 tca;
static AdsAsync adsAct[2], adsPwr[3];
static Adafruit_BNO08x bno(-1);
static Preferences prefs;

struct Settings {
  uint16_t imu_hz = 5;  // v3 published at 5 Hz
  uint16_t ads_hz = 10;
} settings;

struct ImuData {
  float qi = 0, qj = 0, qk = 0, qr = 1, acc_heading = 0;
  float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
  uint32_t last_ms = 0;
} imu;

static volatile bool actEnable = true;    // software enable (MQTT)
static volatile bool estopOk = false;     // loop closed
static volatile bool imuOk = false;
static volatile bool imuResetReq = false;
static volatile uint32_t i2cErrA = 0, i2cErrB = 0;
static float vrefPotRatio = 0;            // VREF_SNS raw (= VREF/2) counts
static int16_t v5Raw = 0;

static const char *DEFAULT_NAMES[6] = {"steer", "lift", "slider", "a4", "a5", "a6"};

// ------------------------------------------------------------------ persistence
static void saveAct(int i) {
  StaticJsonDocument<768> d;
  act[i].cfg.toJson(d.to<JsonObject>());
  String s;
  serializeJson(d, s);
  char key[4] = {'a', char('0' + i), 0};
  prefs.begin("act", false);
  prefs.putString(key, s);
  prefs.end();
}

static void loadAct() {
  for (int i = 0; i < 6; i++) {
    ActConfig &c = act[i].cfg;
    strlcpy(c.name, DEFAULT_NAMES[i], sizeof(c.name));
    c.enabled = i < 3;
    c.r_min = 0.02f;
    c.r_max = 0.98f;
    c.u_min = 0.02f * LEGACY_SCALE;  // default units = legacy counts
    c.u_max = 0.98f * LEGACY_SCALE;
    if (i == 0) c.invert = true;     // v3: steering actuator was CCW
    if (i == 2) c.u_min = -0.10f, c.u_max = 0.10f;  // slider metres (calibrate!)
  }
  prefs.begin("act", true);
  for (int i = 0; i < 6; i++) {
    char key[4] = {'a', char('0' + i), 0};
    String s = prefs.getString(key, "");
    StaticJsonDocument<768> d;
    if (s.length() && !deserializeJson(d, s)) act[i].cfg.fromJson(d.as<JsonObjectConst>());
  }
  prefs.end();
}

static void savePower() {
  DynamicJsonDocument d(1536);
  pm.cfg.toJson(d.to<JsonObject>());
  String s;
  serializeJson(d, s);
  prefs.begin("pwr", false);
  prefs.putString("cfg", s);
  prefs.end();
}

static void loadPower() {
  prefs.begin("pwr", true);
  String s = prefs.getString("cfg", "");
  prefs.end();
  DynamicJsonDocument d(1536);
  if (s.length() && !deserializeJson(d, s)) pm.cfg.fromJson(d.as<JsonObjectConst>());
}

static void saveSettings() {
  prefs.begin("set", false);
  prefs.putUShort("imu_hz", settings.imu_hz);
  prefs.putUShort("ads_hz", settings.ads_hz);
  prefs.end();
}

static void loadSettings() {
  prefs.begin("set", true);
  settings.imu_hz = prefs.getUShort("imu_hz", settings.imu_hz);
  settings.ads_hz = prefs.getUShort("ads_hz", settings.ads_hz);
  prefs.end();
}

// ------------------------------------------------------------------ IMU (bus A)
static void imuReports() {
  const uint32_t us = 10000;  // 100 Hz
  bno.enableReport(SH2_ARVR_STABILIZED_RV, us);
  bno.enableReport(SH2_LINEAR_ACCELERATION, us);
  bno.enableReport(SH2_GYROSCOPE_CALIBRATED, us);
}

static bool imuBegin() {
  imuOk = bno.begin_I2C(i2caddr::BNO085, &Wire);
  if (imuOk) imuReports();
  return imuOk;
}

static void imuService() {
  if (!imuOk) return;
  if (bno.wasReset()) imuReports();
  sh2_SensorValue_t v;
  for (int n = 0; n < 12 && bno.getSensorEvent(&v); n++) {
    Lock l;
    switch (v.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
      case SH2_ROTATION_VECTOR:
        imu.qi = v.un.rotationVector.i;
        imu.qj = v.un.rotationVector.j;
        imu.qk = v.un.rotationVector.k;
        imu.qr = v.un.rotationVector.real;
        imu.acc_heading = v.un.rotationVector.accuracy;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        imu.gx = v.un.gyroscope.x;
        imu.gy = v.un.gyroscope.y;
        imu.gz = v.un.gyroscope.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        imu.ax = v.un.linearAcceleration.x;
        imu.ay = v.un.linearAcceleration.y;
        imu.az = v.un.linearAcceleration.z;
        break;
      default:
        break;
    }
    imu.last_ms = millis();
  }
}

// ------------------------------------------------------------------ control task
static bool readAds(AdsAsync &a, uint8_t ch, int16_t &out) {
  if (!a.ok()) return false;
  a.start(ch);
  uint32_t t0 = millis();
  vTaskDelay(1);
  while (!a.ready()) {
    if (millis() - t0 > 6) {
      i2cErrA++;
      return false;
    }
    vTaskDelay(1);
  }
  out = a.result();
  return true;
}

static void controlTask(void *) {
  const float dt = 0.02f;
  TickType_t last = xTaskGetTickCount();
  int aux = 0;
  int16_t vrefRaw = 13200;  // ~3.3 V / 2 until measured
  uint32_t imuRetryMs = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    uint32_t now = millis();
    imuService();
    if (imuOk && now - imu.last_ms > 2000) {  // no data: hardware reset via expander
      imuOk = false;
      imuResetReq = true;
      imuRetryMs = now + 500;
    }
    if (!imuOk && imuRetryMs && now > imuRetryMs) {
      imuRetryMs = imuBegin() ? 0 : now + 5000;
      imu.last_ms = now;
    }

    // positions: every enabled actuator each cycle + one aux channel round-robin
    float r[6];
    bool got[6] = {};
    for (int i = 0; i < 6; i++) {
      if (!act[i].cfg.enabled) continue;
      int16_t raw;
      if (readAds(adsAct[i / 4], i % 4, raw) && vrefRaw > 1000) {
        r[i] = raw / (2.0f * vrefRaw);
        got[i] = true;
      }
      imuService();
    }
    int16_t raw;
    if (aux == 0 && readAds(adsAct[1], 2, raw)) vrefRaw = raw;
    if (aux == 1 && readAds(adsAct[1], 3, raw)) v5Raw = raw;
    aux = (aux + 1) % 8;
    vrefPotRatio = vrefRaw;

    // IBT-2 IS currents: 1k load, k_ILIS 8500 -> 8.5 A per volt
    float amps[6];
    for (int i = 0; i < 6; i++) amps[i] = analogReadMilliVolts(pins::ACT[i].is_adc) * 0.0085f;

    estopOk = digitalRead(pins::ESTOP_SENSE);
    bool allowed = actEnable && estopOk;
    float duty[6];
    {
      Lock l;
      for (int i = 0; i < 6; i++) {
        if (got[i]) act[i].updatePosition(r[i], now);
        act[i].updateCurrent(amps[i]);
        duty[i] = act[i].control(dt, allowed, now);
      }
    }
    for (int i = 0; i < 6; i++) hbridge::write(i, duty[i]);
    digitalWrite(pins::ACT_EN, actEnable ? HIGH : LOW);
  }
}

// ------------------------------------------------------------------ power task
static void applySwitches(const bool want[NPACK]) {
  // break-before-make: open first, wait for FET turn-off, then close
  for (int k = 0; k < NPACK; k++)
    if (!want[k]) tca.set(expbit::BAT_EN0 + k, false);
  bool anyOn = false;
  for (int k = 0; k < NPACK; k++) anyOn |= want[k] && !tca.get(expbit::BAT_EN0 + k);
  if (anyOn) {
    vTaskDelay(pdMS_TO_TICKS(3));
    for (int k = 0; k < NPACK; k++)
      if (want[k]) tca.set(expbit::BAT_EN0 + k, true);
  }
}

static void powerTask(void *) {
  const float dt = 0.1f;
  TickType_t last = xTaskGetTickCount();
  uint32_t lastVerify = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    uint32_t now = millis();
    PowerRaw raw = {};
    int16_t res[3][4] = {};
    for (int ch = 0; ch < 4; ch++) {
      for (auto &a : adsPwr) a.start(ch);
      vTaskDelay(2);
      for (int d = 0; d < 3; d++) {
        uint32_t t0 = millis();
        while (adsPwr[d].ok() && !adsPwr[d].ready() && millis() - t0 < 5) vTaskDelay(1);
        if (adsPwr[d].ok()) res[d][ch] = adsPwr[d].result();
      }
    }
    const float L = AdsAsync::LSB_V;
    for (int k = 0; k < 2; k++) {
      raw.v[k] = res[0][2 * k] * L;
      raw.i[k] = res[0][2 * k + 1] * L;
      raw.v[k + 2] = res[1][2 * k] * L;
      raw.i[k + 2] = res[1][2 * k + 1] * L;
    }
    raw.v[4] = res[2][0] * L;
    raw.i[4] = res[2][1] * L;
    raw.v[5] = res[2][2] * L;
    raw.temp_v = res[2][3] * L;
    raw.vref = 3.3f;
    for (int d = 0; d < 3; d++) raw.valid[d] = adsPwr[d].ok();

    bool want[NPACK];
    bool pvOn;
    {
      Lock l;
      pm.update(raw, dt, now);
      for (int k = 0; k < NPACK; k++) want[k] = pm.pack[k].on;
      pvOn = pm.pv.on;
    }
    if (tca.ok()) applySwitches(want);
    digitalWrite(pins::PV_EN, pvOn ? HIGH : LOW);

    if (imuResetReq) {
      imuResetReq = false;
      tca.set(expbit::IMU_RST, false);
      vTaskDelay(pdMS_TO_TICKS(10));
      tca.set(expbit::IMU_RST, true);
    }
    if (now - lastVerify > 1000) {
      lastVerify = now;
      if (!tca.verify()) i2cErrB++;
      bool anyOn = false;
      for (bool w : want) anyOn |= w;
      tca.set(expbit::LED_PWR, !(anyOn && (now / 500) % 2));  // blink while a pack is connected
      bool fault = !estopOk || !imuOk;
      for (auto &a : act) fault |= a.st.fault != ActFault::NONE;
      digitalWrite(pins::LED_ERR, fault);
    }
  }
}

// ------------------------------------------------------------------ MQTT
static int actIndex(const char *s) {
  if (s[0] >= '1' && s[0] <= '6' && s[1] == 0) return s[0] - '1';
  for (int i = 0; i < 6; i++)
    if (!strcmp(s, act[i].cfg.name)) return i;
  return -1;
}

static void publishConfigStatus() {
  DynamicJsonDocument d(6144);
  d["deviceName"] = cfg.deviceName;
  d["ssid"] = cfg.ssid;
  d["mqtt_server"] = cfg.mqtt_server;
  d["mqtt_user"] = cfg.mqtt_user;
  d["mqtt_port"] = cfg.mqtt_port;
  d["ip"] = WiFi.localIP().toString();
  d["mac"] = WiFi.macAddress();
  d["imu_hz"] = settings.imu_hz;
  d["ads_hz"] = settings.ads_hz;
  {
    Lock l;
    JsonArray a = d.createNestedArray("actuators");
    for (auto &x : act) x.cfg.toJson(a.createNestedObject());
    pm.cfg.toJson(d.createNestedObject("power"));
  }
  String s;
  serializeJson(d, s);
  char t[80];
  netcfg::topic(t, sizeof(t), "config_status");
  mqtt.publish(t, s.c_str());
}

static void ack(const char *what, bool ok) {
  char t[80], b[128];
  netcfg::topic(t, sizeof(t), "ack");
  snprintf(b, sizeof(b), "{\"cmd\":\"%s\",\"ok\":%s}", what, ok ? "true" : "false");
  mqtt.publish(t, b);
}

static void onMqtt(char *topic, byte *payload, unsigned int len) {
  static char buf[1536];
  len = min(len, (unsigned)sizeof(buf) - 1);
  memcpy(buf, payload, len);
  buf[len] = 0;
  if (netcfg::handleConfigTopic(topic, buf)) return;

  DynamicJsonDocument d(2048);
  bool json = !deserializeJson(d, buf);
  JsonObjectConst o = d.as<JsonObjectConst>();
  float data = json ? (o["data"] | NAN) : atof(buf);

  // ---- v3 compatible
  if (!strcmp(topic, "steer_position")) {
    Lock l;
    act[0].setGoalLegacy(data);
    return;
  }
  if (!strcmp(topic, "lift/set_position")) {
    Lock l;
    act[1].setGoalLegacy(data);
    return;
  }
  if (!strcmp(topic, "motor_direction")) {
    Lock l;
    act[0].cfg.invert = !act[0].cfg.invert;
    return;
  }
  if (!strcmp(topic, "slider/set_position")) {
    Lock l;
    act[2].setGoalUnits(data);
    return;
  }

  // ---- <dev>/...
  size_t dl = strlen(cfg.deviceName);
  if (strncmp(topic, cfg.deviceName, dl) || topic[dl] != '/') return;
  const char *sub = topic + dl + 1;

  if (!strcmp(sub, "get_config")) return publishConfigStatus();
  if (!strcmp(sub, "act/reset")) {
    Lock l;
    for (auto &a : act) a.resetFault();
    return ack("act/reset", true);
  }
  if (!strcmp(sub, "act/enable")) {
    actEnable = data != 0;
    return ack("act/enable", true);
  }
  if (!strncmp(sub, "act/", 4)) {  // act/<n>/set | act/<n>/config
    char id[16];
    const char *slash = strchr(sub + 4, '/');
    if (!slash || slash - (sub + 4) >= (int)sizeof(id)) return;
    strlcpy(id, sub + 4, slash - (sub + 4) + 1);
    int i = actIndex(id);
    if (i < 0 || !json) return ack(sub, false);
    if (!strcmp(slash, "/set")) {
      Lock l;
      if (o.containsKey("pos")) act[i].setGoalUnits(o["pos"]);
      else if (o.containsKey("raw")) act[i].setGoalLegacy(o["raw"]);
      else if (o.containsKey("ratio")) act[i].setGoalRatio(o["ratio"]);
      else if (o.containsKey("data")) act[i].setGoalUnits(o["data"]);
      return;
    }
    if (!strcmp(slash, "/config")) {
      {
        Lock l;
        act[i].cfg.fromJson(o);
        act[i].hold();
      }
      saveAct(i);
      return ack(sub, true);
    }
    return;
  }
  if (!strcmp(sub, "power/cmd") && json) {
    Lock l;
    Mode m;
    if (o.containsKey("pack") && parseMode(o["mode"], m)) pm.setPackMode((int)o["pack"] - 1, m);
    if (parseMode(o["pv"], m)) pm.setPvMode(m);
    if (o["reset_faults"] | false) pm.resetFaults();
    return ack(sub, true);
  }
  if (!strcmp(sub, "power/config") && json) {
    {
      Lock l;
      pm.cfg.fromJson(o);
    }
    savePower();
    return ack(sub, true);
  }
  if (!strcmp(sub, "settings") && json) {
    settings.imu_hz = constrain((int)(o["imu_hz"] | settings.imu_hz), 1, 100);
    settings.ads_hz = constrain((int)(o["ads_hz"] | settings.ads_hz), 1, 50);
    saveSettings();
    return ack(sub, true);
  }
}

static void onMqttConnect() {
  for (const char *t : {"steer_position", "lift/set_position", "motor_direction", "slider/set_position"})
    mqtt.subscribe(t);
  char t[80];
  for (const char *s : {"act/+/set", "act/+/config", "act/reset", "act/enable", "power/cmd", "power/config",
                        "settings"}) {
    netcfg::topic(t, sizeof(t), s);
    mqtt.subscribe(t);
  }
}

// ------------------------------------------------------------------ telemetry
static void publishTelemetry() {
  uint32_t now = millis();
  static uint32_t tImu = 0, tAds = 0, tAct = 0, tPwr = 0;
  char out[1600];

  if (now - tImu >= 1000u / settings.imu_hz) {
    tImu = now;
    StaticJsonDocument<512> d;
    {
      Lock l;
      JsonArray q = d.createNestedArray("orientation");
      q.add(imu.qi), q.add(imu.qj), q.add(imu.qk), q.add(imu.qr);
      d["heading_accuracy"] = imu.acc_heading;
      JsonArray g = d.createNestedArray("angular_velocity");
      g.add(imu.gx), g.add(imu.gy), g.add(imu.gz);
      JsonArray a = d.createNestedArray("linear_acceleration");
      a.add(imu.ax), a.add(imu.ay), a.add(imu.az);
    }
    if (imuOk) {
      serializeJson(d, out);
      mqtt.publish("imu", out);
    }
  }

  if (now - tAds >= 1000u / settings.ads_hz) {
    tAds = now;
    StaticJsonDocument<512> d;
    {
      Lock l;
      // v3 "ads" message: legacy counts, pwm 0..255
      d["WAS"] = lroundf(act[0].st.r * LEGACY_SCALE);
      d["WAS_pwm"] = lroundf(fabsf(act[0].st.duty) * 255);
      d["WAS_pos_goal"] = lroundf(act[0].st.goal_r * LEGACY_SCALE);
      d["LAS"] = lroundf(act[1].st.r * LEGACY_SCALE);
      d["LAS_pwm"] = lroundf(fabsf(act[1].st.duty) * 255);
      d["LAS_pos_goal"] = lroundf(act[1].st.goal_r * LEGACY_SCALE);
      d["TBS"] = act[2].toUnits(act[2].st.r);
      d["TBS_goal"] = act[2].toUnits(act[2].st.goal_r);
      d["estop"] = !estopOk;
    }
    serializeJson(d, out);
    mqtt.publish("ads", out);
    if (act[2].cfg.enabled && act[2].st.valid) {
      snprintf(out, sizeof(out), "{\"data\":%.4f}", act[2].toUnits(act[2].st.r));
      mqtt.publish("slider/position", out);
    }
  }

  if (now - tAct >= 200) {
    tAct = now;
    DynamicJsonDocument d(2048);
    JsonArray arr = d.createNestedArray("act");
    {
      Lock l;
      for (int i = 0; i < 6; i++) {
        if (!act[i].cfg.enabled) continue;
        JsonObject o = arr.createNestedObject();
        o["id"] = i + 1;
        o["name"] = act[i].cfg.name;
        o["pos"] = act[i].toUnits(act[i].st.r);
        o["goal"] = act[i].toUnits(act[i].st.goal_r);
        o["ratio"] = act[i].st.r;
        o["duty"] = act[i].st.duty;
        o["amps"] = act[i].st.current;
        o["fault"] = faultName(act[i].st.fault);
      }
    }
    d["enabled"] = (bool)actEnable;
    d["estop"] = !estopOk;
    d["vref_v"] = vrefPotRatio * 2 * AdsAsync::LSB_V;
    d["v5_v"] = v5Raw * 2 * AdsAsync::LSB_V;
    char t[80];
    netcfg::topic(t, sizeof(t), "act/state");
    serializeJson(d, out);
    mqtt.publish(t, out);
  }

  if (now - tPwr >= 1000) {
    tPwr = now;
    DynamicJsonDocument d(2048);
    {
      Lock l;
      pm.toJson(d.to<JsonObject>());
    }
    d["expander_ok"] = tca.ok();
    d["i2c_err_a"] = i2cErrA;
    d["i2c_err_b"] = i2cErrB;
    serializeJson(d, out);
    mqtt.publish("esp32/power", out);
  }
}

static void diag() {
  static uint32_t lastLoop = 0;
  static bool lastEstop = true;
  uint32_t now = millis();
  if (lastLoop && now - lastLoop > 200 && mqtt.connected()) {
    char b[64];
    snprintf(b, sizeof(b), "{\"loop_gap_ms\":%lu}", (unsigned long)(now - lastLoop));
    mqtt.publish("esp32/diag", b);
  }
  lastLoop = now;
  if (estopOk != lastEstop && mqtt.connected()) {
    lastEstop = estopOk;
    mqtt.publish("esp32/diag", estopOk ? "{\"estop\":\"released\"}" : "{\"estop\":\"pressed\"}");
  }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(pins::ACT_EN, OUTPUT);
  digitalWrite(pins::ACT_EN, LOW);
  pinMode(pins::PV_EN, OUTPUT);
  digitalWrite(pins::PV_EN, LOW);
  pinMode(pins::LED_ERR, OUTPUT);
  pinMode(pins::ESTOP_SENSE, INPUT);
  pinMode(pins::IMU_INT, INPUT);
  for (auto &p : pins::ACT) analogSetPinAttenuation(p.is_adc, ADC_11db);
  hbridge::begin(20000);
  gLock = xSemaphoreCreateMutex();

  Serial.println("\nsolbot_main v4 starting");
  netcfg::loadConfig();
  loadSettings();
  loadAct();
  loadPower();

  Wire.begin(pins::I2CA_SDA, pins::I2CA_SCL, 400000);
  Wire1.begin(pins::I2CB_SDA, pins::I2CB_SCL, 400000);
  Wire.setTimeOut(20);
  Wire1.setTimeOut(20);

  if (tca.begin(&Wire1, i2caddr::TCA9535)) {
    if (tca.adopted()) {  // ESP32 reset while powered: keep packs as they are
      bool on[NPACK];
      for (int k = 0; k < NPACK; k++) on[k] = tca.get(expbit::BAT_EN0 + k);
      pm.adoptSwitchStates(on);
      Serial.println("TCA9535: adopted latched pack switch state");
    }
  } else {
    Serial.println("TCA9535 missing - pack switches not controllable!");
  }
  adsPwr[0].begin(&Wire1, i2caddr::ADS_PWR0);
  adsPwr[1].begin(&Wire1, i2caddr::ADS_PWR1);
  adsPwr[2].begin(&Wire1, i2caddr::ADS_PWR2);
  adsAct[0].begin(&Wire, i2caddr::ADS_ACT0);
  adsAct[1].begin(&Wire, i2caddr::ADS_ACT1);
  Serial.printf("ADS act %d%d pwr %d%d%d\n", adsAct[0].ok(), adsAct[1].ok(), adsPwr[0].ok(), adsPwr[1].ok(),
                adsPwr[2].ok());
  if (!imuBegin()) Serial.println("BNO085 not found");

  // control loops run regardless of network state
  xTaskCreatePinnedToCore(controlTask, "control", 6144, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(powerTask, "power", 6144, nullptr, 4, nullptr, 1);

  netcfg::begin(onMqtt, onMqttConnect);
}

void loop() {
  if (!netcfg::loop()) return;  // AP portal mode
  diag();
  if (mqtt.connected()) publishTelemetry();
  delay(2);
}
