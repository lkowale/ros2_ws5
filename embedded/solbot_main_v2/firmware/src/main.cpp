// solbot_main v2 rev B — compact ESP32-S3 main controller firmware (JLC basic-parts board)
//
// Tasks
//   control (50 Hz): BNO085 (I2C), positions + IS currents (ADC1), H-bridge PWM
//   pv      (10 Hz): bus / 12 V voltage (ESP ADC), PV SSR control
//   loop()         : WiFi/MQTT/OTA, command parsing, telemetry
//
// MQTT (v3-compatible topics marked *)
//   sub  steer_position* {"data":legacy}   lift/set_position* {"data":legacy}
//        motor_direction*                  slider/set_position {"data":metres}
//        <dev>/act/<n|name>/set {"pos":u}|{"raw":legacy}|{"ratio":r}
//        <dev>/act/<n|name>/config {...}   <dev>/act/reset   <dev>/act/enable {"data":0|1}
//        <dev>/power/cmd {"pv":"auto|on|off"} {"ssr2":"mirror|on|off"}
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
#include "pv.h"

constexpr int N = pins::NACT;

static SemaphoreHandle_t gLock;
struct Lock {
  Lock() { xSemaphoreTake(gLock, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(gLock); }
};

static Actuator act[N];
static PvManager pv;
static Adafruit_BNO08x bno(-1);
static Preferences prefs;

struct Settings {
  uint16_t imu_hz = 5;  // v3 published at 5 Hz
  uint16_t ads_hz = 10;
  float pot_mv = 3000;  // voltage at the pot top (3V3A through 1k into a ~10k pot): measure & set
} settings;

struct ImuData {
  float qi = 0, qj = 0, qk = 0, qr = 1, acc_heading = 0;
  float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
  uint32_t last_ms = 0;
} imu;

static volatile bool actEnable = true;
static volatile bool estopOk = false;
static volatile bool imuOk = false;

static const char *DEFAULT_NAMES[N] = {"steer", "lift", "slider", "a4"};

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
  for (int i = 0; i < N; i++) {
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
  for (int i = 0; i < N; i++) {
    char key[4] = {'a', char('0' + i), 0};
    String s = prefs.getString(key, "");
    StaticJsonDocument<768> d;
    if (s.length() && !deserializeJson(d, s)) act[i].cfg.fromJson(d.as<JsonObjectConst>());
  }
  prefs.end();
}

static void savePv() {
  StaticJsonDocument<512> d;
  pv.cfg.toJson(d.to<JsonObject>());
  String s;
  serializeJson(d, s);
  prefs.begin("pv", false);
  prefs.putString("cfg", s);
  prefs.end();
}

static void loadPv() {
  prefs.begin("pv", true);
  String s = prefs.getString("cfg", "");
  prefs.end();
  StaticJsonDocument<512> d;
  if (s.length() && !deserializeJson(d, s)) pv.cfg.fromJson(d.as<JsonObjectConst>());
}

static void saveSettings() {
  prefs.begin("set", false);
  prefs.putUShort("imu_hz", settings.imu_hz);
  prefs.putUShort("ads_hz", settings.ads_hz);
  prefs.putFloat("pot_mv", settings.pot_mv);
  prefs.end();
}

static void loadSettings() {
  prefs.begin("set", true);
  settings.imu_hz = prefs.getUShort("imu_hz", settings.imu_hz);
  settings.ads_hz = prefs.getUShort("ads_hz", settings.ads_hz);
  settings.pot_mv = prefs.getFloat("pot_mv", settings.pot_mv);
  prefs.end();
}

// ------------------------------------------------------------------ IMU
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

static void imuHardReset() {
  digitalWrite(pins::IMU_RST, LOW);
  pinMode(pins::IMU_RST, OUTPUT);
  vTaskDelay(pdMS_TO_TICKS(10));
  pinMode(pins::IMU_RST, INPUT);  // release (breakout has a pull-up)
}

// ------------------------------------------------------------------ control task
static float adcMv(int pin, int n) {
  uint32_t sum = 0;
  for (int k = 0; k < n; k++) sum += analogReadMilliVolts(pin);
  return sum / (float)n;
}

static void controlTask(void *) {
  const float dt = 0.02f;
  TickType_t last = xTaskGetTickCount();
  uint32_t imuRetryMs = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    uint32_t now = millis();
    imuService();
    if (imuOk && now - imu.last_ms > 2000) {  // no data: hardware reset
      imuOk = false;
      imuHardReset();
      imuRetryMs = now + 500;
    }
    if (!imuOk && imuRetryMs && now > imuRetryMs) {
      imuRetryMs = imuBegin() ? 0 : now + 5000;
      imu.last_ms = now;
    }

    // positions: ESP32-S3 ADC1, 16x oversampled (pot top ~pot_mv)
    float r[N];
    bool got[N] = {};
    for (int i = 0; i < N; i++) {
      if (!act[i].cfg.enabled) continue;
      r[i] = adcMv(pins::ACT[i].pos_adc, 16) / settings.pot_mv;
      got[i] = true;
    }
    imuService();

    // IBT-2 IS currents: 1k load, k_ILIS 8500 -> 8.5 A per volt
    float amps[N];
    for (int i = 0; i < N; i++) amps[i] = adcMv(pins::ACT[i].is_adc, 4) * 0.0085f;

    estopOk = digitalRead(pins::ESTOP_SENSE);
    bool allowed = actEnable && estopOk;
    float duty[N];
    {
      Lock l;
      for (int i = 0; i < N; i++) {
        if (got[i]) act[i].updatePosition(r[i], now);
        act[i].updateCurrent(amps[i]);
        duty[i] = act[i].control(dt, allowed, now);
      }
    }
    for (int i = 0; i < N; i++) hbridge::write(i, duty[i]);
    digitalWrite(pins::ACT_EN, actEnable ? HIGH : LOW);
  }
}

// ------------------------------------------------------------------ PV task
static float adcAvgV(int pin) {
  uint32_t s = 0;
  for (int k = 0; k < 16; k++) s += analogReadMilliVolts(pin);
  return s / 16.0f / 1000.0f;
}

static void pvTask(void *) {
  TickType_t last = xTaskGetTickCount();
  float bus = 0, v12 = 0;
  bool first = true;
  uint32_t lastLed = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    float b = adcAvgV(pins::BUS_VS), t = adcAvgV(pins::V12_SNS);
    bus = first ? b : bus + 0.3f * (b - bus);
    v12 = first ? t : v12 + 0.3f * (t - v12);
    first = false;
    bool s1, s2;
    {
      Lock l;
      pv.update(bus, v12, millis());
      s1 = pv.pv_on;
      s2 = pv.ssr2_on;
    }
    digitalWrite(pins::SSR1, s1);
    digitalWrite(pins::SSR2, s2);
    if (millis() - lastLed > 500) {
      lastLed = millis();
      bool fault = !estopOk || !imuOk || pv.low;
      for (auto &a : act) fault |= a.st.fault != ActFault::NONE;
      digitalWrite(pins::LED_ERR, fault);
    }
  }
}

// ------------------------------------------------------------------ MQTT
static int actIndex(const char *s) {
  if (s[0] >= '1' && s[0] < '1' + N && s[1] == 0) return s[0] - '1';
  for (int i = 0; i < N; i++)
    if (!strcmp(s, act[i].cfg.name)) return i;
  return -1;
}

static void publishConfigStatus() {
  DynamicJsonDocument d(4096);
  d["deviceName"] = cfg.deviceName;
  d["ssid"] = cfg.ssid;
  d["mqtt_server"] = cfg.mqtt_server;
  d["mqtt_user"] = cfg.mqtt_user;
  d["mqtt_port"] = cfg.mqtt_port;
  d["ip"] = WiFi.localIP().toString();
  d["mac"] = WiFi.macAddress();
  d["fw"] = "solbot_main v2 revB";
  d["imu_hz"] = settings.imu_hz;
  d["ads_hz"] = settings.ads_hz;
  d["pot_mv"] = settings.pot_mv;
  {
    Lock l;
    JsonArray a = d.createNestedArray("actuators");
    for (auto &x : act) x.cfg.toJson(a.createNestedObject());
    pv.cfg.toJson(d.createNestedObject("power"));
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
  static char buf[1024];
  len = min(len, (unsigned)sizeof(buf) - 1);
  memcpy(buf, payload, len);
  buf[len] = 0;
  if (netcfg::handleConfigTopic(topic, buf)) return;

  DynamicJsonDocument d(1536);
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
    PvMode m;
    Ssr2Mode m2;
    if (parsePvMode(o["pv"], m)) pv.mode = m;
    if (parseSsr2Mode(o["ssr2"], m2)) pv.ssr2_mode = m2;
    return ack(sub, true);
  }
  if (!strcmp(sub, "power/config") && json) {
    {
      Lock l;
      pv.cfg.fromJson(o);
    }
    savePv();
    return ack(sub, true);
  }
  if (!strcmp(sub, "settings") && json) {
    settings.imu_hz = constrain((int)(o["imu_hz"] | settings.imu_hz), 1, 100);
    settings.ads_hz = constrain((int)(o["ads_hz"] | settings.ads_hz), 1, 50);
    settings.pot_mv = constrain((float)(o["pot_mv"] | settings.pot_mv), 1000.0f, 3300.0f);
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
  char out[1200];

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
    DynamicJsonDocument d(1536);
    JsonArray arr = d.createNestedArray("act");
    {
      Lock l;
      for (int i = 0; i < N; i++) {
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
    char t[80];
    netcfg::topic(t, sizeof(t), "act/state");
    serializeJson(d, out);
    mqtt.publish(t, out);
  }

  if (now - tPwr >= 1000) {
    tPwr = now;
    StaticJsonDocument<512> d;
    {
      Lock l;
      pv.toJson(d.to<JsonObject>());
    }
    serializeJson(d, out);
    mqtt.publish("esp32/power", out);
  }
}

static void diag() {
  static uint32_t lastLoop = 0;
  static bool lastEstop = true, lastPv = false;
  uint32_t now = millis();
  if (lastLoop && now - lastLoop > 200 && mqtt.connected()) {
    char b[64];
    snprintf(b, sizeof(b), "{\"loop_gap_ms\":%lu}", (unsigned long)(now - lastLoop));
    mqtt.publish("esp32/diag", b);
  }
  lastLoop = now;
  if (!mqtt.connected()) return;
  if (estopOk != lastEstop) {
    lastEstop = estopOk;
    mqtt.publish("esp32/diag", estopOk ? "{\"estop\":\"released\"}" : "{\"estop\":\"pressed\"}");
  }
  if (pv.pv_on != lastPv) {  // v3 published PV switch changes here
    lastPv = pv.pv_on;
    mqtt.publish("esp32/diag", lastPv ? "{\"pv_switch\": \"on\"}" : "{\"pv_switch\": \"off\"}");
  }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(pins::ACT_EN, OUTPUT);
  digitalWrite(pins::ACT_EN, LOW);
  pinMode(pins::SSR1, OUTPUT);
  digitalWrite(pins::SSR1, LOW);
  pinMode(pins::SSR2, OUTPUT);
  digitalWrite(pins::SSR2, LOW);
  pinMode(pins::LED_ERR, OUTPUT);
  pinMode(pins::ESTOP_SENSE, INPUT);
  pinMode(pins::IMU_INT, INPUT);
  pinMode(pins::IMU_RST, INPUT);
  for (auto &p : pins::ACT) {
    analogSetPinAttenuation(p.is_adc, ADC_11db);
    analogSetPinAttenuation(p.pos_adc, ADC_11db);
  }
  analogSetPinAttenuation(pins::BUS_VS, ADC_11db);
  analogSetPinAttenuation(pins::V12_SNS, ADC_11db);
  hbridge::begin(20000);
  gLock = xSemaphoreCreateMutex();

  Serial.println("\nsolbot_main v2 rev B starting");
  netcfg::loadConfig();
  loadSettings();
  loadAct();
  loadPv();

  Wire.begin(pins::I2C_SDA, pins::I2C_SCL, 400000);
  Wire.setTimeOut(20);
  if (!imuBegin()) Serial.println("BNO085 not found");

  xTaskCreatePinnedToCore(controlTask, "control", 6144, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(pvTask, "pv", 4096, nullptr, 4, nullptr, 1);

  netcfg::begin(onMqtt, onMqttConnect);
}

void loop() {
  if (!netcfg::loop()) return;  // AP portal mode
  diag();
  if (mqtt.connected()) publishTelemetry();
  delay(2);
}
