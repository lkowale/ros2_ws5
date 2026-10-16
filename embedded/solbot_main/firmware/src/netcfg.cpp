#include "netcfg.h"

#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define DEFAULT_SSID "solap"
#define DEFAULT_PASSWORD "solap1234"
#define DEFAULT_MQTT_SERVER "192.168.0.190"
#define DEFAULT_MQTT_USER "mark"
#define DEFAULT_MQTT_PASS "pass"
#define DEFAULT_MQTT_PORT 1883
#define DEFAULT_DEVICE_NAME "ESP32_solbot_main"

DeviceConfig cfg;
static WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

namespace {
Preferences prefs;
WebServer apServer(80);
WebServer staServer(80);
DNSServer dnsServer;
WiFiUDP udp;
bool apModeActive = false;
bool enterAPPending = false;
String apSSID;
void (*onConnectCb)() = nullptr;

const char AP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Config</title>
<style>
  body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:0 20px;background:#1a1a2e;color:#eee}
  input{width:100%;padding:8px;margin:4px 0 12px;box-sizing:border-box;background:#16213e;color:#eee;border:1px solid #0f4c75;border-radius:4px}
  label{font-size:14px;color:#aaa}
  button{width:100%;padding:12px;background:#0f4c75;color:#fff;border:none;border-radius:4px;font-size:16px;cursor:pointer}
  h2{color:#0f4c75}
</style></head><body>
<h2>solbot_main v4 config</h2>
<form method="POST" action="/save">
  <label>Device Name</label><input name="deviceName" value="%DEVNAME%">
  <label>WiFi SSID</label><input name="ssid" value="%SSID%">
  <label>WiFi Password</label><input name="password" type="password">
  <label>MQTT Server</label><input name="mqtt_server" value="%MQTT_SRV%">
  <label>MQTT User</label><input name="mqtt_user" value="%MQTT_USER%">
  <label>MQTT Password</label><input name="mqtt_pass" type="password">
  <label>MQTT Port</label><input name="mqtt_port" type="number" value="%MQTT_PORT%">
  <button type="submit">Save &amp; Reboot</button>
</form></body></html>
)rawliteral";

void copyField(char *dst, size_t n, const char *src) {
  if (src && *src) strlcpy(dst, src, n);
}

// apply a JSON object of config fields; returns true if anything changed
bool applyJson(JsonObjectConst d) {
  bool ch = false;
  struct {
    const char *k;
    char *dst;
    size_t n;
  } f[] = {{"deviceName", cfg.deviceName, sizeof(cfg.deviceName)}, {"ssid", cfg.ssid, sizeof(cfg.ssid)},
           {"password", cfg.password, sizeof(cfg.password)}, {"mqtt_server", cfg.mqtt_server, sizeof(cfg.mqtt_server)},
           {"mqtt_user", cfg.mqtt_user, sizeof(cfg.mqtt_user)}, {"mqtt_pass", cfg.mqtt_pass, sizeof(cfg.mqtt_pass)}};
  for (auto &x : f) {
    const char *v = d[x.k];
    if (v && *v) copyField(x.dst, x.n, v), ch = true;
  }
  if (d.containsKey("mqtt_port")) cfg.mqtt_port = d["mqtt_port"].as<int>(), ch = true;
  return ch;
}

void cors(WebServer &s) { s.sendHeader("Access-Control-Allow-Origin", "*"); }

void startAPMode() {
  apModeActive = true;
  String mac = WiFi.macAddress();
  String apName = "ESP32_Config_" + mac.substring(mac.length() - 5);
  apName.replace(":", "");
  apSSID = apName;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName.c_str());
  delay(100);
  dnsServer.start(53, "*", WiFi.softAPIP());

  apServer.on("/", HTTP_GET, []() {
    String html = String(AP_HTML);
    html.replace("%DEVNAME%", cfg.deviceName);
    html.replace("%SSID%", cfg.ssid);
    html.replace("%MQTT_SRV%", cfg.mqtt_server);
    html.replace("%MQTT_USER%", cfg.mqtt_user);
    html.replace("%MQTT_PORT%", String(cfg.mqtt_port));
    apServer.send(200, "text/html", html);
  });
  apServer.on("/save", HTTP_POST, []() {
    StaticJsonDocument<512> d;
    for (const char *k : {"deviceName", "ssid", "password", "mqtt_server", "mqtt_user", "mqtt_pass", "mqtt_port"})
      if (apServer.hasArg(k) && apServer.arg(k).length()) d[k] = apServer.arg(k);
    if (d.containsKey("mqtt_port")) d["mqtt_port"] = apServer.arg("mqtt_port").toInt();
    applyJson(d.as<JsonObjectConst>());
    netcfg::saveConfig();
    apServer.send(200, "text/html", "<html><body style='background:#1a1a2e;color:#eee;font-family:sans-serif;"
                                    "text-align:center;padding-top:100px'><h2>Config Saved!</h2><p>Rebooting...</p></body></html>");
    delay(1500);
    ESP.restart();
  });
  apServer.on("/api/config", HTTP_GET, []() {
    StaticJsonDocument<512> d;
    d["deviceName"] = cfg.deviceName;
    d["ssid"] = cfg.ssid;
    d["mqtt_server"] = cfg.mqtt_server;
    d["mqtt_user"] = cfg.mqtt_user;
    d["mqtt_port"] = cfg.mqtt_port;
    char buf[512];
    serializeJson(d, buf);
    cors(apServer);
    apServer.send(200, "application/json", buf);
  });
  apServer.on("/api/config", HTTP_POST, []() {
    cors(apServer);
    StaticJsonDocument<512> d;
    if (deserializeJson(d, apServer.arg("plain"))) {
      apServer.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
      return;
    }
    if (applyJson(d.as<JsonObjectConst>())) {
      netcfg::saveConfig();
      apServer.send(200, "application/json", "{\"status\":\"saved\",\"rebooting\":true}");
      delay(1000);
      ESP.restart();
    }
    apServer.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"No changes\"}");
  });
  apServer.on("/api/info", HTTP_GET, []() {
    StaticJsonDocument<256> d;
    d["name"] = cfg.deviceName;
    d["mac"] = WiFi.macAddress();
    d["ap_ssid"] = apSSID;
    char buf[256];
    serializeJson(d, buf);
    cors(apServer);
    apServer.send(200, "application/json", buf);
  });
  apServer.on("/api/config", HTTP_OPTIONS, []() {
    cors(apServer);
    apServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    apServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    apServer.send(204);
  });
  apServer.onNotFound([]() {
    apServer.sendHeader("Location", "/", true);
    apServer.send(302, "text/plain", "");
  });
  apServer.begin();
  Serial.printf("AP mode: %s  %s\n", apName.c_str(), WiFi.softAPIP().toString().c_str());
}

void startSTAServer() {
  staServer.on("/api/info", HTTP_GET, []() {
    StaticJsonDocument<256> d;
    d["name"] = cfg.deviceName;
    d["mac"] = WiFi.macAddress();
    d["ip"] = WiFi.localIP().toString();
    char buf[256];
    serializeJson(d, buf);
    cors(staServer);
    staServer.send(200, "application/json", buf);
  });
  staServer.on("/enter_ap", HTTP_POST, []() {
    cors(staServer);
    staServer.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Switching to AP mode\"}");
    enterAPPending = true;
  });
  staServer.on("/enter_ap", HTTP_OPTIONS, []() {
    cors(staServer);
    staServer.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    staServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    staServer.send(204);
  });
  staServer.begin();
}

void sendUDPBeacon() {
  static uint32_t last = 0;
  if (millis() - last < 5000) return;
  last = millis();
  StaticJsonDocument<128> d;
  d["name"] = cfg.deviceName;
  d["mac"] = WiFi.macAddress();
  d["ip"] = WiFi.localIP().toString();
  char buf[128];
  serializeJson(d, buf);
  udp.beginPacket("255.255.255.255", 12345);
  udp.write((uint8_t *)buf, strlen(buf));
  udp.endPacket();
}

void announce() {
  StaticJsonDocument<256> d;
  d["name"] = cfg.deviceName;
  d["ip"] = WiFi.localIP().toString();
  d["mac"] = WiFi.macAddress();
  d["fw"] = "solbot_main v4";
  char buf[256];
  serializeJson(d, buf);
  mqtt.publish("devices/announce", buf, true);
}

void tryReconnect() {
  static uint32_t last = 0;
  if (last && millis() - last < 5000) return;
  last = millis();
  if (mqtt.connect(cfg.deviceName, cfg.mqtt_user, cfg.mqtt_pass)) {
    Serial.println("MQTT connected");
    char t[80];
    for (const char *s : {"config", "get_config", "enter_ap"}) {
      netcfg::topic(t, sizeof(t), s);
      mqtt.subscribe(t);
    }
    announce();
    if (onConnectCb) onConnectCb();
  } else {
    Serial.printf("MQTT connect failed rc=%d\n", mqtt.state());
  }
}
}  // namespace

namespace netcfg {

void topic(char *buf, size_t n, const char *suffix) { snprintf(buf, n, "%s/%s", cfg.deviceName, suffix); }

void loadConfig() {
  prefs.begin("devcfg", true);
  strlcpy(cfg.ssid, prefs.getString("ssid", DEFAULT_SSID).c_str(), sizeof(cfg.ssid));
  strlcpy(cfg.password, prefs.getString("password", DEFAULT_PASSWORD).c_str(), sizeof(cfg.password));
  strlcpy(cfg.mqtt_server, prefs.getString("mqtt_srv", DEFAULT_MQTT_SERVER).c_str(), sizeof(cfg.mqtt_server));
  strlcpy(cfg.mqtt_user, prefs.getString("mqtt_user", DEFAULT_MQTT_USER).c_str(), sizeof(cfg.mqtt_user));
  strlcpy(cfg.mqtt_pass, prefs.getString("mqtt_pass", DEFAULT_MQTT_PASS).c_str(), sizeof(cfg.mqtt_pass));
  cfg.mqtt_port = prefs.getInt("mqtt_port", DEFAULT_MQTT_PORT);
  strlcpy(cfg.deviceName, prefs.getString("devName", DEFAULT_DEVICE_NAME).c_str(), sizeof(cfg.deviceName));
  prefs.end();
}

void saveConfig() {
  prefs.begin("devcfg", false);
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("password", cfg.password);
  prefs.putString("mqtt_srv", cfg.mqtt_server);
  prefs.putString("mqtt_user", cfg.mqtt_user);
  prefs.putString("mqtt_pass", cfg.mqtt_pass);
  prefs.putInt("mqtt_port", cfg.mqtt_port);
  prefs.putString("devName", cfg.deviceName);
  prefs.end();
}

bool begin(MQTT_CALLBACK_SIGNATURE, void (*onConnect)()) {
  onConnectCb = onConnect;
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setSleep(false);
  WiFi.begin(cfg.ssid, cfg.password);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 60000) delay(250);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi failed -> AP mode");
    startAPMode();
    return false;
  }
  WiFi.setAutoReconnect(true);
  Serial.printf("WiFi IP %s\n", WiFi.localIP().toString().c_str());
  ArduinoOTA.setHostname(cfg.deviceName);
  ArduinoOTA.begin();
  mqtt.setServer(cfg.mqtt_server, cfg.mqtt_port);
  mqtt.setCallback(callback);
  mqtt.setBufferSize(2048);
  startSTAServer();
  return true;
}

bool loop() {
  if (apModeActive) {
    dnsServer.processNextRequest();
    apServer.handleClient();
    return false;
  }
  ArduinoOTA.handle();
  staServer.handleClient();
  sendUDPBeacon();
  if (enterAPPending) {
    enterAPPending = false;
    staServer.stop();
    mqtt.disconnect();
    WiFi.disconnect(true);
    delay(100);
    startAPMode();
    return false;
  }
  if (!mqtt.connected()) tryReconnect();
  mqtt.loop();
  return true;
}

bool handleConfigTopic(const char *t, const char *payload) {
  char want[80], ack[80];
  topic(ack, sizeof(ack), "config_ack");
  topic(want, sizeof(want), "config");
  if (!strcmp(t, want)) {
    StaticJsonDocument<512> d;
    if (!deserializeJson(d, payload) && applyJson(d.as<JsonObjectConst>())) {
      saveConfig();
      mqtt.publish(ack, "{\"status\":\"saved\",\"rebooting\":true}");
      mqtt.loop();
      delay(500);
      ESP.restart();
    }
    return true;
  }
  topic(want, sizeof(want), "enter_ap");
  if (!strcmp(t, want)) {
    mqtt.publish(ack, "{\"status\":\"entering_ap_mode\"}");
    mqtt.loop();
    delay(500);
    enterAPPending = true;
    return true;
  }
  return false;  // get_config handled by main (adds actuator/power config)
}

}  // namespace netcfg
