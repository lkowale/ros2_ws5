// WiFi / MQTT / OTA / AP-mode config portal (compatible with tools/esp32_config from ros2_ws4).
#pragma once
#include <PubSubClient.h>

struct DeviceConfig {
  char ssid[33];
  char password[65];
  char mqtt_server[64];
  char mqtt_user[33];
  char mqtt_pass[65];
  int mqtt_port;
  char deviceName[33];
};

extern DeviceConfig cfg;
extern PubSubClient mqtt;

namespace netcfg {
void loadConfig();
void saveConfig();
// connect WiFi (60 s), start OTA + STA HTTP server; returns false if AP mode was started
bool begin(MQTT_CALLBACK_SIGNATURE, void (*onConnect)());
// returns false while in AP mode (caller should skip the rest of loop)
bool loop();
bool handleConfigTopic(const char *topic, const char *payload);  // <dev>/config|get_config|enter_ap
void topic(char *buf, size_t n, const char *suffix);              // "<dev>/<suffix>"
}  // namespace netcfg
