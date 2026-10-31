// WAS (BMW position sensor) bench test on ESP32 DevKit V1.
// Sensor pin 4 (signal) -> GPIO4 (ADC2, OK while WiFi is off), sensor powered from 3V3.
// Prints calibrated mV, raw ADC, and running min/max. Send 'r' to reset min/max.
#include <Arduino.h>

const int WAS_PIN = 4;
const int SAMPLES = 32;

uint32_t mv_min = UINT32_MAX, mv_max = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(WAS_PIN, ADC_11db);  // ~0-3.1 V range
  delay(500);
  Serial.println("WAS test: GPIO4. 'r' resets min/max.");
}

void loop() {
  uint32_t mv = 0, raw = 0;
  for (int i = 0; i < SAMPLES; i++) {
    mv += analogReadMilliVolts(WAS_PIN);
    raw += analogRead(WAS_PIN);
  }
  mv /= SAMPLES;
  raw /= SAMPLES;

  if (Serial.available() && Serial.read() == 'r') {
    mv_min = UINT32_MAX;
    mv_max = 0;
  }
  mv_min = min(mv_min, mv);
  mv_max = max(mv_max, mv);

  Serial.printf("mV=%4u raw=%4u  min=%4u max=%4u span=%4u\n",
                mv, raw, mv_min, mv_max, mv_max - mv_min);
  delay(100);
}
