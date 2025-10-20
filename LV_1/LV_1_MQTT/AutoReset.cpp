#include "AutoReset.h"
#include <esp_system.h>

static unsigned long resetIntervalMillis = 0;
static unsigned long lastResetTime = 0;

void AutoReset_init(unsigned long resetIntervalMin) {
  resetIntervalMillis = resetIntervalMin * 60UL * 1000UL;
  lastResetTime = millis();
}

void AutoReset_loop() {
  if (millis() - lastResetTime >= resetIntervalMillis) {
    Serial.println("[AutoReset] Interval reached. Restarting...");
    Serial.flush();
    delay(100);
    ESP.restart();
  }
}
