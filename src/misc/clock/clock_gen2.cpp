#include <Arduino.h>
#include <misc/clock.h>

void ClockGen2::waitForSync() {
  // This seems to be necessary only on the ESP32-C5, otherwise NTP will fail 100% of the time
  // Wait until a valid time is received from the NTP server
  // 1577836800 is the Unix time for Jan 1, 2020
  time_t now = 0;
  while (time(&now) < 1577836800) {
    vTaskDelay(50);
  }
}
