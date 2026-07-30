#include <Arduino.h>
#include <ArduinoLog.h>
#include <misc/time.h>

uint32_t getTime(void) {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 200)) {
    Log.info("%s [%d]: Failed to obtain time. \r\n", __FILE__, __LINE__);
    return (0);
  }
  time(&now);
  return now;
}
