#include <Arduino.h>
#include <ArduinoLog.h>
#include <misc/clock.h>

#include "esp_sntp.h"

#define CLOCK_SNTP_TIMEOUT_MS 10000
#define CLOCK_SNTP_POLL_MS    50

uint32_t Clock::getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 200)) {
    Log.info("%s [%d]: Failed to obtain time. \r\n", __FILE__, __LINE__);
    return (0);
  }
  time(&now);
  return now;
}

// Not sure if WiFiClientSecure checks the validity date of the certificate.
// Setting clock just to be sure...
bool Clock::setTimeFromNTP() {
  int iDeltaTime;
  Preferences prefs;

  prefs.begin("data");
  uint32_t u32Epoch = prefs.getUInt("last_sync", 0); // Get the last time sync time
  iDeltaTime = getTime() - u32Epoch;                 // Number of seconds since the last sync
  Log.info("%s [%d]: epoch time: %d iDelta: %d\r\n", __FILE__, __LINE__, getTime(), iDeltaTime);
  if (u32Epoch != 0 && iDeltaTime > 0 && iDeltaTime < 24 * 60 * 60) { // Less than 24h, no need to sync the time
    Log.info("%s [%d]: Skipping time sync\r\n", __FILE__, __LINE__);
    prefs.end();
    return true;
  }
  String ntp = prefs.getString("ntp_server", "time.google.com");

  Log.info("%s [%d]: Using NTP: %s, fallback: pool.ntp.org\r\n", __FILE__, __LINE__, ntp.c_str());

  bool sync_status = sync(prefs, ntp);

  prefs.end();
  return sync_status;
}

bool Clock::sync(Preferences &prefs, const String &ntpServer) {
  bool sync_status = false;
  struct tm timeinfo = {};
  esp_sntp_set_sync_status(SNTP_SYNC_STATUS_RESET);

  configTime(0, 0, ntpServer.c_str(), "pool.ntp.org");

  waitForSync();

  for (int i = 0; i < SNTP_MAX_SERVERS; i++) {
    const char *srv = esp_sntp_getservername(i);
    if (srv && strlen(srv) > 0) {
      Log.info("%s [%d]: SNTP server[%d]: %s\r\n", __FILE__, __LINE__, i, srv);
    }
  }

  Log.info("%s [%d]: Time sync started\r\n", __FILE__, __LINE__);

  const uint32_t start_ms = millis();
  while (millis() - start_ms < CLOCK_SNTP_TIMEOUT_MS) {
    if (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
      break;
    }
    delay(CLOCK_SNTP_POLL_MS);
  }

  if (getLocalTime(&timeinfo)) {
    sync_status = true;
    Log.info("%s [%d]: Time sync succeeded in %d ms\r\n", __FILE__, __LINE__, (int)(millis() - start_ms));
    prefs.putUInt("last_sync", getTime()); // save epoch time of last sync
  } else {
    Log.info("%s [%d]: Time sync failed...\r\n", __FILE__, __LINE__);
  }

  Log.info("%s [%d]: Current time - %s\r\n", __FILE__, __LINE__, asctime(&timeinfo));

  return sync_status;
}

#if defined(INCLUDE_CLOCK_X)
static ClockX clockInstance;
#elif defined(INCLUDE_CLOCK_GEN2)
static ClockGen2 clockInstance;
#else
static Clock clockInstance;
#endif

Clock &systemClock() { return clockInstance; }
