#include <misc/clock.h>

#ifdef INCLUDE_CLOCK_X

#include <Arduino.h>
#include <ArduinoLog.h>
#include <WifiCaptive.h>
#include <globals.h>

#include "modem.h"

bool ClockX::sync(Preferences &prefs, const String &ntpServer) {
  if (!(g_modem && WifiCaptivePortal.getLastCredentials().is5GHz)) {
    return Clock::sync(prefs, ntpServer);
  }

  bool sync_status = false;
  struct tm timeinfo;

  time_t t = g_modem->getSntpTime();
  if (t > 0) {
    struct timeval tv = {t, 0};
    settimeofday(&tv, nullptr);
    getLocalTime(&timeinfo);
    sync_status = true;
    Log.info("%s [%d]: Time synchronization via modem succeed!\r\n", __FILE__, __LINE__);
    prefs.putUInt("last_sync", getTime()); // save epoch time of last sync
  } else {
    Log.info("%s [%d]: Time synchronization via modem failed...\r\n", __FILE__, __LINE__);
  }
  Log.info("%s [%d]: Current time - %s\r\n", __FILE__, __LINE__, asctime(&timeinfo));
  return sync_status;
}

#endif // INCLUDE_CLOCK_X
