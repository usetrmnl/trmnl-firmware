#include <display_session.h>

#include <ArduinoLog.h>
#include <battery.h>
#include <config.h>
#include <device_id.h>
#include <display.h>
#include <globals.h>
#include <logging_parcers.h>
#include <power.h>
#include <trmnl_log.h>
#include <wifi_network.h>

ApiDisplayInputs loadApiDisplayInputs(Preferences &preferences)
{
  ApiDisplayInputs inputs;

  inputs.baseUrl = preferences.getString(PREFERENCES_API_URL, API_BASE_URL);

  Log.info("%s [%d]: baseUrl from preferences: %s\r\n", __FILE__, __LINE__, inputs.baseUrl.c_str());

  char wakeupReasonString[32] = {0};
  if (parseWakeupReasonToStr(wakeupReasonString, sizeof(wakeupReasonString), (esp_sleep_source_t)wakeup_reason))
  {
    inputs.updateSource = String(wakeupReasonString);
  }
  else
  {
    inputs.updateSource = "unknown";
  }

  if (preferences.isKey(PREFERENCES_API_KEY))
  {
    inputs.apiKey = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY, inputs.apiKey.c_str());
  }
  else
  {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY);
  }

  if (preferences.isKey(PREFERENCES_FRIENDLY_ID))
  {
    inputs.friendlyId = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_FRIENDLY_ID, inputs.friendlyId);
  }
  else
  {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_FRIENDLY_ID);
  }

  inputs.refreshRate = refreshInterval.seconds();
  Log.info("%s [%d]: refresh rate: %d\r\n", __FILE__, __LINE__, inputs.refreshRate);

  inputs.macAddress = device_mac_address();
  WiFiStatus wifi = getWiFiStatus();
  inputs.rssi = wifi.rssi;
  inputs.wifiBand = wifi.band;
  inputs.batteryVoltage = vBatt;
  inputs.firmwareVersion = String(FW_VERSION_STRING);
  inputs.firmwareCommit = String(FW_COMMIT);
  inputs.displayWidth = display_width();
  inputs.displayHeight = display_height();
  inputs.model = DEVICE_MODEL;
  inputs.specialFunction = special_function;
  inputs.imageCached = bUsedCachedImage;
  inputs.prevWakeTime = iPrevWakeTime;
  inputs.usbStatus = power().usbStatus();
  inputs.chargingStatus = power().chargingStatus();

#ifdef BOARD_TRMNL_X
  // These getters already return -1 if the last gaugeInit() didn't produce a valid reading.
  inputs.batteryCount = battery_count;
  inputs.stateOfCharge = battery().readSoc();
  inputs.stateOfHealth = battery().readHealth();
  inputs.batteryCurrent = battery().readCurrent();
  inputs.batteryTemperature = battery().readTemperature();
  inputs.currentBatteryCapacity = battery().readCapacityRemain();
  inputs.maxBatteryCapacity = battery().readCapacityFull();
#endif // BOARD_TRMNL_X

  return inputs;
}
