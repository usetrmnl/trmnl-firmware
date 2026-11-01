#include <ArduinoLog.h>
#include <Preferences.h>
#include <api-client/header.h>
#include <battery.h>
#include <config.h>
#include <device_id.h>
#include <display.h>
#include <epd_panel_rev.h>
#include <globals.h>
#include <pairing_runtime.h>
#include <power.h>
#include <wifi_network.h>

#include "logging_parcers.h"

#define RESET_REASON_CASE(reason)                                                                                      \
  case reason:                                                                                                         \
    return #reason;

// Only the reasons every supported ESP-IDF version defines; a newer one is sent as its number.
static String resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    RESET_REASON_CASE(ESP_RST_UNKNOWN)
    RESET_REASON_CASE(ESP_RST_POWERON)
    RESET_REASON_CASE(ESP_RST_EXT)
    RESET_REASON_CASE(ESP_RST_SW)
    RESET_REASON_CASE(ESP_RST_PANIC)
    RESET_REASON_CASE(ESP_RST_INT_WDT)
    RESET_REASON_CASE(ESP_RST_TASK_WDT)
    RESET_REASON_CASE(ESP_RST_WDT)
    RESET_REASON_CASE(ESP_RST_DEEPSLEEP)
    RESET_REASON_CASE(ESP_RST_BROWNOUT)
    RESET_REASON_CASE(ESP_RST_SDIO)
  default:
    return String((int)reason);
  }
}

ApiDisplayInputs createApiHeaders() {
  ApiDisplayInputs inputs;
  char wakeupReasonString[32] = {0};
  if (parseWakeupReasonToStr(wakeupReasonString, sizeof(wakeupReasonString), (esp_sleep_source_t)wakeup_reason)) {
    inputs.updateSource = String(wakeupReasonString);
  } else {
    inputs.updateSource = "unknown";
  }

  if (preferences.isKey(PREFERENCES_API_KEY)) {
    inputs.apiKey = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY, inputs.apiKey.c_str());
  } else {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY);
  }

  if (preferences.isKey(PREFERENCES_FRIENDLY_ID)) {
    inputs.friendlyId = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_FRIENDLY_ID, inputs.friendlyId);
  } else {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_FRIENDLY_ID);
  }

  inputs.refreshRate = refreshInterval.seconds();
  Log.info("%s [%d]: refresh rate: %d\r\n", __FILE__, __LINE__, inputs.refreshRate);

  inputs.bleKey = pairingPersistentKey();
  if (!inputs.bleKey.isEmpty()) {
    inputs.bleSessionId = pairingIdentity().sessionId;
  }
  inputs.macAddress = device_mac_address();
  WiFiStatus wifi = getWiFiStatus();
  inputs.rssi = wifi.rssi;
  inputs.wifiBand = wifi.band;
  // Not re-read here: the voltage is measured early in boot, before the display and WiFi draw
  // current (battery_voltage in globals.h), and /api/display fills that reading in (display_session.cpp's
  // loadApiDisplayInputs).
  inputs.batteryVoltage = 0.0f;
  inputs.firmwareVersion = String(FW_VERSION_STRING);
  inputs.firmwareCommit = String(FW_COMMIT);
  inputs.displayWidth = display_width();
  inputs.displayHeight = display_height();
  inputs.model = DEVICE_MODEL;
  inputs.panelId = display_panel_rev_string();
  inputs.specialFunction = special_function;
  inputs.specialFunctionChosen = special_function_chosen;
  inputs.imageCached = bUsedCachedImage;
  inputs.prevWakeTime = iPrevWakeTime;
  inputs.resetReason = resetReasonName(esp_reset_reason());
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
  inputs.gaugeSoc = battery().readGaugeSoc();
  inputs.gaugeHealth = battery().readGaugeHealth();
  inputs.gaugeCapacityRemain = battery().readGaugeCapacityRemain();
  inputs.gaugeCapacityFull = battery().readGaugeCapacityFull();
#endif // BOARD_TRMNL_X

  return inputs;
}
