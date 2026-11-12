#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include "hardware_types.h"
#include "special_function.h"
#include "trmnl_log.h"

enum class ApiSetupOutcome { Ok, DeserializationError, StatusError };

struct ApiSetupResponse {
  ApiSetupOutcome outcome;
  uint16_t status;
  String api_key;
  String friendly_id;
  String image_url;
  String message;
};

enum class ApiDisplayOutcome {
  Ok,
  DeserializationError,
};

#define DEFAULT_FULL_REFRESH_EVERY 8

struct ApiDisplayResponse {
  ApiDisplayOutcome outcome;
  String error_detail;
  uint64_t status;
  String image_url;
  uint32_t image_url_timeout;
  String filename;
  bool update_firmware;
  bool maximum_compatibility;
  String firmware_url;
  uint64_t refresh_rate;
  uint32_t temp_profile;
  bool reset_firmware;
  SPECIAL_FUNCTION special_function;
  String action;
  String verbose_logging;
  bool keep_wifi_on; // wait for the next check-in awake with Wi-Fi connected, not in deep sleep
  int full_refresh_every; // partial draws between full refreshes, which clear ghosting
};

struct ApiDisplayInputs {
  String baseUrl;
  String apiKey;
  String friendlyId;
  String updateSource;
  uint32_t refreshRate;
  String macAddress;
  float batteryVoltage;
  ChargingStatus chargingStatus;
#ifdef BOARD_TRMNL_X
  int batteryCount;
  int batteryCurrent;
  int currentBatteryCapacity;
  int maxBatteryCapacity;
  float batteryTemperature;
  int stateOfCharge;
  int stateOfHealth;
  // Real gas-gauge (Impedance Track) readings, gathered for comparison even
  // when BYPASS_BQ27427_SOC makes stateOfCharge/etc. above an approximation.
  int gaugeSoc;
  int gaugeHealth;
  int gaugeCapacityRemain;
  int gaugeCapacityFull;
#endif
  String firmwareVersion;
  String firmwareCommit;
  String model;
  String panelId; // EPD revision as hex; empty = unknown (header omitted)
  String panelVcom; // VCOM_DC code from the panel's OTP, "-1" = none programmed; empty = not read (header omitted)
  int rssi;
  String wifiBand;
  int displayWidth;
  int displayHeight;
  SPECIAL_FUNCTION specialFunction;
  bool specialFunctionChosen; // picked on the device: the header names it instead of saying "true"
  UsbStatus usbStatus;
  bool imageCached;
  int prevWakeTime;
  String resetReason; // esp_reset_reason_t name of this boot, e.g. ESP_RST_PANIC
};

using ApiSetupInputs = ApiDisplayInputs;

struct ApiLogInputs {
  String macAddress;
  String apiKey;
};

typedef struct {
  char current_image[100];
  char current_error_message[100];
} ScreenStatus;

typedef struct DeviceStatusStamp {
  int8_t wifi_rssi_level;
  char wifi_status[30];
  uint32_t refresh_rate;
  uint32_t time_since_last_sleep;
  char current_fw_version[10];
  char special_function[100];
  float battery_voltage;
  char wakeup_reason[30];
  uint32_t free_heap_size;
  uint32_t max_alloc_size;

  ScreenStatus screen_status;

} DeviceStatusStamp;

struct LogWithDetails {
  DeviceStatusStamp deviceStatusStamp;
  time_t timestamp;
  int codeline;
  const char *sourceFile;
  const char *logMessage;
  uint32_t logId;
  String filenameCurrent;
  String filenameNew;
  bool logRetry;
  int retryAttempt;
  LogLevel level;
};
