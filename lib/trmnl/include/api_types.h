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

struct ApiSetupInputs {
  String baseUrl;
  String macAddress;
  String firmwareVersion;
  String model;
};

enum class ApiDisplayOutcome {
  Ok,
  DeserializationError,
};

// Protocol v1 (must-have fork): what the server wants the device to do with the panel.
enum V1Action { V1_ACTION_NONE = 0, V1_ACTION_PARTIAL = 1, V1_ACTION_FULL = 2 };
enum V1FullMode { V1_FULL_FULL = 0, V1_FULL_FAST = 1 };
enum V1SleepMode { V1_SLEEP_DEEP = 0, V1_SLEEP_LIGHT = 1 };

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
  String touchbar_mode;
  // protocol v1 (defaults keep stock servers working: full via image_url/filename)
  V1Action v1_action;
  String frame_id;
  String full_url;
  String regions_url;
  V1FullMode full_mode;
  V1SleepMode sleep_mode;
  bool ota_wait;  // server is preparing an update: poll fast, do not draw, do not deep-sleep long
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
  int rssi;
  String wifiBand;
  int displayWidth;
  int displayHeight;
  SPECIAL_FUNCTION specialFunction;
  UsbStatus usbStatus;
  bool imageCached;
  int prevWakeTime;
  String frameId;  // protocol v1: frame currently on the panel (empty = unknown)
};

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
