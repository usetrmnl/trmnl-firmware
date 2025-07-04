#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "special_function.h"

enum class ApiSetupOutcome
{
  Ok,
  DeserializationError,
  StatusError
};

struct ApiSetupResponse
{
  ApiSetupOutcome outcome;
  uint16_t status;
  String api_key;
  String friendly_id;
  String image_url;
  String message;
};

enum class ApiDisplayOutcome
{
  Ok,
  DeserializationError,
};

struct ApiDisplayResponse
{
  ApiDisplayOutcome outcome;
  String error_detail;
  uint64_t status;
  String image_url;
  uint32_t image_url_timeout;
  String filename;
  bool update_firmware;
  String firmware_url;
  uint64_t refresh_rate;
  bool reset_firmware;
  SPECIAL_FUNCTION special_function;
  String action;
};

struct ApiDisplayInputs
{
  String baseUrl;
  String apiKey;
  String friendlyId;
  uint32_t refreshRate;
  String macAddress;
  float batteryVoltage;
  String firmwareVersion;
  String model;
  int rssi;
  int displayWidth;
  int displayHeight;
  SPECIAL_FUNCTION specialFunction;
};

typedef struct
{
  char current_image[100];
  char current_error_message[100];
} ScreenStatus;

typedef struct DeviceStatusStamp
{
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

struct LogWithDetails
{
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
};
