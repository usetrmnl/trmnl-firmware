#include <unity.h>
#include <ArduinoJson.h>
#include <api_types.h>
#include <serialize_log.h>

LogWithDetails input = {
    .deviceStatusStamp = {
        .wifi_rssi_level = -50,
        .wifi_status = "Connected",
        .refresh_rate = 30000,
        .time_since_last_sleep = 120,
        .current_fw_version = "1.2.3",
        .special_function = "None",
        .battery_voltage = 4.2f,
        .wakeup_reason = "Timer",
        .free_heap_size = 50000,
        .max_alloc_size = 40000,
        .screen_status = {
            .current_image = "test.png",
            .current_error_message = "",
        },
    },
    .timestamp = 1609459200, // 2021-01-01 00:00:00
    .codeline = 123,
    .sourceFile = "test.cpp",
    .logMessage = "Test log message",
    .logId = 456,
    .filenameCurrent = "current.png",
    .filenameNew = "new.png",
};

String compact(String input)
{
  JsonDocument doc;
  deserializeJson(doc, input);
  String output;
  serializeJson(doc, output);
  return output;
}

void test_serialize_log(void)
{
  auto expected = compact(R"({
    "created_at": 1609459200,
    "id": 456,
    "message": "Test log message",
    "source_line": 123,
    "source_path": "test.cpp",
    "wifi_signal": -50,
    "wifi_status": "Connected",
    "refresh_rate": 30000,
    "sleep_duration": 120,
    "firmware_version": "1.2.3",
    "special_function": "None",
    "battery_voltage": 4.2,
    "wake_reason": "Timer",
    "free_heap_size": 50000,
    "max_alloc_size": 40000
  })");

  auto result = serialize_log(input);

  TEST_ASSERT_EQUAL_STRING(expected.c_str(), result.c_str());
}

void test_serialize_log_with_retry(void)
{
  auto inputWithRetry = input;
  inputWithRetry.logRetry = true;
  inputWithRetry.retryAttempt = 2;

  auto expected = compact(R"({
    "created_at": 1609459200,
    "id": 456,
    "message": "Test log message",
    "source_line": 123,
    "source_path": "test.cpp",
    "wifi_signal": -50,
    "wifi_status": "Connected",
    "refresh_rate": 30000,
    "sleep_duration": 120,
    "firmware_version": "1.2.3",
    "special_function": "None",
    "battery_voltage": 4.2,
    "wake_reason": "Timer",
    "free_heap_size": 50000,
    "max_alloc_size": 40000,
    "retry": 2
  })");

  String result = serialize_log(inputWithRetry);

  TEST_ASSERT_EQUAL_STRING(expected.c_str(), result.c_str());
}

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void process()
{
  UNITY_BEGIN();
  RUN_TEST(test_serialize_log);
  RUN_TEST(test_serialize_log_with_retry);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}