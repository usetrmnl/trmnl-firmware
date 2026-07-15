// Integration tests for the production /api/display client.
//
// Talks to the real backend at TEST_BACKEND_URL with TEST_MAC_ADDRESS.
// The Access-Token (api_key) is fetched fresh at suite start by calling
// /api/setup once — see fetch_api_key_or_skip(). Avoids needing to hardcode
// a key in test_config.h.

#include <Arduino.h>
#include <unity.h>
#include <WiFi.h>
#include <api-client/display.h>
#include <api-client/setup.h>
#include <special_function.h>
#include <config.h>  // FW_VERSION_STRING, DEVICE_MODEL
#include <display.h> // display_width(), display_height()
#include <test_helpers.h>
#include "tests.h"

// Cached across tests so we only pay for one /api/setup round trip.
static String s_api_key;
static String s_friendly_id;

static void fetch_api_key_or_skip()
{
  if (!s_api_key.isEmpty())
    return;

  ApiSetupInputs setupInputs = {
      .baseUrl = TEST_BACKEND_URL,
      .macAddress = TEST_MAC_ADDRESS,
      .firmwareVersion = FW_VERSION_STRING,
      .model = DEVICE_MODEL,
  };

  ApiSetupResult r = fetchApiSetup(setupInputs);
  TEST_ASSERT_EQUAL_MESSAGE(HTTPS_NO_ERR, r.error,
                            "Pre-test /api/setup call failed at the transport layer");
  TEST_ASSERT_EQUAL_MESSAGE((int)ApiSetupOutcome::Ok, (int)r.response.outcome,
                            "Pre-test /api/setup returned non-Ok — is the test MAC registered?");
  TEST_ASSERT_FALSE_MESSAGE(r.response.api_key.isEmpty(),
                            "Pre-test /api/setup returned an empty api_key");

  s_api_key = r.response.api_key;
  s_friendly_id = r.response.friendly_id;
  Serial.printf("cached api_key (len=%d) + friendly_id=%s from /api/setup\n",
                s_api_key.length(), s_friendly_id.c_str());
}

static ApiDisplayInputs make_display_inputs(float battery_voltage)
{
  // Mirrors the inputs production assembles in bl.cpp around line 1400-1475.
  // Anything that's not under test gets a value the real device would send.
  ApiDisplayInputs inputs = {};
  inputs.baseUrl = TEST_BACKEND_URL;
  inputs.apiKey = s_api_key;
  inputs.friendlyId = s_friendly_id;
  inputs.updateSource = "timer";            // matches wake-from-RTC case
  inputs.refreshRate = SLEEP_TIME_TO_SLEEP; // 900s — production default
  inputs.macAddress = TEST_MAC_ADDRESS;
  inputs.batteryVoltage = battery_voltage;
  inputs.firmwareVersion = FW_VERSION_STRING;
  inputs.firmwareCommit = FW_COMMIT;
  inputs.model = DEVICE_MODEL;
  inputs.rssi = WiFi.RSSI();
  inputs.displayWidth = display_width();
  inputs.displayHeight = display_height();
  inputs.specialFunction = SF_NONE;
  return inputs;
}

static void test_low_battery_returns_low_battery_image(void)
{
  // 3.0V is well below the device's empty threshold — the backend should
  // respond with the low-battery placeholder image.
  ApiDisplayInputs inputs = make_display_inputs(3.0f);

  Serial.printf("calling /api/display with battery=%.2fV\n", inputs.batteryVoltage);

  ApiDisplayResult result = fetchApiDisplay(inputs);

  TEST_ASSERT_EQUAL_MESSAGE(HTTPS_NO_ERR, result.error,
                            "fetchApiDisplay returned a transport error");
  TEST_ASSERT_EQUAL_MESSAGE((int)ApiDisplayOutcome::Ok, (int)result.response.outcome,
                            "Expected Ok outcome from /api/display");

  Serial.printf("response filename: %s\n", result.response.filename.c_str());

  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(0, result.response.filename.indexOf("low_battery"),
                                       "filename should contain 'low_battery' for a low-battery request");
}

void test_api_display(void)
{
  // Setup
  fetch_api_key_or_skip();

  // Tests
  RUN_TEST(test_low_battery_returns_low_battery_image);
}
