// Integration tests for TLS session resumption (ResumableWiFiClientSecure).
//
// Two things are proven on the real backend:
//   1. A resumed connect actually engages (it's ~an order of magnitude faster
//      than a full software-ECDHE handshake on the C3).
//   2. Request headers on a RESUMED session reach the server intact: the
//      battery-voltage header is changed between the priming call and the
//      resumed call, and the server must honor the NEW value (low_battery
//      image for 3.0V). Resumption sits below the byte stream, so a corrupted
//      or replayed header set would fail this.
//
// NOTE: RTC_NOINIT survives soft resets, so call #1 of a run may already
// resume a session saved by a previous run. Full-handshake timing is therefore
// baselined with a stock WiFiClientSecure, never with the resumable client.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <api-client/display.h>
#include <api-client/setup.h>
#include <config.h>
#include <display.h>
#include <resumable_wifi_client_secure.h>
#include <special_function.h>
#include <test_helpers.h>
#include <unity.h>

#include "tests.h"

static const char *HOST = "trmnl.app";

// Mirrors api_display_tests.cpp — kept file-internal per suite convention.
static String s_api_key;
static String s_friendly_id;

static void fetch_api_key_or_skip() {
  if (!s_api_key.isEmpty()) return;

  ApiSetupInputs setupInputs = {
      .baseUrl = TEST_BACKEND_URL,
      .macAddress = TEST_MAC_ADDRESS,
      .firmwareVersion = FW_VERSION_STRING,
      .model = DEVICE_MODEL,
  };

  ApiSetupResult r = fetchApiSetup(setupInputs);
  TEST_ASSERT_EQUAL_MESSAGE(HTTPS_NO_ERR, r.error, "Pre-test /api/setup call failed at the transport layer");
  TEST_ASSERT_EQUAL_MESSAGE((int)ApiSetupOutcome::Ok, (int)r.response.outcome,
                            "Pre-test /api/setup returned non-Ok — is the test MAC registered?");
  s_api_key = r.response.api_key;
  s_friendly_id = r.response.friendly_id;
}

static ApiDisplayInputs make_display_inputs(float battery_voltage) {
  ApiDisplayInputs inputs = {};
  inputs.baseUrl = TEST_BACKEND_URL;
  inputs.apiKey = s_api_key;
  inputs.friendlyId = s_friendly_id;
  inputs.updateSource = "timer";
  inputs.refreshRate = SLEEP_TIME_TO_SLEEP;
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

static unsigned long time_connect(WiFiClientSecure &client) {
  client.setInsecure();
  unsigned long start = millis();
  int ok = client.connect(HOST, 443);
  unsigned long elapsed = millis() - start;
  client.stop();
  TEST_ASSERT_EQUAL_MESSAGE(1, ok, "TLS connect to trmnl.app failed");
  return elapsed;
}

static void test_resumed_connect_is_fast(void) {
  WiFiClientSecure stock;
  unsigned long t_full = time_connect(stock); // full handshake baseline

  ResumableWiFiClientSecure prime;
  time_connect(prime); // guarantees a fresh session is saved in RTC

  ResumableWiFiClientSecure resumed;
  unsigned long t_resumed = time_connect(resumed);

  Serial.printf("full handshake: %lu ms, resumed: %lu ms\n", t_full, t_resumed);

  TEST_ASSERT_LESS_THAN_MESSAGE(t_full / 2, t_resumed,
                                "Resumed connect not meaningfully faster — session resume did not engage");
}

static void test_resumed_session_headers_reach_server(void) {
  // Priming call: healthy battery over a (possibly full-handshake) connection.
  ApiDisplayInputs healthy = make_display_inputs(4.1f);
  ApiDisplayResult first = fetchApiDisplay(healthy);
  TEST_ASSERT_EQUAL_MESSAGE(HTTPS_NO_ERR, first.error, "priming /api/display failed");
  TEST_ASSERT_EQUAL_MESSAGE((int)ApiDisplayOutcome::Ok, (int)first.response.outcome,
                            "priming /api/display returned non-Ok");
  TEST_ASSERT_EQUAL_MESSAGE(-1, first.response.filename.indexOf("low_battery"),
                            "server served low_battery for a 4.1V header");

  // Resumed call: only the battery header changed — the server must see 3.0V.
  ApiDisplayInputs low = make_display_inputs(3.0f);
  ApiDisplayResult second = fetchApiDisplay(low);
  TEST_ASSERT_EQUAL_MESSAGE(HTTPS_NO_ERR, second.error, "resumed /api/display failed");
  TEST_ASSERT_EQUAL_MESSAGE((int)ApiDisplayOutcome::Ok, (int)second.response.outcome,
                            "resumed /api/display returned non-Ok");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(0, second.response.filename.indexOf("low_battery"),
                                       "3.0V header not honored on resumed session — headers corrupted?");
}

void test_tls_resume(void) {
  fetch_api_key_or_skip();

  RUN_TEST(test_resumed_connect_is_fast);
  RUN_TEST(test_resumed_session_headers_reach_server);
}