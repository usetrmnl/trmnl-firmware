#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <unity.h>

#include "WifiCaptive.h"
#include "tests.h"
#include "wifi-types.h"

#if !defined(TEST_WIFI_SSID) || !defined(TEST_WIFI_PASSWORD)
#error "TEST_WIFI_SSID and TEST_WIFI_PASSWORD must be defined in test/integration/test_config.h"
#endif

// Each test starts from a known disconnected state. We don't rely on the
// global setUp/tearDown for this because other groups in this suite (e.g.
// setup_tests) need WiFi connected and would be broken by a suite-wide
// disconnect.
static void disconnect_and_settle() {
  WiFi.disconnect(true, true);
  delay(500);
}

static void test_wifi_connects_with_valid_credentials(void) {
  disconnect_and_settle();

  WifiCredentials creds(TEST_WIFI_SSID, TEST_WIFI_PASSWORD);
  wl_status_t status = WifiCaptivePortal.connect(creds).status;

  TEST_ASSERT_EQUAL_MESSAGE(WL_CONNECTED, status, "Expected WL_CONNECTED after connect() with valid credentials");
  TEST_ASSERT_TRUE_MESSAGE(WiFi.isConnected(), "WiFi.isConnected() should be true after a successful connect()");
}

static void test_wifi_fails_with_wrong_password(void) {
  disconnect_and_settle();

  WifiCredentials creds(TEST_WIFI_SSID, "definitely-not-the-real-password");
  wl_status_t status = WifiCaptivePortal.connect(creds).status;

  TEST_ASSERT_NOT_EQUAL_MESSAGE(WL_CONNECTED, status, "connect() should not return WL_CONNECTED with a wrong password");
  TEST_ASSERT_FALSE_MESSAGE(WiFi.isConnected(), "WiFi.isConnected() should be false after a failed connect()");
}

static void test_wifi_fails_with_wrong_ssid(void) {
  disconnect_and_settle();

  WifiCredentials creds("definitely-not-the-real-ssid", "definitely-not-the-real-password");
  wl_status_t status = WifiCaptivePortal.connect(creds).status;

  TEST_ASSERT_NOT_EQUAL_MESSAGE(WL_CONNECTED, status, "connect() should not return WL_CONNECTED with a wrong SSID");
  TEST_ASSERT_FALSE_MESSAGE(WiFi.isConnected(), "WiFi.isConnected() should be false after a failed connect()");
}

// Mirrors an AP that moved channel between wakes, leaving the cached hint stale.
static void test_stale_fast_connect_hint_still_connects(void) {
  disconnect_and_settle();

  uint32_t now = (uint32_t)time(nullptr);
  // An unset clock makes tryFastConnect skip itself, so this would pass without exercising the fallback.
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, now, "System clock must be set or this test proves nothing");

  WifiCredentials creds(TEST_WIFI_SSID, TEST_WIFI_PASSWORD);
  creds.bssid = "02:00:00:00:00:01"; // locally administered, cannot be on air
  creds.channel = 1;
  creds.lastFullScanEpoch = now;

  wl_status_t status = WifiCaptivePortal.connect(creds).status;

  TEST_ASSERT_EQUAL_MESSAGE(WL_CONNECTED, status,
                            "A stale BSSID must fall back to a full channel scan and still connect");
}

// The stale hint and the network are both gone, so the fallback scan finds nothing either and has
// to report that rather than hang or inherit the fast connect's answer.
static void test_stale_fast_connect_hint_on_an_absent_network_fails(void) {
  disconnect_and_settle();

  WifiCredentials creds("definitely-not-the-real-ssid", "definitely-not-the-real-password");
  creds.bssid = "02:00:00:00:00:01";
  creds.channel = 1;
  creds.lastFullScanEpoch = (uint32_t)time(nullptr);

  wl_status_t status = WifiCaptivePortal.connect(creds).status;

  TEST_ASSERT_NOT_EQUAL_MESSAGE(WL_CONNECTED, status, "connect() should not report a connection it never made");
  TEST_ASSERT_FALSE_MESSAGE(WiFi.isConnected(), "WiFi.isConnected() should be false after a failed connect()");
}

void test_wifi(void) {
  RUN_TEST(test_wifi_connects_with_valid_credentials);
  RUN_TEST(test_wifi_fails_with_wrong_password);
  RUN_TEST(test_wifi_fails_with_wrong_ssid);
  RUN_TEST(test_stale_fast_connect_hint_still_connects);
  RUN_TEST(test_stale_fast_connect_hint_on_an_absent_network_fails);
}
