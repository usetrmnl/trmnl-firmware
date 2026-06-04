#include <Arduino.h>
#include <unity.h>
#include <WiFi.h>
#include "WifiCaptive.h"
#include "wifi-types.h"
#include "tests.h"

#if !defined(TEST_WIFI_SSID) || !defined(TEST_WIFI_PASSWORD)
#error "TEST_WIFI_SSID and TEST_WIFI_PASSWORD must be defined in test/integration/test_config.h"
#endif

// Each test starts from a known disconnected state. We don't rely on the
// global setUp/tearDown for this because other groups in this suite (e.g.
// setup_tests) need WiFi connected and would be broken by a suite-wide
// disconnect.
static void disconnect_and_settle()
{
  WiFi.disconnect(true, true);
  delay(500);
}

static void test_wifi_connects_with_valid_credentials(void)
{
  disconnect_and_settle();

  WifiCredentials creds(TEST_WIFI_SSID, TEST_WIFI_PASSWORD);
  wl_status_t status = WifiCaptivePortal.connect(creds);

  TEST_ASSERT_EQUAL_MESSAGE(WL_CONNECTED, status,
                            "Expected WL_CONNECTED after connect() with valid credentials");
  TEST_ASSERT_TRUE_MESSAGE(WiFi.isConnected(),
                           "WiFi.isConnected() should be true after a successful connect()");
}

static void test_wifi_fails_with_wrong_password(void)
{
  disconnect_and_settle();

  WifiCredentials creds(TEST_WIFI_SSID, "definitely-not-the-real-password");
  wl_status_t status = WifiCaptivePortal.connect(creds);

  TEST_ASSERT_NOT_EQUAL_MESSAGE(WL_CONNECTED, status,
                                "connect() should not return WL_CONNECTED with a wrong password");
  TEST_ASSERT_FALSE_MESSAGE(WiFi.isConnected(),
                            "WiFi.isConnected() should be false after a failed connect()");
}

static void test_wifi_fails_with_wrong_ssid(void)
{
  disconnect_and_settle();

  WifiCredentials creds("definitely-not-the-real-ssid", "definitely-not-the-real-password");
  wl_status_t status = WifiCaptivePortal.connect(creds);

  TEST_ASSERT_NOT_EQUAL_MESSAGE(WL_CONNECTED, status,
                                "connect() should not return WL_CONNECTED with a wrong SSID");
  TEST_ASSERT_FALSE_MESSAGE(WiFi.isConnected(),
                            "WiFi.isConnected() should be false after a failed connect()");
}

void test_wifi(void)
{
  RUN_TEST(test_wifi_connects_with_valid_credentials);
  RUN_TEST(test_wifi_fails_with_wrong_password);
  RUN_TEST(test_wifi_fails_with_wrong_ssid);
}
