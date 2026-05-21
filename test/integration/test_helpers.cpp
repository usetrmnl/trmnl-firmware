#include "test_helpers.h"

#include <Arduino.h>
#include <WiFi.h>
#include <unity.h>

void connect_to_wifi()
{
  if (WiFi.status() == WL_CONNECTED)
    return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(TEST_WIFI_SSID, TEST_WIFI_PASSWORD);

  unsigned long deadline = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline)
  {
    delay(100);
  }
  TEST_ASSERT_EQUAL_MESSAGE(WL_CONNECTED, WiFi.status(),
                            "WiFi did not connect within 15s — check TEST_WIFI_SSID/PSWD");
}
