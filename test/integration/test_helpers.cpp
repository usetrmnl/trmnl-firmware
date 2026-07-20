#include "test_helpers.h"

#include <Arduino.h>
#include <WiFi.h>
#include <unity.h>

void connect_to_wifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(TEST_WIFI_SSID, TEST_WIFI_PASSWORD);

  unsigned long deadline = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    // Dump what the radio can actually see so an SSID mismatch is obvious.
    Serial.printf("connect failed (status %d), configured SSID: '%s'. Visible networks:\n", WiFi.status(),
                  TEST_WIFI_SSID);
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++)
      Serial.printf("  '%s' (ch %d, %d dBm)\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
  }

  TEST_ASSERT_EQUAL_MESSAGE(WL_CONNECTED, WiFi.status(), "WiFi did not connect within 15s — check TEST_WIFI_SSID/PSWD");
}
