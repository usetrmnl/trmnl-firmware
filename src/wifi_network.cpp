#include <wifi_network.h>
#include <WiFi.h>
#include <WifiCaptive.h>
#include <trmnl_log.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <config.h>

#ifdef BOARD_TRMNL_X
// Defined in bl.cpp; set once during bl_init(). Only used on the 5 GHz path.
#include "modem.h"
extern Modem *g_modem;
#endif // BOARD_TRMNL_X

namespace {
constexpr unsigned long WIFI_RECONNECT_TIMEOUT_MS = 10000;
}

String getWifiClientHostname(void)
{
  Preferences prefs;
  if (!prefs.begin("data", true))
    return String(WIFI_CLIENT_HOSTNAME_PREFIX);

  String customHostName = prefs.getString(PREFERENCES_HOSTNAME, "");
  if (customHostName.length() > 0)
  {
    prefs.end();
    return customHostName;
  }

  String hostname = WIFI_CLIENT_HOSTNAME_PREFIX;
  String friendly_id = prefs.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
  prefs.end();
  if (friendly_id.length() > 0)
    hostname += "-" + friendly_id;
  return hostname;
}

WiFiStatus getWiFiStatus(void)
{
  WiFiStatus status;
  status.rssi = WiFi.RSSI();
  status.band = WiFi.channel() >= 36 ? "5" : "2.4";

#ifdef BOARD_TRMNL_X
  // On the 5 GHz path the ESP32-C5 modem owns the Wi-Fi link, so the host's
  // WiFi.RSSI() reads 0; query the modem for the real signal strength.
  if (g_modem && WifiCaptivePortal.getLastCredentials().is5GHz)
  {
    status.rssi = g_modem->getSignalRssi();
    status.band = "5";
  }
#endif // BOARD_TRMNL_X

  return status;
}

bool connectWithSavedCredentials(void) {
  if (!WifiCaptivePortal.isSaved())
    return false;

#ifdef BOARD_TRMNL_X
  if (g_modem)
  {
    WifiCredentials creds = WifiCaptivePortal.getLastCredentials();
    if (creds.is5GHz)
      return g_modem->connectToNetwork(creds.ssid, creds.pswd, getWifiClientHostname());
  }
#endif

  bool connected = WifiCaptivePortal.autoConnect();
  if (connected) {
    String hostname = getWifiClientHostname();
    if (MDNS.begin(hostname.c_str())) {
      Log_info("mDNS started: %s.local", hostname.c_str());
    } else {
      Log_error("mDNS failed to start");
    }
  }
  return connected;
}

bool ensureWifiConnected(void) {
  if (WiFi.status() == WL_CONNECTED)
    return true;

  Log_info("%s [%d]: WiFi disconnected, reconnecting...\r\n", __FILE__, __LINE__);

  if (!WifiCaptivePortal.isSaved())
  {
    Log_error("%s [%d]: No saved WiFi credentials\r\n", __FILE__, __LINE__);
    return false;
  }

  WiFi.reconnect();
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_RECONNECT_TIMEOUT_MS)
    delay(100);

  if (WiFi.status() == WL_CONNECTED)
    return true;

  return connectWithSavedCredentials();
}
