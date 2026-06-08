#include <wifi_network.h>
#include <WiFi.h>
#include <WifiCaptive.h>

#ifdef BOARD_TRMNL_X
// Defined in bl.cpp; set once during bl_init(). Only used on the 5 GHz path.
#include "modem.h"
extern Modem *g_modem;
#endif // BOARD_TRMNL_X

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
