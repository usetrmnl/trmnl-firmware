#include "device_id.h"

#include <esp_mac.h>

String device_mac_address() {
#ifdef TEST_MAC_ADDRESS
  return TEST_MAC_ADDRESS;
#else
  uint8_t mac[6];
  char macStr[18];

  // Not WiFi.macAddress(): on Arduino 3.x it answers all zeros until the station interface starts, as on the X's modem
  // path.
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
#endif
}
