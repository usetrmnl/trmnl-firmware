#include "device_id.h"

#include "esp_mac.h"

String device_mac_address()
{
#ifdef TEST_MAC_ADDRESS
  return TEST_MAC_ADDRESS;
#else
  uint8_t mac[6] = {};
  esp_efuse_mac_get_default(mac);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
#endif
}