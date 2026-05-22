#include "device_id.h"

#include <WiFi.h>

String device_mac_address()
{
#ifdef TEST_MAC_ADDRESS
  return TEST_MAC_ADDRESS;
#else
  return WiFi.macAddress();
#endif
}