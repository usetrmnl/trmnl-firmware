#pragma once

#include <Arduino.h>

/// Returns the device's MAC address; overridable by defining TEST_MAC_ADDRESS.
String device_mac_address();