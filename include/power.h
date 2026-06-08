#pragma once

#include <hardware_types.h>

/// @brief Resolve USB/VBUS presence for the running board.
///        Safe to call on any board with no guards at the call site — returns
///        UNKNOWN where the hardware isn't supported.
///          - TRMNL X: BQ25616 read via the TCA9535 I/O expander
///          - gen2:    BQ25616 read via a dedicated ESP32-C5 GPIO
UsbStatus get_usb_status(void);

/// @brief Resolve the battery charge state for the running board.
///        Safe to call on any board with no guards at the call site — returns
///        UNKNOWN where the hardware isn't supported.
ChargingStatus get_charging_status(void);
