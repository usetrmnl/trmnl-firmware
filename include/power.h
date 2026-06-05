#ifndef POWER_H
#define POWER_H

/// USB / VBUS presence. UNKNOWN on boards without USB-detection support.
enum class UsbStatus
{
  UNKNOWN,      // not implemented / not readable on this board
  CONNECTED,    // VBUS (USB) power present
  DISCONNECTED, // running on battery
};

/// Battery charge state reported by the charger IC (BQ25616 on TRMNL X / gen2).
enum class ChargingStatus
{
  UNKNOWN,      // not implemented / not readable on this board
  CHARGING,     // actively charging
  NOT_CHARGING, // charge complete, disabled, or no battery
  FAULT,        // charger fault — not yet implemented
};

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

#endif // POWER_H
