#include <power.h>
#include <Arduino.h>
#include <DEV_Config.h>

#ifdef BOARD_TRMNL_X
#include "FastEPD.h"
// Defined in display.cpp. On TRMNL X the BQ25616 PG/STAT lines are wired to the
// TCA9535 I/O expander, which is reached through the FastEPD io* helpers.
extern FASTEPD bbep;
// TCA9535 expander pins (open-drain, LOW = active).
#define BQ25616_PG_PIN 0   // P0_0 — LOW = VBUS OK
#define BQ25616_STAT_PIN 2 // P0_2 — LOW = charging in progress
#endif // BOARD_TRMNL_X

#ifdef BOARD_TRMNL_GEN2
// On Gen2, BQ25616 STAT/PG are on TCA9555 expander1 pins 14/15 (open-drain, LOW = active).
// Use gen2_battery.h helpers which read via the expander directly.
#include "gen2_battery.h"
#endif

UsbStatus get_usb_status(void)
{
#if defined(BOARD_TRMNL_X)
  bbep.ioPinMode(BQ25616_PG_PIN, INPUT);
  return (bbep.ioRead(BQ25616_PG_PIN) == 0) ? UsbStatus::CONNECTED : UsbStatus::DISCONNECTED;
#elif defined(BOARD_TRMNL_GEN2)
  // PG# on expander1 pin 15: LOW = VBUS present
  Gen2BatteryStatus s = gen2_batteryRead();
  return (s.valid && s.power_good) ? UsbStatus::CONNECTED : UsbStatus::DISCONNECTED;
#else
  return UsbStatus::UNKNOWN;
#endif
}

ChargingStatus get_charging_status(void)
{
  // BQ25616 STAT: LOW = actively charging, HIGH = charge complete/disabled.
#if defined(BOARD_TRMNL_X)
  bbep.ioPinMode(BQ25616_STAT_PIN, INPUT);
  return (bbep.ioRead(BQ25616_STAT_PIN) == 0) ? ChargingStatus::CHARGING : ChargingStatus::NOT_CHARGING;
#elif defined(BOARD_TRMNL_GEN2)
  // STAT on expander1 pin 14: LOW = charging
  Gen2BatteryStatus s = gen2_batteryRead();
  return (s.valid && s.charging) ? ChargingStatus::CHARGING : ChargingStatus::NOT_CHARGING;
#else
  return ChargingStatus::UNKNOWN;
#endif
}
