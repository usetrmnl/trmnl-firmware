#include <DEV_Config.h> // defines FAKE_BATTERY_VOLTAGE for some boards
#include <battery.h>
#include <config.h>

#if defined(FAKE_BATTERY_VOLTAGE)
static FakeBattery batteryInstance;
#elif defined(BOARD_TRMNL_X) || defined(BOARD_FRAME)
static BQ27427Battery batteryInstance;
#elif defined(PIN_VBAT_SWITCH)
static SeeedBattery batteryInstance(PIN_BATTERY, PIN_VBAT_SWITCH, VBAT_SWITCH_LEVEL);
#elif defined(PIN_BATTERY)
static ADCBattery batteryInstance(PIN_BATTERY);
#else
static FakeBattery batteryInstance;
#endif

#ifdef INCLUDE_BQ27427
BQ27427Battery &battery() { return batteryInstance; }
#else
BaseBattery &battery() { return batteryInstance; }
#endif
