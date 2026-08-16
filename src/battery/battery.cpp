#include <config.h>
#include <battery.h>

#if defined(BOARD_TRMNL_X)
static BQ27427Battery batteryInstance;
#else
static ADCBattery batteryInstance;
#endif

#ifdef INCLUDE_BQ27427
BQ27427Battery &battery() { return batteryInstance; }
#else
BaseBattery &battery() { return batteryInstance; }
#endif
