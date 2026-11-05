#include <battery.h>
#include <config.h>

#if defined(BOARD_TRMNL_X) || defined ( BOARD_TRMNL_GEN2 )
static BQ27427Battery batteryInstance;
#else
static ADCBattery batteryInstance;
#endif

#ifdef INCLUDE_BQ27427
BQ27427Battery &battery() { return batteryInstance; }
#else
BaseBattery &battery() { return batteryInstance; }
#endif
