#include <config.h>     // TCA9535_PG_PIN / TCA9535_STAT_PIN (TRMNL X)
#include <power.h>
#include <trmnl_log.h>

#if defined(INCLUDE_TCA9535_POWER)
#include "FastEPD.h"
// Defined in display.cpp. On TRMNL X the BQ25616 PG/STAT lines are wired to the
// TCA9535 I/O expander, which is reached through the FastEPD io* helpers.
extern FASTEPD bbep;

static TCA9535Power powerInstance(bbep, TCA9535_PG_PIN, TCA9535_STAT_PIN);
#elif defined(INCLUDE_GPIO_POWER)
static GPIOPower powerInstance(BQ25616_PG_PIN, BQ25616_STAT_PIN);
#else
static BasePower powerInstance;
#endif

BasePower &power() { return powerInstance; }

#if defined(INCLUDE_TCA9535_POWER)
void otg_turn_on() {
  bbep.ioPinMode(1, OUTPUT);
  bbep.ioWrite(1, HIGH);
  Log_info("OTG turned on");
}

void otg_turn_off() {
  bbep.ioPinMode(1, OUTPUT);
  bbep.ioWrite(1, LOW);
  Log_info("OTG turned off");
}
#endif // INCLUDE_TCA9535_POWER
