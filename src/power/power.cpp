#include <DEV_Config.h> // BQ25616_PG_PIN / BQ25616_STAT_PIN (gen2)
#include <config.h>     // TCA9535_PG_PIN / TCA9535_STAT_PIN (TRMNL X)
#include <power.h>

#if defined(INCLUDE_TCA9535_POWER)
#include "FastEPD.h"
// Defined in display.cpp. On TRMNL X the BQ25616 PG/STAT lines are wired to the
// TCA9535 I/O expander, which is reached through the FastEPD io* helpers.
extern FASTEPD bbep;

static TCA9535Power powerInstance(bbep, TCA9535_PG_PIN, TCA9535_STAT_PIN);
#elif defined(INCLUDE_GPIO_POWER)
static GPIOPower powerInstance(BQ25616_PG_PIN, BQ25616_STAT_PIN);
#elif defined(CHARGER_SY6974)
static SY6974Power powerInstance;
#else
static BasePower powerInstance;
#endif

BasePower &power() { return powerInstance; }
