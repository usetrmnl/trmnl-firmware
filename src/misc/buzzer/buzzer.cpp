#include <misc/buzzer.h>

#ifndef BUZZER_FREQ
#define BUZZER_FREQ 2700
#endif

#if defined(PIN_BUZZER)
static PWMBuzzer buzzerInstance(PIN_BUZZER, BUZZER_FREQ);
#else
static BaseBuzzer buzzerInstance;
#endif

BaseBuzzer &buzzer() { return buzzerInstance; }
