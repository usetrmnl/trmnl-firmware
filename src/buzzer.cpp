#include <Arduino.h>
#include <config.h>

#ifdef PIN_BUZZER

#include "buzzer.h"
#include "trmnl_log.h"

#ifndef BUZZER_FREQ
#define BUZZER_FREQ 2700
#endif

#define BUZZER_CHANNEL 0
#define BUZZER_RESOLUTION 8
#define BUZZER_DUTY 128 // 50% duty cycle

void buzzer_init(void)
{
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    Log_info("Buzzer initialized on GPIO %d", PIN_BUZZER);
}

void buzzer_on(void)
{
    ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RESOLUTION);
    ledcAttachPin(PIN_BUZZER, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, BUZZER_DUTY);
}

void buzzer_off(void)
{
    ledcWrite(BUZZER_CHANNEL, 0);
    ledcDetachPin(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
}

void buzzer_beep(unsigned long duration_ms)
{
    buzzer_on();
    delay(duration_ms);
    buzzer_off();
}

void buzzer_beep_pattern(int count, unsigned long on_ms, unsigned long off_ms)
{
    for (int i = 0; i < count; i++) {
        buzzer_on();
        delay(on_ms);
        buzzer_off();
        if (i < count - 1) {
            delay(off_ms);
        }
    }
}

#endif // PIN_BUZZER
