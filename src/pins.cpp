#include <Arduino.h>
#include <pins.h>
#include <config.h>

void pins_init(void)
{
    pinMode(PIN_INTERRUPT, INPUT);
}