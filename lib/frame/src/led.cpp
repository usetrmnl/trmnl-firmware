//
// Control the Frame's RGB LED
//
#include <Arduino.h>
#include <trmnl_log.h>
#include "frame.h"
#include "comm.h"
#include "pindefine.h"
#include "TCA9555.h"
extern TCA9555 expander2;

// Initialize the Frame's RGB LED
void FrameLEDInit(void)
{
    ledcAttach(LED_PWM_PIN, 5000, 8); // attach pin to channel 0
    ledcWrite(LED_PWM_PIN, 0); // set to off brightness to begin
    expander2.pinMode1(LED_R_IO, OUTPUT);
    expander2.pinMode1(LED_G_IO, OUTPUT);
    expander2.pinMode1(LED_B_IO, OUTPUT);
    setGpioLevel_EXPANDER_2(LED_R_IO, GPIO_HIGH); // disable all LEDs
    setGpioLevel_EXPANDER_2(LED_G_IO, GPIO_HIGH);
    setGpioLevel_EXPANDER_2(LED_B_IO, GPIO_HIGH);
} /* FrameLEDInit() */

//
// Set the color and PWM frequency of the Frame's RGB LED
//
void FrameLEDSet(uint8_t color, uint8_t brightness)
{
// Enable the correct LEDs for this color

    ledcWrite(LED_PWM_PIN, brightness);
    setGpioLevel_EXPANDER_2(LED_R_IO, (color & 1) ? GPIO_LOW : GPIO_HIGH); 
    setGpioLevel_EXPANDER_2(LED_G_IO, (color & 2) ? GPIO_LOW : GPIO_HIGH); 
    setGpioLevel_EXPANDER_2(LED_B_IO, (color & 4) ? GPIO_LOW : GPIO_HIGH); 
} /* FrameLEDSet() */


