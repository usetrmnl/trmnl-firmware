#pragma once

#include <Arduino.h>

//
// TRMNL X hardware reached through the TCA9535 IO expander (x_io_expander.cpp)
//

void config_bma530_interrupt();

/**
 * @brief Rotate subsequent drawing to match the device's physical orientation
 * @param orientation One of PORTRAIT_UP_RIGHT / LANDSCAPE_LEFT / PORTRAIT_UP_DOWN / LANDSCAPE_RIGHT (accelerometer.h)
 * @return none
 */
void display_set_orientation(uint8_t orientation);
void config_tca95535_pins_for_lp();
void enter_shipment_sleep();
void BQ27427_reset();
void otg_turn_on();
void otg_turn_off();

typedef enum {
  BATTERY_NONE = 0, // no battery — pin stayed HIGH (timeout >6000 µs)
  BATTERY_ONE = 1,  // one 6K cell — discharge > 750 µs
  BATTERY_TWO = 2,  // two 6K cells — discharge ≤ 750 µs
} battery_count_t;

battery_count_t detect_battery_count();

extern "C" {
  void modem_enter_bootloader();
  void modem_reset_target();
}

// Number of progress dots shown by display_show_orientation_confirm(), one
// filling in per second over the orientation-change confirm window.
#define ORIENTATION_CONFIRM_DOT_COUNT 5

/**
 * @brief Draw one frame of the "Changing device orientation..." confirmation screen
 * @param logo_buffer G5-compressed logo image (e.g. storedLogoOrDefault(1))
 * @param dots_filled Number of progress dots to show filled (0..ORIENTATION_CONFIRM_DOT_COUNT).
 *                  0 does a full refresh (establishes the base image); any other
 *                  value does a fast partial refresh.
 * @return none
 */
void display_show_orientation_confirm(uint8_t *logo_buffer, int dots_filled);
