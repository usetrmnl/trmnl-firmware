#pragma once

#include <Arduino.h>

//
// TRMNL X hardware reached through the TCA9535 IO expander (x_io_expander.cpp)
//

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
