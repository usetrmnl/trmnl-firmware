#ifndef E1003_SLEEP_H
#define E1003_SLEEP_H

// Deep-sleep power fix for the Seeed reTerminal E1003 (trmnl-firmware#572).
//
// The problem: in deep sleep the E1003 drew about 5 mA. A pin set with
// digitalWrite() is released the moment the ESP32-S3 sleeps, so the peripheral
// enable lines float. The worst offender is the GT911 touch controller's reset
// line, which has an external pull-up: once released it goes HIGH and the
// controller scans for touches all night, on a board where TRMNL never uses
// touch.
//
// The fix: before sleeping, drive the enable lines LOW and latch them with the
// ESP32's GPIO hold feature so they stay LOW through deep sleep. On boot,
// release the latches before the display driver powers the panel back up.
//
// The same approach was measured on this board by ar0v3r/reTerminal-E1003-ESPHome
// (sleep current dropped below 1 mA) and applied by dmellok/tesserae-device-firmware.
//
// Only compiled for BOARD_SEEED_RETERMINAL_E1003; every call site is guarded.

// Call first thing in setup(), before display_init().
void e1003_release_sleep_holds(void);

// Call in goToSleep(), after display_sleep() and just before esp_deep_sleep_start().
void e1003_park_pins_for_sleep(void);

// Arm the button wake. Uses ext1; build with -D E1003_SLEEP_USE_EXT0 for stock ext0.
void e1003_enable_button_wakeup(int interrupt_pin);

#endif // E1003_SLEEP_H
