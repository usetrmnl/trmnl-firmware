#pragma once

/**
 * Deep sleep entry and low-power GPIO configuration.
 *
 * GME: goToSleep / goToSleepButtonOnly / config_gpio_for_lp. wifiErrorDeepSleep
 * remains in bl.cpp until a follow-up PR.
 */

/** Prepare peripherals, enable timer + GPIO wake, enter deep sleep. */
void goToSleep(void);

/** Deep sleep until button only (no timer); used after Wi-Fi retry limit. */
void goToSleepButtonOnly(void);

/** Float/tristate GPIOs for low power (TRMNL X panel/I2C pins). */
void config_gpio_for_lp(void);
