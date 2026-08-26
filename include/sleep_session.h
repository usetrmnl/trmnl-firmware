#pragma once

/**
 * Deep sleep entry, low-power GPIO configuration, and Wi-Fi-fail sleep schedule.
 *
 * GME: goToSleep / goToSleepButtonOnly / config_gpio_for_lp / wifiErrorDeepSleep.
 */

/** Prepare peripherals, enable timer + GPIO wake, enter deep sleep. */
void goToSleep(void);

/** Deep sleep until button only (no timer); used after Wi-Fi retry limit. */
void goToSleepButtonOnly(void);

/** Float/tristate GPIOs for low power (TRMNL X panel/I2C pins). */
void config_gpio_for_lp(void);

/**
 * Wi-Fi connect failure: advance retry schedule via RefreshInterval, then
 * goToSleep or goToSleepButtonOnly at the limit. Does not return.
 */
void wifiErrorDeepSleep(void);
