#pragma once

/**
 * Deep sleep entry and low-power GPIO configuration.
 *
 * GME2 G4: goToSleep / goToSleepButtonOnly / config_gpio_for_lp.
 * GME2 G5: wifiErrorDeepSleep.
 *
 * Public goToSleep remains declared in bl.h for touchbar_actions / WifiCaptive.
 */

/** Prepare peripherals, enable timer + GPIO wake, enter deep sleep. */
void goToSleep(void);

/** Deep sleep until button only (no timer); used after Wi-Fi retry limit. */
void goToSleepButtonOnly(void);

/** Float/tristate GPIOs for low power (TRMNL X panel/I2C pins). */
void config_gpio_for_lp(void);

/**
 * Wi-Fi connect failure: apply retry backoff sleeps, or button-only sleep at the
 * limit (with WIFI_RETRY_LIMIT UI). Does not return on sleep paths.
 */
void wifiErrorDeepSleep(void);
