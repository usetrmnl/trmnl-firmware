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

/** Deep sleep until button only (no timer). Currently has no callers (wifiErrorDeepSleep now
 * sleeps on the slow-retry timer at the limit). */
void goToSleepButtonOnly(void);

/** Float/tristate GPIOs for low power (TRMNL X panel/I2C pins). */
void config_gpio_for_lp(void);

/**
 * Wi-Fi connect failure: sleep on the Wi-Fi retry backoff; at MAX_QUIET_SLOW_RETRIES show
 * WIFI_FAILED and reset the count (still a timed sleep). Does not return.
 */
void wifiErrorDeepSleep(void);
