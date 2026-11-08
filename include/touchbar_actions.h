#ifndef TOUCHBAR_ACTIONS_H
#define TOUCHBAR_ACTIONS_H

#include <esp_sleep.h>

// What the TRMNL X touch bar does on a wake - the app-level side effects built on top of
// touchbar_gesture.h: left and right taps ask /api/display for the previous or next screen (shown from
// the cache when it's there), a middle tap opens the menu (menu.h) with the touch bar's control scheme.
// Closing the menu keeps the device awake until 20 s pass without a touch, for the next tap.
// Until the user finishes the touch bar tutorial, any touch shows it instead. While locked (Settings)
// touches are ignored, except a 5 s middle hold, which asks to confirm with a 3 s hold, then unlocks.
// Any reset but a deliberate restart (e.g. OTA) also unlocks it.

// Logs the touch bar status the wake stub read before the app started. Called once per wake.
void touchbar_log_wake_stub_status(void);

// Starts the IQS323 task and, on a GPIO wake, handles the touch that woke the device. Restarts the
// device if the task fails to start.
void touchbar_start(esp_sleep_wakeup_cause_t wakeup_reason, bool gpio_wakeup);

// Resolves the current navigation intent and dispatches it. Called once per wake. If that closes the
// menu, waits for a tap to dispatch instead, and sleeps (does not return) if none comes or the menu
// locked the touch bar. While locked, returns only when a refresh is due.
void process_iqs323_data(void);

// After a touch, stays awake until 20 s pass without a touch
void touchbar_stay_awake(void (*show_screen)(void));

// Whether a touch's arrow indicator is waiting to redraw
bool touchbar_has_pending_indicator(void);

// Redraws the touchbar indicator if a full-screen refresh would have erased it.
void touchbar_redraw_pending_indicator(void);

// Polled while the setup portal is up: a 2 s middle press opens the factory's ship check (menu.h,
// menu_run_ship_check), which shuts the device down once each zone has been tapped, unless the touch
// bar is locked. Returns true if the check timed out instead, leaving its screen up for the caller to
// replace.
bool touchbar_poll_ship_check(void);

#endif // TOUCHBAR_ACTIONS_H
