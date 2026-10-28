#ifndef TOUCHBAR_ACTIONS_H
#define TOUCHBAR_ACTIONS_H

#include <esp_sleep.h>

// What the TRMNL X touch bar does on a wake - the app-level side effects built on top of
// touchbar_gesture.h: left and right taps browse the cached images (cached_images.h), a middle tap
// opens the menu (menu.h) with the touch bar's control scheme. Until the user finishes the touch bar
// tutorial, any touch shows it instead.

// Logs the touch bar status the wake stub read before the app started. Called once per wake.
void touchbar_log_wake_stub_status(void);

// Starts the IQS323 task and, on a GPIO wake, handles the touch that woke the device. Restarts the
// device if the task fails to start.
void touchbar_start(esp_sleep_wakeup_cause_t wakeup_reason, bool gpio_wakeup);

// Resolves the current navigation intent and dispatches it. Called once per wake.
void process_iqs323_data(void);

// Redraws the touchbar indicator if a full-screen refresh would have erased it.
void touchbar_redraw_pending_indicator(void);

// Polled while the setup portal is up: a 2 s middle press opens the factory's ship check (menu.h,
// menu_run_ship_check), which shuts the device down once each zone has been tapped. Returns true if
// the check timed out instead, leaving its screen up for the caller to replace.
bool touchbar_poll_ship_check(void);

#endif // TOUCHBAR_ACTIONS_H
