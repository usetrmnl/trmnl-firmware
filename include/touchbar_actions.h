#ifndef TOUCHBAR_ACTIONS_H
#define TOUCHBAR_ACTIONS_H

// Confirmation-flow orchestration, cached-image cycling, and intent dispatch
// - the app-level side effects built on top of touchbar_gesture.h.

// Inserts/refreshes new_path in the cached-image playlist order, anchored
// after prev_path's position. Called whenever a new image is cached.
void update_playlist_order(const char *new_path, const char *prev_path);

// Resolves the current navigation intent and dispatches it. Called once per wake.
void process_iqs323_data(void);

void handle_wifi_reset_confirmation();
void handle_power_off_confirmation();

// Redraws the touchbar indicator if a full-screen refresh would have erased it.
void touchbar_redraw_pending_indicator(void);

// Wires up the captive portal's both-corners-held power-off confirmation hook.
void touchbar_init_captive_portal_power_off_hook(void);

#endif // TOUCHBAR_ACTIONS_H
