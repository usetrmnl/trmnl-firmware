#ifndef TOUCHBAR_ACTIONS_H
#define TOUCHBAR_ACTIONS_H

// Confirmation-flow orchestration and intent dispatch - the app-level side effects built on top
// of touchbar_gesture.h. Left and right taps browse the cached images (cached_images.h).

// Resolves the current navigation intent and dispatches it. Called once per wake.
void process_iqs323_data(void);

void handle_wifi_reset_confirmation();
void handle_power_off_confirmation();

// Redraws the touchbar indicator if a full-screen refresh would have erased it.
void touchbar_redraw_pending_indicator(void);

// Wires up the captive portal's both-corners-held power-off confirmation hook.
void touchbar_init_captive_portal_power_off_hook(void);

#endif // TOUCHBAR_ACTIONS_H
