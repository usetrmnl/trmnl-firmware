#ifndef TOUCHBAR_ACTIONS_H
#define TOUCHBAR_ACTIONS_H

// What the TRMNL X touch bar does on a wake - the app-level side effects built on top of
// touchbar_gesture.h: left and right taps browse the cached images (cached_images.h), a middle tap
// opens the menu (menu.h) with the touch bar's control scheme.

// Resolves the current navigation intent and dispatches it. Called once per wake.
void process_iqs323_data(void);

// Redraws the touchbar indicator if a full-screen refresh would have erased it.
void touchbar_redraw_pending_indicator(void);

#endif // TOUCHBAR_ACTIONS_H
