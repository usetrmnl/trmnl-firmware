#ifndef MENU_H
#define MENU_H

// The device menu: next/previous screen, the control scheme's tutorial, Special (the special functions), Settings
// (diagnostics, Wi-Fi reset, shut down). It knows nothing of how it is driven: a control scheme (menu_controls_t) turns
// a board's inputs into up/down/select, and says what to draw as hints for them. The TRMNL X touch bar's is in
// touchbar_actions.cpp.

// The icon left of a menu hint's label
typedef enum {
  MENU_HINT_ICON_NONE,
  MENU_HINT_ICON_UP,
  MENU_HINT_ICON_DOWN,
} menu_hint_icon_t;

// A hint along the bottom of a menu screen telling what a control does
typedef struct {
  const char *label; // nullptr leaves the slot empty
  menu_hint_icon_t icon;
} menu_hint_t;

// Hint slots: under the left, middle and right of the bottom edge (the TRMNL X touch bar's regions)
#define MENU_HINT_SLOTS 3

// An input from a control scheme
enum class menu_input_t {
  NONE,
  UP, // move the arrow up (it wraps around)
  DOWN, // move the arrow down
  SELECT, // choose the item the arrow points at
};

// What one of the controls does, for the tutorial. The tutorial's controls are in hint slot order: the
// ones that read UP, SELECT and DOWN in the menu.
typedef struct {
  const char *title; // e.g. "Left Tap"
  const char *action; // what it does, e.g. "Previous screen", or nullptr
  const char *label; // its pill, filled once it is tried, e.g. "Previous"
} menu_tutorial_control_t;

// A control scheme's tutorial: a screen that explains the controls, then has the user try each one,
// in hint slot order. Using all of them, then any one once more, closes it and marks it done (in
// NVS, `done_key`).
typedef struct {
  const char *item; // its item in the main menu
  const char *title;
  const char *subtitle;
  menu_tutorial_control_t controls[MENU_HINT_SLOTS];
  const char *practice; // asks the user to try each control, in order
  const char *finish; // once they have: how to close it (any control)
  // Once they have: fireworks replace the screen, then `finish` is shown at the bottom; without
  // them, `finish` is shown under the practice prompt
  bool fireworks;
  const char *done_key; // the NVS (Preferences) key for its being done
} menu_tutorial_t;

// How a board's inputs drive the menu
struct menu_controls_t {
  // Drawn at the bottom of every menu screen: MENU_HINT_SLOTS of them, left to right
  const menu_hint_t *hints;
  // Called as the menu opens and closes, e.g. to poll the inputs faster while it is open. The
  // input that opened the menu may still be going on at begin(): it must not be read as an input.
  void (*begin)(void);
  void (*end)(void);
  // Polled while a menu screen is up: an input as it starts, then NONE until the next one starts.
  // The menu draws its response (moves the arrow, shows the next screen) before polling again, so
  // an input that starts (and ends) during a display update should still be reported after it.
  menu_input_t (*read)(void);
  // The controls' tutorial, or nullptr
  const menu_tutorial_t *tutorial;
};

// Opens the menu with `controls` and does what is chosen. Next Screen and the special functions
// return, for the wake to go on and fetch the next screen (with the special function, if chosen);
// everything else sleeps or restarts and does not return.
void menu_open(const menu_controls_t &controls);

// Whether the user finished the tutorial of `controls` (true without one)
bool menu_tutorial_done(const menu_controls_t &controls);

// Shows the tutorial of `controls`, then the current image again, and sleeps: does not return
void menu_open_tutorial(const menu_controls_t &controls);

// The factory's last check before shipping: has the user try each control of `controls` in order, on
// a screen laid out like a tutorial with `check`'s text (its item and done_key unused), then shuts
// down for shipment like Settings' Shut Down (does not return). Returns after a timeout without
// them, with the check still on the screen.
void menu_run_ship_check(const menu_controls_t &controls, const menu_tutorial_t &check);

#endif // MENU_H
