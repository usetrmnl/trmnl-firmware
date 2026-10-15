#ifndef MENU_H
#define MENU_H

// The device menu: next/previous screen, Special (the special functions). It knows nothing of how
// it is driven: a control scheme (menu_controls_t) turns a board's inputs into up/down/select, and
// says what to draw as hints for them. The TRMNL X touch bar's is in touchbar_actions.cpp.

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
  UP,     // move the arrow up (it wraps around)
  DOWN,   // move the arrow down
  SELECT, // choose the item the arrow points at
};

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
};

// Opens the menu with `controls` and does what is chosen. Next Screen and the special functions
// return, for the wake to go on and fetch the next screen (with the special function, if chosen);
// everything else sleeps and does not return.
void menu_open(const menu_controls_t &controls);

#endif // MENU_H
