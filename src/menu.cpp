#include "menu.h"

// The TRMNL X is the only board with a control scheme for the menu so far. Another board needs one
// (menu_controls_t), and a look at the X-only parts here: the drawing (FastEPD, sized for the X's
// panel), Shut Down (shipment mode).
#ifdef BOARD_TRMNL_X

#include <Preferences.h>
#include <WifiCaptive.h>
#include <battery.h>
#include <bl.h>
#include <config.h>
#include <display.h>
#include <globals.h>
#include <power.h>
#include <qa.h>
#include <refresh_interval.h>
#include <trmnl_log.h>

#include "FastEPD.h"
#include "cached_images.h"
#include "fonts.h"
#include "messages.h"

extern FASTEPD bbep; // display.cpp

#define MENU_TIMEOUT_MS       15000
#define MENU_POLL_INTERVAL_MS 20

// The control scheme of the menu that is open
static const menu_controls_t *menu_controls = nullptr;

// A menu screen
typedef struct {
  const char *const *items; // item labels, top to bottom
  int count;
  int selected;             // the item the arrow points at
  const char *const *body;  // lines of text above the items, or nullptr; a "label\tvalue" line
  int body_count;           // shows the value in a second column
  const menu_hint_t *hints; // MENU_HINT_SLOTS of them, or nullptr
  bool battery;             // the battery icon in the top right corner
} menu_view_t;

// Menu screen layout
#define MENU_ITEM_SPACING  110 // baseline to baseline
#define MENU_ARROW_SIZE    40    // arrow height; it is half as wide
#define MENU_ARROW_GAP     40     // between the arrow and the item text
#define MENU_HINT_MARGIN   40   // from the bottom of the screen to the hint baselines
#define MENU_HINT_PAD_X    56    // around a hint, inside its pill
#define MENU_HINT_PAD_Y    18
#define MENU_HINT_BORDER   3    // the pill's outline
#define MENU_HINT_ICON_GAP 16 // between a hint's icon and its label
#define MENU_BODY_SPACING  64  // baseline to baseline of the text above the items
#define MENU_BODY_GAP      90      // from the last body line to the first item
#define MENU_BODY_VALUE_X  380 // a "label\tvalue" line's value, from the text column

// Body lines of the menu on the screen, for the arrow's position
static int menu_body_count = 0;

// Fill a triangle pointing in the direction dir (0 = right, 1 = up, 2 = down) centered on (cx, cy).
// size is its extent along the base; the tip is size/2 from the base.
static void menu_fill_triangle(int cx, int cy, int size, int dir, uint8_t color) {
  int half = size / 2;
  for (int i = 0; i <= half; i++) {
    int len = size - 2 * i; // length of the line i pixels from the base
    switch (dir) {
    case 0:
      bbep.fillRect(cx - half / 2 + i, cy - len / 2, 1, len, color);
      break;
    case 1:
      bbep.fillRect(cx - len / 2, cy + half / 2 - i, len, 1, color);
      break;
    case 2:
      bbep.fillRect(cx - len / 2, cy - half / 2 + i, len, 1, color);
      break;
    }
  }
}

// From the first body line's baseline to the first item's, 0 without a body
static int menu_body_height(void) {
  return menu_body_count > 0 ? (menu_body_count - 1) * MENU_BODY_SPACING + MENU_BODY_GAP : 0;
}

// Baseline of the top line: the body, if any, then the items are centered vertically
static int menu_top_baseline(int count) {
  return (bbep.height() - menu_body_height() - (count - 1) * MENU_ITEM_SPACING) / 2;
}

// Baseline of the first item
static int menu_first_baseline(int count) { return menu_top_baseline(count) + menu_body_height(); }

// The body: lines of text above the items. A "label\tvalue" line shows the value in a second column.
static void menu_draw_body(const char *const *body, int count) {
  const int x = bbep.width() / 3;
  int baseline = menu_top_baseline(count);
  for (int i = 0; i < menu_body_count; i++, baseline += MENU_BODY_SPACING) {
    const char *tab = strchr(body[i], '\t');
    bbep.setFont(Inter_18);
    if (!tab) {
      bbep.drawString(body[i], x, baseline);
      continue;
    }
    char label[48];
    size_t len = min((size_t)(tab - body[i]), sizeof(label) - 1);
    memcpy(label, body[i], len);
    label[len] = '\0';
    bbep.drawString(label, x, baseline);
    bbep.setFont(Roboto_Black_24);
    bbep.drawString(tab + 1, x + MENU_BODY_VALUE_X, baseline);
  }
}

// The arrow box for an item, left of the text column
static void menu_arrow_rect(int count, int index, BB_RECT *pRect) {
  bbep.setFont(Roboto_Black_24);
  BB_RECT cap;
  bbep.setCursor(0, 0);
  bbep.getStringBox("E", &cap); // cap.y is the (negative) cap height
  int baseline = menu_first_baseline(count) + index * MENU_ITEM_SPACING;
  pRect->w = MENU_ARROW_SIZE / 2 + 1;
  pRect->h = MENU_ARROW_SIZE + 1;
  pRect->x = bbep.width() / 3 - MENU_ARROW_GAP - pRect->w;
  pRect->y = baseline + cap.y / 2 - MENU_ARROW_SIZE / 2;
}

static void menu_draw_arrow(int count, int index, uint8_t color) {
  BB_RECT r;
  menu_arrow_rect(count, index, &r);
  if (color == BBEP_WHITE) {
    bbep.fillRect(r.x, r.y, r.w, r.h, BBEP_WHITE);
  } else {
    menu_fill_triangle(r.x + r.w / 2, r.y + r.h / 2, MENU_ARROW_SIZE, 0, color);
  }
}

// One of the control scheme's hints along the bottom edge, at the same x positions as the TRMNL X's
// touchbar pills: an up/down triangle, if any, then the label, in an outlined pill
static void menu_draw_hint(const menu_hint_t &hint, int slot) {
  if (!hint.label) return;
  const int xs[MENU_HINT_SLOTS] = {bbep.width() / 6, bbep.width() / 2, bbep.width() * 5 / 6};
  const int y = bbep.height() - MENU_HINT_MARGIN;
  BB_RECT rect, line;
  bbep.setFont(Inter_18);
  bbep.setCursor(0, y);
  bbep.getStringBox(hint.label, &rect);
  bbep.getStringBox("Ag", &line); // the same height for every hint's pill
  const int tri = hint.icon != MENU_HINT_ICON_NONE ? line.h * 2 / 3 : 0;
  const int content_w = rect.w + (tri ? tri + MENU_HINT_ICON_GAP : 0);
  const int left = xs[slot] - content_w / 2;
  const int px = left - MENU_HINT_PAD_X, py = line.y - MENU_HINT_PAD_Y;
  const int pw = content_w + 2 * MENU_HINT_PAD_X, ph = line.h + 2 * MENU_HINT_PAD_Y;
  const int b = MENU_HINT_BORDER;
  display_fill_pill(px, py, pw, ph, BBEP_BLACK);
  display_fill_pill(px + b, py + b, pw - 2 * b, ph - 2 * b, BBEP_WHITE);
  if (tri) {
    menu_fill_triangle(left + tri / 2, line.y + line.h / 2, tri, hint.icon == MENU_HINT_ICON_UP ? 1 : 2, BBEP_BLACK);
  }
  bbep.drawString(hint.label, left + content_w - rect.w, y);
}

static void menu_draw_hints(const menu_hint_t *hints) {
  if (!hints) return;
  for (int i = 0; i < MENU_HINT_SLOTS; i++) {
    menu_draw_hint(hints[i], i);
  }
}

// Draws a menu screen on a white background over the current content. `replace`: replace a menu
// already on the screen (clearing it) instead of drawing over the image.
static void menu_show_screen(const menu_view_t &view, bool replace) {
  Log_info("menu_show_screen start");
  menu_body_count = view.body ? view.body_count : 0;
  bbep.setMode(BB_MODE_1BPP);
  bbep.fillScreen(BBEP_WHITE);
  bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
  menu_draw_body(view.body, view.count);
  bbep.setFont(Roboto_Black_24);
  int baseline = menu_first_baseline(view.count);
  for (int i = 0; i < view.count; i++) {
    bbep.drawString(view.items[i], bbep.width() / 3, baseline + i * MENU_ITEM_SPACING);
  }
  menu_draw_arrow(view.count, view.selected, BBEP_BLACK);
  menu_draw_hints(view.hints);
  if (view.battery) {
    display_draw_battery_icon(bbep.width() - DISPLAY_BATTERY_ICON_W - DISPLAY_BATTERY_ICON_MARGIN,
                              DISPLAY_BATTERY_ICON_MARGIN);
  }
    // Over the image: like the loading screen, no clearing passes, straight over the old content.
    // Over another menu: clear it, since a partial update leaves its text ghosting through.
    // Either way the current image becomes the previous one, for the arrow's partial updates.
  bbep.fullUpdate(replace ? CLEAR_FAST : CLEAR_NONE, false);
  Log_info("menu_show_screen end");
}

// Moves the arrow of the menu screen on the display (of `count` items) with a partial update
static void menu_move_arrow(int count, int old_selected, int new_selected) {
  menu_draw_arrow(count, old_selected, BBEP_WHITE);
  menu_draw_arrow(count, new_selected, BBEP_BLACK);
  bbep.partialUpdate(false);
}

// Shows a menu screen and runs it until an item is chosen: UP/DOWN move the arrow, SELECT chooses.
// Returns the chosen index, or -1 after timeout_ms without input.
// `replace`: clear the menu on the screen instead of drawing over the image.
// `body`: lines of text above the items (see menu_view_t).
// `selected`: the item the arrow starts on.
// `battery`: show the battery icon.
static int run_menu_screen(const char *const *labels, int count, bool replace, const char *const *body = nullptr,
                           int body_count = 0, uint32_t timeout_ms = MENU_TIMEOUT_MS, int selected = 0,
                           bool battery = false) {
  menu_view_t view = {labels, count, selected, body, body_count, menu_controls->hints, battery};
  menu_show_screen(view, replace);

  int chosen = -1;
  unsigned long last_input = millis();
  while (millis() - last_input < timeout_ms) {
    delay(MENU_POLL_INTERVAL_MS);
    menu_input_t input = menu_controls->read();
    if (input == menu_input_t::NONE) continue;

    last_input = millis();
    if (input == menu_input_t::SELECT) {
      chosen = selected;
      Log_info("Menu: chose %s", labels[selected]);
      break;
    }
    if (count < 2) continue;
    int next = (selected + (input == menu_input_t::UP ? -1 : 1) + count) % count;
    Log_info("Menu: %s -> %s", labels[selected], labels[next]);
    menu_move_arrow(count, selected, next);
    selected = next;
  }

  if (chosen < 0) Log_info("Menu: timeout - closing");
  return chosen;
}

enum main_menu_item_t {
  MAIN_MENU_NEXT_SCREEN,
  MAIN_MENU_PREVIOUS_SCREEN,
  MAIN_MENU_SPECIAL,
  MAIN_MENU_SETTINGS,
  MAIN_MENU_CLOSE,
  MAIN_MENU_COUNT,
};

static const char *const main_menu_labels[MAIN_MENU_COUNT] = {
    "Next Screen",
    "Previous Screen",
    "Special...",
    "Settings...",
    "Close",
};

// The Special submenu: the special functions, then Back, to the main menu
struct special_menu_item_t {
  const char *label;
  SPECIAL_FUNCTION function; // SF_NONE for Back
};

static const special_menu_item_t special_menu_items[] = {
    {"Identify", SF_IDENTIFY},
    {"Restart Playlist", SF_RESTART_PLAYLIST},
    {"Send to Me", SF_SEND_TO_ME},
    {"Guest Mode", SF_GUEST_MODE},
    {"Back", SF_NONE},
};
#define SPECIAL_MENU_COUNT ((int)(sizeof(special_menu_items) / sizeof(special_menu_items[0])))

// The Settings submenu; Back goes to the main menu
enum settings_menu_item_t {
  SETTINGS_MENU_RESET_WIFI,
  SETTINGS_MENU_SHUT_DOWN,
  SETTINGS_MENU_BACK,
  SETTINGS_MENU_COUNT,
};

static const char *const settings_menu_labels[SETTINGS_MENU_COUNT] = {
    "Reset Wi-Fi...",
    "Shut Down...",
    "Back",
};

// The confirmations of Reset Wi-Fi and Shut Down; Back goes to Settings
#define CONFIRM_MENU_CONFIRM 0
#define CONFIRM_MENU_BACK    1 // where the arrow starts, so a stray input doesn't confirm
#define CONFIRM_MENU_COUNT   2

static const char *const reset_wifi_menu_labels[CONFIRM_MENU_COUNT] = {"Confirm Reset Wi-Fi", "Back"};
static const char *const reset_wifi_menu_body[] = {
    "Forget the saved Wi-Fi network and open the setup portal?",
};

static const char *const shut_down_menu_labels[CONFIRM_MENU_COUNT] = {"Confirm Shut Down", "Back"};
static const char *const shut_down_menu_body[] = {
    "Shut down the device?",
    "Reconnect USB-C to power to wake it up.",
};

enum menu_screen_t {
  MENU_SCREEN_MAIN,
  MENU_SCREEN_SPECIAL,
  MENU_SCREEN_SETTINGS,
  MENU_SCREEN_RESET_WIFI,
  MENU_SCREEN_SHUT_DOWN,
};

enum menu_outcome_t {
  MENU_OUTCOME_NEXT_SCREEN,
  MENU_OUTCOME_PREVIOUS_SCREEN,
  MENU_OUTCOME_SPECIAL_FUNCTION,
  MENU_OUTCOME_RESET_WIFI,
  MENU_OUTCOME_SHUT_DOWN,
  MENU_OUTCOME_CLOSE, // Close on the main menu, or a timeout on any screen
};

// Runs the menu screens until one ends the menu. Sets *function for MENU_OUTCOME_SPECIAL_FUNCTION.
static menu_outcome_t run_menu_screens(SPECIAL_FUNCTION *function) {
  const char *special_labels[SPECIAL_MENU_COUNT];
  for (int i = 0; i < SPECIAL_MENU_COUNT; i++) {
    special_labels[i] = special_menu_items[i].label;
  }

  menu_screen_t screen = MENU_SCREEN_MAIN;
  bool replace = false; // the first menu goes over the image, like the loading screen
  while (true) {
    switch (screen) {
    case MENU_SCREEN_MAIN: {
      int choice = run_menu_screen(main_menu_labels, MAIN_MENU_COUNT, replace, nullptr, 0, MENU_TIMEOUT_MS, 0, true);
      replace = true;
      switch (choice) {
      case MAIN_MENU_NEXT_SCREEN:
        return MENU_OUTCOME_NEXT_SCREEN;
      case MAIN_MENU_PREVIOUS_SCREEN:
        return MENU_OUTCOME_PREVIOUS_SCREEN;
      case MAIN_MENU_SPECIAL:
        screen = MENU_SCREEN_SPECIAL;
        break;
      case MAIN_MENU_SETTINGS:
        screen = MENU_SCREEN_SETTINGS;
        break;
      default: // Close, or timeout
        return MENU_OUTCOME_CLOSE;
      }
      break;
    }
    case MENU_SCREEN_SPECIAL: {
      int item = run_menu_screen(special_labels, SPECIAL_MENU_COUNT, true);
      if (item < 0) return MENU_OUTCOME_CLOSE;
      if (special_menu_items[item].function == SF_NONE) { // Back
        screen = MENU_SCREEN_MAIN;
        break;
      }
      *function = special_menu_items[item].function;
      return MENU_OUTCOME_SPECIAL_FUNCTION;
    }
    case MENU_SCREEN_SETTINGS: {
      int item = run_menu_screen(settings_menu_labels, SETTINGS_MENU_COUNT, true);
      switch (item) {
      case SETTINGS_MENU_RESET_WIFI:
        screen = MENU_SCREEN_RESET_WIFI;
        break;
      case SETTINGS_MENU_SHUT_DOWN:
        screen = MENU_SCREEN_SHUT_DOWN;
        break;
      case SETTINGS_MENU_BACK:
        screen = MENU_SCREEN_MAIN;
        break;
      default: // timeout
        return MENU_OUTCOME_CLOSE;
      }
      break;
    }
    case MENU_SCREEN_RESET_WIFI: {
      int choice = run_menu_screen(reset_wifi_menu_labels, CONFIRM_MENU_COUNT, true, reset_wifi_menu_body,
                                   sizeof(reset_wifi_menu_body) / sizeof(reset_wifi_menu_body[0]), MENU_TIMEOUT_MS,
                                   CONFIRM_MENU_BACK);
      if (choice < 0) return MENU_OUTCOME_CLOSE;
      if (choice == CONFIRM_MENU_CONFIRM) return MENU_OUTCOME_RESET_WIFI;
      screen = MENU_SCREEN_SETTINGS;
      break;
    }
    case MENU_SCREEN_SHUT_DOWN: {
      int choice = run_menu_screen(shut_down_menu_labels, CONFIRM_MENU_COUNT, true, shut_down_menu_body,
                                   sizeof(shut_down_menu_body) / sizeof(shut_down_menu_body[0]), MENU_TIMEOUT_MS,
                                   CONFIRM_MENU_BACK);
      if (choice < 0) return MENU_OUTCOME_CLOSE;
      if (choice == CONFIRM_MENU_CONFIRM) return MENU_OUTCOME_SHUT_DOWN;
      screen = MENU_SCREEN_SETTINGS;
      break;
    }
    }
  }
}

void menu_open(const menu_controls_t &controls) {
  Log_info("Opening menu");
  SPECIAL_FUNCTION function = SF_NONE;
  menu_controls = &controls;
  // For the battery icon: the wake detects the battery and reads the gauge after the
  // menu, and the gauge isn't read without a battery detected
  battery_count = detect_battery_count();
  battery().gaugeInit();
  if (controls.begin) controls.begin();
  menu_outcome_t outcome = run_menu_screens(&function);
  if (controls.end) controls.end();
  menu_controls = nullptr;

  switch (outcome) {
  case MENU_OUTCOME_NEXT_SCREEN:
    // The rest of the wake fetches the next screen
    break;
  case MENU_OUTCOME_PREVIOUS_SCREEN:
    show_cached_image_by_offset(-1);
    break;
  case MENU_OUTCOME_SPECIAL_FUNCTION:
    // The API request names this function, so the server runs it rather than the saved one
    special_function = function;
    special_function_chosen = true;
    break;
  case MENU_OUTCOME_RESET_WIFI:
    Log_info("Resetting WiFi from the menu");
    resetDeviceCredentials();
    break;
  case MENU_OUTCOME_SHUT_DOWN:
    // With the shipment status cleared, the restart goes back to shipment mode: off until USB-C power
    Log_info("Shutting down from the menu");
    clearShipmentStatus();
    ESP.restart();
    break;
  case MENU_OUTCOME_CLOSE:
    // Re-show the current image and go back to sleep
    show_last_image_and_sleep();
    break;
  }
}

#endif // BOARD_TRMNL_X
