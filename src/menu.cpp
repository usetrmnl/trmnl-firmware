#include "menu.h"

// The TRMNL X is the only board with a control scheme for the menu so far. Another board needs one
// (menu_controls_t), and a look at the X-only parts here: the drawing (FastEPD, sized for the X's
// panel), Shut Down (shipment mode) and the fuel gauge in Diagnostics.
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

// Center of a hint slot: the same x positions as the TRMNL X's touchbar pills
static int menu_hint_x(int slot) { return bbep.width() * (2 * slot + 1) / 6; }

// Draws text centered on cx in the current font
static void menu_draw_centered(const char *text, int cx, int baseline) {
  BB_RECT rect;
  bbep.setCursor(0, baseline);
  bbep.getStringBox(text, &rect);
  bbep.drawString(text, cx - rect.w / 2, baseline);
}

// How a hint is drawn
enum menu_hint_style_t {
  MENU_HINT_PLAIN,    // its content alone
  MENU_HINT_OUTLINED, // in an outlined pill
  MENU_HINT_FILLED,   // white in a black pill
};

// One of the control scheme's hints along the bottom edge: an up/down triangle, if any, then the
// label, drawn in `style`
static void menu_draw_hint(const menu_hint_t &hint, int slot, menu_hint_style_t style = MENU_HINT_OUTLINED) {
  if (!hint.label) return;
  const int y = bbep.height() - MENU_HINT_MARGIN;
  BB_RECT rect, line;
  bbep.setFont(Inter_18);
  bbep.setCursor(0, y);
  bbep.getStringBox(hint.label, &rect);
  bbep.getStringBox("Ag", &line); // the same height for every hint's pill
  const int tri = hint.icon != MENU_HINT_ICON_NONE ? line.h * 2 / 3 : 0;
  const int content_w = rect.w + (tri ? tri + MENU_HINT_ICON_GAP : 0);
  const int left = menu_hint_x(slot) - content_w / 2;
  const int px = left - MENU_HINT_PAD_X, py = line.y - MENU_HINT_PAD_Y;
  const int pw = content_w + 2 * MENU_HINT_PAD_X, ph = line.h + 2 * MENU_HINT_PAD_Y;
  const int b = MENU_HINT_BORDER;
  const bool filled = style == MENU_HINT_FILLED;
  const uint8_t fg = filled ? BBEP_WHITE : BBEP_BLACK, bg = filled ? BBEP_BLACK : BBEP_WHITE;
  if (style != MENU_HINT_PLAIN) {
    display_fill_pill(px, py, pw, ph, BBEP_BLACK);
    display_fill_pill(px + b, py + b, pw - 2 * b, ph - 2 * b, bg);
  }
  if (tri) {
    menu_fill_triangle(left + tri / 2, line.y + line.h / 2, tri, hint.icon == MENU_HINT_ICON_UP ? 1 : 2, fg);
  }
  bbep.setTextColor(fg, bg);
  bbep.drawString(hint.label, left + content_w - rect.w, y);
  bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
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
  MAIN_MENU_TUTORIAL, // only with a control scheme that has a tutorial
  MAIN_MENU_SPECIAL,
  MAIN_MENU_SETTINGS,
  MAIN_MENU_CLOSE,
  MAIN_MENU_COUNT,
};

static const char *const main_menu_labels[MAIN_MENU_COUNT] = {
    "Next Screen", "Previous Screen",
    nullptr, // the tutorial's own item
    "Special...",  "Settings...",     "Close",
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
  SETTINGS_MENU_DIAGNOSTICS,
  SETTINGS_MENU_RESET_WIFI,
  SETTINGS_MENU_SHUT_DOWN,
  SETTINGS_MENU_BACK,
  SETTINGS_MENU_COUNT,
};

static const char *const settings_menu_labels[SETTINGS_MENU_COUNT] = {
    "Diagnostics...",
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

#define DIAGNOSTICS_TIMEOUT_MS 60000 // time to read them
#define DIAGNOSTICS_MAX_LINES  16

static const char *const diagnostics_menu_labels[] = {"Back"};

// Fills lines with the Diagnostics screen's "label\tvalue" lines; returns how many
static int diagnostics_lines(String *lines) {
  int n = 0;
  auto add = [&](const char *label, const String &value) {
    if (n < DIAGNOSTICS_MAX_LINES) lines[n++] = String(label) + "\t" + value;
  };
  char buf[64];

  add("Firmware", Messages::firmware_version());
  String friendly_id = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
  add("Friendly ID", friendly_id.isEmpty() ? String("not set up") : friendly_id);

  float volts = battery().readVoltage();
  int soc = battery().readSoc();
  if (volts < 0) {
    add("Battery", "unknown");
  } else if (soc < 0) {
    snprintf(buf, sizeof(buf), "%.2f V", volts);
    add("Battery", buf);
  } else {
    snprintf(buf, sizeof(buf), "%.2f V, %d%%", volts, soc);
    add("Battery", buf);
  }
  float temperature = battery().readTemperature();
  if (volts >= 0) {
    snprintf(buf, sizeof(buf), "%.1f C", temperature);
    add("Battery Temperature", buf);
  }
  UsbStatus usb = power().usbStatus();
  ChargingStatus charging = power().chargingStatus();
  add("Power", usb == UsbStatus::CONNECTED      ? (charging == ChargingStatus::CHARGING ? "USB-C, charging" : "USB-C")
               : usb == UsbStatus::DISCONNECTED ? "Battery"
                                                : "unknown");

  if (WifiCaptivePortal.isSaved()) {
    WifiCredentials creds = WifiCaptivePortal.getLastCredentials();
    add("Wi-Fi Network", creds.ssid);
    add("Wi-Fi Band", creds.is5GHz ? "5 GHz" : "2.4 GHz");
  } else {
    add("Wi-Fi Network", "not set up");
  }
  if (last_wifi_rssi != 0) {
    snprintf(buf, sizeof(buf), "%d dBm (last connection)", last_wifi_rssi);
    add("Wi-Fi Signal", buf);
  } else {
    add("Wi-Fi Signal", "unknown");
  }

  uint32_t refresh = refreshInterval.seconds();
  if (refresh % 60 == 0) {
    snprintf(buf, sizeof(buf), "every %lu min", (unsigned long)(refresh / 60));
  } else {
    snprintf(buf, sizeof(buf), "every %lu s", (unsigned long)refresh);
  }
  add("Refresh", buf);
  add("Server", preferences.getString(PREFERENCES_API_URL, API_BASE_URL));
  String panel = display_panel_rev_string();
  add("Panel", panel.isEmpty() ? String("unknown") : panel);
  snprintf(buf, sizeof(buf), "%lu KB heap, %lu KB PSRAM", (unsigned long)(ESP.getFreeHeap() / 1024),
           (unsigned long)(ESP.getFreePsram() / 1024));
  add("Free Memory", buf);
  return n;
}

// The tutorial screen layout: the title and subtitle, a column above each control's hint slot,
// the prompt to try them in order (then the one to close it, below it), and each control's label in
// its hint slot: plain, outlined when it is the one to tap next, filled once tapped. The partial
// updates only add black: white over black leaves a ghost.
#define TUTORIAL_TIMEOUT_MS   60000
#define TUTORIAL_TITLE_Y      240 // baselines
#define TUTORIAL_SUBTITLE_Y   330
#define TUTORIAL_CONTROLS_Y   580
#define TUTORIAL_LINE_SPACING 80
#define TUTORIAL_PRACTICE_Y   1000
#define TUTORIAL_FINISH_Y     1100

// The label of a tutorial control, in its hint slot
static void tutorial_draw_pill(const menu_tutorial_t &tutorial, int slot, menu_hint_style_t style) {
  menu_hint_t pill = {tutorial.controls[slot].label, MENU_HINT_ICON_NONE};
  menu_draw_hint(pill, slot, style);
}

static void tutorial_show_screen(const menu_tutorial_t &tutorial, bool replace) {
  Log_info("tutorial_show_screen start");
  bbep.setMode(BB_MODE_1BPP);
  bbep.fillScreen(BBEP_WHITE);
  bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
  bbep.setFont(Roboto_Black_24);
  menu_draw_centered(tutorial.title, bbep.width() / 2, TUTORIAL_TITLE_Y);
  bbep.setFont(Inter_18);
  menu_draw_centered(tutorial.subtitle, bbep.width() / 2, TUTORIAL_SUBTITLE_Y);
  for (int slot = 0; slot < MENU_HINT_SLOTS; slot++) {
    const menu_tutorial_control_t &control = tutorial.controls[slot];
    const int x = menu_hint_x(slot);
    bbep.setFont(Roboto_Black_24);
    menu_draw_centered(control.title, x, TUTORIAL_CONTROLS_Y);
    bbep.setFont(Inter_18);
    if (control.action) menu_draw_centered(control.action, x, TUTORIAL_CONTROLS_Y + TUTORIAL_LINE_SPACING);
    tutorial_draw_pill(tutorial, slot, slot == 0 ? MENU_HINT_OUTLINED : MENU_HINT_PLAIN);
  }
  bbep.setFont(Roboto_Black_24);
  menu_draw_centered(tutorial.practice, bbep.width() / 2, TUTORIAL_PRACTICE_Y);
  bbep.fullUpdate(replace ? CLEAR_FAST : CLEAR_NONE, false);
  Log_info("tutorial_show_screen end");
}

// Shows `tutorial` and has the user try its controls in order. Returns true once they have and, with
// `confirm`, then used any control to close it; false after a timeout. `replace`: replace a screen on
// the display instead of the image.
static bool run_tutorial_screen(const menu_tutorial_t &tutorial, bool replace, bool confirm) {
  tutorial_show_screen(tutorial, replace);

  int next = 0; // the hint slot of the control to tap next; MENU_HINT_SLOTS once all are
  unsigned long last_input = millis();
  while (millis() - last_input < TUTORIAL_TIMEOUT_MS) {
    delay(MENU_POLL_INTERVAL_MS);
    menu_input_t input = menu_controls->read();
    if (input == menu_input_t::NONE) continue;

    last_input = millis();
    const int slot = input == menu_input_t::UP ? 0 : input == menu_input_t::SELECT ? 1 : 2;
    if (next == MENU_HINT_SLOTS) {
      Log_info("Tutorial: done");
      return true;
    }
    if (slot != next) {
      Log_info("Tutorial: tapped %s, waiting for %s", tutorial.controls[slot].label, tutorial.controls[next].label);
      continue;
    }
    Log_info("Tutorial: tried %s", tutorial.controls[slot].label);
    tutorial_draw_pill(tutorial, slot, MENU_HINT_FILLED);
    if (++next < MENU_HINT_SLOTS) {
      tutorial_draw_pill(tutorial, next, MENU_HINT_OUTLINED);
    } else {
      bbep.setFont(Roboto_Black_24);
      menu_draw_centered(tutorial.finish, bbep.width() / 2, TUTORIAL_FINISH_Y);
    }
    bbep.partialUpdate(false);
    if (next == MENU_HINT_SLOTS && !confirm) {
      Log_info("Tutorial: done");
      return true;
    }
  }
  Log_info("Tutorial: timeout - closing");
  return false;
}

// Shows the tutorial of the menu's control scheme and runs it until the user closes it (true; it is
// marked done) or timeout (false). `replace`: replace a menu on the screen instead of the image.
static bool run_tutorial(bool replace) {
  const menu_tutorial_t &tutorial = *menu_controls->tutorial;
  if (!run_tutorial_screen(tutorial, replace, true)) return false;
  preferences.putBool(tutorial.done_key, true);
  return true;
}

// With the shipment status cleared, the restart goes back to shipment mode: off until USB-C power.
// Does not return.
static void menu_shut_down(void) {
  clearShipmentStatus();
  ESP.restart();
}

enum menu_screen_t {
  MENU_SCREEN_MAIN,
  MENU_SCREEN_SPECIAL,
  MENU_SCREEN_SETTINGS,
  MENU_SCREEN_DIAGNOSTICS,
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

  // The main menu's items: the tutorial's only if the control scheme has one
  const char *main_labels[MAIN_MENU_COUNT];
  main_menu_item_t main_items[MAIN_MENU_COUNT];
  int main_count = 0;
  for (int i = 0; i < MAIN_MENU_COUNT; i++) {
    if (i == MAIN_MENU_TUTORIAL && !menu_controls->tutorial) continue;
    main_labels[main_count] = i == MAIN_MENU_TUTORIAL ? menu_controls->tutorial->item : main_menu_labels[i];
    main_items[main_count++] = (main_menu_item_t)i;
  }

  menu_screen_t screen = MENU_SCREEN_MAIN;
  bool replace = false; // the first menu goes over the image, like the loading screen
  while (true) {
    switch (screen) {
    case MENU_SCREEN_MAIN: {
      int index = run_menu_screen(main_labels, main_count, replace, nullptr, 0, MENU_TIMEOUT_MS, 0, true);
      replace = true;
      switch (index < 0 ? MAIN_MENU_CLOSE : main_items[index]) {
      case MAIN_MENU_NEXT_SCREEN:
        return MENU_OUTCOME_NEXT_SCREEN;
      case MAIN_MENU_PREVIOUS_SCREEN:
        return MENU_OUTCOME_PREVIOUS_SCREEN;
      case MAIN_MENU_TUTORIAL:
        run_tutorial(true);
        return MENU_OUTCOME_CLOSE; // closing the tutorial closes the menu, as does its timeout
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
      case SETTINGS_MENU_DIAGNOSTICS:
        screen = MENU_SCREEN_DIAGNOSTICS;
        break;
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
    case MENU_SCREEN_DIAGNOSTICS: {
      String lines[DIAGNOSTICS_MAX_LINES];
      const char *body[DIAGNOSTICS_MAX_LINES];
      int n = diagnostics_lines(lines);
      for (int i = 0; i < n; i++) {
        Log_info("Diagnostics: %s", lines[i].c_str());
        body[i] = lines[i].c_str();
      }
      if (run_menu_screen(diagnostics_menu_labels, 1, true, body, n, DIAGNOSTICS_TIMEOUT_MS) < 0) {
        return MENU_OUTCOME_CLOSE;
      }
      screen = MENU_SCREEN_SETTINGS;
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
  // For the battery icon and Diagnostics: the wake detects the battery and reads the gauge after the
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
    Log_info("Shutting down from the menu");
    menu_shut_down();
    break;
  case MENU_OUTCOME_CLOSE:
    // Re-show the current image and go back to sleep
    show_last_image_and_sleep();
    break;
  }
}

bool menu_tutorial_done(const menu_controls_t &controls) {
  return !controls.tutorial || preferences.getBool(controls.tutorial->done_key, false);
}

void menu_open_tutorial(const menu_controls_t &controls) {
  Log_info("Opening the tutorial");
  menu_controls = &controls;
  if (controls.begin) controls.begin();
  run_tutorial(false);
  if (controls.end) controls.end();
  menu_controls = nullptr;
  show_last_image_and_sleep();
}

void menu_run_ship_check(const menu_controls_t &controls, const menu_tutorial_t &check) {
  Log_info("Opening the ship check");
  menu_controls = &controls;
  if (controls.begin) controls.begin();
  bool passed = run_tutorial_screen(check, true, false);
  if (controls.end) controls.end();
  menu_controls = nullptr;
  if (!passed) return;
  Log_info("Ship check passed - shutting down");
  menu_shut_down();
}

#endif // BOARD_TRMNL_X
