#include "touchbar_actions.h"

#ifdef BOARD_TRMNL_X

#include <config.h>
#include <display.h>
#include <globals.h>
#include <status_screens.h>
#include <touchbar_indicator.h>
#include <trmnl_log.h>

#include "IQS323.h"
#include "displayed_image.h"
#include "iqs323_task.h"
#include "menu.h"
#include "rtc_wake_stub_trmnl_x.h"
#include "touchbar_gesture.h"

// Touchbar indicator to redraw after a full-refresh (e.g. logo screen)
static touchbar_side_t pending_indicator_side = TOUCHBAR_LEFT;
static bool has_pending_indicator = false;
static bool staying_awake = false;

// Draws the touch arrow indicator now, then queues up an arrow redraw after the loading logo (if needed)
static void show_touchbar_indicator(touchbar_side_t side) {
  display_draw_touchbar_indicator(side);
  DisplayedImage::cover();
  pending_indicator_side = side;
  has_pending_indicator = true;
}

// The touch bar's control scheme for the menu (menu.h): left is up, right is down, the middle selects

static const menu_hint_t touchbar_menu_hints[MENU_HINT_SLOTS] = {
    {"Up", MENU_HINT_ICON_UP},
    {"Select", MENU_HINT_ICON_NONE},
    {"Down", MENU_HINT_ICON_DOWN},
};

// Touch starts not reported yet (bit n: channel n). The menu reads the bar between display
// updates, so it goes by touch starts: a tap during an update still counts.
static uint8_t touchbar_menu_pending_starts = 0;

static void touchbar_menu_begin(void) {
  iqs323_task_set_streaming_mode(true); // fresh touch data while the menu polls
  iqs323_task_take_touch_starts();      // the middle tap that opened the menu
  touchbar_menu_pending_starts = 0;
}

static void touchbar_menu_end(void) { iqs323_task_set_streaming_mode(false); }

static menu_input_t touchbar_menu_read(void) {
  touchbar_menu_pending_starts |= iqs323_task_take_touch_starts();
  for (uint8_t channel = 0; channel < 3; channel++) {
    if (!(touchbar_menu_pending_starts & (1 << channel))) continue;
    touchbar_menu_pending_starts &= ~(1 << channel);
    switch (channel) {
    case 0:
      return menu_input_t::UP;
    case 2:
      return menu_input_t::DOWN;
    default:
      return menu_input_t::SELECT;
    }
  }
  return menu_input_t::NONE;
}

// Shown on the first touch, since the touch bar used to work differently, and from the main menu
static const menu_tutorial_t touchbar_tutorial = {
    "Touchbar Tutorial",
    "Learn how to use the touchbar",
    "Touch controls have recently been improved. Here's how it works.",
    {
        {"Left Tap", "Previous screen", "Previous"},
        {"Center Tap", "Open the menu", "Menu"},
        {"Right Tap", "Next screen", "Next"},
    },
    "Try it: tap left, then center, then right.",
    "Tap anywhere to continue",
    true,
    PREFERENCES_TOUCHBAR_TUTORIAL_KEY,
};

static const menu_controls_t touchbar_menu_controls = {
    touchbar_menu_hints, touchbar_menu_begin, touchbar_menu_end, touchbar_menu_read, &touchbar_tutorial,
};

// The factory's check that each zone works before the device ships, from the setup portal
static const menu_tutorial_t touchbar_ship_check = {
    nullptr,
    "Prepare for Shipping",
    "Test the touchbar, then the device shuts down for shipment.",
    {
        {"Left Tap", nullptr, "Left"},
        {"Center Tap", nullptr, "Center"},
        {"Right Tap", nullptr, "Right"},
    },
    "Tap left, then center, then right.",
    "Touchbar OK! Shutting down for shipment.",
    false,
    nullptr,
};

#define TOUCHBAR_SHIP_CHECK_HOLD_MS 2000
#define TOUCHBAR_STAY_AWAKE_MS      20000

// Single dispatch point for the resolved navigation intent.
static void dispatch_touchbar_intent(const touchbar_resolution_t &resolved) {
  if (resolved.intent != touchbar_intent_t::NONE && !menu_tutorial_done(touchbar_menu_controls)) {
    Log_info("Touch bar used before its tutorial - showing it");
    menu_open_tutorial(touchbar_menu_controls);
    return;
  }
  if (resolved.intent != touchbar_intent_t::NONE) stay_awake_for_touchbar = true;
  has_pending_indicator = false;
  switch (resolved.intent) {
  case touchbar_intent_t::BACK:
    // The API request asks the server for the previous screen
    show_touchbar_indicator(TOUCHBAR_LEFT);
    Log_info(resolved.held ? "Back button hold" : "Back button tapped");
    special_function = SF_REWIND;
    special_function_chosen = true;
    break;
  case touchbar_intent_t::NEXT:
    // The API request gets the next screen
    show_touchbar_indicator(TOUCHBAR_RIGHT);
    Log_info(resolved.held ? "Next button hold" : "Next button tapped");
    break;
  case touchbar_intent_t::MIDDLE:
    if (!resolved.held) {
      Log_info("Middle button tapped - opening the menu");
      menu_open(touchbar_menu_controls);
      if (staying_awake) showWakeLogo(); // over the menu
      break;
    }
    show_touchbar_indicator(TOUCHBAR_MIDDLE);
    Log_info("Middle button hold");
    // TODO: CH1 hold was previously reserved for an OTG toggle feature -
    // not implemented yet. See otg_state/otg_turn_on()/otg_turn_off().
    break;
  case touchbar_intent_t::NONE:
  default:
    break;
  }
}

void process_iqs323_data(void) { dispatch_touchbar_intent(touchbar_resolve_intent()); }

// The intent of the first touch started since the last call, as a tap
static touchbar_intent_t take_tap_intent(void) {
  uint8_t starts = iqs323_task_take_touch_starts();
  if (starts & (1 << 0)) return touchbar_intent_t::BACK;
  if (starts & (1 << 2)) return touchbar_intent_t::NEXT;
  if (starts & (1 << 1)) return touchbar_intent_t::MIDDLE;
  return touchbar_intent_t::NONE;
}

void touchbar_stay_awake(void (*show_screen)(void)) {
  if (!stay_awake_for_touchbar) return;
  Log_info("Staying awake %d s for more touch bar taps", TOUCHBAR_STAY_AWAKE_MS / 1000);
  staying_awake = true;
  iqs323_task_set_streaming_mode(true);
  iqs323_task_take_touch_starts(); // ignore touches from before the screen was up
  uint32_t last_touch_ms = millis();
  uint32_t screen_shown_ms = last_touch_ms;
  // A short refresh (e.g. the server's "Catching you up" screen) comes due while still awake
  uint32_t refresh_s = refreshInterval.seconds();
  while (millis() - last_touch_ms < TOUCHBAR_STAY_AWAKE_MS) {
    touchbar_snapshot_t snap;
    iqs323_task_read_snapshot(&snap);
    if (snap.ch0_touch || snap.ch1_touch || snap.ch2_touch) last_touch_ms = millis(); // a hold counts too
    touchbar_intent_t intent = take_tap_intent();
    bool refresh_due = refresh_s > 0 && (millis() - screen_shown_ms) / 1000 >= refresh_s;
    if (intent == touchbar_intent_t::NONE && !refresh_due) {
      delay(20);
      continue;
    }
    special_function = SF_NONE;
    special_function_chosen = false;
    if (intent != touchbar_intent_t::NONE) {
      dispatch_touchbar_intent({intent, false});
      iqs323_task_set_streaming_mode(true); // the menu turns it off when it closes
    } else {
      Log_info("Refresh of %u s due while awake - fetching the screen", (unsigned)refresh_s);
      has_pending_indicator = false; // no known tap location, so skip the indicator arrow
    }
    show_screen();
    iqs323_task_take_touch_starts(); // ignore taps made while the screen was loading
    last_touch_ms = screen_shown_ms = millis();
    refresh_s = refreshInterval.seconds();
  }
  staying_awake = false;
  iqs323_task_set_streaming_mode(false);
  Log_info("No touches for %d s", TOUCHBAR_STAY_AWAKE_MS / 1000);
}

bool touchbar_poll_ship_check(void) {
  static TouchbarHoldTracker middle_hold(1 << 1, TOUCHBAR_SHIP_CHECK_HOLD_MS);
  touchbar_snapshot_t snap;
  iqs323_task_read_snapshot(&snap);
  if (!middle_hold.poll(snap.ch1_touch ? 1 << 1 : 0)) return false;
  Log_info("Middle button held in the setup portal - opening the ship check");
  menu_run_ship_check(touchbar_menu_controls, touchbar_ship_check);
  Log_info("Ship check: timeout - back to the setup portal");
  return true;
}

bool touchbar_has_pending_indicator(void) { return has_pending_indicator; }

void touchbar_redraw_pending_indicator(void) {
  if (has_pending_indicator) {
    display_draw_touchbar_indicator(pending_indicator_side);
    has_pending_indicator = false;
  }
}

void touchbar_log_wake_stub_status(void) {
  Log_info("wakeup_stub_iqs_status.status: 0x%02X 0x%02X", wakeup_stub_iqs_status.status[0],
           wakeup_stub_iqs_status.status[1]);
  Log_info("wakeup_stub_iqs_status.gestures: 0x%02X 0x%02X", wakeup_stub_iqs_status.gestures[0],
           wakeup_stub_iqs_status.gestures[1]);
  Log_info("wakeup_stub_iqs_status.slider_cords: 0x%02X 0x%02X", wakeup_stub_iqs_status.slider_cords[0],
           wakeup_stub_iqs_status.slider_cords[1]);
  Log_info("wakeup_stub_iqs_status.ch0_cnts: 0x%02X 0x%02X 0x%02X 0x%02X", wakeup_stub_iqs_status.ch0_cnts[0],
           wakeup_stub_iqs_status.ch0_cnts[1], wakeup_stub_iqs_status.ch0_cnts[2], wakeup_stub_iqs_status.ch0_cnts[3]);
  Log_info("wakeup_stub_iqs_status.ch1_cnts: 0x%02X 0x%02X 0x%02X 0x%02X", wakeup_stub_iqs_status.ch1_cnts[0],
           wakeup_stub_iqs_status.ch1_cnts[1], wakeup_stub_iqs_status.ch1_cnts[2], wakeup_stub_iqs_status.ch1_cnts[3]);
  Log_info("wakeup_stub_iqs_status.ch2_cnts: 0x%02X 0x%02X 0x%02X 0x%02X", wakeup_stub_iqs_status.ch2_cnts[0],
           wakeup_stub_iqs_status.ch2_cnts[1], wakeup_stub_iqs_status.ch2_cnts[2], wakeup_stub_iqs_status.ch2_cnts[3]);
}

void touchbar_start(esp_sleep_wakeup_cause_t wakeup_reason, bool gpio_wakeup) {
  // Notify IQS323 task about wakeup type BEFORE starting the task
  if (gpio_wakeup) {
    Log_info("GPIO wakeup detected (%d) - using wake stub data", wakeup_reason);
    iqs323_task_notify_gpio_wakeup(true);
  } else {
    Log_info("Non-GPIO wakeup (%d)", wakeup_reason);
  }

  // Start IQS323 task manager
  if (!iqs323_task_init(NULL)) {
    Log_error("IQS323 Task: Failed to start - rebooting");
    delay(1000);
    ESP.restart();
  }

  // Wait for IQS323 initialization to complete
  if (!iqs323_task_wait_ready(5000)) {
    Log_error("IQS323 Task: Initialization timeout - rebooting");
    delay(1000);
    ESP.restart();
  }

  if (gpio_wakeup) {
    process_iqs323_data();
  }
}

#endif // BOARD_TRMNL_X
