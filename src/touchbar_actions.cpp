#include "touchbar_actions.h"

#ifdef BOARD_TRMNL_X

#include <config.h>
#include <display.h>
#include <trmnl_log.h>

#include "IQS323.h"
#include "cached_images.h"
#include "iqs323_task.h"
#include "menu.h"
#include "rtc_wake_stub_trmnl_x.h"
#include "touchbar_gesture.h"

// Touchbar indicator to redraw after a full-refresh (e.g. logo screen)
static touchbar_side_t pending_indicator_side = TOUCHBAR_LEFT;
static bool has_pending_indicator = false;

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

// Single dispatch point for the resolved navigation intent.
static void dispatch_touchbar_intent(const touchbar_resolution_t &resolved) {
  if (resolved.intent != touchbar_intent_t::NONE && !menu_tutorial_done(touchbar_menu_controls)) {
    Log_info("Touch bar used before its tutorial - showing it");
    menu_open_tutorial(touchbar_menu_controls);
    return;
  }
  switch (resolved.intent) {
  case touchbar_intent_t::BACK:
    display_draw_touchbar_indicator(TOUCHBAR_LEFT);
    Log_info(resolved.held ? "Back button hold" : "Back button tapped");
    show_cached_image_by_offset(-1);
    break;
  case touchbar_intent_t::NEXT:
    display_draw_touchbar_indicator(TOUCHBAR_RIGHT);
    Log_info(resolved.held ? "Next button hold" : "Next button tapped");
    show_cached_image_by_offset(+1);
    break;
  case touchbar_intent_t::MIDDLE:
    if (!resolved.held) {
      Log_info("Middle button tapped - opening the menu");
      menu_open(touchbar_menu_controls);
      break;
    }
    display_draw_touchbar_indicator(TOUCHBAR_MIDDLE);
    Log_info("Middle button hold");
    pending_indicator_side = TOUCHBAR_MIDDLE;
    has_pending_indicator = true;
    // TODO: CH1 hold was previously reserved for an OTG toggle feature -
    // not implemented yet. See otg_state/otg_turn_on()/otg_turn_off().
    break;
  case touchbar_intent_t::NONE:
  default:
    break;
  }
}

void process_iqs323_data(void) { dispatch_touchbar_intent(touchbar_resolve_intent()); }

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
