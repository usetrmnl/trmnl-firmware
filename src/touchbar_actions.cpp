#include "touchbar_actions.h"

#ifdef BOARD_TRMNL_X

#include <config.h>
#include <display.h>
#include <esp_system.h>
#include <globals.h>
#include <status_screens.h>
#include <touchbar_indicator.h>
#include <trmnl_log.h>

#include "IQS323.h"
#include "cached_images.h"
#include "displayed_image.h"
#include "iqs323_task.h"
#include "menu.h"
#include "rtc_wake_stub_trmnl_x.h"
#include "sleep_session.h"
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

static bool touchbar_menu_select_down(void) {
  touchbar_snapshot_t snap;
  iqs323_task_read_snapshot(&snap);
  return snap.ch1_touch;
}

static const menu_controls_t touchbar_menu_controls = {
    touchbar_menu_hints, touchbar_menu_begin,       touchbar_menu_end,
    touchbar_menu_read,  touchbar_menu_select_down, &touchbar_tutorial,
};

// The wake stub can read too early to see the middle touched
#define TOUCHBAR_LOCKED_WAKE_WAIT_MS 500

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

static bool touchbar_locked(void) { return preferences.getBool(PREFERENCES_TOUCHBAR_LOCKED_KEY, false); }

// Whether the middle stays held for MENU_UNLOCK_HOLD_MS from boot
static bool touchbar_unlock_held(void) {
  TouchbarHoldTracker hold(1 << 1, MENU_UNLOCK_HOLD_MS, startup_time);
  const unsigned long start = millis();
  bool touched = false;
  while (true) {
    const bool down = touchbar_menu_select_down();
    if (hold.poll(down ? 1 << 1 : 0)) return true;
    if (down) {
      touched = true;
    } else if (touched || millis() - start >= TOUCHBAR_LOCKED_WAKE_WAIT_MS) {
      return false;
    }
    delay(20);
  }
}

// Ignores the touch unless the middle is held to unlock. Returns only if a refresh is due, for the
// wake to fetch it.
static void touchbar_locked_wake(void) {
  iqs323_task_set_streaming_mode(true);
  const bool unlock = touchbar_unlock_held();
  iqs323_task_set_streaming_mode(false);
  if (!unlock) {
    Log_info("Touch bar locked - ignoring the touch");
    goToSleepKeepingTimer();
    return;
  }
  Log_info("Touch bar locked - asking to unlock");
  if (menu_run_unlock_prompt(touchbar_menu_controls)) Log_info("Touch bar unlocked");
  show_last_image_and_sleep();
}

// Single dispatch point for the resolved navigation intent. Returns false if the screen needs no
// fetching: the menu closed, with the current image back up.
static bool dispatch_touchbar_intent(const touchbar_resolution_t &resolved) {
  if (resolved.intent != touchbar_intent_t::NONE && !menu_tutorial_done(touchbar_menu_controls)) {
    Log_info("Touch bar used before its tutorial - showing it");
    menu_open_tutorial(touchbar_menu_controls);
    return true;
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
      if (!menu_open(touchbar_menu_controls)) {
        if (touchbar_locked()) goToSleep(); // taps would be ignored
        return false;
      }
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
  return true;
}

// The intent of the first touch started since the last call, as a tap
static touchbar_intent_t take_tap_intent(void) {
  uint8_t starts = iqs323_task_take_touch_starts();
  if (starts & (1 << 0)) return touchbar_intent_t::BACK;
  if (starts & (1 << 2)) return touchbar_intent_t::NEXT;
  if (starts & (1 << 1)) return touchbar_intent_t::MIDDLE;
  return touchbar_intent_t::NONE;
}

static bool refresh_is_due(uint32_t screen_shown_ms, uint32_t refresh_s) {
  return refresh_s > 0 && (millis() - screen_shown_ms) / 1000 >= refresh_s;
}

// The next tap, or NONE once TOUCHBAR_STAY_AWAKE_MS pass without a touch or a refresh of refresh_s (0: none)
// comes due. Touches from before the call don't count. For streaming mode.
static touchbar_intent_t wait_for_tap(uint32_t screen_shown_ms, uint32_t refresh_s) {
  iqs323_task_take_touch_starts();
  touchbar_intent_t intent = touchbar_intent_t::NONE;
  uint32_t last_touch_ms = millis();
  while (intent == touchbar_intent_t::NONE && millis() - last_touch_ms < TOUCHBAR_STAY_AWAKE_MS &&
         !refresh_is_due(screen_shown_ms, refresh_s)) {
    delay(20);
    touchbar_snapshot_t snap;
    iqs323_task_read_snapshot(&snap);
    if (snap.ch0_touch || snap.ch1_touch || snap.ch2_touch) last_touch_ms = millis(); // a hold counts too
    intent = take_tap_intent();
  }
  if (millis() - last_touch_ms >= TOUCHBAR_STAY_AWAKE_MS) {
    Log_info("No touches for %d s", TOUCHBAR_STAY_AWAKE_MS / 1000);
  }
  return intent;
}

void process_iqs323_data(void) {
  if (touchbar_locked()) {
    touchbar_locked_wake();
    return;
  }
  touchbar_resolution_t resolved = touchbar_resolve_intent();
  // The menu closed before the wake went online: a tap from here goes on with the wake like the
  // one that woke the device
  while (!dispatch_touchbar_intent(resolved)) {
    Log_info("Menu closed - staying awake %d s for a touch bar tap", TOUCHBAR_STAY_AWAKE_MS / 1000);
    iqs323_task_set_streaming_mode(true);
    resolved = {wait_for_tap(0, 0), false};
    iqs323_task_set_streaming_mode(false);
    if (resolved.intent == touchbar_intent_t::NONE) goToSleep();
  }
}

void touchbar_stay_awake(void (*show_screen)(void)) {
  if (!stay_awake_for_touchbar) return;
  Log_info("Staying awake %d s for more touch bar taps", TOUCHBAR_STAY_AWAKE_MS / 1000);
  staying_awake = true;
  iqs323_task_set_streaming_mode(true);
  uint32_t screen_shown_ms = millis();
  // A short refresh (e.g. the server's "Catching you up" screen) comes due while still awake
  uint32_t refresh_s = refreshInterval.seconds();
  while (true) {
    touchbar_intent_t intent = wait_for_tap(screen_shown_ms, refresh_s);
    if (intent == touchbar_intent_t::NONE && !refresh_is_due(screen_shown_ms, refresh_s)) break;
    special_function = SF_NONE;
    special_function_chosen = false;
    if (intent != touchbar_intent_t::NONE) {
      bool fetch_screen = dispatch_touchbar_intent({intent, false});
      iqs323_task_set_streaming_mode(true); // the menu turns it off when it closes
      if (!fetch_screen) continue;
    } else {
      Log_info("Refresh of %u s due while awake - fetching the screen", (unsigned)refresh_s);
      has_pending_indicator = false; // no known tap location, so skip the indicator arrow
    }
    show_screen();
    screen_shown_ms = millis();
    refresh_s = refreshInterval.seconds();
  }
  staying_awake = false;
  iqs323_task_set_streaming_mode(false);
}

bool touchbar_poll_ship_check(void) {
  if (touchbar_locked()) return false;
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
  // Only sleep wakes and deliberate restarts (e.g. OTA) keep the lock: better unlocked than locked out
  const esp_reset_reason_t reset_reason = esp_reset_reason();
  if (reset_reason != ESP_RST_DEEPSLEEP && reset_reason != ESP_RST_SW && touchbar_locked()) {
    Log_info("Reset (reason %d) - unlocking the touch bar", (int)reset_reason);
    preferences.remove(PREFERENCES_TOUCHBAR_LOCKED_KEY);
  }

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
