#include "touchbar_actions.h"

#ifdef BOARD_TRMNL_X

#include <Preferences.h>
#include <WifiCaptive.h>
#include <bl.h>
#include <config.h>
#include <display.h>
#include <qa.h>
#include <trmnl_log.h>

#include "IQS323.h"
#include "cached_images.h"
#include "iqs323_task.h"
#include "menu.h"
#include "messages.h"
#include "touchbar_gesture.h"

extern Preferences preferences;

#define WIFI_RESET_CONFIRMATION_TIMEOUT_MS 15000
#define WIFI_RESET_POLL_INTERVAL_MS        20

// Touchbar indicator to redraw after a full-refresh (e.g. logo screen)
static touchbar_side_t pending_indicator_side = TOUCHBAR_LEFT;
static bool pending_indicator_filled = false;
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

static const menu_controls_t touchbar_menu_controls = {
    touchbar_menu_hints,
    touchbar_menu_begin,
    touchbar_menu_end,
    touchbar_menu_read,
};

// Static flag to prevent re-entry during power-off confirmation
static bool in_power_off_confirmation = false;
// Cooldown timestamp after cancel to prevent immediate re-trigger
static uint32_t s_power_off_cooldown_until = 0;

static void confirm_power_off() {
  clearShipmentStatus();
  ESP.restart();
}

static bool handle_confirmation_flow(bool &in_flag, MSG message, void (*on_confirm)(void)) {
  in_flag = true;
  showMessageWithLogo(message);

  // Wait for the triggering hold to release, so lifting fingers doesn't register as a cancel tap.
  {
    const uint32_t RELEASE_TIMEOUT_MS = 2000;
    unsigned long release_start = millis();
    touchbar_snapshot_t snap;
    do {
      delay(200);
      iqs323_task_read_snapshot(&snap);
    } while ((snap.ch0_touch || snap.ch1_touch || snap.ch2_touch) && millis() - release_start < RELEASE_TIMEOUT_MS);
  }

  reset_confirmation_resolvers();
  iqs323_task_set_streaming_mode(true);

  touchbar_intent_t outcome = touchbar_intent_t::NONE;
  unsigned long start_time = millis();
  while (millis() - start_time < WIFI_RESET_CONFIRMATION_TIMEOUT_MS) {
    delay(WIFI_RESET_POLL_INTERVAL_MS);
    touchbar_intent_t intent = resolve_confirmation_tap_mode();
    if (intent == touchbar_intent_t::CONFIRM || intent == touchbar_intent_t::CANCEL) {
      outcome = intent;
      break;
    }
  }

  iqs323_task_set_streaming_mode(false);

  if (outcome == touchbar_intent_t::NONE) {
    Log_info("Confirmation timeout - cancelling");
  }
  in_flag = false;
  if (outcome == touchbar_intent_t::CONFIRM) {
    on_confirm();
    return true;
  }
  return false;
}

void handle_power_off_confirmation() {
  Log_info("Entering power-off confirmation mode");
  handle_confirmation_flow(in_power_off_confirmation, POWER_OFF_CONFIRM, confirm_power_off);
}

// Single dispatch point for both modes' resolved navigation intent.
static void dispatch_touchbar_intent(const touchbar_resolution_t &resolved) {
  switch (resolved.intent) {
  case touchbar_intent_t::BACK:
    display_draw_touchbar_indicator(TOUCHBAR_LEFT, resolved.held);
    Log_info(resolved.held ? "Back button hold" : "Back button tapped");
    show_cached_image_by_offset(-1);
    break;
  case touchbar_intent_t::NEXT:
    display_draw_touchbar_indicator(TOUCHBAR_RIGHT, resolved.held);
    Log_info(resolved.held ? "Next button hold" : "Next button tapped");
    show_cached_image_by_offset(+1);
    break;
  case touchbar_intent_t::MIDDLE:
    if (!resolved.held) {
      Log_info("Middle button tapped - opening the menu");
      menu_open(touchbar_menu_controls);
      break;
    }
    display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, true);
    Log_info("Middle button hold");
    pending_indicator_side = TOUCHBAR_MIDDLE;
    pending_indicator_filled = true;
    has_pending_indicator = true;
      // TODO: CH1 hold was previously reserved for an OTG toggle feature -
      // not implemented yet. See otg_state/otg_turn_on()/otg_turn_off().
    break;
  case touchbar_intent_t::NONE:
  case touchbar_intent_t::CANCEL:
  case touchbar_intent_t::CONFIRM:
  default:
    break;
  }
}

void process_iqs323_data(void) { dispatch_touchbar_intent(touchbar_resolve_intent()); }

void touchbar_redraw_pending_indicator(void) {
  if (has_pending_indicator) {
    display_draw_touchbar_indicator(pending_indicator_side, pending_indicator_filled);
    has_pending_indicator = false;
  }
}

void touchbar_init_captive_portal_power_off_hook(void) {
  iqs323_task_set_gesture_config();

  static const uint8_t CORNER_MASK = (1 << IQS323_CH0) | (1 << IQS323_CH2);
  static TouchbarHoldTracker corner_hold_tracker(CORNER_MASK, 600);
  WifiCaptivePortal.setPortalTickCallback([]() {
    if (in_power_off_confirmation) return;
    if (millis() < s_power_off_cooldown_until) return;

    touchbar_snapshot_t snap;
    iqs323_task_read_snapshot(&snap);
    uint8_t touched_mask = 0;
    if (snap.ch0_touch) touched_mask |= (1 << IQS323_CH0);
    if (snap.ch2_touch) touched_mask |= (1 << IQS323_CH2);

    if (corner_hold_tracker.poll(touched_mask)) {
      handle_power_off_confirmation();
      // Only reached on cancel — confirmed path calls ESP.restart()
      s_power_off_cooldown_until = millis() + 2000;
      showMessageWithLogo(WIFI_CONNECT, "", false, Messages::firmware_version().c_str(), WifiCaptivePortal.getAPSSID());
      return;
    }
    if (touched_mask == CORNER_MASK) {
      float progress = corner_hold_tracker.progress();
      display_draw_touchbar_progress_pair(TOUCHBAR_LEFT, progress, TOUCHBAR_RIGHT, progress);
    }
  });
}

#endif // BOARD_TRMNL_X
