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
#include "messages.h"
#include "touchbar_gesture.h"

extern Preferences preferences;

#define WIFI_RESET_CONFIRMATION_TIMEOUT_MS 15000
#define WIFI_RESET_POLL_INTERVAL_MS        20

// Touchbar indicator to redraw after a full-refresh (e.g. logo screen)
static touchbar_side_t pending_indicator_side = TOUCHBAR_LEFT;
static bool pending_indicator_filled = false;
static bool has_pending_indicator = false;

// Static flag to prevent re-entry during WiFi reset confirmation
static bool in_wifi_reset_confirmation = false;
// Static flag to prevent re-entry during power-off confirmation
static bool in_power_off_confirmation = false;
// Cooldown timestamp after cancel to prevent immediate re-trigger
static uint32_t s_power_off_cooldown_until = 0;

static void confirm_wifi_reset() { resetDeviceCredentials(); }

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

void handle_wifi_reset_confirmation() {
  Log_info("Entering WiFi reset confirmation mode");
  bool confirmed = handle_confirmation_flow(in_wifi_reset_confirmation, WIFI_RESET_CONFIRM, confirm_wifi_reset);

  if (!confirmed) {
    Log_info("WiFi reset cancelled - redrawing last image and sleeping");
    show_last_image_and_sleep();
  }
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
    display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, resolved.held);
    Log_info(resolved.held ? "Middle button hold" : "Middle button tapped");
    pending_indicator_side = TOUCHBAR_MIDDLE;
    pending_indicator_filled = resolved.held;
    has_pending_indicator = true;
      // TODO: CH1 hold was previously reserved for an OTG toggle feature -
      // not implemented yet. See otg_state/otg_turn_on()/otg_turn_off().
    break;
  case touchbar_intent_t::CORNER_HOLD_CONFIRMED:
  case touchbar_intent_t::NONE:
  case touchbar_intent_t::CANCEL:
  case touchbar_intent_t::CONFIRM:
  default:
    break;
  }
}

void process_iqs323_data(void) {
  touchbar_resolution_t resolved = touchbar_resolve_intent();

  if (resolved.intent == touchbar_intent_t::CORNER_HOLD_CONFIRMED) {
    if (!in_wifi_reset_confirmation) {
      handle_wifi_reset_confirmation();
    }
    return;
  }

  dispatch_touchbar_intent(resolved);
}

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
