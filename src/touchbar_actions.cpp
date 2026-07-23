#include "touchbar_actions.h"

#ifdef BOARD_TRMNL_X

#include <cstring>
#include <cstdlib>
#include "touchbar_gesture.h"
#include "iqs323_task.h"
#include "IQS323.h"
#include <bl.h>
#include <display.h>
#include <config.h>
#include <filesystem.h>
#include <qa.h>
#include <Preferences.h>
#include <WifiCaptive.h>
#include <trmnl_log.h>
#include "displayed_image.h"
#include "messages.h"

extern Preferences preferences;
extern uint8_t *buffer;

#define WIFI_RESET_CONFIRMATION_TIMEOUT_MS 15000
#define WIFI_RESET_POLL_INTERVAL_MS 100

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

static void confirm_wifi_reset()
{
  resetDeviceCredentials();
}

static void confirm_power_off()
{
  clearShipmentStatus();
  ESP.restart();
}

static bool handle_confirmation_flow(bool &in_flag, MSG message, void (*on_confirm)(void))
{
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
    } while ((snap.ch0_touch || snap.ch1_touch || snap.ch2_touch) &&
             millis() - release_start < RELEASE_TIMEOUT_MS);
    iqs323_task_clear_slider_event();
  }

  reset_confirmation_resolvers();
  touchbar_resolve_confirmation_fn resolve_confirmation =
      touchbar_tap_mode ? resolve_confirmation_tap_mode : resolve_confirmation_slide_mode;
  const uint32_t poll_ms = touchbar_tap_mode ? 20 : WIFI_RESET_POLL_INTERVAL_MS;

  // Streaming mode only benefits tap mode's RDY-gated reads; slide mode reads
  // its gesture register directly every tick regardless, so skip it there.
  if (touchbar_tap_mode) {
    iqs323_task_set_streaming_mode(true);
  }

  touchbar_intent_t outcome = touchbar_intent_t::NONE;
  unsigned long start_time = millis();
  while (millis() - start_time < WIFI_RESET_CONFIRMATION_TIMEOUT_MS) {
    delay(poll_ms);
    touchbar_intent_t intent = resolve_confirmation();
    if (intent == touchbar_intent_t::CONFIRM || intent == touchbar_intent_t::CANCEL) {
      outcome = intent;
      break;
    }
  }

  if (touchbar_tap_mode) {
    iqs323_task_set_streaming_mode(false);
  }

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

static void showLastImageAndSleep()
{
  int file_size = 0;
  String curPath = preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "");
  if (!curPath.isEmpty()) {
    uint8_t *buf = display_read_file(curPath.c_str(), &file_size);
    if (buf && file_size > 0) {
      display_show_image(buf, file_size, true);
      free(buf);
      DisplayedImage::remember(curPath.c_str());
    }
  }
  goToSleep();
}

void handle_wifi_reset_confirmation()
{
  Log_info("Entering WiFi reset confirmation mode");
  bool confirmed = handle_confirmation_flow(in_wifi_reset_confirmation, WIFI_RESET_CONFIRM, confirm_wifi_reset);

  if (!confirmed) {
    Log_info("WiFi reset cancelled - redrawing last image and sleeping");
    showLastImageAndSleep();
  }
}

void handle_power_off_confirmation()
{
  Log_info("Entering power-off confirmation mode");
  handle_confirmation_flow(in_power_off_confirmation, POWER_OFF_CONFIRM, confirm_power_off);
}

void update_playlist_order(const char *new_path, const char *prev_path) {
  String order = preferences.getString(PREFERENCES_PLAYLIST_ORDER_KEY, "");
  String newStr = String(new_path);
  String prefix = newStr.substring(0, 14); // same-plugin identity (matches purge logic)
  String prevStr = String(prev_path);

  if (order.isEmpty()) {
    preferences.putString(PREFERENCES_PLAYLIST_ORDER_KEY, newStr);
    return;
  }

  // Scan list: update in-place if prefix matches (refresh), otherwise build a cleaned list
  // dropping entries whose files no longer exist (except prev_path, which anchors insertion).
  bool found = false;
  String result = "";
  int start = 0;
  while (start <= (int)order.length()) {
    int sep = order.indexOf('|', start);
    String entry = (sep < 0) ? order.substring(start) : order.substring(start, sep);
    if (!entry.isEmpty()) {
      if (!found && entry.startsWith(prefix)) {
        result += (result.isEmpty() ? "" : "|") + newStr;
        found = true;
      } else if (entry == prevStr || filesystem_file_exists(entry.c_str())) {
        result += (result.isEmpty() ? "" : "|") + entry;
      }
      // else: file was purged from filesystem — drop from list
    }
    if (sep < 0) break;
    start = sep + 1;
  }
  if (found) { preferences.putString(PREFERENCES_PLAYLIST_ORDER_KEY, result); return; }

  // New plugin — insert right after prev_path's position in the cleaned list
  String result2 = "";
  bool inserted = false;
  start = 0;
  while (start <= (int)result.length()) {
    int sep = result.indexOf('|', start);
    String entry = (sep < 0) ? result.substring(start) : result.substring(start, sep);
    if (!entry.isEmpty()) {
      result2 += (result2.isEmpty() ? "" : "|") + entry;
      if (!inserted && entry == prevStr) {
        result2 += "|" + newStr;
        inserted = true;
      }
    }
    if (sep < 0) break;
    start = sep + 1;
  }
  if (!inserted) result2 += (result2.isEmpty() ? "" : "|") + newStr;
  preferences.putString(PREFERENCES_PLAYLIST_ORDER_KEY, result2);
}

static void show_cached_image_by_offset(int offset) {
  String order = preferences.getString(PREFERENCES_PLAYLIST_ORDER_KEY, "");

  if (order.isEmpty()) {
    String path = (offset > 0)
      ? preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "")
      : preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    if (path.isEmpty()) { Log_info("No cached image for gesture"); return; }
    int file_size = 0;
    buffer = display_read_file(path.c_str(), &file_size);
    if (buffer && file_size > 0) {
      display_show_image(buffer, file_size, true);
      DisplayedImage::remember(path.c_str());
      goToSleep();
    }
    return;
  }

  char images[MAX_CACHED_IMAGES][36];
  int count = 0;
  int start = 0;
  while (start <= (int)order.length() && count < MAX_CACHED_IMAGES) {
    int sep = order.indexOf('|', start);
    String entry = (sep < 0) ? order.substring(start) : order.substring(start, sep);
    if (!entry.isEmpty() && filesystem_file_exists(entry.c_str())) {
      strncpy(images[count], entry.c_str(), 35);
      images[count][35] = '\0';
      count++;
    }
    if (sep < 0) break;
    start = sep + 1;
  }

  if (count == 0) { Log_info("No cached images available"); return; }

  String browsePath = preferences.getString(PREFERENCES_BROWSE_PATH_KEY, "");
  if (browsePath.isEmpty()) {
    // Seed from last_path so first RIGHT shows curr_path (forward) and first LEFT shows older (backward).
    // Falls back to curr_path if last_path is absent (e.g. only one image cached).
    String lp = preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    browsePath = lp.isEmpty() ? preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "") : lp;
  }

  int cur_idx = count - 1;
  for (int i = 0; i < count; i++) {
    if (browsePath == String(images[i])) { cur_idx = i; break; }
  }

  int new_idx = (cur_idx + offset + count) % count;
  Log_info("Playlist browse: %d/%d -> %d (%s)", cur_idx, count, new_idx, images[new_idx]);

  int file_size = 0;
  buffer = display_read_file(images[new_idx], &file_size);
  if (!buffer || file_size == 0) { Log_info("Failed to read %s", images[new_idx]); return; }

  preferences.putString(PREFERENCES_BROWSE_PATH_KEY, String(images[new_idx]));
  display_show_image(buffer, file_size, true);
  DisplayedImage::remember(images[new_idx]);
  goToSleep();
}

// Single dispatch point for both modes' resolved navigation intent.
static void dispatch_touchbar_intent(const touchbar_resolution_t &resolved)
{
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

void process_iqs323_data(void)
{
  touchbar_snapshot_t snap;
  iqs323_task_read_snapshot(&snap);
  Log_info("Slider position: %d", snap.slider_position);
  log_slider_gesture(snap.slider_event);

  touchbar_resolution_t resolved = touchbar_resolve_intent();

  if (resolved.intent == touchbar_intent_t::CORNER_HOLD_CONFIRMED) {
    if (!in_wifi_reset_confirmation) {
      handle_wifi_reset_confirmation();
    }
    return;
  }

  dispatch_touchbar_intent(resolved);
}

void touchbar_redraw_pending_indicator(void)
{
  if (has_pending_indicator) {
    display_draw_touchbar_indicator(pending_indicator_side, pending_indicator_filled);
    has_pending_indicator = false;
  }
}

void touchbar_init_captive_portal_power_off_hook(void)
{
  // set TAP mode as default
  iqs323_task_set_gesture_config(true);
  set_touchbar_mode(true);

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
