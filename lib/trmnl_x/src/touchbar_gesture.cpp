#include "touchbar_gesture.h"
#include "iqs323_task.h"
#include <Arduino.h>
#include <trmnl_log.h>

// display.h lives in the project's include/ dir, not reachable from lib/* normally.
#define _NO_DEV_CONFIG_
#define UWORD uint16_t
#include "../../../include/display.h"

extern unsigned long startup_time; // set in bl_init()

bool otg_message = false;

static bool snapshot_channel_touched(const touchbar_snapshot_t &snap, uint8_t channel_index)
{
  switch (channel_index) {
    case 0: return snap.ch0_touch;
    case 1: return snap.ch1_touch;
    case 2: return snap.ch2_touch;
    default: return false;
  }
}

static touchbar_side_t channel_touchbar_side(uint8_t channel_index)
{
  switch (channel_index) {
    case 0: return TOUCHBAR_LEFT;
    case 2: return TOUCHBAR_RIGHT;
    default: return TOUCHBAR_MIDDLE;
  }
}

// True if channel i has been held since boot (not since first observed) for
// hold_threshold_ms, to account for wake-stub touches that started before boot.
// Shows a left or right touch's indicator while the channel is still touched. A middle tap opens
// the menu, which is drawn over the screen without clearing it, so its pill would show through.
static bool tap_mode_is_hold(uint8_t channel_index, uint32_t hold_threshold_ms = 600)
{
  const uint32_t POLL_INTERVAL_MS = 20;
  touchbar_snapshot_t snap;
  TouchbarHoldTracker tracker(1 << channel_index, hold_threshold_ms, startup_time);
  bool indicator_drawn = channel_index == 1;

  while (true) {
    iqs323_task_read_snapshot(&snap);
    bool touched = snapshot_channel_touched(snap, channel_index);
    uint8_t mask = touched ? (1 << channel_index) : 0;
    if (tracker.poll(mask)) return true;
    if (!touched) return false;
    if (!indicator_drawn) {
      display_draw_touchbar_indicator(channel_touchbar_side(channel_index));
      indicator_drawn = true;
    }
    delay(POLL_INTERVAL_MS);
  }
}

static touchbar_resolution_t resolve_intent_tap_mode(void)
{
  iqs323_task_set_streaming_mode(true);

  touchbar_resolution_t result = { touchbar_intent_t::NONE, false };

  // A quick tap may have already released by boot; also check the
  // wake-stub-time snapshot for which channel was touched.
  touchbar_snapshot_t snap;
  iqs323_task_read_snapshot(&snap);
  touchbar_snapshot_t wake_snap;
  iqs323_task_read_wake_stub_snapshot(&wake_snap);
  for (uint8_t i = 0; i < 3; i++) {
    if (!snapshot_channel_touched(snap, i) && !snapshot_channel_touched(wake_snap, i)) continue;
    bool hold = tap_mode_is_hold(i, 2000);
    switch (i) {
      case 0: result = { touchbar_intent_t::BACK, hold }; break;
      case 2: result = { touchbar_intent_t::NEXT, hold }; break;
      case 1: result = { touchbar_intent_t::MIDDLE, hold }; break;
    }
    break; // only the first touched channel is handled per resolve
  }

  iqs323_task_set_streaming_mode(false);
  return result;
}

touchbar_resolution_t touchbar_resolve_intent(void)
{
  return resolve_intent_tap_mode();
}

void touchbar_prepare_for_sleep(void)
{
  iqs323_task_set_gesture_config();
}

bool TouchbarHoldTracker::poll(uint8_t current_channel_mask)
{
    bool held = (current_channel_mask & channel_mask_) == channel_mask_;

    if (!held) {
        reset();
        return false;
    }

    if (!active_) {
        active_ = true;
        fired_ = false;
        start_ms_ = has_reference_ ? start_reference_ms_ : millis();
    }

    if (!fired_ && (millis() - start_ms_ >= threshold_ms_)) {
        fired_ = true;
        return true;
    }

    return false;
}

void TouchbarHoldTracker::reset()
{
    active_ = false;
    fired_ = false;
    start_ms_ = 0;
}
