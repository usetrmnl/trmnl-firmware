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
bool touchbar_tap_mode = true; // false = "slide", true = "tap" (default)

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
static bool tap_mode_is_hold(uint8_t channel_index, uint32_t hold_threshold_ms = 600)
{
  const uint32_t POLL_INTERVAL_MS = 20;
  touchbar_snapshot_t snap;
  TouchbarHoldTracker tracker(1 << channel_index, hold_threshold_ms, startup_time);

  while (true) {
    iqs323_task_read_snapshot(&snap);
    bool touched = snapshot_channel_touched(snap, channel_index);
    uint8_t mask = touched ? (1 << channel_index) : 0;
    if (tracker.poll(mask)) return true;
    if (!touched) return false;
    display_draw_touchbar_progress(channel_touchbar_side(channel_index), tracker.progress());
    delay(POLL_INTERVAL_MS);
  }
}

// Polls both corners together so one finger landing slightly after the other
// doesn't fail the gesture; a corner that releases before the threshold still cancels it.
static bool corner_hold_confirmed(uint32_t hold_threshold_ms = 600)
{
  const uint32_t POLL_INTERVAL_MS = 20;
  touchbar_snapshot_t snap;
  bool ch0_was_touched = false;
  bool ch2_was_touched = false;

  while (millis() - startup_time < hold_threshold_ms) {
    iqs323_task_read_snapshot(&snap);
    bool ch0_touched = snapshot_channel_touched(snap, 0);
    bool ch2_touched = snapshot_channel_touched(snap, 2);

    if ((ch0_was_touched && !ch0_touched) || (ch2_was_touched && !ch2_touched)) {
      return false;
    }
    ch0_was_touched = ch0_touched;
    ch2_was_touched = ch2_touched;

    // Only draw once both corners are actually down together, so a lone
    // corner (a plain BACK/NEXT hold) doesn't animate here at all.
    if (ch0_touched && ch2_touched) {
      uint32_t elapsed = millis() - startup_time;
      float progress = (float)elapsed / (float)hold_threshold_ms;
      display_draw_touchbar_progress_pair(TOUCHBAR_LEFT, progress, TOUCHBAR_RIGHT, progress);
    }
    delay(POLL_INTERVAL_MS);
  }

  iqs323_task_read_snapshot(&snap);
  return snapshot_channel_touched(snap, 0) && snapshot_channel_touched(snap, 2);
}

void log_slider_gesture(iqs323_gesture_events event)
{
  switch (event) {
    case IQS323_GESTURE_UNKNOWN:
      Log_info("SLIDER: UNKNOWN (something went wrong?)");
      break;
    case IQS323_GESTURE_TAP:
      Log_info("SLIDER: Tap");
      break;
    case IQS323_GESTURE_SWIPE_NEGATIVE:
      Log_info("SLIDER: Swipe <-");
      break;
    case IQS323_GESTURE_SWIPE_POSITIVE:
      Log_info("SLIDER: Swipe ->");
      break;
    case IQS323_GESTURE_FLICK_NEGATIVE:
      Log_info("SLIDER: Flick <-");
      break;
    case IQS323_GESTURE_FLICK_POSITIVE:
      Log_info("SLIDER: Flick ->");
      break;
    case IQS323_GESTURE_HOLD:
      Log_info("SLIDER: Hold");
      break;
    case IQS323_GESTURE_NONE:
      Log_info("SLIDER: None");
      break;
  }
}

static TouchbarHoldTracker g_confirmation_middle_hold_tracker(1 << IQS323_CH1, 600);
static bool g_confirmation_middle_was_touched = false;

void reset_confirmation_resolvers(void)
{
  g_confirmation_middle_hold_tracker.reset();
  g_confirmation_middle_was_touched = false;
}

touchbar_intent_t resolve_confirmation_tap_mode(void)
{
  touchbar_snapshot_t snap;
  iqs323_task_read_snapshot(&snap);

  if (snap.ch0_touch || snap.ch2_touch) {
    Log_info("Confirmation cancelled - outer button in tap mode, status: left=%d right=%d", snap.ch0_touch, snap.ch2_touch);
    display_draw_touchbar_indicator(snap.ch0_touch ? TOUCHBAR_LEFT : TOUCHBAR_RIGHT, false);
    return touchbar_intent_t::CANCEL;
  }

  if (g_confirmation_middle_was_touched && !snap.ch1_touch) {
    Log_info("Confirmation cancelled - tap on middle button in tap mode");
    g_confirmation_middle_hold_tracker.reset();
    g_confirmation_middle_was_touched = false;
    display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, false);
    return touchbar_intent_t::CANCEL;
  }
  g_confirmation_middle_was_touched = snap.ch1_touch;

  if (g_confirmation_middle_hold_tracker.poll(snap.ch1_touch ? (1 << IQS323_CH1) : 0)) {
    Log_info("Confirmed - holding middle button in tap mode");
    display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, true);
    return touchbar_intent_t::CONFIRM;
  }
  if (snap.ch1_touch) {
    display_draw_touchbar_progress(TOUCHBAR_MIDDLE, g_confirmation_middle_hold_tracker.progress());
  }
  return touchbar_intent_t::NONE;
}

touchbar_intent_t resolve_confirmation_slide_mode(void)
{
  touchbar_snapshot_t snap;
  iqs323_task_read_snapshot(&snap);

  if (snap.slider_event == IQS323_GESTURE_HOLD && snap.ch1_touch) {
    Log_info("WiFi reset confirmed by user - holding middle button");
    display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, true);
    return touchbar_intent_t::CONFIRM;
  }

  if (snap.slider_event == IQS323_GESTURE_TAP) {
    Log_info("WiFi reset cancelled by user - tap detected");
    display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, false);
    return touchbar_intent_t::CANCEL;
  }

  return touchbar_intent_t::NONE;
}

static touchbar_resolution_t resolve_intent_tap_mode(void)
{
  iqs323_task_set_streaming_mode(true);

  touchbar_resolution_t result = { touchbar_intent_t::NONE, false };

  if (corner_hold_confirmed()) {
    result = { touchbar_intent_t::CORNER_HOLD_CONFIRMED, false };
  } else {
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
  }

  iqs323_task_set_streaming_mode(false);
  return result;
}

static touchbar_resolution_t resolve_intent_slide_mode(void)
{
  touchbar_snapshot_t snap;
  iqs323_task_read_snapshot(&snap);

  if (snap.slider_event == IQS323_GESTURE_HOLD && snap.ch0_touch && snap.ch2_touch) {
    return { touchbar_intent_t::CORNER_HOLD_CONFIRMED, false };
  }

  if (snap.slider_event == IQS323_GESTURE_SWIPE_NEGATIVE) {
    return { touchbar_intent_t::BACK, false };
  }
  if (snap.slider_event == IQS323_GESTURE_SWIPE_POSITIVE) {
    return { touchbar_intent_t::NEXT, false };
  }

  if (snap.slider_event == IQS323_GESTURE_TAP || snap.slider_event == IQS323_GESTURE_HOLD) {
    bool held = (snap.slider_event == IQS323_GESTURE_HOLD);
    for (uint8_t i = 0; i < 3; i++) {
      if (!snapshot_channel_touched(snap, i)) continue;
      switch (i) {
        case 0: return { touchbar_intent_t::BACK, held };
        case 2: return { touchbar_intent_t::NEXT, held };
        case 1: return { touchbar_intent_t::MIDDLE, held };
      }
    }
  }
  return { touchbar_intent_t::NONE, false };
}

static touchbar_resolve_intent_fn g_intent_resolver = resolve_intent_tap_mode;

touchbar_resolution_t touchbar_resolve_intent(void)
{
  return g_intent_resolver();
}

void set_touchbar_mode(bool tap_mode)
{
  touchbar_tap_mode = tap_mode;
  g_intent_resolver = tap_mode ? resolve_intent_tap_mode : resolve_intent_slide_mode;
}

void touchbar_prepare_for_sleep(void)
{
  iqs323_task_set_gesture_config(touchbar_tap_mode);
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
        start_ms_ = start_reference_ms_ ? start_reference_ms_ : millis();
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
