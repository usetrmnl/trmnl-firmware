#ifndef TOUCHBAR_GESTURE_H
#define TOUCHBAR_GESTURE_H

#include <stdint.h>
#include "IQS323.h"

// User-level intent derived from touch bar state. CANCEL/CONFIRM only come
// from the confirmation-flow resolvers; the rest come from the normal
// per-wake navigation resolvers.
enum class touchbar_intent_t {
    NONE,
    BACK,
    NEXT,
    MIDDLE,
    CORNER_HOLD_CONFIRMED,
    CANCEL,
    CONFIRM,
};

// Resolved intent plus whether it was a hold (vs. a plain tap/swipe).
struct touchbar_resolution_t {
    touchbar_intent_t intent;
    bool held;
};

typedef touchbar_resolution_t (*touchbar_resolve_intent_fn)(void);
typedef touchbar_intent_t (*touchbar_resolve_confirmation_fn)(void);

// "N channels held continuously for >= threshold_ms" detector. Not tied to
// any polling cadence - poll() just timestamps via millis().
class TouchbarHoldTracker {
public:
    // start_reference_ms: 0 = time from first active poll ("live"); or pass
    // a fixed reference (e.g. startup_time) for boot-relative timing.
    TouchbarHoldTracker(uint8_t channel_mask, uint32_t threshold_ms, uint32_t start_reference_ms = 0)
        : channel_mask_(channel_mask), threshold_ms_(threshold_ms), start_reference_ms_(start_reference_ms) {}

    // Returns true exactly once, when all tracked channels have been held
    // continuously for >= threshold_ms. Releasing any channel resets it.
    bool poll(uint8_t current_channel_mask);
    void reset();

    // 0.0-1.0 fraction of threshold_ms elapsed since the hold became active; 0 if not active.
    float progress() const {
        if (!active_) return 0.0f;
        float p = (float)(millis() - start_ms_) / (float)threshold_ms_;
        return p > 1.0f ? 1.0f : p;
    }

private:
    uint8_t channel_mask_;
    uint32_t threshold_ms_;
    uint32_t start_reference_ms_;
    bool active_ = false;
    bool fired_ = false;
    uint32_t start_ms_ = 0;
};

// Pure gesture-resolution layer: decides tap/hold/swipe/corner outcomes from
// the IQS323 task's published snapshot. See touchbar_actions.h for the
// confirmation-flow orchestration and dispatch built on top of this.

extern bool otg_message;
extern bool touchbar_tap_mode; // false = "slide", true = "tap" (default)

void log_slider_gesture(iqs323_gesture_events event);

touchbar_resolution_t touchbar_resolve_intent(void);

touchbar_intent_t resolve_confirmation_tap_mode(void);
touchbar_intent_t resolve_confirmation_slide_mode(void);
void reset_confirmation_resolvers(void);

void set_touchbar_mode(bool tap_mode);

// Configures the IQS323's gesture mode for touchbar_tap_mode before deep sleep.
void touchbar_prepare_for_sleep(void);

#endif // TOUCHBAR_GESTURE_H
