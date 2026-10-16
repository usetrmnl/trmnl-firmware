#ifndef TOUCHBAR_GESTURE_H
#define TOUCHBAR_GESTURE_H

#include <stdint.h>
#include "IQS323.h"

// User-level intent derived from touch bar state when a touch wakes the device.
enum class touchbar_intent_t {
    NONE,
    BACK,
    NEXT,
    MIDDLE,
};

// Resolved intent plus whether it was a hold (vs. a plain tap).
struct touchbar_resolution_t {
    touchbar_intent_t intent;
    bool held;
};

// "N channels held continuously for >= threshold_ms" detector. Not tied to
// any polling cadence - poll() just timestamps via millis().
class TouchbarHoldTracker {
public:
    // Times the hold from the first active poll ("live").
    TouchbarHoldTracker(uint8_t channel_mask, uint32_t threshold_ms)
        : channel_mask_(channel_mask), threshold_ms_(threshold_ms) {}

    // Times the hold from a fixed millis() reference instead, e.g. startup_time for
    // boot-relative timing of a touch that woke the device (startup_time can be 0).
    TouchbarHoldTracker(uint8_t channel_mask, uint32_t threshold_ms, uint32_t start_reference_ms)
        : channel_mask_(channel_mask), threshold_ms_(threshold_ms), has_reference_(true),
          start_reference_ms_(start_reference_ms) {}

    // Returns true exactly once, when all tracked channels have been held
    // continuously for >= threshold_ms. Releasing any channel resets it.
    bool poll(uint8_t current_channel_mask);
    void reset();

private:
    uint8_t channel_mask_;
    uint32_t threshold_ms_;
    bool has_reference_ = false;
    uint32_t start_reference_ms_ = 0;
    bool active_ = false;
    bool fired_ = false;
    uint32_t start_ms_ = 0;
};

// Pure gesture-resolution layer: decides tap/hold outcomes from the IQS323
// task's published snapshot. See touchbar_actions.h for the touch menu and
// dispatch built on top of this.

extern bool otg_message;

touchbar_resolution_t touchbar_resolve_intent(void);

// Configures the IQS323 for tap gestures before deep sleep.
void touchbar_prepare_for_sleep(void);

#endif // TOUCHBAR_GESTURE_H
