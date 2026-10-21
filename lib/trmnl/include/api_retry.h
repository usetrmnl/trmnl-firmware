#pragma once

#include <stdint.h>

#define MAX_QUIET_API_RETRIES 3 // failing timer wakes that keep the current image before the error shows

enum class ApiRetryStep {
  Reset,        // request worked (or only the image host failed): count back to 1
  ShowNow,      // manual wake: show the error now, leave the count alone
  RetryQuietly, // timer wake: keep the current image and sleep on the retry interval
  GiveUp,       // timer wake, quiet retries used up: show the error
};

// retries is the stored 1-based count before this wake's attempt.
ApiRetryStep apiRetryStep(bool requestFailed, bool showErrorNow, uint8_t retries);
