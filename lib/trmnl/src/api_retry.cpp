#include <api_retry.h>

ApiRetryStep apiRetryStep(bool requestFailed, bool showErrorNow, uint8_t retries) {
  if (!requestFailed) return ApiRetryStep::Reset;
  if (showErrorNow) return ApiRetryStep::ShowNow;
  return retries <= MAX_QUIET_API_RETRIES ? ApiRetryStep::RetryQuietly : ApiRetryStep::GiveUp;
}
