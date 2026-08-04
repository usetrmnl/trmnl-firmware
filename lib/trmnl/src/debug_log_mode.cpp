#include <debug_log_mode.h>

LogLevel debug_log_threshold(uint32_t expires_at, uint32_t now) {
  // An unsynced clock cannot confirm the window is still open, and staying
  // quiet is the cheaper mistake: the next synced cycle re-arms capture, but a
  // device stuck verbose burns battery and flash unattended.
  if (now == 0) return LOG_ERROR;

  return expires_at > now ? LOG_VERBOSE : LOG_ERROR;
}

uint32_t debug_log_expiry_to_store(uint32_t stored, uint32_t from_response) {
  return from_response == 0 ? stored : from_response;
}
