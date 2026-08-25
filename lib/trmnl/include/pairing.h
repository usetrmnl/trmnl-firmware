#pragma once

#include <string.h>

// Decide whether to (re)paint the pairing-code screen for a NO_REGISTER
// (/api/display HTTP 200 body status 202, or otherwise unregistered) response.
//
// Repaint when the device is unregistered AND either a repaint is forced
// (power-on, or content just cleared the last-shown tracker) or the stored
// friendly id differs from the code we last painted. This surfaces the pairing
// code after a server-side device deletion without e-ink thrash on fast polls.
inline bool shouldShowPairingCode(bool no_register, bool force_refresh, const char *stored_friendly_id,
                                  const char *last_shown_id) {
  if (!no_register) return false;
  if (force_refresh) return true;
  return strcmp(stored_friendly_id ? stored_friendly_id : "", last_shown_id ? last_shown_id : "") != 0;
}
