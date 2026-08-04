#pragma once

#include <stdint.h>
#include <trmnl_log.h>

/// @brief Severity at or above which entries are stored for submission.
///
/// Verbose capture is bounded by a deadline rather than a flag so that a device
/// which stops hearing from the server - often the reason it is being debugged -
/// still returns to normal on its own.
///
/// @param expires_at epoch seconds the server captured until, 0 when never set
/// @param now epoch seconds, 0 when the clock has not been synced
LogLevel debug_log_threshold(uint32_t expires_at, uint32_t now);

/// @brief The expiry to persist, given what is already stored and what the
///        latest display response carried.
///
/// A response without the field leaves the stored value alone. The server's
/// error paths (maintenance mode, 202, technical difficulties) render different
/// response shapes, and treating their silence as a cancel would drop a device
/// out of capture exactly when it is being watched. The deadline still
/// guarantees capture ends; the server cancels early by sending a past value.
///
/// @param stored currently persisted expiry
/// @param from_response expiry from the display response, 0 when absent
uint32_t debug_log_expiry_to_store(uint32_t stored, uint32_t from_response);
