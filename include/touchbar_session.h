#pragma once

/**
 * TRMNL X touchbar confirmation flows (Wi-Fi reset / power-off).
 *
 * GME: confirmation handlers only. Gesture processing, playlist browse, and
 * portal tick remain in bl.cpp until follow-up PRs.
 */

#ifdef BOARD_TRMNL_X

#include <stdint.h>

/** Re-entrancy flags / cooldown shared with bl portal-tick and gesture code. */
extern bool in_wifi_reset_confirmation;
extern bool in_power_off_confirmation;
extern uint32_t s_power_off_cooldown_until;

void handle_wifi_reset_confirmation(void);
void handle_power_off_confirmation(void);

#endif // BOARD_TRMNL_X
