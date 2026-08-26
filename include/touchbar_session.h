#pragma once

/**
 * TRMNL X touchbar playlist browse helpers.
 *
 * GME: show_cached_image_by_offset for gesture→browse wiring. Confirmation
 * flows and full gesture processing remain in bl / other PRs as applicable.
 */

#ifdef BOARD_TRMNL_X

/**
 * Show a cached playlist image relative to the current browse position.
 * @param offset -1 previous, +1 next (wraps within playlist_order)
 * On success displays the image and enters deep sleep via goToSleep().
 */
void show_cached_image_by_offset(int offset);

#endif // BOARD_TRMNL_X
