/******************************************************************************
 * @file        snake.h
 * @brief       Easter egg: Snake, played on the touchbar.
 *
 *              Entry: hold the middle of the touchbar for 5 s. Left tap
 *              turns the snake 90° counter-clockwise, right tap turns
 *              clockwise.
 ******************************************************************************/

#ifndef SNAKE_H
#define SNAKE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Poll for the launch hold, having already seen a middle-channel hold.
 *
 * The device handles one touch event per wake, so entry has to resolve
 * inside a single gesture: this keeps polling CH1 until the finger has been
 * down for 5 s in total, and bails the moment it lifts. Call it from the
 * middle-hold branch of check_channel_states(), before the normal hold
 * behavior, with however long the caller's own hold detection already spent.
 *
 * @param already_held_ms hold time the caller has already confirmed
 * @return true if the finger stayed down for the full duration
 */
bool snake_entry_hold_confirmed(uint32_t already_held_ms);

/**
 * @brief Run the game. Blocking; returns when the game ends.
 *
 * Owns the display (switches to 1bpp mode) and polls the IQS323 with the
 * same I2C-lock dance as tap_mode_is_hold(). Any touch on the game-over
 * screen returns. The caller is expected to redraw the normal screen and
 * go to sleep after this returns; the result is stashed in NVS for
 * snake_submit_pending_score().
 */
void snake_run(void);

/**
 * @brief POST a stashed game result to /api/log, if one is waiting.
 *
 * The game runs before WiFi is up, so the upload is deferred to the next
 * wake with a connection. No-op when nothing is pending; the stash is
 * cleared only after a successful POST.
 *
 * @param api_key device API key
 * @param api_url base server URL (same value passed to submitLogToApi)
 */
void snake_submit_pending_score(const char *api_key, const char *api_url);

#endif // SNAKE_H
