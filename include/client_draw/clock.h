#pragma once

#include <client_draw/spec.h>
#include <stdint.h>

// Which touchbar zone ended the clock loop (returned by client_draw_clock_touched_zone).
#define CLIENT_DRAW_CLOCK_TOUCH_NONE   -1
#define CLIENT_DRAW_CLOCK_TOUCH_LEFT   0
#define CLIENT_DRAW_CLOCK_TOUCH_MIDDLE 1
#define CLIENT_DRAW_CLOCK_TOUCH_RIGHT  2

bool client_draw_clock_prepare(const uint8_t *png, int size);
void client_draw_clock_show(void);
bool client_draw_clock_active(void);
void client_draw_clock_run_loop(uint32_t budget_s);
uint32_t client_draw_clock_consumed_s(void);
void client_draw_clock_release(void);
// Touchbar zone that ended the loop (CLIENT_DRAW_CLOCK_TOUCH_*); valid after run_loop.
int client_draw_clock_touched_zone(void);
