#pragma once

#include <stddef.h>
#include <stdint.h>
#include <time.h>

enum ClientDrawClockStyle { CLIENT_DRAW_CLOCK_NONE = 0, CLIENT_DRAW_CLOCK_DIGITAL, CLIENT_DRAW_CLOCK_ANALOG };

typedef struct {
  ClientDrawClockStyle style;
  int x;
  int y;
  int w;
  int h;
  int32_t tz; // UTC offset seconds (Ruby Time#utc_offset)
} ClientDrawClock;

/// Report whether a PNG chunk at @p off, plus its 12 bytes of overhead, fits in
/// @p size. @p len comes from the chunk header, so it is attacker controlled.
/// The parameters are uint32_t to keep the 32-bit wrap of the device, where
/// "off + 12 + len" overflows and lets an oversized chunk pass the check.
bool client_draw_png_chunk_fits(uint32_t off, uint32_t len, uint32_t size);

void client_draw_clock_wall_time(const ClientDrawClock *spec, time_t utc_now, struct tm *out);
bool client_draw_clock_parse_json(const char *text, ClientDrawClock *out);
bool client_draw_clock_parse_png(const uint8_t *png, size_t size, ClientDrawClock *out);
