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

/// @brief Report whether a PNG chunk at @p off fits inside a @p size byte buffer.
/// @param off offset of the chunk header
/// @param len payload length taken from the chunk header (attacker controlled)
/// @param size total buffer size
/// @return true when the 12 bytes of chunk overhead plus @p len bytes fit
/// Takes uint32_t so the arithmetic is 32-bit on every host. This matches the
/// device, where "off + 12 + len" wraps on a hostile length and lets an
/// oversized chunk pass a bounds check written that way.
bool client_draw_png_chunk_fits(uint32_t off, uint32_t len, uint32_t size);

void client_draw_clock_wall_time(const ClientDrawClock *spec, time_t utc_now, struct tm *out);
bool client_draw_clock_parse_json(const char *text, ClientDrawClock *out);
bool client_draw_clock_parse_png(const uint8_t *png, size_t size, ClientDrawClock *out);
