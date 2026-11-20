#ifndef REVEAL_X_H
#define REVEAL_X_H

#include <stdint.h>

// Plays the server's screen transition (lib/trmnl reveal.h) on the TRMNL X, after the PNG is drawn
// into the frame buffer and before the panel shows it. TRMNL X only: it draws with FastEPD.

enum RevealPlayOutcome {
  REVEAL_NOT_PLAYED,          // no transition asked, no regions, or no memory: refresh as usual
  REVEAL_NEEDS_GRAY_REFRESH,  // played in black and white; the frame buffer holds the gray image again
  REVEAL_PANEL_SHOWS_IMAGE    // played to the end; the panel already shows the whole image
};

RevealPlayOutcome reveal_x_play(const uint8_t *png, int png_size);

#endif // REVEAL_X_H
