#ifndef FIREWORKS_H
#define FIREWORKS_H

// A fireworks animation, white on black, for the finished touch bar tutorial (menu.cpp). TRMNL X
// only: it draws with FastEPD.

// Wipes the screen to black, then draws the fireworks over 5 seconds, leaving them on the screen
// (and the display powered) for whatever is drawn over them next
void fireworks_show(void);

// Wipes the fireworks off with a black/white clearing cycle, leaving the screen white for what
// comes next, so the lines do not ghost into it
void fireworks_wipe(void);

#endif // FIREWORKS_H
