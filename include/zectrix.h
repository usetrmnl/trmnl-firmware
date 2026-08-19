#pragma once

#ifdef BOARD_ZECTRIX

// Latch the board power rail and enable the e-paper rail. This must run at the
// very beginning of setup(), before the physical power button is released.
void zectrix_power_init();

// Power down peripherals while keeping the board power latch asserted across
// deep sleep.
void zectrix_prepare_deep_sleep();

#endif
