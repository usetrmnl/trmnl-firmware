#ifndef SHIP_MODE_H
#define SHIP_MODE_H

#include <config.h>

#ifdef SHIP_MODE_SUPPORTED

// Runs the full user-triggered shipping-mode handshake: prompt the user to
// unplug USB, show the shipping screen, then enter a low-power sleep until a
// charger is reconnected, at which point the device marks shipment complete and
// restarts into normal operation. This call does not return.
void enter_ship_mode();

// Must be called early in setup() on ship-mode boards. If a shipment is in
// progress it either completes it (charger present / button pressed) or returns
// the device to deep sleep. May not return.
void ship_mode_boot_check();

#endif // SHIP_MODE_SUPPORTED

#endif // SHIP_MODE_H
