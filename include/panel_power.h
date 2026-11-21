#pragma once

// TRMNL X: makes a draw fail when the panel's power supply never reports power good, where FastEPD would wait
// forever. Call after bbep.initPanel().
void panel_power_limit_wait();
