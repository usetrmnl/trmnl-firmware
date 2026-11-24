#pragma once

// TRMNL X QA: powers the panel up 30 times and fails the unit if the TPS65185 is slow to report
// power good on any try. Returns true on PASS. The result stays on screen and goes to the server
// log; a panel that never powers up draws nothing.
bool checkPanelPower(void);

// Override per build, e.g. PLATFORMIO_BUILD_FLAGS='-D QA_NETWORK_SSID=\"MY_QA\"' pio run -e TRMNL_X
#ifndef QA_NETWORK_SSID
#define QA_NETWORK_SSID "TRMNL_QA"
#endif
