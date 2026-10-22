#pragma once

// TRMNL X: drive the ESP32-C5 modem's enable and boot pins through the TCA9535 IO expander. C linkage
// because components/esp-serial-flasher calls them.
extern "C" {
  void modem_enter_bootloader();
  void modem_reset_target();
}
