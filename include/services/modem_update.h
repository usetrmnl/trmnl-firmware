#pragma once

#include <Arduino.h>
#include <config.h>

// Modem (ESP32-C5) firmware update, TRMNL X only. Two steps, split across boots:
//   1. modemUpdateStage()        - if the modem doesn't report MODEM_FW_VERSION, download its image
//                                  to MODEM_IMAGE_PATH and mark it pending in NVS, then restart.
//   2. modemUpdateApplyPending() - early in the next boot: flash the pending image and check the modem.

#define MODEM_IMAGE_DIR  "/system"
#define MODEM_IMAGE_PATH MODEM_IMAGE_DIR "/factory_ESP32C5-4MB.bin" // also in the merged binary, from data/

void modemUpdateStage(void);
void modemUpdateApplyPending(void);

// Flashes the image over UART, filling the progress bar of the MODEM_FLASHING screen the caller shows first
bool modemFlash(const char *path, String &error);
