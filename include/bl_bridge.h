#pragma once

/**
 * Shared symbols between bl.cpp and modules extracted from it.
 *
 * Temporary bridge while god-module extraction continues. Prefer shrinking
 * this surface (pass context structs) over adding more globals.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <api_types.h>
#include <api-client/display.h>
#include <display.h>
#include <special_function.h>
#include <types.h>
#include "displayed_image.h"

// --- Session / device state (defined in bl.cpp) ---
extern Preferences preferences;
extern String new_filename;
extern ApiDisplayResult apiDisplayResult;
extern uint8_t *buffer;
extern char filename[1024];
extern bool status;          // need to download a new image
extern bool reset_firmware;
extern SPECIAL_FUNCTION special_function;
extern bool touchbar_tap_mode;
extern float vBatt;
extern esp_sleep_wakeup_cause_t wakeup_reason;
extern RTC_DATA_ATTR uint8_t need_to_refresh_display;
extern RTC_DATA_ATTR bool bUsedCachedImage;
extern RTC_DATA_ATTR int iPrevWakeTime;

// --- Helpers still owned by bl.cpp ---
void showMessageWithLogo(MSG message_type);
void showMessageWithLogo(MSG message_type, String friendly_id, bool id, const char *fw_version,
                         String message);
void submitStoredLogs(void);
void writeSpecialFunction(SPECIAL_FUNCTION function);

// X-only battery / modem (defined in bl.cpp under BOARD_TRMNL_X)
#ifdef BOARD_TRMNL_X
#include "BQ27427.h"
#include "modem.h"
extern Modem *g_modem;
extern battery_count_t battery_count;
extern BQ27427 lipo;
#endif
