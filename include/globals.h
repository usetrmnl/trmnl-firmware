#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <api-client/display.h>
#include <config.h>
#include <display.h>
#include <esp_sleep.h>
#include <firmware_update.h>
#include <preferences_persistence.h>
#include <refresh_interval.h>
#include <special_function.h>
#include <stored_logs.h>

#ifdef SENSOR_SDA
#include <bb_scd41.h>
#include <bb_temperature.h>
#endif

#ifdef BOARD_TRMNL_X
#include "IQS323.h"
class Modem;
#endif

//
// Shared application state, defined in globals.cpp. Include this header
// instead of declaring ad-hoc externs against bl.cpp or display.cpp.
//

// --- Persistence & long-lived services ---
extern Preferences preferences;
extern PreferencesPersistence preferencesPersistence;
extern StoredLogs storedLogs;
extern RefreshInterval refreshInterval;
extern FirmwareUpdateService firmwareUpdateService;

// --- Image download / API state ---
extern String new_filename;
extern ApiDisplayResult apiDisplayResult;
extern uint8_t *buffer;
extern char filename[1024];      // image URL
extern char message_buffer[128]; // message to show on the screen
extern bool status;              // need to download a new image
extern bool reset_firmware;      // need to reset credentials
extern bool log_retry;           // need to log connection retry

// --- Wake / sleep state ---
extern uint32_t time_since_sleep;
extern esp_sleep_wakeup_cause_t wakeup_reason;
extern SPECIAL_FUNCTION special_function;
extern int iPrevWakeTime;               // RTC: total wake time of the last cycle (for statistics collection)
extern bool bUsedCachedImage;           // RTC: last image displayed was read from cache (for statistics collection)
extern uint8_t need_to_refresh_display; // RTC
extern bool otg_state;                  // RTC: OTG state across deep sleep

// --- UI / input ---
extern bool touchbar_tap_mode; // false = "slide", true = "tap" (default)
extern bool otg_message;

// --- Display refresh bookkeeping ---
extern int iUpdateCount;   // RTC: partial updates since the last full refresh
extern bool bCanDoPartial; // RTC
extern uint32_t iTempProfile;

#ifdef SENSOR_SDA
// --- Environmental sensors ---
extern SCD41 scd41;
extern BBTemp bbt;
extern bool bCO2;
extern int iSensorType;
extern int lastCO2, lastSCDTemp, lastTemp, lastSCDHumid, lastHumid, lastPressure, lastType, lastTime;
#endif // SENSOR_SDA

#ifdef BOARD_TRMNL_X
// --- TRMNL X hardware ---
extern Modem *g_modem;
extern battery_count_t battery_count;
extern bool battery_charging;
extern IQS323 iqs323;
extern uint16_t slider_position;
extern iqs323_gesture_events slider_event;
#endif // BOARD_TRMNL_X
