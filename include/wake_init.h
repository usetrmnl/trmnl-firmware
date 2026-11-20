#pragma once

#include <Arduino.h>
#include <esp_sleep.h>

// Why the device woke, read once at boot
struct WakeContext {
  esp_sleep_wakeup_cause_t reason;
  bool gpio;          // a button or the touch bar woke it
  bool showErrorsNow; // not a timer wake, so errors show right away instead of retrying quietly first
};

// Holds the power on (Sticky), stamps the boot time and starts dev serial logging. Returns the boot
// time stamp in µs for the TRMNL X init time log.
uint32_t bootEarly(void);

// Finds this board in the device list, then starts its pins, buzzer and sensors
void initPeripherals(void);

// Reads the wake cause (also into wakeup_reason)
WakeContext readWakeContext(void);

// Opens the "data" preferences namespace, restarting the device if it can't
void openPreferences(void);

// Reads the buttons or starts the touch bar, reads the battery and starts the display and
// filesystem, each board in its own order
void startBoardHardware(const WakeContext &wake, uint32_t init_time);

// Logs the firmware and framework versions, the files in flash and the NVS usage
void logVersions(void);
