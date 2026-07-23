#ifndef BL_H
#define BL_H

#include "trmnl_log.h"
#include "display.h"

/**
 * @brief Function to init business logic module
 * @param none
 * @return none
 */
void bl_init(void);

/**
 * @brief Function to process business logic module
 * @param none
 * @return none
 */
void bl_process(void);

/**
 * @brief Prepares peripherals for deep sleep and enters it. Does not return.
 */
void goToSleep(void);

/**
 * @brief Shows a full-screen message with the TRMNL logo.
 * @param message_type Which message to show.
 */
void showMessageWithLogo(MSG message_type);

/**
 * @brief Shows a full-screen message with the TRMNL logo and extra device
 * details (friendly ID, firmware version, an extra message line).
 */
void showMessageWithLogo(MSG message_type, String friendly_id, bool id, const char *fw_version, String message);

/**
 * @brief Clears stored WiFi/API credentials and restarts the device.
 */
void resetDeviceCredentials(void);

uint32_t getTime(void);

enum LogAction {
  LOG_ACTION_STORE,
  LOG_ACTION_SUBMIT,
  LOG_ACTION_SUBMIT_OR_STORE
};

void logWithAction(LogAction action, LogLevel level, const char *message, time_t time, int line, const char *file);

bool submitLogString(const char *log_buffer);
bool storeLogString(const char *log_buffer);

void fixFileName(const char *src, char *dest);

#endif