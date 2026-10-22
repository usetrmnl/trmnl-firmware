#ifndef BL_H
#define BL_H

#include "display.h"
#include "trmnl_log.h"

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
 * @brief Clears stored WiFi/API credentials and restarts the device.
 */
void resetDeviceCredentials(void);

enum LogAction { LOG_ACTION_STORE, LOG_ACTION_SUBMIT, LOG_ACTION_SUBMIT_OR_STORE };

void logWithAction(LogAction action, LogLevel level, const char *message, time_t time, int line, const char *file);

bool submitLogString(const char *log_buffer);
bool storeLogString(const char *log_buffer);

#endif