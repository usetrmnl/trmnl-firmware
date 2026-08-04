#ifndef BL_H
#define BL_H

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

enum LogAction { LOG_ACTION_STORE, LOG_ACTION_SUBMIT, LOG_ACTION_SUBMIT_OR_STORE };

void logWithAction(LogAction action, LogLevel level, const char *message, time_t time, int line, const char *file);

bool submitLogString(const char *log_buffer);
bool storeLogString(const char *log_buffer);

/**
 * @brief Decide whether this wake cycle captures verbose logs, from the expiry
 *        the server last sent. Call once the filesystem and preferences are up:
 *        capture needs somewhere to write that is not the small NVS log store.
 * @param none
 * @return none
 */
void app_logger_begin(void);

/**
 * @brief Whether this wake cycle is capturing verbose logs.
 * @param none
 * @return bool true while capture is active
 */
bool app_logger_capturing(void);

#endif