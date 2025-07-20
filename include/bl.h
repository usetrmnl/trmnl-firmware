#ifndef BL_H
#define BL_H

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

uint32_t getTime(void);

enum LogAction {
  LOG_ACTION_STORE,
  LOG_ACTION_SUBMIT,
  LOG_ACTION_SUBMIT_OR_STORE
};

void logWithAction(LogAction action, const char *message, time_t time, int line, const char *file);

bool submitLogString(const char *log_buffer);
bool storeLogString(const char *log_buffer);

#endif