/******************************************************************************
 * @file        iqs323_task.h
 * @brief       FreeRTOS task manager for IQS323 touchbar initialization and
 *              RDY-based event handling during power cycles.
 * @date        2026
 ******************************************************************************/

#ifndef IQS323_TASK_H
#define IQS323_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IQS323 task states
 */
typedef enum {
    IQS323_TASK_STATE_IDLE = 0,
    IQS323_TASK_STATE_INITIALIZING,
    IQS323_TASK_STATE_RUNNING,
    IQS323_TASK_STATE_ERROR,
    IQS323_TASK_STATE_PREPARING_SLEEP,
    IQS323_TASK_STATE_SLEEPING,
} iqs323_task_state_t;

/**
 * @brief IQS323 task configuration
 */
typedef struct {
    uint8_t i2c_address;        // I2C address (default 0x44)
    int sda_pin;                // SDA pin number
    int scl_pin;                // SCL pin number
    uint8_t ready_pin;          // Ready/interrupt pin
    uint32_t init_timeout_ms;   // Initialization timeout in milliseconds
    uint8_t max_init_retries;   // Maximum initialization retry attempts
    bool use_wake_stub_data;    // Use data from RTC wake stub on GPIO wakeup
} iqs323_task_config_t;

/**
 * @brief Default configuration values
 */
#define IQS323_TASK_DEFAULT_CONFIG() { \
    .i2c_address = 0x44, \
    .sda_pin = 39, \
    .scl_pin = 40, \
    .ready_pin = 3, \
    .init_timeout_ms = 5000, \
    .max_init_retries = 3, \
    .use_wake_stub_data = true \
}

/**
 * @brief Initialize the IQS323 task manager
 *
 * Creates a FreeRTOS task that handles IQS323 initialization and maintenance.
 * Should be called early in setup(), before any other IQS323 operations.
 *
 * @param config Pointer to configuration structure (NULL uses defaults)
 * @return true if task was created successfully
 */
bool iqs323_task_init(const iqs323_task_config_t *config);

/**
 * @brief Wait for IQS323 initialization to complete
 *
 * Blocks until initialization is complete or timeout expires.
 *
 * @param timeout_ms Maximum time to wait in milliseconds (0 = infinite)
 * @return true if initialization completed successfully
 */
bool iqs323_task_wait_ready(uint32_t timeout_ms);

/**
 * @brief Check if IQS323 is ready for use
 *
 * Non-blocking check of IQS323 status.
 *
 * @return true if IQS323 is initialized and running
 */
bool iqs323_task_is_ready(void);

/**
 * @brief Get current task state
 *
 * @return Current state of the IQS323 task
 */
iqs323_task_state_t iqs323_task_get_state(void);

/**
 * @brief Request IQS323 to prepare for deep sleep
 *
 * Signals the task to prepare IQS323 for sleep mode. This includes:
 * - Reading final values
 * - Checking for resets
 * - Verifying I2C bus health
 *
 * @param timeout_ms Maximum time to wait for preparation
 * @return true if sleep preparation completed successfully
 */
bool iqs323_task_prepare_sleep(uint32_t timeout_ms);

/**
 * @brief Force re-initialization of IQS323
 *
 * Use this when IQS323 becomes unresponsive or after detecting issues.
 *
 * @return true if re-initialization was triggered
 */
bool iqs323_task_reinit(void);

/**
 * @brief Notify task that GPIO wakeup occurred
 *
 * Call this after a GPIO wakeup to let the task know it should
 * use wake stub data instead of full reinitialization.
 *
 * @param wake_stub_data_valid true if wake stub data is valid
 */
void iqs323_task_notify_gpio_wakeup(bool wake_stub_data_valid);

/**
 * @brief Get error count since last successful init
 *
 * @return Number of errors/retries encountered
 */
uint32_t iqs323_task_get_error_count(void);

/**
 * @brief Cleanup and delete the IQS323 task
 *
 * Call before system shutdown or if task is no longer needed.
 */
void iqs323_task_deinit(void);

/**
 * @brief Lock I2C bus - pauses IQS323 task's I2C operations
 *
 * Call this before using I2C with other devices (e.g., display GPIO expander).
 * While locked, the IQS323 task will not process RDY events or do any I2C.
 * Make sure to call iqs323_task_i2c_unlock() when done.
 */
void iqs323_task_i2c_lock(void);

/**
 * @brief Unlock I2C bus - resumes IQS323 task's I2C operations
 *
 * Call this after finishing I2C operations with other devices.
 * The IQS323 task will resume processing RDY events.
 */
void iqs323_task_i2c_unlock(void);

/**
 * @brief Check if I2C is currently locked
 *
 * @return true if I2C is locked by external code
 */
bool iqs323_task_i2c_is_locked(void);

/**
 * @brief Callback function type for new data notifications
 *
 * Called from IQS323 task context when new data is available after health checks pass.
 * Keep callback execution brief to avoid blocking the IQS323 task.
 */
typedef void (*iqs323_data_callback_t)(void);

/**
 * @brief Set callback function to be called when new data is available
 *
 * The callback will be invoked from the IQS323 task whenever:
 * - iqs323.run() processes new data (new_data_available becomes true)
 * - Health checks pass (no reset, no I2C lockup)
 * - The mutex is held, so it's safe to access IQS323 data
 *
 * @param callback Function to call when new data is available (NULL to disable)
 */
void iqs323_task_set_data_callback(iqs323_data_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif // IQS323_TASK_H
