#include "iqs323_task.h"
#include "IQS323.h"
#include "rtc_wake_stub_trmnl_x.h"

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// Event bits for task synchronization
#define IQS323_EVT_INIT_DONE        (1 << 0)
#define IQS323_EVT_INIT_FAILED      (1 << 1)
#define IQS323_EVT_SLEEP_REQUEST    (1 << 2)
#define IQS323_EVT_SLEEP_DONE       (1 << 3)
#define IQS323_EVT_REINIT_REQUEST   (1 << 4)
#define IQS323_EVT_GPIO_WAKEUP      (1 << 5)
#define IQS323_EVT_SHUTDOWN         (1 << 6)

// Task parameters
#define IQS323_TASK_STACK_SIZE      4096
#define IQS323_TASK_PRIORITY        5
#define IQS323_TASK_CORE            1

// Timing constants
#define IQS323_INIT_LOOP_DELAY_MS   10
#define IQS323_RUN_LOOP_DELAY_MS    100    // How often to check for RDY events
#define IQS323_MCLR_SWITCHOVER_US   150    // Time for RDY->MCLR switchover
#define IQS323_MCLR_PULSE_US        500    // MCLR pulse width (min 250ns, using 500us for safety)
#define IQS323_RESET_RECOVERY_MS    50     // Time for device to recover after reset
#define IQS323_POST_RESET_DELAY_MS  100    // Additional delay after hardware reset before reinit

// Private variables
static TaskHandle_t iqs323_task_handle = NULL;
static EventGroupHandle_t iqs323_event_group = NULL;
static SemaphoreHandle_t iqs323_mutex = NULL;
static iqs323_task_config_t task_config;
static volatile iqs323_task_state_t current_state = IQS323_TASK_STATE_IDLE;
static volatile uint32_t error_count = 0;
static volatile bool gpio_wakeup_pending = false;
static volatile bool wake_stub_data_valid = false;
static volatile bool task_running = false;  // Track if task is actually running
static volatile bool i2c_locked = false;    // When true, task skips I2C operations
static iqs323_data_callback_t data_callback = NULL;  // Callback for new data notifications

// External IQS323 instance (defined in bl.cpp)
extern IQS323 iqs323;

// Forward declarations
static void iqs323_task_main(void *pvParameters);
static bool iqs323_do_init(bool use_wake_stub);
static void iqs323_do_prepare_sleep(void);
static bool iqs323_hardware_reset(void);

/**
 * @brief Initialize the IQS323 task manager
 */
bool iqs323_task_init(const iqs323_task_config_t *config)
{
    // Use default config if none provided
    if (config == NULL) {
        iqs323_task_config_t default_config = IQS323_TASK_DEFAULT_CONFIG();
        task_config = default_config;
    } else {
        task_config = *config;
    }

    // Create synchronization primitives
    iqs323_event_group = xEventGroupCreate();
    if (iqs323_event_group == NULL) {
        Serial.println("IQS323 Task: Failed to create event group");
        return false;
    }

    iqs323_mutex = xSemaphoreCreateMutex();
    if (iqs323_mutex == NULL) {
        Serial.println("IQS323 Task: Failed to create mutex");
        vEventGroupDelete(iqs323_event_group);
        iqs323_event_group = NULL;
        return false;
    }

    // Create the task
    task_running = true;
    BaseType_t result = xTaskCreatePinnedToCore(
        iqs323_task_main,
        "iqs323_task",
        IQS323_TASK_STACK_SIZE,
        NULL,
        IQS323_TASK_PRIORITY,
        &iqs323_task_handle,
        IQS323_TASK_CORE
    );

    if (result != pdPASS) {
        Serial.println("IQS323 Task: Failed to create task");
        task_running = false;
        vSemaphoreDelete(iqs323_mutex);
        iqs323_mutex = NULL;
        vEventGroupDelete(iqs323_event_group);
        iqs323_event_group = NULL;
        return false;
    }

    Serial.println("IQS323 Task: Created successfully");
    return true;
}

/**
 * @brief Wait for IQS323 initialization to complete
 */
bool iqs323_task_wait_ready(uint32_t timeout_ms)
{
    if (iqs323_event_group == NULL) {
        return false;
    }

    TickType_t wait_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    EventBits_t bits = xEventGroupWaitBits(
        iqs323_event_group,
        IQS323_EVT_INIT_DONE | IQS323_EVT_INIT_FAILED,
        pdFALSE,  // Don't clear bits
        pdFALSE,  // Wait for any bit
        wait_ticks
    );

    return (bits & IQS323_EVT_INIT_DONE) != 0;
}

/**
 * @brief Check if IQS323 is ready
 */
bool iqs323_task_is_ready(void)
{
    return current_state == IQS323_TASK_STATE_RUNNING;
}

/**
 * @brief Get current task state
 */
iqs323_task_state_t iqs323_task_get_state(void)
{
    return current_state;
}

/**
 * @brief Request sleep preparation
 */
bool iqs323_task_prepare_sleep(uint32_t timeout_ms)
{
    if (iqs323_event_group == NULL || current_state == IQS323_TASK_STATE_SLEEPING) {
        return true;  // Already sleeping or no event group
    }

    // Clear any previous sleep done event
    xEventGroupClearBits(iqs323_event_group, IQS323_EVT_SLEEP_DONE);

    // Request sleep preparation
    xEventGroupSetBits(iqs323_event_group, IQS323_EVT_SLEEP_REQUEST);

    // Wait for completion
    TickType_t wait_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    EventBits_t bits = xEventGroupWaitBits(
        iqs323_event_group,
        IQS323_EVT_SLEEP_DONE,
        pdTRUE,   // Clear the bit
        pdFALSE,  // Wait for this bit
        wait_ticks
    );

    return (bits & IQS323_EVT_SLEEP_DONE) != 0;
}

/**
 * @brief Force re-initialization
 */
bool iqs323_task_reinit(void)
{
    if (iqs323_event_group == NULL) {
        return false;
    }

    xEventGroupSetBits(iqs323_event_group, IQS323_EVT_REINIT_REQUEST);
    return true;
}

/**
 * @brief Notify of GPIO wakeup
 */
void iqs323_task_notify_gpio_wakeup(bool data_valid)
{
    gpio_wakeup_pending = true;
    wake_stub_data_valid = data_valid;

    if (iqs323_event_group != NULL) {
        xEventGroupSetBits(iqs323_event_group, IQS323_EVT_GPIO_WAKEUP);
    }
}

/**
 * @brief Get error count
 */
uint32_t iqs323_task_get_error_count(void)
{
    return error_count;
}

/**
 * @brief Cleanup task - must be called before deep sleep
 */
void iqs323_task_deinit(void)
{
    if (iqs323_task_handle == NULL) {
        return;  // Already cleaned up
    }

    // Signal shutdown and wait for task to acknowledge
    if (iqs323_event_group != NULL && task_running) {
        xEventGroupSetBits(iqs323_event_group, IQS323_EVT_SHUTDOWN);

        // Wait for task to suspend (it will set task_running = false)
        uint32_t wait_start = millis();
        while (task_running && (millis() - wait_start) < 500) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Delete the task (it's either suspended or timed out)
    if (iqs323_task_handle != NULL) {
        vTaskDelete(iqs323_task_handle);
        iqs323_task_handle = NULL;
    }
    task_running = false;

    // Clean up synchronization primitives
    if (iqs323_mutex != NULL) {
        vSemaphoreDelete(iqs323_mutex);
        iqs323_mutex = NULL;
    }

    if (iqs323_event_group != NULL) {
        vEventGroupDelete(iqs323_event_group);
        iqs323_event_group = NULL;
    }

    current_state = IQS323_TASK_STATE_IDLE;
    i2c_locked = false;
    Serial.println("IQS323 Task: Cleanup complete");
}

/**
 * @brief Lock I2C bus - pauses IQS323 task's I2C operations
 */
void iqs323_task_i2c_lock(void)
{
    i2c_locked = true;
}

/**
 * @brief Unlock I2C bus - resumes IQS323 task's I2C operations
 */
void iqs323_task_i2c_unlock(void)
{
    i2c_locked = false;
}

/**
 * @brief Check if I2C is currently locked
 */
bool iqs323_task_i2c_is_locked(void)
{
    return i2c_locked;
}

/**
 * @brief Set callback function for new data notifications
 */
void iqs323_task_set_data_callback(iqs323_data_callback_t callback)
{
    data_callback = callback;
}

/**
 * @brief Main task function
 */
static void iqs323_task_main(void *pvParameters)
{
    Serial.println("IQS323 Task: Starting...");

    current_state = IQS323_TASK_STATE_INITIALIZING;

    // Check if this is a GPIO wakeup with valid wake stub data
    bool use_wake_stub = gpio_wakeup_pending &&
                         wake_stub_data_valid &&
                         task_config.use_wake_stub_data;

    // Perform initialization with retries
    uint8_t retry_count = 0;
    bool init_success = false;

    while (retry_count < task_config.max_init_retries && !init_success) {
        Serial.printf("IQS323 Task: Init attempt %d/%d (use_wake_stub=%d)\n",
                      retry_count + 1, task_config.max_init_retries, use_wake_stub);

        init_success = iqs323_do_init(use_wake_stub);

        if (!init_success) {
            retry_count++;
            error_count = error_count + 1;
            use_wake_stub = false;  // Don't try wake stub again after failure

            if (retry_count < task_config.max_init_retries) {
                Serial.println("IQS323 Task: Init failed, attempting hardware reset...");
                if (iqs323_hardware_reset()) {
                    Serial.println("IQS323 Task: Hardware reset completed");
                }
                vTaskDelay(pdMS_TO_TICKS(IQS323_POST_RESET_DELAY_MS));
            }
        }
    }

    if (init_success) {
        current_state = IQS323_TASK_STATE_RUNNING;
        xEventGroupSetBits(iqs323_event_group, IQS323_EVT_INIT_DONE);
        Serial.println("IQS323 Task: Initialization complete");
    } else {
        current_state = IQS323_TASK_STATE_ERROR;
        xEventGroupSetBits(iqs323_event_group, IQS323_EVT_INIT_FAILED);
        Serial.println("IQS323 Task: Initialization failed after all retries");
    }

    // Main task loop
    // The IQS323 library handles RDY interrupt internally (sets iqs323_deviceRDY flag)
    // We periodically call run() which only does I2C when the flag is set
    while (task_running) {
        // Wait for control events with timeout
        EventBits_t events = xEventGroupWaitBits(
            iqs323_event_group,
            IQS323_EVT_SLEEP_REQUEST | IQS323_EVT_REINIT_REQUEST | IQS323_EVT_SHUTDOWN,
            pdTRUE,   // Clear bits on return
            pdFALSE,  // Wait for any bit
            pdMS_TO_TICKS(IQS323_RUN_LOOP_DELAY_MS)
        );

        // Handle shutdown request - highest priority
        if (events & IQS323_EVT_SHUTDOWN) {
            Serial.println("IQS323 Task: Shutdown requested");
            break;
        }

        // Handle sleep request
        if (events & IQS323_EVT_SLEEP_REQUEST) {
            Serial.println("IQS323 Task: Sleep preparation requested");
            current_state = IQS323_TASK_STATE_PREPARING_SLEEP;
            iqs323_do_prepare_sleep();
            current_state = IQS323_TASK_STATE_SLEEPING;
            xEventGroupSetBits(iqs323_event_group, IQS323_EVT_SLEEP_DONE);
            Serial.println("IQS323 Task: Sleep preparation complete");
            continue;
        }

        // Handle reinit request
        if (events & IQS323_EVT_REINIT_REQUEST) {
            Serial.println("IQS323 Task: Reinit requested");
            current_state = IQS323_TASK_STATE_INITIALIZING;
            gpio_wakeup_pending = false;

            if (iqs323_do_init(false)) {
                current_state = IQS323_TASK_STATE_RUNNING;
                Serial.println("IQS323 Task: Reinit successful");
            } else {
                current_state = IQS323_TASK_STATE_ERROR;
                error_count = error_count + 1;
                Serial.println("IQS323 Task: Reinit failed");
            }
            continue;
        }

        // Process IQS323 if running and I2C not locked by external code
        // The library's run() only does I2C when RDY flag is set (from library's ISR)
        if (current_state == IQS323_TASK_STATE_RUNNING && !i2c_locked) {
            if (xSemaphoreTake(iqs323_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Run IQS323 state machine - only does I2C if iqs323_deviceRDY is true
                iqs323.run();

                // Only do health checks if run() actually processed a RDY event
                if (iqs323.new_data_available) {
                    // Check for unexpected reset
                    if (iqs323.checkReset()) {
                        Serial.println("IQS323 Task: Unexpected reset detected");
                        xSemaphoreGive(iqs323_mutex);
                        xEventGroupSetBits(iqs323_event_group, IQS323_EVT_REINIT_REQUEST);
                        continue;
                    }

                    // Check for I2C lockup
                    if (iqs323.check_i2c_lockup()) {
                        Serial.println("IQS323 Task: I2C lockup detected");
                        xSemaphoreGive(iqs323_mutex);
                        xEventGroupSetBits(iqs323_event_group, IQS323_EVT_REINIT_REQUEST);
                        continue;
                    }

                    // Call user callback if registered - mutex is still held
                    if (data_callback != NULL) {
                        data_callback();
                    }
                    iqs323.new_data_available = false;
                }

                xSemaphoreGive(iqs323_mutex);
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }

    Serial.println("IQS323 Task: Exiting");

    task_running = false;
    vTaskSuspend(NULL);  // Suspend task, deinit() will delete it
}

/**
 * @brief Perform IQS323 initialization
 */
static bool iqs323_do_init(bool use_wake_stub)
{
    if (xSemaphoreTake(iqs323_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.println("IQS323 Task: Failed to acquire mutex for init");
        return false;
    }

    bool success = false;

    if (use_wake_stub) {
        // Use data from wake stub - faster path for GPIO wakeup
        Serial.println("IQS323 Task: Using wake stub data");

        // Set up IQS323 without full init (just configures pins and address)
        iqs323.begin(
            task_config.i2c_address,
            task_config.sda_pin,
            task_config.scl_pin,
            task_config.ready_pin,
            false  // to_init = false - skip full initialization
        );

        // Load data from wake stub
        iqs323.setIQSMemoryMap(wakeup_stub_iqs_status);

        // Check if IQS323 reset during sleep
        if (iqs323.checkReset()) {
            Serial.println("IQS323 Task: Reset detected in wake stub data, need full init");
            use_wake_stub = false;
        } else {
            // Check I2C health by doing a quick communication check
            if (iqs323.check_i2c_lockup()) {
                Serial.println("IQS323 Task: I2C lockup detected after wake, need full init");
                use_wake_stub = false;
            } else {
                // Data is valid, set state to running
                iqs323.iqs323_state.state = IQS323_STATE_RUN;
                iqs323.iqs323_state.init_state = IQS323_INIT_DONE;
                iqs323.new_data_available = true;
                success = true;
                Serial.println("IQS323 Task: Wake stub data validated OK");
            }
        }
    }

    if (!success) {
        // Full initialization
        Serial.println("IQS323 Task: Starting full initialization");

        // Initialize IQS323 with interrupt
        iqs323.begin(
            task_config.i2c_address,
            task_config.sda_pin,
            task_config.scl_pin,
            task_config.ready_pin,
            true  // to_init = true
        );

        // Request software reset to ensure clean state
        Serial.println("IQS323 Task: Requesting SW reset");
        iqs323.SW_Reset(true);  // STOP
        vTaskDelay(pdMS_TO_TICKS(100));

        // Run initialization state machine with timeout
        uint32_t start_time = millis();
        uint32_t timeout = task_config.init_timeout_ms;

        while ((millis() - start_time) < timeout) {
            iqs323.run();

            if (iqs323.new_data_available) {
                success = true;
                break;
            }

            if (iqs323.iqs323_state.init_state == IQS323_INIT_NONE) {
                Serial.println("IQS323 Task: Init state machine error");
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(IQS323_INIT_LOOP_DELAY_MS));
        }

        if (!success && (millis() - start_time) >= timeout) {
            Serial.println("IQS323 Task: Initialization timeout");
        }
    }

    xSemaphoreGive(iqs323_mutex);
    return success;
}

/**
 * @brief Prepare IQS323 for deep sleep
 */
static void iqs323_do_prepare_sleep(void)
{
    if (xSemaphoreTake(iqs323_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.println("IQS323 Task: Failed to acquire mutex for sleep prep");
        return;
    }

    Serial.println("IQS323 Task: Reading final values before sleep");
    iqs323.queueValueUpdates();

    iqs323.check_i2c_lockup();

    if (iqs323.checkReset()) {
        Serial.println("IQS323 Task: Reset detected before sleep");
    }

    // Activate event mode for low-power operation during sleep
    // In event mode, IQS323 only opens communication windows on touch events
    Serial.println("IQS323 Task: Activating event mode for sleep");
    iqs323.setEventMode(true);  // STOP = true

    Serial.println("IQS323 Task: IQS323 ready for sleep");

    xSemaphoreGive(iqs323_mutex);
}

/**
 * @brief Perform hardware reset of IQS323 via MCLR pin
 */
static bool iqs323_hardware_reset(void)
{
    Serial.println("IQS323 Task: Performing hardware reset via MCLR");

    gpio_num_t mclr_pin = (gpio_num_t)task_config.ready_pin;

    // Step 1: Wait for RDY to go HIGH (communication window closed)
    uint32_t timeout_start = millis();
    while (gpio_get_level(mclr_pin) == 0) {
        if (millis() - timeout_start > 500) {
            Serial.println("IQS323 Task: Timeout waiting for RDY to go HIGH");
            break;
        }
        delayMicroseconds(10);
    }

    // Step 2: Wait for RDY->MCLR switchover (150µs typical)
    delayMicroseconds(IQS323_MCLR_SWITCHOVER_US);

    // Step 3: Detach any existing interrupt
    detachInterrupt(mclr_pin);

    // Step 4: Configure pin as output and pull LOW
    gpio_set_direction(mclr_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(mclr_pin, 0);

    // Step 5: Hold LOW for MCLR pulse (min 250ns, using 500µs for safety)
    delayMicroseconds(IQS323_MCLR_PULSE_US);

    // Step 6: Release pin - configure as input with pull-up
    gpio_set_direction(mclr_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(mclr_pin, GPIO_PULLUP_ONLY);

    // Step 7: Wait for device to recover after reset
    delay(IQS323_RESET_RECOVERY_MS);

    // Verify device responds on I2C
    Wire.beginTransmission(task_config.i2c_address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
        Serial.println("IQS323 Task: Hardware reset successful, device responding");
        return true;
    }

    // Give it more time
    delay(100);
    Wire.beginTransmission(task_config.i2c_address);
    error = Wire.endTransmission();

    if (error == 0) {
        Serial.println("IQS323 Task: Hardware reset successful after additional wait");
        return true;
    }

    Serial.println("IQS323 Task: Hardware reset failed - device not responding");
    return false;
}
