/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "usb_private.h"
#include "hcd.h"

#if CONFIG_USB_HOST_HUBS_SUPPORTED
#define ENABLE_USB_HUBS                     1
#endif // CONFIG_USB_HOST_HUBS_SUPPORTED

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------- Events ----------------------------------------
typedef enum {
    HUB_EVENT_CONNECTED,            /**< Device has been connected */
    HUB_EVENT_RESET_COMPLETED,      /**< Device reset completed */
    HUB_EVENT_DISCONNECTED,         /**< Device has been disconnected */
} hub_event_t;

typedef struct {
    hub_event_t event;                          /**< HUB event ID */
    union {
        struct {
            unsigned int uid;                   /**< Unique device ID */
        } connected;                            /**< HUB_EVENT_CONNECTED specific data */

        struct {
            unsigned int uid;                   /**< Unique device ID */
        } reset_completed;                      /**< HUB_EVENT_RESET_COMPLETED specific data */

        struct {
            unsigned int uid;                   /**< Unique device ID */
        } disconnected;                         /**< HUB_EVENT_DISCONNECTED specific data */
    };
} hub_event_data_t;

// ---------------------- Callbacks ------------------------

/**
 * @brief Callback used to indicate that the USBH has an event
 *
 * @note This callback is called from within usbh_process()
 */
typedef void (*hub_event_cb_t)(hub_event_data_t *event_data, void *arg);

// ------------------------------------------------------ Types --------------------------------------------------------

/**
 * @brief Hub driver configuration
 */
typedef struct {
    unsigned port_map;                              /**< Bitmap of root ports to enable */
    usb_proc_req_cb_t proc_req_cb;                  /**< Processing request callback */
    void *proc_req_cb_arg;                          /**< Processing request callback argument */
    hub_event_cb_t event_cb;                        /**< Hub event callback */
    void *event_cb_arg;                             /**< Hub event callback argument */
    int intr_flags;                                 /**< Interrupt flags for HCD interrupt */
    const hcd_fifo_settings_t *fifo_config;         /**< Optional pointer to custom FIFO config. If NULL, default configuration is used. */
} hub_config_t;

// ---------------------------------------------- Hub Driver Functions -------------------------------------------------

/**
 * @brief Install Hub driver
 *
 * Entry:
 * - USBH must already be installed
 * Exit:
 * - Install Hub driver memory resources
 * - Initializes the HCD root ports selected by hub_config->port_map
 *
 * @param[in] hub_config Hub driver configuration
 * @param[out] client_ret Unique pointer to identify the Hub as a USB Host client
 *
 * @return
 *    - ESP_OK: Hub driver installed successfully
 *    - ESP_ERR_INVALID_STATE: Hub driver is not in correct state to be installed
 *    - ESP_ERR_NO_MEM: Insufficient memory
 *    - ESP_ERR_INVALID_ARG: Arguments are invalid
 */
esp_err_t hub_install(hub_config_t *hub_config, void **client_ret);

/**
 * @brief Uninstall Hub driver
 *
 * This must be called before uninstalling the USBH
 * Entry:
 * - Must have stopped all enabled root ports
 * Exit:
 * - HCD root ports deinitialized
 *
 * @return
 *    - ESP_OK: Hub driver uninstalled successfully
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed, or any enabled root port is in other state than not powered
 */
esp_err_t hub_uninstall(void);

/**
 * @brief Start the Hub driver's root ports
 *
 * This will power ON all enabled root ports.
 *
 * @return
 *    - ESP_OK: All enabled root ports have been powered on
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed, or any enabled root port is in other state than not powered
 */
esp_err_t hub_root_start(void);

/**
 * @brief Stops the Hub driver's root ports
 *
 * This will power OFF all enabled root ports.
 *
 * @return
 *    - ESP_OK: All enabled root ports have been powered off
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed, or all enabled root ports are already not powered
 */
esp_err_t hub_root_stop(void);

/**
 * @brief Check if a root port is in suspended state
 *
 * This will check root port state. In multi-port configurations, this currently
 * checks the first enabled root port only.
 *
 * @return
 *    - true: Root port is in suspended state
 *    - false: Root port is in any other state
 */
bool hub_root_is_suspended(void);

/**
 * @brief Check if the Hub driver's root port can be suspended
 *
 * This will check if all HCD pipes are idle and which state the root port is in.
 * In multi-port configurations, this currently checks the first enabled root port only.
 *
 * @return
 *    - ESP_OK: Hub driver's root port can be suspended
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed
 *    - ESP_ERR_NOT_FINISHED: HCD pipes routed through this port are still executing
 *    - ESP_ERR_TIMEOUT: Root port is already issuing a suspend command and is within a SUSPEND_ENTRY_MS timeout
 *    - ESP_ERR_NOT_ALLOWED: Root port is in other than enabled state
 */
esp_err_t hub_root_can_suspend(void);

/**
 * @brief Check if the Hub driver's root port can be resumed
 *
 * In multi-port configurations, this currently checks the first enabled root port only.
 *
 * @return
 *    - ESP_OK: Hub driver's root port can be resumed
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed
 *    - ESP_ERR_TIMEOUT: Root port is already issuing a resume command and is within a RESUME_RECOVERY_MS,
 *      or a RESUME_HOLD_MS timeout
 *    - ESP_ERR_NOT_ALLOWED: Root port is in other than suspended state, or HCD port is in incorrect state
 */
esp_err_t hub_root_can_resume(void);

/**
 * @brief Mark the Hub driver's root port as ready for suspend
 *
 * This will mark the root port as ready to be suspended and will be processed by the hub processing loop.
 * In multi-port configurations, this currently targets the first enabled root port only.
 *
 * @return
 *    - ESP_OK: Hub driver suspended successfully
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed
 *    - ESP_ERR_NOT_ALLOWED: Root port is in other than enabled state
 */
esp_err_t hub_root_mark_suspend(void);

/**
 * @brief Mark the Hub driver's root port as ready for resume
 *
 * This will mark the root port as ready to be resumed and will be processed by the hub processing loop.
 * In multi-port configurations, this currently targets the first enabled root port only.
 *
 * @return
 *    - ESP_OK: Hub driver resumed successfully
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed
 *    - ESP_ERR_NOT_ALLOWED: Root port is in other than suspended state
 */
esp_err_t hub_root_mark_resume(void);

/**
 * @brief Indicate to the Hub driver that a device's port can be recycled
 *
 * The device connected to the port has been freed.
 * The Hub driver can now recycle the port, related to the node.
 *
 * @note This function should only be called from the Host Library task
 *
 * @param[in] node_uid  Device's node unique ID
 *
 * @return
 *    - ESP_OK: Device's port can be recycled
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed
 *    - ESP_ERR_NOT_SUPPORTED: Recycling External Port is not available (External Hub support disabled),
 *      or ext hub port error
 *    - ESP_ERR_NOT_FOUND: Device's node is not found
 */
esp_err_t hub_node_recycle(unsigned int node_uid);

/**
 * @brief Reset the device in the port, related to the specific Device Tree node
 *
 * @note This function should only be called from the Host Library task
 *
 * @param[in] node_uid  Device's node unique ID
 *
 * @return
 *    - ESP_OK: If port reset was successful
 *    - ESP_ERR_INVALID_STATE: Hub driver is not installed
 *    - ESP_ERR_NOT_SUPPORTED: Resetting External Port is not available (External Hub support disabled),
 *      or ext hub port error
 *    - ESP_ERR_NOT_FOUND: Device's node is not found
 */
esp_err_t hub_node_reset(unsigned int node_uid);

/**
 * @brief Port, related to the specific Device Tree node, has an active device
 *
 * @note This function should only be called from the Host Library task
 *
 * @param[in] node_uid  Device's node unique ID
 *
 * @return
 *    - ESP_OK: If Port, related to the Device Tree node was activated successfully
 *    - ESP_ERR_NOT_SUPPORTED: If activating Port is not available (External Hub support disabled),
*      or ext hub port error
 *    - ESP_ERR_NOT_FOUND: If Device's node is not found
 */
esp_err_t hub_node_active(unsigned int node_uid);

/**
 * @brief Disable the port, related to the specific Device Tree node
 *
 * @note This function should only be called from the Host Library task
 *
 * @param[in] node_uid  Device's node unique ID
 *
 * @return
 *    - ESP_ERR_INVALID_STATE: If Hub driver is not installed
 *    - ESP_ERR_NOT_SUPPORTED: If the function is not support by the selected port
 *    - ESP_ERR_NOT_FOUND: If device's tree node is not found by uid
 *    - ESP_OK: If Port has been disabled without error
 */
esp_err_t hub_node_disable(unsigned int node_uid);

/**
 * @brief Notify Hub driver that new device has been attached
 *
 * If device is has a HUB class, then it will be added as External Hub to Hub Driver.
 *
 * @note This function should only be called from the Host Library task
 * @note If the Hub support feature is disabled and device has a Hub class, only the warning message will be shown.
 *
 * @param[in] dev_addr  Device bus address
 *
 * @return
 *    - ESP_OK: New device added successfully
 *    - ESP_ERR_INVALID_STATE: Hub driver is not in a correct state
 */
esp_err_t hub_dev_new(uint8_t dev_addr);

/**
 * @brief Notify Hub driver that device has been removed
 *
 * If the device was an External Hub, then it will be removed from the Hub Driver.
 *
 * @note This function should only be called from the Host Library task
 * @note If the Hub support feature is disabled, no additional logic requires here.
 *
 * @param[in] dev_addr  Device bus address
 *
 * @return
 *    - ESP_OK: A device removed successfully
 *    - ESP_ERR_INVALID_STATE: Hub driver is not in a correct state
 */
esp_err_t hub_dev_gone(uint8_t dev_addr);

#if ENABLE_USB_HUBS
/**
 * @brief Notify Hub driver that all devices should be freed
 *
 * @note This function should only be called from the Host Library task
 *
 * @return
 *    - ESP_OK: All the devices can be freed
 *    - ESP_ERR_INVALID_STATE: Hub driver is not in a correct state
 */
esp_err_t hub_notify_all_free(void);
#endif // ENABLE_USB_HUBS

/**
 * @brief Hub driver's processing function
 *
 * Hub driver handling function that must be called repeatedly to process the Hub driver's events. If blocking, the
 * caller can block on the notification callback of source USB_PROC_REQ_SOURCE_HUB to run this function.
 *
 * @return
 *    - ESP_OK: All events handled
 */
esp_err_t hub_process(void);

#ifdef __cplusplus
}
#endif
