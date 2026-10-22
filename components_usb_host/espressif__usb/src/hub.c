/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include <stdlib.h>
#include <stdbool.h>
#include <sys/queue.h>
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_bit_defs.h"
#include "esp_private/critical_section.h"
#include "esp_log.h"
#include "usb_private.h"
#include "hcd.h"
#include "hub.h"
#include "usbh.h"

#if ENABLE_USB_HUBS
#include "ext_hub.h"
#else
// Forward declaration when hubs are disabled
typedef struct ext_hub_s *ext_hub_handle_t;
#endif // ENABLE_USB_HUBS

/*
Implementation of the HUB driver that supports Root HUB with 2 Root Ports.
*/

#ifdef CONFIG_USB_HOST_HW_BUFFER_BIAS_IN
#define HUB_ROOT_HCD_PORT_FIFO_BIAS                 HCD_PORT_FIFO_BIAS_RX
#elif CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT
#define HUB_ROOT_HCD_PORT_FIFO_BIAS                 HCD_PORT_FIFO_BIAS_PTX
#else   // CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED
#define HUB_ROOT_HCD_PORT_FIFO_BIAS                 HCD_PORT_FIFO_BIAS_BALANCED
#endif

#define PORT_REQ_DISABLE                            0x01
#define PORT_REQ_RECOVER                            0x02
#define PORT_REQ_SUSPEND                            0x04
#define PORT_REQ_RESUME                             0x08

/**
 * @brief Hub driver action flags
 */
typedef enum {
    HUB_DRIVER_ACTION_NONE                = 0,
    HUB_DRIVER_ACTION_ROOT0_EVENT         = BIT0,
    HUB_DRIVER_ACTION_ROOT0_REQ           = BIT1,
#if HCD_NUM_PORTS > 1
    HUB_DRIVER_ACTION_ROOT1_EVENT         = BIT2,
    HUB_DRIVER_ACTION_ROOT1_REQ           = BIT3,
#endif // HCD_NUM_PORTS > 1
#if ENABLE_USB_HUBS
    HUB_DRIVER_ACTION_EXT_HUB             = BIT6,
    HUB_DRIVER_ACTION_EXT_PORT            = BIT7
#endif // ENABLE_USB_HUBS
} hub_flag_action_t;

/**
 * @brief Root port states
 */
typedef enum {
    ROOT_PORT_STATE_NOT_POWERED,    /**< Root port initialized and/or not powered */
    ROOT_PORT_STATE_POWERED,        /**< Root port is powered, device is not connected */
    ROOT_PORT_STATE_DISABLED,       /**< A device is connected but is disabled (i.e., not reset, no SOFs are sent) */
    ROOT_PORT_STATE_ENABLED,        /**< A device is connected, port has been reset, SOFs are sent */
    ROOT_PORT_STATE_SUSPENDED,      /**< Root port is suspended, no SOFs are sent */
} root_port_state_t;

/**
 * @brief Hub device tree node
 *
 * Object type of a node in the USB device tree that is maintained by the Hub driver
 */
struct dev_tree_node_s {
    TAILQ_ENTRY(dev_tree_node_s) tailq_entry;   /**< Entry for the device tree node object tailq */
    unsigned int uid;                           /**< Device's unique ID */
    ext_hub_handle_t parent;                    /**< Device's parent context (NULL for root hub ports, ext_hub_handle_t for external hub ports) */
    uint8_t port_num;                           /**< Device's parent port number */
    hcd_port_handle_t root_port_hdl;            /**< Root port handle this device ultimately hangs off */
};

typedef struct dev_tree_node_s dev_tree_node_t;

typedef struct {
    struct {
        hcd_port_handle_t hdl;
        int index;
    } constant;
    struct {
        unsigned int reqs;
        root_port_state_t state;                    /**< Root port state */
    } dynamic;
} root_hub_port_t;

typedef struct {
    struct {
        union {
            struct {
                hub_flag_action_t actions: 8;       /**< Hub actions */
                uint32_t reserved24: 24;            /**< Reserved */
            };
            uint32_t val;                           /**< Hub flag action value */
        } flags;                                    /**< Hub flags */
    } dynamic;                                      /**< Dynamic members. Require a critical section */

    struct {
        TAILQ_HEAD(tailhead_devs, dev_tree_node_s) dev_nodes_tailq;     /**< Tailq of attached devices */
    } single_thread;                                /**< Single thread members don't require a critical section so long as they are never accessed from multiple threads */

    root_hub_port_t root_hub_ports[HCD_NUM_PORTS];

    struct {
        usb_proc_req_cb_t proc_req_cb;              /**< Process request callback */
        void *proc_req_cb_arg;                      /**< Process request callback argument */
        hub_event_cb_t event_cb;                    /**< Hub Driver event callback */
        void *event_cb_arg;                         /**< Event callback argument */
    } constant;                                     /**< Constant members. Do not change after installation thus do not require a critical section or mutex */
} hub_driver_t;

static hub_driver_t *p_hub_driver_obj = NULL;

const char *HUB_DRIVER_TAG = "HUB";

DEFINE_CRIT_SECTION_LOCK_STATIC(hub_driver_lock);
#define HUB_DRIVER_ENTER_CRITICAL_ISR()       esp_os_enter_critical_isr(&hub_driver_lock)
#define HUB_DRIVER_EXIT_CRITICAL_ISR()        esp_os_exit_critical_isr(&hub_driver_lock)
#define HUB_DRIVER_ENTER_CRITICAL()           esp_os_enter_critical(&hub_driver_lock)
#define HUB_DRIVER_EXIT_CRITICAL()            esp_os_exit_critical(&hub_driver_lock)
#define HUB_DRIVER_ENTER_CRITICAL_SAFE()      esp_os_enter_critical_safe(&hub_driver_lock)
#define HUB_DRIVER_EXIT_CRITICAL_SAFE()       esp_os_exit_critical_safe(&hub_driver_lock)

#define HUB_DRIVER_CHECK(cond, ret_val) ({              \
            if (unlikely(!(cond))) {                    \
                return (ret_val);                       \
            }                                           \
})
#define HUB_DRIVER_CHECK_FROM_CRIT(cond, ret_val) ({    \
            if (unlikely(!(cond))) {                    \
                HUB_DRIVER_EXIT_CRITICAL();             \
                return ret_val;                         \
            }                                           \
})

// ------------------------------------------------- Forward Declare ---------------------------------------------------

/**
 * @brief HCD port callback for a root port
 *
 * - This callback is called from the context of the HCD, so any event handling should be deferred to hub_process()
 * - Under the current HCD implementation, this callback should only be ever be called in an ISR
 * - This callback needs to call proc_req_cb to ensure that hub_process() gets a chance to run
 *
 * @param port_hdl HCD port handle
 * @param port_event HCD port event
 * @param user_arg Callback argument
 * @param in_isr Whether callback is in an ISR context
 * @return Whether a yield is required
 */
static bool root_port_callback(hcd_port_handle_t port_hdl, hcd_port_event_t port_event, void *user_arg, bool in_isr);

// ---------------------- Internal Logic ------------------------

static dev_tree_node_t *dev_tree_node_get_by_uid(unsigned int node_uid)
{
    dev_tree_node_t *dev_tree_node = NULL;
    dev_tree_node_t *dev_tree_iter;
    // Search the device tree nodes list for a device node with the specified parent
    TAILQ_FOREACH(dev_tree_iter, &p_hub_driver_obj->single_thread.dev_nodes_tailq, tailq_entry) {
        if (dev_tree_iter->uid == node_uid) {
            dev_tree_node = dev_tree_iter;
            break;
        }
    }
    return dev_tree_node;
}

/**
 * @brief Creates new device tree node and propagate HUB_EVENT_CONNECTED event
 *
 * @param parent Parent hub handle (NULL for root port devices)
 * @param port_num Root port index (for root devices) or downstream port number (for external hubs)
 * @param speed Enumerated device speed
 * @return esp_err_t
 */
static esp_err_t dev_tree_node_new(ext_hub_handle_t parent, uint8_t port_num, usb_speed_t speed)
{
    esp_err_t ret;

    // Get root port handle of the new device
    hcd_port_handle_t root_port_hdl = NULL;
    if (parent == NULL) {
        assert(port_num < HCD_NUM_PORTS);
        root_port_hdl = p_hub_driver_obj->root_hub_ports[port_num].constant.hdl;
    } else {
#ifdef ENABLE_USB_HUBS
        root_port_hdl = ext_hub_get_root_port(parent);
#else
        abort(); // External hub support is not enabled. parent must be NULL for root port devices
#endif // ENABLE_USB_HUBS
    }
    HUB_DRIVER_CHECK(root_port_hdl != NULL, ESP_ERR_INVALID_ARG);

    // Allocate memory for a new device tree node
    dev_tree_node_t *dev_tree_node = heap_caps_calloc(1, sizeof(dev_tree_node_t), MALLOC_CAP_DEFAULT);
    if (dev_tree_node == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // Assign initial UID based on the current number of registered devices
    int device_num = 0;
    ESP_ERROR_CHECK(usbh_devs_num(&device_num));
    dev_tree_node->uid = device_num + 1;
    // Ensure the UID is unique
    while (usbh_devs_is_uid_in_use(dev_tree_node->uid)) {
        dev_tree_node->uid++;
        assert(dev_tree_node->uid != 0); // No overflow possible
    }

    dev_tree_node->parent = parent;
    dev_tree_node->port_num = port_num;
    dev_tree_node->root_port_hdl = root_port_hdl;

    // Initialize and register a new USBH Device with the assigned UID
    usbh_dev_params_t params = {
        .uid = dev_tree_node->uid,
        .speed = speed,
        .root_port_hdl = root_port_hdl,
        // TODO: IDF-10023 Move parent-child tree management responsibility to Hub Driver
        .parent_dev_hdl = NULL,
        .parent_port_num = port_num,
    };

    ret = usbh_devs_add(&params);
    if (ret != ESP_OK) {
        // Device registration may fail if there are no available HCD channels.
        goto fail;
    }

    TAILQ_INSERT_TAIL(&p_hub_driver_obj->single_thread.dev_nodes_tailq, dev_tree_node, tailq_entry);

    ESP_LOGD(HUB_DRIVER_TAG, "Device tree node (port %d, uid %d): new", dev_tree_node->port_num, dev_tree_node->uid);

    hub_event_data_t event_data = {
        .event = HUB_EVENT_CONNECTED,
        .connected = {
            .uid = dev_tree_node->uid,
        },
    };
    p_hub_driver_obj->constant.event_cb(&event_data, p_hub_driver_obj->constant.event_cb_arg);

    return ret;

fail:
    heap_caps_free(dev_tree_node);
    return ret;
}

static esp_err_t dev_tree_node_reset_completed(ext_hub_handle_t parent, uint8_t port_num)
{
    dev_tree_node_t *dev_tree_node = NULL;
    dev_tree_node_t *dev_tree_iter;
    // Search the device tree nodes list for a device node with the specified parent
    TAILQ_FOREACH(dev_tree_iter, &p_hub_driver_obj->single_thread.dev_nodes_tailq, tailq_entry) {
        if (dev_tree_iter->parent == parent &&
                dev_tree_iter->port_num == port_num) {
            dev_tree_node = dev_tree_iter;
            break;
        }
    }

    if (dev_tree_node == NULL) {
        ESP_LOGE(HUB_DRIVER_TAG, "Reset completed, but device tree node (port %d) not found", port_num);
        return ESP_ERR_NOT_FOUND;
    }

    hub_event_data_t event_data = {
        .event = HUB_EVENT_RESET_COMPLETED,
        .reset_completed = {
            .uid = dev_tree_node->uid,
        },
    };
    p_hub_driver_obj->constant.event_cb(&event_data, p_hub_driver_obj->constant.event_cb_arg);
    return ESP_OK;
}

static esp_err_t dev_tree_node_dev_gone(ext_hub_handle_t parent, uint8_t port_num)
{
    dev_tree_node_t *dev_tree_node = NULL;
    dev_tree_node_t *dev_tree_iter;
    // Search the device tree nodes list for a device node with the specified parent
    TAILQ_FOREACH(dev_tree_iter, &p_hub_driver_obj->single_thread.dev_nodes_tailq, tailq_entry) {
        if (dev_tree_iter->parent == parent &&
                dev_tree_iter->port_num == port_num) {
            dev_tree_node = dev_tree_iter;
            break;
        }
    }

    if (dev_tree_node == NULL) {
        ESP_LOGW(HUB_DRIVER_TAG, "Device tree node (port %d): not found", port_num);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGD(HUB_DRIVER_TAG, "Device tree node (port %d, uid=%d): device gone", port_num, dev_tree_node->uid);

    hub_event_data_t event_data = {
        .event = HUB_EVENT_DISCONNECTED,
        .disconnected = {
            .uid = dev_tree_node->uid,
        },
    };
    p_hub_driver_obj->constant.event_cb(&event_data, p_hub_driver_obj->constant.event_cb_arg);

    return ESP_OK;
}

/**
 * @brief Frees device tree node
 *
 * @return esp_err_t
 */
static esp_err_t dev_tree_node_remove_by_parent(ext_hub_handle_t parent, uint8_t port_num)
{
    dev_tree_node_t *dev_tree_node = NULL;
    dev_tree_node_t *dev_tree_iter;
    // Search the device tree nodes list for a device node with the specified parent
    TAILQ_FOREACH(dev_tree_iter, &p_hub_driver_obj->single_thread.dev_nodes_tailq, tailq_entry) {
        if (dev_tree_iter->parent == parent &&
                dev_tree_iter->port_num == port_num) {
            dev_tree_node = dev_tree_iter;
            break;
        }
    }

    if (dev_tree_node == NULL) {
        ESP_LOGW(HUB_DRIVER_TAG, "Device tree node (port %d): not found", port_num);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGD(HUB_DRIVER_TAG, "Device tree node (port %d, uid %d): freeing", port_num, dev_tree_node->uid);

    TAILQ_REMOVE(&p_hub_driver_obj->single_thread.dev_nodes_tailq, dev_tree_node, tailq_entry);
    heap_caps_free(dev_tree_node);
    return ESP_OK;
}

// ---------------------- Callbacks ------------------------

static bool root_port_callback(hcd_port_handle_t port_hdl, hcd_port_event_t port_event, void *user_arg, bool in_isr)
{
    root_hub_port_t *root_hub_port = (root_hub_port_t *)user_arg;
    HUB_DRIVER_ENTER_CRITICAL_SAFE();
    if (root_hub_port->constant.index == 0) {
        p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_ROOT0_EVENT;
    } else {
#if HCD_NUM_PORTS > 1
        p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_ROOT1_EVENT;
#else
        abort();    // Should never occur
#endif // HCD_NUM_PORTS > 1
    }
    HUB_DRIVER_EXIT_CRITICAL_SAFE();
    assert(in_isr); // Currently, this callback should only ever be called from an ISR context
    return p_hub_driver_obj->constant.proc_req_cb(USB_PROC_REQ_SOURCE_HUB, in_isr, p_hub_driver_obj->constant.proc_req_cb_arg);
}

#ifdef ENABLE_USB_HUBS
static bool ext_hub_callback(bool in_isr, void *user_arg)
{
    HUB_DRIVER_ENTER_CRITICAL_SAFE();
    p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_EXT_HUB;
    HUB_DRIVER_EXIT_CRITICAL_SAFE();
    return p_hub_driver_obj->constant.proc_req_cb(USB_PROC_REQ_SOURCE_HUB, in_isr, p_hub_driver_obj->constant.proc_req_cb_arg);
}

static void ext_port_callback(void *user_arg)
{
    HUB_DRIVER_ENTER_CRITICAL();
    p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_EXT_PORT;
    HUB_DRIVER_EXIT_CRITICAL();
    p_hub_driver_obj->constant.proc_req_cb(USB_PROC_REQ_SOURCE_HUB, false, p_hub_driver_obj->constant.proc_req_cb_arg);
}

static void ext_port_event_callback(ext_port_hdl_t port_hdl, ext_port_event_t event, void *arg)
{
    uint8_t port_num;
    ext_hub_handle_t ext_hub_hdl = (ext_hub_handle_t) ext_port_get_context(port_hdl);
    ESP_ERROR_CHECK(ext_port_get_port_num(port_hdl, &port_num));

    switch (event) {
    case EXT_PORT_CONNECTED:
        // First reset is done by ext_port logic
        usb_speed_t port_speed;

        if (ext_hub_port_get_speed(ext_hub_hdl, port_num, &port_speed) != ESP_OK) {
            goto new_ds_dev_err;
        }

        usb_speed_t parent_speed;
        ESP_ERROR_CHECK(ext_hub_get_speed(ext_hub_hdl, &parent_speed));
        if (parent_speed == USB_SPEED_HIGH) {
            if (port_speed != parent_speed) {
                ESP_LOGE(HUB_DRIVER_TAG, "Connected device is %s, transaction translator (TT) is not supported",
                (const char *[]) {
                    "LS", "FS", "HS"
                }[port_speed]);
                goto new_ds_dev_err;
            }
        }

        if (dev_tree_node_new(ext_hub_hdl, port_num, port_speed) != ESP_OK) {
            ESP_LOGE(HUB_DRIVER_TAG, "Failed to add new downstream device");
            goto new_ds_dev_err;
        }
        break;
new_ds_dev_err:
        ext_hub_port_disable(ext_hub_hdl, port_num);
        break;
    case EXT_PORT_RESET_COMPLETED:
        ESP_ERROR_CHECK(dev_tree_node_reset_completed(ext_hub_hdl, port_num));
        break;
    case EXT_PORT_DISCONNECTED:
        // The node could be freed by now, no need to verify the result here
        dev_tree_node_dev_gone(ext_hub_hdl, port_num);
        break;
    default:
        // Should never occur
        abort();
        break;
    }
}
#endif // ENABLE_USB_HUBS

// ---------------------- Handlers -------------------------
static void root_port_handle_events(root_hub_port_t *root_hub_port)
{
    hcd_port_handle_t root_port_hdl = root_hub_port->constant.hdl;
    hcd_port_event_t port_event = hcd_port_handle_event(root_port_hdl);
    switch (port_event) {
    case HCD_PORT_EVENT_NONE:
        // Nothing to do
        break;
    case HCD_PORT_EVENT_CONNECTION: {
        if (hcd_port_command(root_port_hdl, HCD_PORT_CMD_RESET) != ESP_OK) {
            ESP_LOGE(HUB_DRIVER_TAG, "Root port reset failed");
            goto reset_err;
        }
        ESP_LOGD(HUB_DRIVER_TAG, "Root port reset");
        usb_speed_t speed;
        if (hcd_port_get_speed(root_port_hdl, &speed) != ESP_OK) {
            goto new_dev_err;
        }

        if (dev_tree_node_new(NULL, root_hub_port->constant.index, speed) != ESP_OK) {
            ESP_LOGE(HUB_DRIVER_TAG, "Failed to add new device");
            goto new_dev_err;
        }

        // Change Port state
        HUB_DRIVER_ENTER_CRITICAL();
        root_hub_port->dynamic.state = ROOT_PORT_STATE_ENABLED;
        HUB_DRIVER_EXIT_CRITICAL();
        break;
new_dev_err:
        // We allow this to fail in case a disconnect/port error happens while disabling.
        hcd_port_command(root_port_hdl, HCD_PORT_CMD_DISABLE);
reset_err:
        break;
    }
    case HCD_PORT_EVENT_DISCONNECTION:
    case HCD_PORT_EVENT_ERROR:
    case HCD_PORT_EVENT_OVERCURRENT: {
        bool port_has_device = false;
        HUB_DRIVER_ENTER_CRITICAL();
        switch (root_hub_port->dynamic.state) {
        case ROOT_PORT_STATE_POWERED: // This occurred before enumeration
        case ROOT_PORT_STATE_DISABLED: // This occurred after the device has already been disabled
            // Therefore, there's no device object to clean up, and we can go straight to port recovery
            root_hub_port->dynamic.reqs |= PORT_REQ_RECOVER;
            if (root_hub_port->constant.index == 0) {
                p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_ROOT0_REQ;
            } else {
#if HCD_NUM_PORTS > 1
                p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_ROOT1_REQ;
#else
                abort();    // Should never occur
#endif // HCD_NUM_PORTS > 1
            }
            break;
        case ROOT_PORT_STATE_NOT_POWERED: // The user turned off ports' power. Indicate to USBH that the device is gone
        case ROOT_PORT_STATE_ENABLED: // There is an enabled (active) device. Indicate to USBH that the device is gone
        case ROOT_PORT_STATE_SUSPENDED: // The user called usb host suspend, Indicate to USBH that the device is suspended
            port_has_device = true;
            break;
        default:
            abort();    // Should never occur
            break;
        }
        HUB_DRIVER_EXIT_CRITICAL();

        if (port_has_device) {
            ESP_ERROR_CHECK(dev_tree_node_dev_gone(NULL, root_hub_port->constant.index));
        }

        break;
    }
#ifdef REMOTE_WAKE_HAL_SUPPORTED
    case HCD_PORT_EVENT_REMOTE_WAKEUP:
        // Mark root port as ready to be resumed
        // We allow this to fail in case a disconnect/port error happens while remote wakeup.
        hub_root_mark_resume();
        break;
#endif // REMOTE_WAKE_HAL_SUPPORTED
    default:
        abort();    // Should never occur
        break;
    }
}

static void root_port_req(root_hub_port_t *root_hub_port)
{
    unsigned int port_reqs;
    hcd_port_handle_t root_port_hdl = root_hub_port->constant.hdl;

    HUB_DRIVER_ENTER_CRITICAL();
    port_reqs = root_hub_port->dynamic.reqs;
    root_hub_port->dynamic.reqs = 0;
    HUB_DRIVER_EXIT_CRITICAL();

    if (port_reqs & PORT_REQ_DISABLE) {
        ESP_LOGD(HUB_DRIVER_TAG, "Disabling root port %d",  root_hub_port->constant.index);
        // We allow this to fail in case a disconnect/port error happens while disabling.
        hcd_port_command(root_port_hdl, HCD_PORT_CMD_DISABLE);
    }
    if (port_reqs & PORT_REQ_RECOVER) {
        ESP_LOGD(HUB_DRIVER_TAG, "Recovering root port %d",  root_hub_port->constant.index);
        ESP_ERROR_CHECK(hcd_port_recover(root_port_hdl));

        // In case the port's power was turned off with usb_host_lib_set_root_port_power(false)
        // we will not turn on the power during port recovery
        HUB_DRIVER_ENTER_CRITICAL();
        const root_port_state_t root_state = root_hub_port->dynamic.state;
        HUB_DRIVER_EXIT_CRITICAL();

        if (root_state != ROOT_PORT_STATE_NOT_POWERED) {
            ESP_ERROR_CHECK(hcd_port_command(root_port_hdl, HCD_PORT_CMD_POWER_ON));
            HUB_DRIVER_ENTER_CRITICAL();
            root_hub_port->dynamic.state = ROOT_PORT_STATE_POWERED;
            HUB_DRIVER_EXIT_CRITICAL();
        }
    }
    if (port_reqs & PORT_REQ_SUSPEND) {
        ESP_LOGD(HUB_DRIVER_TAG, "Suspending root port %d",  root_hub_port->constant.index);

        HUB_DRIVER_ENTER_CRITICAL();
        const root_port_state_t root_state = root_hub_port->dynamic.state;
        HUB_DRIVER_EXIT_CRITICAL();

        // In case the port's state changed (disconnection, reconnection) prior to issuing PORT_REQ_SUSPEND request,
        // we will not proceed with the resuming sequence
        if (root_state != ROOT_PORT_STATE_ENABLED) {
            return;
        }

        const esp_err_t port_cmd_err = hcd_port_command(root_port_hdl, HCD_PORT_CMD_SUSPEND);
        switch (port_cmd_err) {
        case ESP_OK:
            // Root port suspended correctly
            break;
        case ESP_ERR_INVALID_RESPONSE:
            // Root port's state machine changed it's state during suspend command execution (port disconnect)
            // The root port is already in recovery state, which was set by dconn event interrupt, no further action needed
            return;
        default:
            // Fatal error occurred: Do ESP_ERROR_CHECK for backtrace
            ESP_ERROR_CHECK(port_cmd_err);
            return;
        }

        // Suspend command executed successfully, finish the suspend procedure
        HUB_DRIVER_ENTER_CRITICAL();
        root_hub_port->dynamic.state = ROOT_PORT_STATE_SUSPENDED;
        HUB_DRIVER_EXIT_CRITICAL();

        // Root port, including all the connected devices were suspended (global suspend)
        // Propagate the suspended event to clients
        usbh_devs_set_pm_actions_all(USBH_DEV_SUSPEND_EVT);
    }
    if (port_reqs & PORT_REQ_RESUME) {
        ESP_LOGD(HUB_DRIVER_TAG, "Resuming root port %d",  root_hub_port->constant.index);

        HUB_DRIVER_ENTER_CRITICAL();
        const root_port_state_t root_state = root_hub_port->dynamic.state;
        HUB_DRIVER_EXIT_CRITICAL();

        // In case the port's state changed (disconnection, reconnection) prior to issuing PORT_REQ_RESUME request,
        // we will not proceed with the resuming sequence
        if (root_state != ROOT_PORT_STATE_SUSPENDED) {
            return;
        }

        const esp_err_t port_cmd_err = hcd_port_command(root_port_hdl, HCD_PORT_CMD_RESUME);
        switch (port_cmd_err) {
        case ESP_OK:
            // Root port resumed correctly
            break;
        case ESP_ERR_INVALID_RESPONSE:
            // Root port's state machine changed it's state during suspend command execution (port disconnect)
            // The root port is already in recovery state, which was set by dconn event interrupt, no further action needed
            return;
        default:
            // Fatal error occurred: Do ESP_ERROR_CHECK for backtrace
            ESP_ERROR_CHECK(port_cmd_err);
            return;
        }

        // Resume command executed successfully, finish the resume procedure
        HUB_DRIVER_ENTER_CRITICAL();
        root_hub_port->dynamic.state = ROOT_PORT_STATE_ENABLED;
        HUB_DRIVER_EXIT_CRITICAL();

        // Root port, including all the connected devices were resumed (global resume)
        // Clear all EPs and propagate the resumed event to clients
        usbh_devs_set_pm_actions_all(USBH_DEV_RESUME | USBH_DEV_RESUME_EVT);    // NOLINT(clang-analyzer-optin.core.EnumCastOutOfRange)
    }
}

static esp_err_t root_port_recycle(root_hub_port_t *root_hub_port)
{
    // Device is free, we can now request its port be recycled
    hcd_port_state_t port_state = hcd_port_get_state(root_hub_port->constant.hdl);
    HUB_DRIVER_ENTER_CRITICAL();
    // How the port is recycled will depend on the port's state
    switch (port_state) {
    case HCD_PORT_STATE_ENABLED:
    case HCD_PORT_STATE_SUSPENDED:
        root_hub_port->dynamic.reqs |= PORT_REQ_DISABLE;
        break;
    case HCD_PORT_STATE_RECOVERY:
        root_hub_port->dynamic.reqs |= PORT_REQ_RECOVER;
        break;
    default:
        abort();    // Should never occur
        break;
    }
    if (root_hub_port->constant.index == 0) {
        p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_ROOT0_REQ;
    } else {
#if HCD_NUM_PORTS > 1
        p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_ROOT1_REQ;
#else
        abort();    // Should never occur
#endif // HCD_NUM_PORTS > 1
    }
    HUB_DRIVER_EXIT_CRITICAL();

    p_hub_driver_obj->constant.proc_req_cb(USB_PROC_REQ_SOURCE_HUB, false, p_hub_driver_obj->constant.proc_req_cb_arg);

    return ESP_OK;
}

static esp_err_t root_port_disable(root_hub_port_t *root_hub_port)
{
    hcd_port_state_t port_state = hcd_port_get_state(root_hub_port->constant.hdl);
    HUB_DRIVER_CHECK(port_state == HCD_PORT_STATE_ENABLED, ESP_ERR_INVALID_STATE);

    HUB_DRIVER_ENTER_CRITICAL();
    root_hub_port->dynamic.reqs |= PORT_REQ_DISABLE;
    if (root_hub_port->constant.index == 0) {
        p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_ROOT0_REQ;
    } else {
#if HCD_NUM_PORTS > 1
        p_hub_driver_obj->dynamic.flags.actions |= HUB_DRIVER_ACTION_ROOT1_REQ;
#else
        abort();    // Should never occur
#endif // HCD_NUM_PORTS > 1
    }
    HUB_DRIVER_EXIT_CRITICAL();

    p_hub_driver_obj->constant.proc_req_cb(USB_PROC_REQ_SOURCE_HUB, false, p_hub_driver_obj->constant.proc_req_cb_arg);
    return ESP_OK;
}

// ---------------------------------------------- Hub Driver Functions -------------------------------------------------

esp_err_t hub_install(hub_config_t *hub_config, void **client_ret)
{
    HUB_DRIVER_CHECK(hub_config != NULL && client_ret != NULL && hub_config->port_map != 0, ESP_ERR_INVALID_ARG);
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj == NULL, ESP_ERR_INVALID_STATE);
    HUB_DRIVER_EXIT_CRITICAL();
    esp_err_t ret = ESP_FAIL;

    // Allocate Hub driver object
    hub_driver_t *hub_driver_obj = heap_caps_calloc(1, sizeof(hub_driver_t), MALLOC_CAP_DEFAULT);
    if (hub_driver_obj == NULL) {
        return ESP_ERR_NO_MEM;
    }

#if ENABLE_USB_HUBS
    // Install External Port driver
    ext_port_driver_config_t ext_port_config = {
        .proc_req_cb = ext_port_callback,
        .event_cb = ext_port_event_callback,
        .hub_request_cb = ext_hub_request,
    };
    ret = ext_port_install(&ext_port_config);
    if (ret != ESP_OK) {
        goto err_ext_port;
    }

    // Install External HUB driver
    ext_hub_config_t ext_hub_config = {
        .proc_req_cb = ext_hub_callback,
        .port_driver = ext_port_get_driver(),
    };
    ret = ext_hub_install(&ext_hub_config);
    if (ret != ESP_OK) {
        ESP_LOGE(HUB_DRIVER_TAG, "Ext hub install error: %s", esp_err_to_name(ret));
        goto err_ext_hub;
    }
    *client_ret = ext_hub_get_client();
#else
    *client_ret = NULL;
#endif // ENABLE_USB_HUBS

    // Initialize HCD port(s)
    root_hub_port_t *root_hub_port = hub_driver_obj->root_hub_ports;
    for (int i = 0; i < HCD_NUM_PORTS; i++, root_hub_port++) {
        if (hub_config->port_map & BIT(i)) {
            const hcd_port_config_t port_config = {
                .callback = root_port_callback,
                .callback_arg = root_hub_port, // To access the root hub port from the callback which is called from an ISR
                .context = root_hub_port,
                .intr_flags = hub_config->intr_flags,
                .fifo_config = hub_config->fifo_config,
            };
            ret = hcd_port_init(i, &port_config, &root_hub_port->constant.hdl);
            if (ret != ESP_OK) {
                ESP_LOGE(HUB_DRIVER_TAG, "HCD Port %d init error: %s", i, esp_err_to_name(ret));
                goto err;
            }
            root_hub_port->constant.index = i;
            root_hub_port->dynamic.state = ROOT_PORT_STATE_NOT_POWERED;
        }
    }

    // Initialize Hub driver object
    hub_driver_obj->constant.proc_req_cb = hub_config->proc_req_cb;
    hub_driver_obj->constant.proc_req_cb_arg = hub_config->proc_req_cb_arg;
    hub_driver_obj->constant.event_cb = hub_config->event_cb;
    hub_driver_obj->constant.event_cb_arg = hub_config->event_cb_arg;
    TAILQ_INIT(&hub_driver_obj->single_thread.dev_nodes_tailq);

    HUB_DRIVER_ENTER_CRITICAL();
    if (p_hub_driver_obj != NULL) {
        HUB_DRIVER_EXIT_CRITICAL();
        ret = ESP_ERR_INVALID_STATE;
        goto err;
    }
    p_hub_driver_obj = hub_driver_obj;
    HUB_DRIVER_EXIT_CRITICAL();

    return ret;

err:
    for (int i = 0; i < HCD_NUM_PORTS; i++) {
        if (hub_driver_obj->root_hub_ports[i].constant.hdl) {
            ESP_ERROR_CHECK(hcd_port_deinit(hub_driver_obj->root_hub_ports[i].constant.hdl));
        }
    }
#if ENABLE_USB_HUBS
    ext_hub_uninstall();
err_ext_hub:
    ext_port_uninstall();
err_ext_port:
#endif // ENABLE_USB_HUBS
    heap_caps_free(hub_driver_obj);
    return ret;
}

esp_err_t hub_uninstall(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    for (int i = 0; i < HCD_NUM_PORTS; i++) {
        if (p_hub_driver_obj->root_hub_ports[i].constant.hdl == NULL) {
            continue;
        }
        HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj->root_hub_ports[i].dynamic.state == ROOT_PORT_STATE_NOT_POWERED,
                                   ESP_ERR_INVALID_STATE);
    }
    hub_driver_t *hub_driver_obj = p_hub_driver_obj;
    p_hub_driver_obj = NULL;
    HUB_DRIVER_EXIT_CRITICAL();

#if ENABLE_USB_HUBS
    ESP_ERROR_CHECK(ext_hub_uninstall());
    ESP_ERROR_CHECK(ext_port_uninstall());
#endif // ENABLE_USB_HUBS

    for (int i = 0; i < HCD_NUM_PORTS; i++) {
        if (hub_driver_obj->root_hub_ports[i].constant.hdl) {
            ESP_ERROR_CHECK(hcd_port_deinit(hub_driver_obj->root_hub_ports[i].constant.hdl));
        }
    }
    // Free Hub driver resources
    heap_caps_free(hub_driver_obj);
    return ESP_OK;
}

esp_err_t hub_root_start(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    for (int i = 0; i < HCD_NUM_PORTS; i++) {
        if (p_hub_driver_obj->root_hub_ports[i].constant.hdl == NULL) {
            continue;
        }
        HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj->root_hub_ports[i].dynamic.state == ROOT_PORT_STATE_NOT_POWERED,
                                   ESP_ERR_INVALID_STATE);
    }
    HUB_DRIVER_EXIT_CRITICAL();
    // Power ON all enabled root ports
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < HCD_NUM_PORTS; i++) {
        hcd_port_handle_t root_port_hdl = p_hub_driver_obj->root_hub_ports[i].constant.hdl;
        if (root_port_hdl == NULL) {
            continue;
        }
        const esp_err_t port_ret = hcd_port_command(root_port_hdl, HCD_PORT_CMD_POWER_ON);
        if (port_ret != ESP_OK) {
            ESP_LOGD(HUB_DRIVER_TAG, "Root port %d power on failed", i);
            ret = port_ret;
            goto fail;
        }
        HUB_DRIVER_ENTER_CRITICAL();
        p_hub_driver_obj->root_hub_ports[i].dynamic.state = ROOT_PORT_STATE_POWERED;
        HUB_DRIVER_EXIT_CRITICAL();
    }
    return ret;
fail:
    // Roll back to a consistent state
    hub_root_stop();
    return ret;
}

esp_err_t hub_root_stop(void)
{
    bool needs_power_off[HCD_NUM_PORTS] = {0};
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    bool all_stopped = true;
    for (int i = 0; i < HCD_NUM_PORTS; i++) {
        if (p_hub_driver_obj->root_hub_ports[i].constant.hdl == NULL) {
            continue;
        }
        if (p_hub_driver_obj->root_hub_ports[i].dynamic.state != ROOT_PORT_STATE_NOT_POWERED) {
            all_stopped = false;
            needs_power_off[i] = true;
            p_hub_driver_obj->root_hub_ports[i].dynamic.state = ROOT_PORT_STATE_NOT_POWERED;
        }
    }
    if (all_stopped) {
        // The HUB was already stopped by usb_host_lib_set_root_port_power(false)
        HUB_DRIVER_EXIT_CRITICAL();
        return ESP_OK;
    }
    HUB_DRIVER_EXIT_CRITICAL();

    // HCD_PORT_CMD_POWER_OFF will only fail if the port is already powered_off or if HW fails to enable clock for the port (if stopping suspended port).
    // This should never happen, so we assert ret == ESP_OK
    for (int i = 0; i < HCD_NUM_PORTS; i++) {
        hcd_port_handle_t root_port_hdl = p_hub_driver_obj->root_hub_ports[i].constant.hdl;
        if (root_port_hdl == NULL || !needs_power_off[i]) {
            continue;
        }
        const esp_err_t ret = hcd_port_command(root_port_hdl, HCD_PORT_CMD_POWER_OFF);
        assert(ret == ESP_OK);
    }
    return ESP_OK;
}

bool hub_root_is_suspended(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, false);

    // TODO: Suspend/Resume API is currently available only for single host configuration.
    // Pick the first root port that is enabled
    root_hub_port_t *root_hub_port = &p_hub_driver_obj->root_hub_ports[0];
#if HCD_NUM_PORTS > 1
    if (!root_hub_port->constant.hdl) {
        root_hub_port = &p_hub_driver_obj->root_hub_ports[1];
    }
#endif
    assert(root_hub_port->constant.hdl);
    HUB_DRIVER_CHECK_FROM_CRIT(root_hub_port->dynamic.state == ROOT_PORT_STATE_SUSPENDED, false);
    HUB_DRIVER_EXIT_CRITICAL();

    return true;
}

esp_err_t hub_root_can_suspend(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);

    // TODO: Suspend/Resume API is currently available only for single host configuration.
    // Pick the first root port that is enabled
    root_hub_port_t *root_hub_port = &p_hub_driver_obj->root_hub_ports[0];
#if HCD_NUM_PORTS > 1
    if (!root_hub_port->constant.hdl) {
        root_hub_port = &p_hub_driver_obj->root_hub_ports[1];
    }
#endif
    assert(root_hub_port->constant.hdl);
    HUB_DRIVER_CHECK_FROM_CRIT(root_hub_port->dynamic.state == ROOT_PORT_STATE_ENABLED, ESP_ERR_NOT_ALLOWED);
    HUB_DRIVER_EXIT_CRITICAL();

    const hcd_port_state_t port_state = hcd_port_get_state(root_hub_port->constant.hdl);
    if (port_state == HCD_PORT_STATE_SUSPENDING) {
        // Root port is already issuing suspend command and is within a SUSPEND_ENTRY_MS timeout,
        // no need to issue the suspend command again
        return ESP_ERR_TIMEOUT;
    }

    return hcd_port_check_all_pipes_idle(root_hub_port->constant.hdl);
}

esp_err_t hub_root_can_resume(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);

    // TODO: Suspend/Resume API is currently available only for single host configuration.
    // Pick the first root port that is enabled
    root_hub_port_t *root_hub_port = &p_hub_driver_obj->root_hub_ports[0];
#if HCD_NUM_PORTS > 1
    if (!root_hub_port->constant.hdl) {
        root_hub_port = &p_hub_driver_obj->root_hub_ports[1];
    }
#endif
    assert(root_hub_port->constant.hdl);
    HUB_DRIVER_CHECK_FROM_CRIT(root_hub_port->dynamic.state == ROOT_PORT_STATE_SUSPENDED, ESP_ERR_NOT_ALLOWED);
    HUB_DRIVER_EXIT_CRITICAL();

    const hcd_port_state_t port_state = hcd_port_get_state(root_hub_port->constant.hdl);
    esp_err_t ret;
    switch (port_state) {
    case HCD_PORT_STATE_RESUMING:
        // Root port is already issuing resume command and is within a RESUME_RECOVERY_MS or a RESUME_HOLD_MS timeout,
        // no need to issue the resume command again
        ret = ESP_ERR_TIMEOUT;
        break;
    case HCD_PORT_STATE_SUSPENDING:
        // Root port is within a SUSPEND_ENTRY_MS timeout
        // We can't resume the port right now, as it would have caused an undefined state
        ret = ESP_ERR_NOT_ALLOWED;
        break;
    default:
        ret = ESP_OK;
        break;
    }

    return ret;
}

esp_err_t hub_root_mark_suspend(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);

    // TODO: Suspend/Resume API is currently available only for single host configuration.
    // Pick the first root port that is enabled
    root_hub_port_t *root_hub_port = &p_hub_driver_obj->root_hub_ports[0];
    hub_flag_action_t action = HUB_DRIVER_ACTION_ROOT0_REQ;
#if HCD_NUM_PORTS > 1
    if (!root_hub_port->constant.hdl) {
        root_hub_port = &p_hub_driver_obj->root_hub_ports[1];
        action = HUB_DRIVER_ACTION_ROOT1_REQ;
    }
#endif
    assert(root_hub_port->constant.hdl);
    HUB_DRIVER_CHECK_FROM_CRIT(root_hub_port->dynamic.state == ROOT_PORT_STATE_ENABLED, ESP_ERR_NOT_ALLOWED);

    // Set port request to suspend
    root_hub_port->dynamic.reqs |= PORT_REQ_SUSPEND;
    p_hub_driver_obj->dynamic.flags.actions |= action;
    HUB_DRIVER_EXIT_CRITICAL();

    // Call processing callback
    p_hub_driver_obj->constant.proc_req_cb(USB_PROC_REQ_SOURCE_HUB, false, p_hub_driver_obj->constant.proc_req_cb_arg);
    return ESP_OK;
}

esp_err_t hub_root_mark_resume(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);

    // TODO: Suspend/Resume API is currently available only for single host configuration.
    // Pick the first root port that is enabled
    root_hub_port_t *root_hub_port = &p_hub_driver_obj->root_hub_ports[0];
    hub_flag_action_t action = HUB_DRIVER_ACTION_ROOT0_REQ;
#if HCD_NUM_PORTS > 1
    if (!root_hub_port->constant.hdl) {
        root_hub_port = &p_hub_driver_obj->root_hub_ports[1];
        action = HUB_DRIVER_ACTION_ROOT1_REQ;
    }
#endif
    assert(root_hub_port->constant.hdl);
    HUB_DRIVER_CHECK_FROM_CRIT(root_hub_port->dynamic.state == ROOT_PORT_STATE_SUSPENDED, ESP_ERR_NOT_ALLOWED);

    // Set port request to resume
    root_hub_port->dynamic.reqs |= PORT_REQ_RESUME;
    p_hub_driver_obj->dynamic.flags.actions |= action;
    HUB_DRIVER_EXIT_CRITICAL();

    // Call processing callback
    p_hub_driver_obj->constant.proc_req_cb(USB_PROC_REQ_SOURCE_HUB, false, p_hub_driver_obj->constant.proc_req_cb_arg);
    return ESP_OK;
}

esp_err_t hub_node_recycle(unsigned int node_uid)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    HUB_DRIVER_EXIT_CRITICAL();
    esp_err_t ret;

    dev_tree_node_t *dev_tree_node = dev_tree_node_get_by_uid(node_uid);
    HUB_DRIVER_CHECK(dev_tree_node != NULL, ESP_ERR_NOT_FOUND);

    if (dev_tree_node->parent == NULL) {
        // Root Hub Ports
        root_hub_port_t *root_hub_port = (root_hub_port_t *)hcd_port_get_context(dev_tree_node->root_port_hdl);
        HUB_DRIVER_CHECK(root_hub_port != NULL, ESP_ERR_NOT_FOUND);
        ret = root_port_recycle(root_hub_port);
    } else {
#if ENABLE_USB_HUBS
        ret = ext_hub_port_recycle(dev_tree_node->parent, dev_tree_node->port_num);
        if (ret != ESP_OK) {
            ESP_LOGE(HUB_DRIVER_TAG, "Ext hub port recycle error: %s", esp_err_to_name(ret));
        }
#else
        ESP_LOGW(HUB_DRIVER_TAG, "Recycling External Port is not available (External Hub support disabled)");
        ret = ESP_ERR_NOT_SUPPORTED;
#endif // ENABLE_USB_HUBS
    }

    if (ret == ESP_OK) {
        ESP_ERROR_CHECK(dev_tree_node_remove_by_parent(dev_tree_node->parent, dev_tree_node->port_num));
    }

    return ret;
}

esp_err_t hub_node_reset(unsigned int node_uid)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    HUB_DRIVER_EXIT_CRITICAL();
    esp_err_t ret;

    dev_tree_node_t *dev_tree_node = dev_tree_node_get_by_uid(node_uid);
    HUB_DRIVER_CHECK(dev_tree_node != NULL, ESP_ERR_NOT_FOUND);

    if (dev_tree_node->parent == NULL) {
        ret = hcd_port_command(dev_tree_node->root_port_hdl, HCD_PORT_CMD_RESET);
        if (ret != ESP_OK) {
            ESP_LOGE(HUB_DRIVER_TAG, "Failed to issue root port reset");
        } else {
            ret = dev_tree_node_reset_completed(NULL, dev_tree_node->port_num);
        }
    } else {
#if ENABLE_USB_HUBS
        ret = ext_hub_port_reset(dev_tree_node->parent, dev_tree_node->port_num);
        if (ret != ESP_OK) {
            ESP_LOGE(HUB_DRIVER_TAG, "Ext hub port reset error: %s", esp_err_to_name(ret));
        }
#else
        ESP_LOGW(HUB_DRIVER_TAG, "Resetting External Port is not available (External Hub support disabled)");
        ret = ESP_ERR_NOT_SUPPORTED;
#endif // ENABLE_USB_HUBS
    }
    return ret;
}

esp_err_t hub_node_active(unsigned int node_uid)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    HUB_DRIVER_EXIT_CRITICAL();
    esp_err_t ret;

    dev_tree_node_t *dev_tree_node = dev_tree_node_get_by_uid(node_uid);
    HUB_DRIVER_CHECK(dev_tree_node != NULL, ESP_ERR_NOT_FOUND);

    if (dev_tree_node->parent == NULL) {
        // Root port no need to be activated
        ret = ESP_OK;
    } else {
#if ENABLE_USB_HUBS
        // External Hub port
        ret = ext_hub_port_active(dev_tree_node->parent, dev_tree_node->port_num);
        if (ret != ESP_OK) {
            ESP_LOGE(HUB_DRIVER_TAG, "Ext hub port activation error: %s", esp_err_to_name(ret));
        }
#else
        ESP_LOGW(HUB_DRIVER_TAG, "Activating External Port is not available (External Hub support disabled)");
        ret = ESP_ERR_NOT_SUPPORTED;
#endif // ENABLE_USB_HUBS
    }
    return ret;
}

esp_err_t hub_node_disable(unsigned int node_uid)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    HUB_DRIVER_EXIT_CRITICAL();
    esp_err_t ret;

    dev_tree_node_t *dev_tree_node = dev_tree_node_get_by_uid(node_uid);
    HUB_DRIVER_CHECK(dev_tree_node != NULL, ESP_ERR_NOT_FOUND);

    if (dev_tree_node->parent == NULL) {
        root_hub_port_t *root_hub_port = (root_hub_port_t *)hcd_port_get_context(dev_tree_node->root_port_hdl);
        HUB_DRIVER_CHECK(root_hub_port != NULL, ESP_ERR_NOT_FOUND);
        ret = root_port_disable(root_hub_port);
    } else {
#if ENABLE_USB_HUBS
        // External Hub port
        ret = ext_hub_port_disable(dev_tree_node->parent, dev_tree_node->port_num);
#else
        ESP_LOGW(HUB_DRIVER_TAG, "Disabling External Port is not available (External Hub support disabled)");
        ret = ESP_ERR_NOT_SUPPORTED;
#endif // ENABLE_USB_HUBS
    }
    return ret;
}

esp_err_t hub_dev_new(uint8_t dev_addr)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    HUB_DRIVER_EXIT_CRITICAL();

    esp_err_t ret;
#if ENABLE_USB_HUBS
    ret = ext_hub_new_dev(dev_addr);
#else
    // Verify the device descriptor and if the bDeviceClass is a Hub class,
    // show the warning message, that Hub support feature is not enabled
    usb_device_handle_t dev_hdl = NULL;
    const usb_device_desc_t *device_desc = NULL;
    // Open device
    if (usbh_devs_open(dev_addr, &dev_hdl) == ESP_OK) {
        // Get Device Descriptor
        if (usbh_dev_get_desc(dev_hdl, &device_desc) == ESP_OK) {
            if (device_desc->bDeviceClass == USB_CLASS_HUB) {
                ESP_LOGW(HUB_DRIVER_TAG, "External Hubs support disabled, Hub device was not initialized");
            }
        }
        // Close device
        ESP_ERROR_CHECK(usbh_dev_close(dev_hdl));
    }
    // Nothing to do, while Hubs support is not enabled
    ret = ESP_ERR_NOT_SUPPORTED;
#endif // ENABLE_USB_HUBS
    return ret;
}

esp_err_t hub_dev_gone(uint8_t dev_addr)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    HUB_DRIVER_EXIT_CRITICAL();

    esp_err_t ret;
#if ENABLE_USB_HUBS
    ret = ext_hub_dev_gone(dev_addr);
#else
    // Nothing to do, while Hubs support is not enabled
    ret = ESP_ERR_NOT_SUPPORTED;
#endif // ENABLE_USB_HUBS
    return ret;
}

#if (ENABLE_USB_HUBS)
esp_err_t hub_notify_all_free(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    HUB_DRIVER_CHECK_FROM_CRIT(p_hub_driver_obj != NULL, ESP_ERR_INVALID_STATE);
    HUB_DRIVER_EXIT_CRITICAL();

    ext_hub_mark_all_free();

    return ESP_OK;
}
#endif // ENABLE_USB_HUBS

esp_err_t hub_process(void)
{
    HUB_DRIVER_ENTER_CRITICAL();
    uint32_t action_flags = p_hub_driver_obj->dynamic.flags.actions;
    p_hub_driver_obj->dynamic.flags.actions = HUB_DRIVER_ACTION_NONE;
    HUB_DRIVER_EXIT_CRITICAL();

    while (action_flags) {
#if ENABLE_USB_HUBS
        if (action_flags & HUB_DRIVER_ACTION_EXT_PORT) {
            ESP_ERROR_CHECK(ext_port_process());
        }
        if (action_flags & HUB_DRIVER_ACTION_EXT_HUB) {
            ESP_ERROR_CHECK(ext_hub_process());
        }
#endif // ENABLE_USB_HUBS
        if (action_flags & HUB_DRIVER_ACTION_ROOT0_EVENT) {
            root_port_handle_events(&p_hub_driver_obj->root_hub_ports[0]);
        }
        if (action_flags & HUB_DRIVER_ACTION_ROOT0_REQ) {
            root_port_req(&p_hub_driver_obj->root_hub_ports[0]);
        }
#if HCD_NUM_PORTS > 1
        if (action_flags & HUB_DRIVER_ACTION_ROOT1_EVENT) {
            root_port_handle_events(&p_hub_driver_obj->root_hub_ports[1]);
        }
        if (action_flags & HUB_DRIVER_ACTION_ROOT1_REQ) {
            root_port_req(&p_hub_driver_obj->root_hub_ports[1]);
        }
#endif // HCD_NUM_PORTS > 1
        HUB_DRIVER_ENTER_CRITICAL();
        action_flags = p_hub_driver_obj->dynamic.flags.actions;
        p_hub_driver_obj->dynamic.flags.actions = HUB_DRIVER_ACTION_NONE;
        HUB_DRIVER_EXIT_CRITICAL();
    }

    return ESP_OK;
}
