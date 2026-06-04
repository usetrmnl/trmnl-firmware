/**
 * @file connection_manager.h
 * @brief Arbitrates default network route between Wi-Fi and Ethernet.
 *
 * Priority: Ethernet > Wi-Fi > none.
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include <stdbool.h>

#define WIFI_READY_BIT  BIT0  // 0b001
#define ETH_READY_BIT   BIT1  // 0b010

typedef enum {
    CONNECTION_INTERFACE_WIFI = 0,
    CONNECTION_INTERFACE_ETH  = 1,
} connection_interface_t;

/**
 * @brief Initialize the connection manager.
 *
 * Creates the event group if it does not exist.
 */
void conn_mgr_init(void);

/**
 * @brief Mark the given interface as ready (sets its event bit).
 *
 * @param interface CONNECTION_INTERFACE_WIFI or CONNECTION_INTERFACE_ETH
 */
void conn_mgr_mark_ready(connection_interface_t interface);

/**
 * @brief Mark the given interface as lost (clears its event bit).
 *
 * @param interface CONNECTION_INTERFACE_WIFI or CONNECTION_INTERFACE_ETH
 */
void conn_mgr_mark_lost(connection_interface_t interface);

/**
 * @brief Wait until either Wi-Fi or Ethernet connection is ready.
 *
 * @param ticks Timeout in FreeRTOS ticks (use portMAX_DELAY to wait forever).
 * @return Current event bits state.
 */
EventBits_t conn_mgr_wait_any(TickType_t ticks);

/**
 * @brief Check if any interface is currently ready.
 *
 * @return true if at least one interface is ready, false otherwise.
 */
bool conn_mgr_is_ready(void);

/**
 * @brief Update the system default route based on current connection state.
 *
 * Priority: Ethernet > Wi-Fi > none.
 */
void conn_mgr_update_default_route(void);

/**
 * @brief Get bitmask of currently ready interfaces.
 *
 * @return 0=none, WIFI_READY_BIT=WiFi only, ETH_READY_BIT=ETH only, both bits=both
 */
int conn_mgr_get_bits(void);

/**
 * @brief Check if Ethernet is the active interface.
 *
 * @return true if ETH_READY_BIT is set.
 */
bool conn_mgr_is_eth(void);

/**
 * @brief Check if Wi-Fi is the active interface.
 *
 * @return true if WIFI_READY_BIT is set.
 */
bool conn_mgr_is_wifi(void);

#ifdef __cplusplus
}
#endif
