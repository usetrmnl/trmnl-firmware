/**
 * @file ethernet_config.h
 * @brief Ethernet-over-USB (CDC-ECM host) bootstrap API.
 *
 * Supports RTL8152/RTL8153 USB dongles (VID 0x0BDA, PID 0x8152/0x8153).
 * Call ethernet_start() once at boot before waiting for connectivity.
 */
#pragma once
#include <stdint.h>
#include "esp_event.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize CDC-ECM driver and start listening for USB dongle.
 *
 * Non-blocking. Connection state is reported via connection_manager event bits.
 */
void ethernet_start(void);

/**
 * @brief Query whether the CDC-ECM interface currently has a non-zero IPv4 address.
 *
 * @return true if interface exists and has a valid IPv4 address, false otherwise.
 */
bool ethernet_is_connected(void);

/**
 * @brief Internal netif event handler for the CDC-ECM Ethernet interface.
 *
 * Registered as cdc_ecm_params.event_cb. Do not call directly.
 *
 * ETH_EVENT:
 *   ETHERNET_EVENT_CONNECTED    — physical link up; sets ETH_LINK_UP_BIT
 *   ETHERNET_EVENT_DISCONNECTED — link lost; clears bits, marks ETH lost,
 *                                 triggers default-route update
 *   ETHERNET_EVENT_STOP         — driver stopped; same as disconnected
 *
 * IP_EVENT:
 *   IP_EVENT_ETH_GOT_IP  — DHCP succeeded; sets ETH_GOT_IP_BIT,
 *                          marks ETH ready, triggers default-route update
 *   IP_EVENT_ETH_LOST_IP — IP lease lost; clears ETH_GOT_IP_BIT,
 *                          marks ETH lost, triggers default-route update
 */
static void netif_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data);

/**
 * @brief Get the MAC address of the CDC-ECM Ethernet interface.
 *
 * @param[out] mac  6-byte buffer to receive the MAC address.
 * @return true if the netif exists and MAC was retrieved, false otherwise.
 */
bool ethernet_get_mac(uint8_t mac[6]);

#ifdef __cplusplus
}
#endif
