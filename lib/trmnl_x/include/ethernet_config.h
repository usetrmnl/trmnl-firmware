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
 * @brief Get the MAC address of the CDC-ECM Ethernet interface.
 *
 * @param[out] mac  6-byte buffer to receive the MAC address.
 * @return true if the netif exists and MAC was retrieved, false otherwise.
 */
bool ethernet_get_mac(uint8_t mac[6]);

/**
 * @brief Query whether the CDC-ECM physical link is up (independent of DHCP).
 *
 * @return true if ETHERNET_EVENT_CONNECTED has fired and link is still up.
 */
bool ethernet_is_link_up(void);

#ifdef __cplusplus
}
#endif
