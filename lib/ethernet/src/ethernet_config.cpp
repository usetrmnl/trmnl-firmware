#include "ethernet_config.h"
#include "connection_manager.h"

#include "trmnl_log.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "cdc_ecm_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static EventGroupHandle_t eth_event_group;
#define ETH_LINK_UP_BIT BIT0
#define ETH_GOT_IP_BIT  BIT1

static void netif_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data);

static cdc_ecm_params_t cdc_ecm_params = {
    .vid          = 0x0BDA,
    .pids         = {0x8152, 0x8153},
    .event_cb     = netif_event_handler,
    .callback_arg = NULL,
    .if_key       = "cdc_ecm_host",
};

static void netif_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT) {
        switch (event_id) {
            case ETHERNET_EVENT_CONNECTED:
                Log_info("Ethernet connected, waiting for IP...");
                xEventGroupSetBits(eth_event_group, ETH_LINK_UP_BIT);
                break;
            case ETHERNET_EVENT_DISCONNECTED:
                Log_info("Ethernet disconnected");
                xEventGroupClearBits(eth_event_group, ETH_LINK_UP_BIT | ETH_GOT_IP_BIT);
                conn_mgr_mark_lost(CONNECTION_INTERFACE_ETH);
                conn_mgr_update_default_route();
                break;
            case ETHERNET_EVENT_START:
                Log_info("Ethernet started");
                break;
            case ETHERNET_EVENT_STOP:
                Log_info("Ethernet stopped");
                xEventGroupClearBits(eth_event_group, ETH_LINK_UP_BIT | ETH_GOT_IP_BIT);
                conn_mgr_mark_lost(CONNECTION_INTERFACE_ETH);
                conn_mgr_update_default_route();
                break;
            default:
                Log_info("Unhandled Ethernet event: %ld", event_id);
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_ETH_GOT_IP: {
                ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
                Log_info("Got IP: " IPSTR ", mask: " IPSTR ", gw: " IPSTR,
                         IP2STR(&event->ip_info.ip),
                         IP2STR(&event->ip_info.netmask),
                         IP2STR(&event->ip_info.gw));
                xEventGroupSetBits(eth_event_group, ETH_GOT_IP_BIT);
                conn_mgr_mark_ready(CONNECTION_INTERFACE_ETH);
                conn_mgr_update_default_route();
                break;
            }
            case IP_EVENT_ETH_LOST_IP:
                Log_info("Ethernet lost IP");
                xEventGroupClearBits(eth_event_group, ETH_GOT_IP_BIT);
                conn_mgr_mark_lost(CONNECTION_INTERFACE_ETH);
                conn_mgr_update_default_route();
                break;
            default:
                break;
        }
    }
}

bool ethernet_is_connected(void)
{
    esp_netif_t *n = esp_netif_get_handle_from_ifkey("cdc_ecm_host");
    if (!n) return false;
    esp_netif_ip_info_t ip;
    return (esp_netif_get_ip_info(n, &ip) == ESP_OK) && (ip.ip.addr != 0);
}

bool ethernet_get_mac(uint8_t mac[6])
{
    esp_netif_t *n = esp_netif_get_handle_from_ifkey("cdc_ecm_host");
    if (!n) return false;
    return esp_netif_get_mac(n, mac) == ESP_OK;
}

void ethernet_start(void)
{
    Log_info("Starting CDC-ECM Ethernet (RTL8152/RTL8153)");
    eth_event_group = xEventGroupCreate();
    cdc_ecm_init(&cdc_ecm_params);
    Log_info("CDC-ECM driver initialized, waiting for dongle...");
}
