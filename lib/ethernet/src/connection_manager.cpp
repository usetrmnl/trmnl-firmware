#include "connection_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "trmnl_log.h"
#include "esp_netif.h"

static EventGroupHandle_t connection_group;

void conn_mgr_init(void)
{
    if (!connection_group) connection_group = xEventGroupCreate();
}

void conn_mgr_mark_ready(connection_interface_t interface)
{
    if (connection_group == NULL) return;
    if (interface == CONNECTION_INTERFACE_WIFI) {
        xEventGroupSetBits(connection_group, WIFI_READY_BIT);
    } else {
        xEventGroupSetBits(connection_group, ETH_READY_BIT);
    }
}

void conn_mgr_mark_lost(connection_interface_t interface)
{
    if (connection_group == NULL) return;
    if (interface == CONNECTION_INTERFACE_WIFI) {
        xEventGroupClearBits(connection_group, WIFI_READY_BIT);
    } else {
        xEventGroupClearBits(connection_group, ETH_READY_BIT);
    }
}

EventBits_t conn_mgr_wait_any(TickType_t ticks)
{
    if (connection_group == NULL) return 0;
    return xEventGroupWaitBits(connection_group, WIFI_READY_BIT | ETH_READY_BIT, false, false, ticks);
}

bool conn_mgr_is_ready(void)
{
    if (connection_group == NULL) return false;
    EventBits_t state = xEventGroupGetBits(connection_group);
    return (state & (WIFI_READY_BIT | ETH_READY_BIT)) != 0;
}

int conn_mgr_get_bits(void)
{
    if (connection_group == NULL) return 0;
    return (int)xEventGroupGetBits(connection_group);
}

bool conn_mgr_is_eth(void)
{
    if (connection_group == NULL) return false;
    return (xEventGroupGetBits(connection_group) & ETH_READY_BIT) != 0;
}

bool conn_mgr_is_wifi(void)
{
    if (connection_group == NULL) return false;
    return (xEventGroupGetBits(connection_group) & WIFI_READY_BIT) != 0;
}

void conn_mgr_update_default_route(void)
{
    if (connection_group == NULL) return;
    esp_netif_t *wifi = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_t *eth  = esp_netif_get_handle_from_ifkey("cdc_ecm_host");
    EventBits_t state = xEventGroupGetBits(connection_group);

    if ((state & ETH_READY_BIT) && eth) {
        esp_netif_set_default_netif(eth);
        Log_info("Default route -> Ethernet");
    } else if ((state & WIFI_READY_BIT) && wifi) {
        esp_netif_set_default_netif(wifi);
        Log_info("Default route -> Wi-Fi");
    } else {
        Log_error("No default route (no ready interfaces)");
    }
}
