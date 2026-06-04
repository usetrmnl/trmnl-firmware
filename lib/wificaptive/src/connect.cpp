#include "connect.h"
#include "WifiCaptive.h"
#include <trmnl_log.h>
#include "WebServer.h"
#include "wifi-helpers.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_sntp.h"

/**
 * @brief Configure static IP with smart defaults
 *
 * If gateway/subnet/dns are not specified, derives sensible defaults:
 * - Gateway: x.x.x.1 (same network as static IP)
 * - Subnet: 255.255.255.0
 * - DNS1: 8.8.8.8 (Google DNS)
 * - DNS2: 8.8.8.8 (Google DNS)
 *
 * @param credentials WiFi credentials containing static IP settings
 * @return true if static IP was configured, false if using DHCP
 */
static bool configureStaticIP(const WifiCredentials &credentials)
{
    if (!credentials.useStaticIP || credentials.staticIP.length() == 0)
    {
        Log_info("WiFi: Using DHCP");
        return false;
    }

    IPAddress ip, gateway, subnet, dns1, dns2;

    // Parse static IP (required)
    if (!ip.fromString(credentials.staticIP))
    {
        Log_error("WiFi: Invalid static IP address: %s", credentials.staticIP.c_str());
        return false;
    }

    // Parse or derive gateway (default: x.x.x.1)
    if (credentials.gateway.length() > 0 && gateway.fromString(credentials.gateway))
    {
        Log_info("WiFi: Using specified gateway: %s", credentials.gateway.c_str());
    }
    else
    {
        gateway = IPAddress(ip[0], ip[1], ip[2], 1);
        Log_info("WiFi: Using default gateway: %s", gateway.toString().c_str());
    }

    // Parse or use default subnet (default: 255.255.255.0)
    if (credentials.subnet.length() > 0 && subnet.fromString(credentials.subnet))
    {
        Log_info("WiFi: Using specified subnet: %s", credentials.subnet.c_str());
    }
    else
    {
        subnet = IPAddress(255, 255, 255, 0);
        Log_info("WiFi: Using default subnet: %s", subnet.toString().c_str());
    }

    // Parse or use default DNS1 (default: 8.8.8.8 Google DNS)
    if (credentials.dns1.length() > 0 && dns1.fromString(credentials.dns1))
    {
        Log_info("WiFi: Using specified DNS1: %s", credentials.dns1.c_str());
    }
    else
    {
        dns1 = IPAddress(8, 8, 8, 8);
        Log_info("WiFi: Using default DNS1: %s", dns1.toString().c_str());
    }

    // Parse or use default DNS2 (default: 8.8.8.8 Google DNS)
    if (credentials.dns2.length() > 0 && dns2.fromString(credentials.dns2))
    {
        Log_info("WiFi: Using specified DNS2: %s", credentials.dns2.c_str());
    }
    else
    {
        dns2 = IPAddress(8, 8, 8, 8);
        Log_info("WiFi: Using default DNS2: %s", dns2.toString().c_str());
    }

    // Apply static IP configuration
    Log_info("WiFi: Configuring static IP: %s", ip.toString().c_str());
    if (!WiFi.config(ip, gateway, subnet, dns1, dns2))
    {
        Log_error("WiFi: Failed to configure static IP");
        return false;
    }

    // Explicitly stop DHCP client to prevent any DHCP requests on boot/wake
    // This saves battery by avoiding DHCP negotiation delays
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif != nullptr)
    {
        esp_err_t err = esp_netif_dhcpc_stop(netif);
        if (err == ESP_OK)
        {
            Log_info("WiFi: DHCP client disabled (static IP mode)");
        }
        else if (err == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
        {
            Log_info("WiFi: DHCP client already disabled");
        }
        else
        {
            Log_error("WiFi: Failed to disable DHCP client: %d", err);
        }
    }

    Log_info("WiFi: Static IP configured - IP: %s, GW: %s, Subnet: %s, DNS1: %s, DNS2: %s",
             ip.toString().c_str(),
             gateway.toString().c_str(),
             subnet.toString().c_str(),
             dns1.toString().c_str(),
             dns2.toString().c_str());
    return true;
}

void disableWpa2Enterprise()
{
    Log_info("WiFi: Disabling WPA2 Enterprise");
    esp_wifi_sta_wpa2_ent_disable();

    esp_wifi_sta_wpa2_ent_clear_identity();
    esp_wifi_sta_wpa2_ent_clear_username();
    esp_wifi_sta_wpa2_ent_clear_password();
    esp_wifi_sta_wpa2_ent_clear_ca_cert();
}

void captureEventData(WiFiEvent_t event, WiFiEventInfo_t info, WifiEventData *eventData)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Log_info("Wifi: Event STA_GOT_IP, IP: %s, Gateway: %s",
                 (IPAddress(info.got_ip.ip_info.ip.addr)).toString().c_str(),
                 (IPAddress(info.got_ip.ip_info.gw.addr)).toString().c_str());
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Log_info("Wifi: Event STA_CONNECTED SSID: %s, BSSID: %s, channel: %d, authmode: %d",
                 String((char *)info.wifi_sta_connected.ssid).c_str(),
                 WiFi.BSSIDstr().c_str(),
                 info.wifi_sta_connected.channel,
                 info.wifi_sta_connected.authmode);
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        eventData->disconnected = true;
        eventData->disconnectReason = (wifi_err_reason_t)info.wifi_sta_disconnected.reason;
        Log_info("Wifi: Event STA_DISCONNECTED, reason: %s", WiFi.disconnectReasonName((wifi_err_reason_t)info.wifi_sta_disconnected.reason));
        break;
    default:
        Log_info("Wifi: Event (other): %s", WiFi.eventName((arduino_event_id_t)event));
        break;
    }
}

WifiConnectionResult initiateConnectionAndWaitForOutcome(const WifiCredentials credentials)
{
    WifiEventData eventData;

    for (int i = ARDUINO_EVENT_WIFI_READY; i < ARDUINO_EVENT_MAX; i++)
    {
        WiFi.onEvent([i, &eventData](WiFiEvent_t event, WiFiEventInfo_t info)
                     {
                         eventData.eventCount++;

                         captureEventData(event, info, &eventData); },
                     (arduino_event_id_t)i);
    }

    if (!credentials.useStaticIP)
    {
        sntp_servermode_dhcp(1);
    }

    // always start with a clean state - disable any previous configuration
    disableWpa2Enterprise();

    wl_status_t beginResult;

    if (credentials.isEnterprise)
    {
        Log_info("WiFi: Connecting to WPA2 Enterprise network: %s", credentials.ssid.c_str());

        if (credentials.identity.length() == 0)
        {
            Log_error("WiFi: Enterprise mode requires an identity");
            // clean up event handlers
            for (int i = ARDUINO_EVENT_WIFI_READY; i < ARDUINO_EVENT_MAX; i++)
            {
                WiFi.removeEvent(i);
            }
            return {WL_CONNECT_FAILED, eventData};
        }

        // configure WPA2 Enterprise
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        delay(100);

        esp_err_t err = esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)credentials.identity.c_str(), credentials.identity.length());
        if (err != ESP_OK)
        {
            Log_error("WiFi: Failed to set identity, error: %d", err);
        }
        else
        {
            Log_info("WiFi: Set identity: %s", credentials.identity.c_str());
        }

        if (credentials.username.length() > 0)
        {
            err = esp_wifi_sta_wpa2_ent_set_username((uint8_t *)credentials.username.c_str(), credentials.username.length());
            if (err != ESP_OK)
            {
                Log_error("WiFi: Failed to set username, error: %d", err);
            }
            else
            {
                Log_info("WiFi: Set username: %s", credentials.username.c_str());
            }
        }
        else
        {
            err = esp_wifi_sta_wpa2_ent_set_username((uint8_t *)credentials.identity.c_str(), credentials.identity.length());
            if (err != ESP_OK)
            {
                Log_error("WiFi: Failed to set username (from identity), error: %d", err);
            }
            else
            {
                Log_info("WiFi: Set username (from identity): %s", credentials.identity.c_str());
            }
        }

        if (credentials.pswd.length() > 0)
        {
            err = esp_wifi_sta_wpa2_ent_set_password((uint8_t *)credentials.pswd.c_str(), credentials.pswd.length());
            if (err != ESP_OK)
            {
                Log_error("WiFi: Failed to set password, error: %d", err);
            }
            else
            {
                Log_info("WiFi: Password set");
            }
        }

        esp_wifi_sta_wpa2_ent_set_ca_cert(NULL, 0);
        Log_info("WiFi: CA certificate verification disabled");

        err = esp_wifi_sta_wpa2_ent_enable();
        if (err != ESP_OK)
        {
            Log_error("WiFi: Failed to enable WPA2 Enterprise, error: %d", err);
            disableWpa2Enterprise();
            // clean up event handlers
            for (int i = ARDUINO_EVENT_WIFI_READY; i < ARDUINO_EVENT_MAX; i++)
            {
                WiFi.removeEvent(i);
            }
            return {WL_CONNECT_FAILED, eventData};
        }

        // Configure static IP if specified (must be before WiFi.begin)
        configureStaticIP(credentials);

        WiFi.begin(credentials.ssid.c_str());

        beginResult = WiFi.status();
        Log_info("WiFi: WPA2 Enterprise configured, starting from status %s", wifiStatusStr(beginResult));
    }
    else
    {
        // regular connection
        WiFi.mode(WIFI_STA);

        // Configure static IP if specified (must be before WiFi.begin)
        configureStaticIP(credentials);

        beginResult = WiFi.begin(credentials.ssid.c_str(), credentials.pswd.c_str());
        Log_info("WiFi: begin (WPA2-Personal), starting from status %s", wifiStatusStr(beginResult));
    }

    auto result = waitForConnectResult(CONNECTION_TIMEOUT);

    // if connection failed and we were using enterprise, clean up
    if (result != WL_CONNECTED && credentials.isEnterprise)
    {
        Log_info("WiFi: Enterprise connection failed, cleaning up WPA2 Enterprise state");
        disableWpa2Enterprise();
    }

    // Clean up Arduino event handlers
    for (int i = ARDUINO_EVENT_WIFI_READY; i < ARDUINO_EVENT_MAX; i++)
    {
        WiFi.removeEvent(i);
    }

    return {result, eventData};
}

wl_status_t waitForConnectResult(uint32_t timeout)
{

    unsigned long timeoutmillis = millis() + timeout;
    wl_status_t status = WiFi.status();

    while (millis() < timeoutmillis)
    {
        wl_status_t newStatus = WiFi.status();
        if (newStatus != status)
        {
            Log_info("WiFi: status changed from %s to %s", wifiStatusStr(status), wifiStatusStr(newStatus));
        }
        status = newStatus;
        // @todo detect additional states, connect happens, then dhcp then get ip, there is some delay here, make sure not to timeout if waiting on IP
        if (status == WL_CONNECTED || status == WL_CONNECT_FAILED)
        {
            return status;
        }
        delay(100);
    }

    Log_info("WiFi: connect timed out after %d ms", timeout);
    return status;
}
