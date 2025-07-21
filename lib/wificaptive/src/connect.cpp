#include "connect.h"
#include "WifiCaptive.h"
#include <trmnl_log.h>
#include "WebServer.h"
#include "wifi-helpers.h"
#include "esp_event.h"
#include "esp_wifi.h"

void captureEventData(WiFiEvent_t event, WiFiEventInfo_t info, WifiEventData *eventData)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Log_info("Wifi: Event STA_GOT_IP, IP: %s, Gateway: %s",
                 (IPAddress(info.got_ip.ip_info.ip.addr)).toString(),
                 (IPAddress(info.got_ip.ip_info.gw.addr)).toString());
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

    auto beginResult = WiFi.begin(credentials.ssid.c_str(), credentials.pswd.c_str());
    Log_info("WiFi: begin, starting from status %s", wifiStatusStr(beginResult));

    auto result = waitForConnectResult(CONNECTION_TIMEOUT);

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