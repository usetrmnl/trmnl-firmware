#pragma once

#include <Arduino.h>
#include <WiFiGeneric.h>
#include <WiFiType.h>
#include <WiFi.h>

struct WifiCredentials
{
    String ssid;
    String pswd;
    // WPA2 Enterprise fields
    bool isEnterprise = false;
    String username;
    String identity;
    // Static IP fields (optional - leave empty for DHCP)
    bool useStaticIP = false;
    String staticIP;    // e.g., "192.168.1.100"
    String gateway;     // e.g., "192.168.1.1" (optional - defaults to x.x.x.1)
    String subnet;      // e.g., "255.255.255.0" (optional - defaults to 255.255.255.0)
    String dns1;        // optional - defaults to gateway
    String dns2;        // optional - defaults to 8.8.8.8
};

struct Network
{
    String ssid;
    int32_t rssi;
    bool open;
    bool saved;
    bool enterprise;
};

struct WifiEventData
{
    bool disconnected = false;
    wifi_err_reason_t disconnectReason = wifi_err_reason_t::WIFI_REASON_UNSPECIFIED;
    int eventCount = 0;
};

struct WifiConnectionResult
{
    wl_status_t status;
    WifiEventData eventData;
};
