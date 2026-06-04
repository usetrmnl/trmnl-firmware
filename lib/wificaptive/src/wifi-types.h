#pragma once

#include <Arduino.h>
#include <WiFi.h>

struct WifiCredentials
{
    String ssid;
    String pswd;
    bool   is5GHz;
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
    WifiCredentials() : is5GHz(false) {}
    WifiCredentials(String ssid, String pswd, bool is5GHz = false)
        : ssid(ssid), pswd(pswd), is5GHz(is5GHz) {}
};

struct WifiNetwork
{
    String ssid;
    int32_t rssi;
    bool open;
    bool saved;
    bool is5GHz;
    bool enterprise;
    WifiNetwork(String ssid, int32_t rssi, bool open, bool saved, bool is5GHz = false, bool enterprise = false)
        : ssid(ssid), rssi(rssi), open(open), saved(saved), is5GHz(is5GHz), enterprise(enterprise) {}
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
