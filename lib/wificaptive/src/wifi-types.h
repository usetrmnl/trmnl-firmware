#pragma once

#include <Arduino.h>
#include <WiFi.h>

struct WifiCredentials
{
    String ssid;
    String pswd;
    bool   is5GHz;
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
    WifiNetwork(String ssid, int32_t rssi, bool open, bool saved, bool is5GHz = false)
        : ssid(ssid), rssi(rssi), open(open), saved(saved), is5GHz(is5GHz) {}
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