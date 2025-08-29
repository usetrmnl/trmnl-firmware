#pragma once

#include <Arduino.h>
#include <WiFiGeneric.h>
#include <WiFiType.h>
#include <WiFi.h>

struct WifiCredentials
{
    String ssid;
    String pswd;
};

struct Network
{
    String ssid;
    int32_t rssi;
    bool open;
    bool saved;
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