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
    // Fast-connect hint: populated after each successful connection
    String bssid;        // "AA:BB:CC:DD:EE:FF"; empty if not yet known
    uint8_t channel = 0; // 0 if not yet known
    // Epoch (UTC seconds) of the last full channel scan for this network; 0 = never scanned.
    // Used to force a periodic roam scan even when the cached BSSID/channel still connects fine.
    uint32_t lastFullScanEpoch = 0;
    // Cached DHCP lease from the last successful connection (0 = none). Applied as a static
    // config on the fast-connect path so association skips the DHCP round trip; a stale lease
    // is recovered in-wake via WifiCaptive::recoverFromStaleLease().
    uint32_t leaseIP = 0;
    uint32_t leaseGW = 0;
    uint32_t leaseMask = 0;
    uint32_t leaseDNS = 0;
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
    bool usedFullScan;     // true if this attempt performed a full channel scan (not fast connect)
    bool fastLeaseApplied; // connected using the cached DHCP lease (DHCP round trip skipped)
};
