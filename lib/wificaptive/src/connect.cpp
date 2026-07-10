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
#include <vector>
#include <Preferences.h>

extern void trace(const char *name); // timing checkpoints (bl.cpp), persisted across deep sleep

// T1 — fast-connect cache: after any successful connect we remember the AP's
// channel + BSSID (keyed by SSID) so an interactive tap can skip the ~1-2 s
// WIFI_ALL_CHANNEL_SCAN and associate directly. Stored in the same
// "wificaptive" NVS namespace as the saved creds. Invalidated on first failure
// (router rebooted / AP moved channel) → one slow tap, then fast again.
#define FC_NS "wificaptive"
#define FAST_CONNECT_TIMEOUT 3000 // ms; fall back to full autoConnect after this

String getDeviceHostname() {
    Preferences prefs;
    prefs.begin("data", true);
    String saved = prefs.getString("hostname", "");
    prefs.end();
    if (saved.length() > 0) return saved;

    String mac = WiFi.macAddress(); // "AA:BB:CC:DD:EE:FF"
    String suffix = mac.substring(12, 14) + mac.substring(15, 17);
    suffix.toLowerCase();
    return "trmnl-" + suffix;
}

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

// T2 — DHCP lease cache. Stored in the SAME single fast-connect entry, so it is
// implicitly keyed by fc_bssid (we only ever keep the last AP we joined). Set
// true when a cached lease was applied this wake, so the fetch path can detect
// a stale lease (associated but can't route) and re-run DHCP.
bool g_fast_lease_applied = false;

void saveFastConnectCache(const String &ssid)
{
    if (WiFi.status() != WL_CONNECTED) return;
    uint8_t *bssid = WiFi.BSSID();
    if (!bssid) return;
    Preferences p;
    if (!p.begin(FC_NS, false)) return;
    p.putString("fc_ssid", ssid);
    p.putBytes("fc_bssid", bssid, 6);
    p.putUChar("fc_ch", (uint8_t)WiFi.channel());
    // T2: remember the current IP config so the next tap can skip DHCP.
    p.putUInt("fc_ip", (uint32_t)WiFi.localIP());
    p.putUInt("fc_gw", (uint32_t)WiFi.gatewayIP());
    p.putUInt("fc_mask", (uint32_t)WiFi.subnetMask());
    p.putUInt("fc_dns", (uint32_t)WiFi.dnsIP(0));
    p.end();
}

static bool loadFastConnectCache(const String &ssid, uint8_t bssid[6], uint8_t *channel)
{
    Preferences p;
    if (!p.begin(FC_NS, true)) return false;
    bool ok = false;
    if (p.getString("fc_ssid", "") == ssid && p.getBytesLength("fc_bssid") == 6)
    {
        p.getBytes("fc_bssid", bssid, 6);
        *channel = p.getUChar("fc_ch", 0);
        ok = (*channel > 0);
    }
    p.end();
    return ok;
}

static void invalidateFastConnectCache()
{
    Preferences p;
    if (!p.begin(FC_NS, false)) return;
    p.remove("fc_ssid");
    p.remove("fc_bssid");
    p.remove("fc_ch");
    p.remove("fc_ip");
    p.remove("fc_gw");
    p.remove("fc_mask");
    p.remove("fc_dns");
    p.end();
}

// T2 — apply the cached DHCP lease (static config + stop dhcpc) so association
// skips the DHCP round trip. Returns true if a lease was applied. DHCP users
// only — static-IP users are already handled by configureStaticIP.
static bool applyCachedLease()
{
    Preferences p;
    if (!p.begin(FC_NS, true)) return false;
    uint32_t ip = p.getUInt("fc_ip", 0), gw = p.getUInt("fc_gw", 0),
             mask = p.getUInt("fc_mask", 0), dns = p.getUInt("fc_dns", 0);
    p.end();
    if (!ip || !gw || !mask) return false;

    IPAddress dnsAddr = dns ? IPAddress(dns) : IPAddress(gw);
    if (!WiFi.config(IPAddress(ip), IPAddress(gw), IPAddress(mask), dnsAddr))
        return false;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) esp_netif_dhcpc_stop(netif);
    trace("fastconnect lease");
    return true;
}

// Direct BSSID/channel association for the tap fast path. Returns WL_CONNECTED
// on success (cache refreshed) or WL_CONNECT_FAILED (cache invalidated) so the
// caller can fall back to the full autoConnect() scan. WPA2-personal only —
// enterprise/5 GHz keep the normal path.
wl_status_t fastConnectAndWait(const WifiCredentials &credentials)
{
    uint8_t bssid[6];
    uint8_t channel;
    if (credentials.isEnterprise || credentials.is5GHz) return WL_CONNECT_FAILED;
    if (!loadFastConnectCache(credentials.ssid, bssid, &channel)) return WL_CONNECT_FAILED;

    trace("fastconnect begin");
    if (!credentials.useStaticIP) sntp_servermode_dhcp(1);
    WiFi.mode(WIFI_STA);
    g_fast_lease_applied = false;
    // Static-IP users keep configureStaticIP; DHCP users try the cached lease.
    if (!configureStaticIP(credentials))
        g_fast_lease_applied = applyCachedLease();
    String hostname = getDeviceHostname();
    WiFi.setHostname(hostname.c_str());
    WiFi.setScanMethod(WIFI_FAST_SCAN);
    WiFi.begin(credentials.ssid.c_str(), credentials.pswd.c_str(), channel, bssid);

    wl_status_t res = waitForConnectResult(FAST_CONNECT_TIMEOUT, 10);
    if (res == WL_CONNECTED)
    {
        saveFastConnectCache(credentials.ssid);
        trace("fastconnect ok");
    }
    else
    {
        invalidateFastConnectCache();
        if (g_fast_lease_applied)
        {
            // Undo the static lease + restart dhcpc so the fallback
            // autoConnect() does a normal DHCP bind. WiFi.config(0,0,0)
            // re-enables the DHCP client.
            WiFi.config((uint32_t)0, (uint32_t)0, (uint32_t)0);
            g_fast_lease_applied = false;
        }
        WiFi.disconnect();
        trace("fastconnect fail");
    }
    return res;
}

// T2 — a cached DHCP lease turned out stale: association succeeded but the fetch
// couldn't route (wrong/duplicate IP). Drop the lease, re-enable DHCP, and
// reconnect so this SAME wake can still succeed. Returns true if a fresh lease
// was obtained. WiFi.config(0,0,0) re-enables the DHCP client.
bool recoverFromStaleLease(const WifiCredentials &credentials)
{
    trace("lease recover begin");
    invalidateFastConnectCache();
    g_fast_lease_applied = false;
    WiFi.disconnect();
    WiFi.config((uint32_t)0, (uint32_t)0, (uint32_t)0);
    WiFi.mode(WIFI_STA);
    WiFi.setScanMethod(WIFI_FAST_SCAN);
    WiFi.begin(credentials.ssid.c_str(), credentials.pswd.c_str());
    wl_status_t res = waitForConnectResult(CONNECTION_TIMEOUT, 10);
    if (res == WL_CONNECTED)
    {
        saveFastConnectCache(credentials.ssid);
        trace("lease recover ok");
        return true;
    }
    trace("lease recover fail");
    return false;
}

void captureEventData(WiFiEvent_t event, WiFiEventInfo_t info, WifiEventData *eventData)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        trace("wifi: GOT_IP");
        Log_info("Wifi: Event STA_GOT_IP, IP: %s, Gateway: %s",
                 (IPAddress(info.got_ip.ip_info.ip.addr)).toString().c_str(),
                 (IPAddress(info.got_ip.ip_info.gw.addr)).toString().c_str());
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        trace("wifi: STA_CONNECTED");
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

    // Register WiFi event handlers and remember each registration id.
    std::vector<wifi_event_id_t> handlerIds;
    for (int i = ARDUINO_EVENT_WIFI_READY; i < ARDUINO_EVENT_MAX; i++)
    {
        wifi_event_id_t id = WiFi.onEvent([&eventData](WiFiEvent_t event, WiFiEventInfo_t info)
                     {
                         eventData.eventCount++;

                         captureEventData(event, info, &eventData); },
                     (arduino_event_id_t)i);
        handlerIds.push_back(id);
    }

    auto removeEventHandlers = [&handlerIds]()
    {
        for (wifi_event_id_t id : handlerIds)
        {
            WiFi.removeEvent(id);
        }
    };

    if (!credentials.useStaticIP)
    {
        sntp_servermode_dhcp(1);
    }

    // always start with a clean state - disable any previous configuration
    disableWpa2Enterprise();

    // Pick the strongest AP when an SSID is broadcast by multiple access points
    // (mesh/roaming networks). The arduino-esp32 default is WIFI_FAST_SCAN, which
    // associates with the FIRST AP found for the SSID regardless of signal strength,
    // so the device can latch onto a weak/distant AP. WIFI_ALL_CHANNEL_SCAN scans
    // every channel first, then WIFI_CONNECT_AP_BY_SIGNAL connects to the AP with the
    // highest RSSI. Trade-off: a full-channel scan adds ~1-2s to each connect, which is
    // an acceptable cost for reliably joining the nearest AP. See issue #285.
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

    wl_status_t beginResult;

    if (credentials.isEnterprise)
    {
        Log_info("WiFi: Connecting to WPA2 Enterprise network: %s", credentials.ssid.c_str());

        if (credentials.identity.length() == 0)
        {
            Log_error("WiFi: Enterprise mode requires an identity");
            // clean up event handlers
            removeEventHandlers();
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
            removeEventHandlers();
            return {WL_CONNECT_FAILED, eventData};
        }

        // Configure static IP if specified (must be before WiFi.begin)
        configureStaticIP(credentials);

        String hostname = getDeviceHostname();
        WiFi.setHostname(hostname.c_str());
        Log_info("WiFi: hostname set to %s", hostname.c_str());

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

        String hostname = getDeviceHostname();
        WiFi.setHostname(hostname.c_str());
        Log_info("WiFi: hostname set to %s", hostname.c_str());

        beginResult = WiFi.begin(credentials.ssid.c_str(), credentials.pswd.c_str());
        Log_info("WiFi: begin (WPA2-Personal), starting from status %s", wifiStatusStr(beginResult));
    }

    auto result = waitForConnectResult(CONNECTION_TIMEOUT);

    // Refresh the T1 fast-connect cache so the next tap can skip the full scan.
    if (result == WL_CONNECTED && !credentials.isEnterprise && !credentials.is5GHz)
    {
        saveFastConnectCache(credentials.ssid);
    }

    // if connection failed and we were using enterprise, clean up
    if (result != WL_CONNECTED && credentials.isEnterprise)
    {
        Log_info("WiFi: Enterprise connection failed, cleaning up WPA2 Enterprise state");
        disableWpa2Enterprise();
    }

    // Clean up Arduino event handlers
    removeEventHandlers();

    return {result, eventData};
}

wl_status_t waitForConnectResult(uint32_t timeout, uint32_t pollMs)
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
        delay(pollMs);
    }

    Log_info("WiFi: connect timed out after %d ms", timeout);
    return status;
}
