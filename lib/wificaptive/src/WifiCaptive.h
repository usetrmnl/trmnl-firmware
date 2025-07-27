#ifndef WiFiCaptive_h
#define WiFiCaptive_h

#include <AsyncTCP.h> //https://github.com/me-no-dev/AsyncTCP using the latest dev version from @me-no-dev
#include <DNSServer.h>
#include <ESPAsyncWebServer.h> //https://github.com/me-no-dev/ESPAsyncWebServer using the latest dev version from @me-no-dev
#include <esp_wifi.h>          //Used for mpdu_rx_disable android workaround
#include <AsyncJson.h>
#include "Preferences.h"
#include "WifiCaptivePage.h"
#include <ArduinoJson.h>
#include "wifi-types.h"

#define WIFI_SSID "TRMNL"
#define WIFI_PASSWORD NULL

// Define the DNS interval in milliseconds between processing DNS requests
#define DNS_INTERVAL 60
// Define the maximum number of clients that can connect to the server
#define MAX_CLIENTS 4
// Define the WiFi channel to be used (channel 6 in this case)
#define WIFI_CHANNEL 6
// Define the maximum number of possible saved credentials
#define WIFI_MAX_SAVED_CREDS 5
// Define the maximum number of connection attempts
#define WIFI_CONNECTION_ATTEMPTS 3
// Define max connection timeout
#define CONNECTION_TIMEOUT 15000

#define WIFI_SSID_KEY(i) ("wifi_" + String(i) + "_ssid").c_str()
#define WIFI_PSWD_KEY(i) ("wifi_" + String(i) + "_pswd").c_str()

#define WIFI_LAST_INDEX "wifi_last_index"

class WifiCaptive
{
private:
    DNSServer *_dnsServer;
    AsyncWebServer *_server;
    String _ssid = "";
    String _password = "";
    String _api_server = "";

    std::function<void()> _resetcallback;

    WifiCredentials _savedWifis[WIFI_MAX_SAVED_CREDS];

    void setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP);
    void readWifiCredentials();
    void saveWifiCredentials(const WifiCredentials credentials);
    void saveLastUsedWifiIndex(int index);
    int readLastUsedWifiIndex();
    void saveApiServer(String url);
    bool tryConnectWithRetries(const WifiCredentials creds, int last_used_index);
    std::vector<WifiCredentials> matchNetworks(std::vector<Network> &scanResults, WifiCredentials wifiCredentials[]);
    std::vector<Network> getScannedUniqueNetworks(bool runScan);
    std::vector<Network> combineNetworks(std::vector<Network> &scanResults, WifiCredentials wifiCredentials[]);

public:
    wl_status_t connect(const WifiCredentials credentials);

    /// @brief Starts WiFi configuration portal.
    /// @return True if successfully connected to provided SSID, false otherwise.
    bool startPortal();

    /// @brief Checks if any ssid is saved
    /// @return True if any ssis is saved, false otherwise
    bool isSaved();

    /// @brief Resets all saved credentials
    void resetSettings();

    /// @brief sets the function callback that is triggered when uses performs soft reset
    /// @param func reset callback
    void setResetSettingsCallback(std::function<void()> func);

    /// @brief Connects to the saved SSID with the best signal strength
    /// @return True if successfully connected to saved SSID, false otherwise.
    bool autoConnect();
};

extern WifiCaptive WifiCaptivePortal;

#endif
