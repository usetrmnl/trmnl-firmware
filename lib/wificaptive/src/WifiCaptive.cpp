#include "WifiCaptive.h"
#include <WiFi.h>
#include <trmnl_log.h>
#include "WebServer.h"
#include "wifi-helpers.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "connect.h"

void WifiCaptive::setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP)
{
    dnsServer.setTTL(3600);
    dnsServer.start(53, "*", localIP);
}

bool WifiCaptive::startPortal()
{
    _dnsServer = new DNSServer();
    _server = new AsyncWebServer(80);

    // Set the WiFi mode to access point and station
    WiFi.mode(WIFI_MODE_AP);

    // Define the subnet mask for the WiFi network
    const IPAddress subnetMask(255, 255, 255, 0);
    const IPAddress localIP(4, 3, 2, 1);
    const IPAddress gatewayIP(4, 3, 2, 1);

    WiFi.disconnect();
    delay(50);

    // Configure the soft access point with a specific IP and subnet mask
    WiFi.softAPConfig(localIP, gatewayIP, subnetMask);
    delay(50);

    // Start the soft access point with the given ssid, password, channel, max number of clients
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, 0, MAX_CLIENTS);
    delay(50);

    // Disable AMPDU RX on the ESP32 WiFi to fix a bug on Android
    esp_wifi_stop();
    esp_wifi_deinit();
    wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
    my_config.ampdu_rx_enable = false;
    esp_wifi_init(&my_config);
    esp_wifi_start();
    vTaskDelay(100 / portTICK_PERIOD_MS); // Add a small delay

    // configure DSN and WEB server
    setUpDNSServer(*_dnsServer, localIP);

    WifiOperationCallbacks callbacks = {
        .resetSettings = [this]()
        {
            resetSettings();
            if (_resetcallback != NULL)
            {
                _resetcallback(); // @CALLBACK
            } },
        .setConnectionCredentials = [this](const WifiCredentials credentials, const String api_server)
        {
            _ssid = credentials.ssid;
            _password = credentials.pswd;
            _api_server = api_server; },
        .getAnnotatedNetworks = [this](bool runScan)
        {
            // Warning: DO NOT USE true on this function in an async context!
            std::vector<Network> uniqueNetworks = getScannedUniqueNetworks(false);
            std::vector<Network> combinedNetworks = combineNetworks(uniqueNetworks, _savedWifis);
            return combinedNetworks; }};

    setUpWebserver(*_server, localIP, callbacks);

    // begin serving
    _server->begin();

    // start async network scan
    WiFi.scanNetworks(true);

    readWifiCredentials();

    bool succesfullyConnected = false;
    // wait until SSID is provided
    while (1)
    {
        _dnsServer->processNextRequest();

        if (_ssid == "")
        {
            delay(DNS_INTERVAL);
        }
        else
        {
            WifiCredentials credentials = {_ssid, _password};
            bool res = connect(credentials) == WL_CONNECTED;
            if (res)
            {
                saveWifiCredentials(credentials);
                saveApiServer(_api_server);
                succesfullyConnected = true;
                break;
            }
            else
            {
                _ssid = "";
                _password = "";

                WiFi.disconnect();
                WiFi.enableSTA(false);
                break;
            }
        }
    }

    // SSID provided, stop server
    WiFi.scanDelete();
    WiFi.softAPdisconnect(true);
    delay(1000);

    auto status = WiFi.status();
    if (status != WL_CONNECTED)
    {
        Log_info("Not connected after AP disconnect");
        WiFi.mode(WIFI_STA);

        auto result = initiateConnectionAndWaitForOutcome({_ssid, _password});
        status = result.status;
    }

    // stop dsn
    _dnsServer->stop();
    delete _dnsServer;
    _dnsServer = nullptr;

    // stop server
    _server->end();
    delete _server;
    _server = nullptr;

    return succesfullyConnected;
}

void WifiCaptive::resetSettings()
{
    Preferences preferences;
    preferences.begin("wificaptive", false);
    preferences.remove("api_url");
    preferences.remove(WIFI_LAST_INDEX);
    for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
    {
        preferences.remove(WIFI_SSID_KEY(i));
        preferences.remove(WIFI_PSWD_KEY(i));
    }
    preferences.end();

    for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
    {
        _savedWifis[i] = {"", ""};
    }

    WiFi.disconnect(true, true);
    WiFi.eraseAP();
}

wl_status_t WifiCaptive::connect(const WifiCredentials credentials)
{
    wl_status_t connRes = WL_NO_SSID_AVAIL;

    if (credentials.ssid != "")
    {
        WiFi.enableSTA(true);

        auto result = initiateConnectionAndWaitForOutcome(credentials);
        connRes = result.status;
    }

    return connRes;
}

void WifiCaptive::setResetSettingsCallback(std::function<void()> func)
{
    _resetcallback = func;
}

bool WifiCaptive::isSaved()
{
    readWifiCredentials();
    return _savedWifis[0].ssid != "";
}

void WifiCaptive::readWifiCredentials()
{
    Preferences preferences;
    preferences.begin("wificaptive", true);

    for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
    {
        _savedWifis[i].ssid = preferences.getString(WIFI_SSID_KEY(i), "");
        _savedWifis[i].pswd = preferences.getString(WIFI_PSWD_KEY(i), "");
    }

    preferences.end();
}

void WifiCaptive::saveWifiCredentials(const WifiCredentials credentials)
{
    Log_info("Saving wifi credentials: %s", credentials.ssid.c_str());

    // Check if the credentials already exist
    for (u16_t i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
    {
        if (_savedWifis[i].ssid == credentials.ssid && _savedWifis[i].pswd == credentials.pswd)
        {
            return; // Avoid saving duplicate networks
        }
    }

    for (u16_t i = WIFI_MAX_SAVED_CREDS - 1; i > 0; i--)
    {
        _savedWifis[i] = _savedWifis[i - 1];
    }

    _savedWifis[0] = credentials;

    Preferences preferences;
    preferences.begin("wificaptive", false);
    for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
    {
        preferences.putString(WIFI_SSID_KEY(i), _savedWifis[i].ssid);
        preferences.putString(WIFI_PSWD_KEY(i), _savedWifis[i].pswd);
    }
    preferences.putInt(WIFI_LAST_INDEX, 0);
    preferences.end();
}

void WifiCaptive::saveLastUsedWifiIndex(int index)
{
    Preferences preferences;
    preferences.begin("wificaptive", false);

    // if index is out of bounds, set to 0
    if (index < 0 || index >= WIFI_MAX_SAVED_CREDS)
    {
        index = 0;
    }

    // if index is greater than the total number of saved wifis, set to 0
    if (index > 0)
    {
        readWifiCredentials();
        if (_savedWifis[index].ssid == "")
        {
            index = 0;
        }
    }

    preferences.putInt(WIFI_LAST_INDEX, index);
}

int WifiCaptive::readLastUsedWifiIndex()
{
    Preferences preferences;
    preferences.begin("wificaptive", true);
    int index = preferences.getInt(WIFI_LAST_INDEX, 0);
    // if index is out of range, return 0
    if (index < 0 || index >= WIFI_MAX_SAVED_CREDS)
    {
        index = 0;
    }

    // if index is greater than the total number of saved wifis, set to 0
    if (index > 0)
    {
        readWifiCredentials();
        if (_savedWifis[index].ssid == "")
        {
            index = 0;
        }
    }
    preferences.end();
    return index;
}

void WifiCaptive::saveApiServer(String url)
{
    // if not URL is provided, don't save a preference and fall back to API_BASE_URL in config.h
    if (url == "")
        return;
    Preferences preferences;
    preferences.begin("data", false);
    preferences.putString("api_url", url);
    preferences.end();
}

std::vector<Network> WifiCaptive::getScannedUniqueNetworks(bool runScan)
{
    std::vector<Network> uniqueNetworks;
    int n = WiFi.scanComplete();
    if (runScan == true)
    {
        WiFi.scanNetworks(false);
        delay(100);
        int n = WiFi.scanComplete();
        while (n == WIFI_SCAN_RUNNING || n == WIFI_SCAN_FAILED)
        {
            delay(100);
            if (n == WIFI_SCAN_RUNNING)
            {
                n = WiFi.scanComplete();
            }
            else if (n == WIFI_SCAN_FAILED)
            {
                // There is a race coniditon that can occur, particularly if you use the async flag of WiFi.scanNetworks(true),
                // where you can race before the data is parsed. scanComplete will be -2, we'll see that and fail out, but then a few microseconds later it actually
                // fills in. This fixes that, in case we ever move back to the async version of scanNetworks, but as long as it's sync above it'll work
                // first shot always.
                Log_verbose("Supposedly failed to finish scan, let's wait 10 seconds before checking again");
                delay(10000);
                n = WiFi.scanComplete();
                if (n > 0)
                {
                    Log_verbose("Scan actually did complete, we have %d networks, breaking loop.", n);
                    // it didn't actually fail, we just raced before the scan was done filling in data
                    break;
                }
                WiFi.scanNetworks(false);
                delay(500);
                n = WiFi.scanComplete();
            }
        }
    }

    n = WiFi.scanComplete();
    Log_verbose("Scanning networks, final scan result: %d", n);

    // Process each found network
    for (int i = 0; i < n; ++i)
    {
        if (!WiFi.SSID(i).equals("TRMNL"))
        {
            String ssid = WiFi.SSID(i);
            int32_t rssi = WiFi.RSSI(i);
            bool open = WiFi.encryptionType(i);
            bool found = false;
            for (auto &network : uniqueNetworks)
            {
                if (network.ssid == ssid)
                {
                    Serial.println("Equal SSID");
                    found = true;
                    if (network.rssi < rssi)
                    {
                        network.rssi = rssi; // Update to higher RSSI
                    }
                    break;
                }
            }
            if (!found)
            {
                uniqueNetworks.push_back({ssid, rssi, open, false});
            }
        }
    }

    Log_info("Unique networks found: %d", uniqueNetworks.size());
    for (auto &network : uniqueNetworks)
    {
        Log_info("SSID: %s, RSSI: %d, Open: %d", network.ssid.c_str(), network.rssi, network.open);
    }

    return uniqueNetworks;
}

std::vector<WifiCredentials> WifiCaptive::matchNetworks(
    std::vector<Network> &scanResults,
    WifiCredentials savedWifis[])
{
    // sort scan results by RSSI
    std::sort(scanResults.begin(), scanResults.end(), [](const Network &a, const Network &b)
              { return a.rssi > b.rssi; });

    std::vector<WifiCredentials> sortedWifis;
    for (auto &network : scanResults)
    {
        for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
        {
            if (network.ssid == savedWifis[i].ssid)
            {
                sortedWifis.push_back(savedWifis[i]);
            }
        }
    }

    return sortedWifis;
}

std::vector<Network> WifiCaptive::combineNetworks(
    std::vector<Network> &scanResults,
    WifiCredentials savedWifis[])
{
    std::vector<Network> combinedNetworks;
    for (auto &network : scanResults)
    {
        bool found = false;
        for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
        {
            if (network.ssid == savedWifis[i].ssid)
            {
                combinedNetworks.push_back({network.ssid, network.rssi, network.open, true});
                found = true;
                break;
            }
        }
        if (!found)
        {
            combinedNetworks.push_back({network.ssid, network.rssi, network.open, false});
        }
    }
    // add saved wifis that are not combinedNetworks
    for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
    {
        bool found = false;
        for (auto &network : combinedNetworks)
        {
            if (network.ssid == savedWifis[i].ssid)
            {
                found = true;
                break;
            }
        }
        if (!found && savedWifis[i].ssid != "")
        {
            combinedNetworks.push_back({savedWifis[i].ssid, -200, false, true});
        }
    }

    return combinedNetworks;
}

bool WifiCaptive::autoConnect()
{
    Log_info("Trying to autoconnect to wifi...");
    readWifiCredentials();

    int last_used_index = readLastUsedWifiIndex();

    if (_savedWifis[last_used_index].ssid != "")
    {
        Log_info("Trying to connect to last used %s...", _savedWifis[last_used_index].ssid.c_str());
        WiFi.setSleep(0);
        WiFi.setMinSecurity(WIFI_AUTH_OPEN);
        WiFi.mode(WIFI_STA);

        if (tryConnectWithRetries(_savedWifis[last_used_index], last_used_index))
        {
            return true;
        }
    }

    Log_info("Last used network unavailable, scanning for known networks...");
    std::vector<Network> scanResults = getScannedUniqueNetworks(true);
    std::vector<WifiCredentials> sortedNetworks = matchNetworks(scanResults, _savedWifis);
    if (sortedNetworks.size() == 0)
    {
        Log_info("No matched networks found in scan, trying all saved networks...");
        sortedNetworks = std::vector<WifiCredentials>(_savedWifis, _savedWifis + WIFI_MAX_SAVED_CREDS);
    }

    WiFi.mode(WIFI_STA);
    for (auto &network : sortedNetworks)
    {
        if (network.ssid == "" || (network.ssid == _savedWifis[last_used_index].ssid && network.pswd == _savedWifis[last_used_index].pswd))
        {
            continue;
        }

        Log_info("Trying to connect to saved network %s...", network.ssid.c_str());
        int found_index = -1;
        for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++)
        {
            if (_savedWifis[i].ssid == network.ssid)
            {
                found_index = i;
                break;
            }
        }
        if (tryConnectWithRetries(network, found_index))
        {
            return true;
        }
    }

    Log_info("Failed to connect to any network");
    return false;
}

bool WifiCaptive::tryConnectWithRetries(const WifiCredentials creds, int last_used_index)
{
    for (int attempt = 0; attempt < WIFI_CONNECTION_ATTEMPTS; attempt++)
    {
        Log_info("Attempt %d to connect to %s", attempt + 1, creds.ssid.c_str());
        connect(creds);
        if (WiFi.status() == WL_CONNECTED)
        {
            Log_info("Connected to %s", creds.ssid.c_str());
            if (last_used_index >= 0)
            {
                saveLastUsedWifiIndex(last_used_index);
            }
            return true;
        }
        WiFi.disconnect();
        if (attempt < WIFI_CONNECTION_ATTEMPTS - 1)
        {
            uint32_t backoff_delay = 2000 * (1 << attempt);
            Log_info("Connection failed, waiting %d ms before retry...", backoff_delay);
            delay(backoff_delay);
        }
    }
    return false;
}

WifiCaptive WifiCaptivePortal;
