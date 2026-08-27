#include <Arduino.h>
#include <ArduinoLog.h>
#include <WiFi.h>
#include <WifiCaptive.h>
#include <bl.h>
#include <config.h>
#include <globals.h>
#include <messages.h>
#include <trmnl_log.h>
#include <wifi-helpers.h>
#include <wifi_network.h>
#include <wifi_session.h>

#ifdef BOARD_TRMNL_X
#include <modem.h>
#include <touchbar_actions.h>
#include <vector>
#endif

// --- Helpers still owned by bl.cpp ---
void wifiErrorDeepSleep(void);

void wifiSessionInit(void) { WifiCaptivePortal.setHostname(getWifiClientHostname()); }

#ifdef BOARD_TRMNL_X
static bool wifiSessionModemNeeded(void) {
  if (WifiCaptivePortal.isSaved()) {
    WifiCredentials lastCreds = WifiCaptivePortal.getLastCredentials();
    return lastCreds.is5GHz;
  }
  return true; // captive portal needs modem for 5 GHz
}

static void wifiSessionPrepareModem(void) {
  Log.info("%s [%d]: Checking if we need to use the ESP32-C5 modem...\r\n", __FILE__, __LINE__);
  const bool bModemNeeded = wifiSessionModemNeeded();
  Log.info("%s [%d]: modem needed = %d\n\r", __FILE__, __LINE__, bModemNeeded);

  if (bModemNeeded) {
    modem_reset_target(); // Must be done BEFORE the instantiation of the class since it expects the modem to be ready
    static Modem modemInstance(115200);
    if (modemInstance.isInitialized()) {
      g_modem = &modemInstance;
    } else {
      Log_info("Modem init failed — falling back to 2.4 GHz mode");
      g_modem = nullptr;
    }
  } else {
    g_modem = nullptr;
  }

  // Only scan when no credentials are saved (i.e. captive portal will be shown).
  if (g_modem && !WifiCaptivePortal.isSaved()) {
    // The modem is a separate radio and can see the device's own captive-portal
    // SoftAP over the air; exclude it so it never shows up as a connectable network.
    String ownApSsid = WifiCaptivePortal.getAPSSID();

    Log_info("No saved credentials — scanning networks via modem...");
    auto modemNets = g_modem->scanNetworks();
    Log_info("Modem found %d network(s)", modemNets.size());
    std::vector<ExternalNetwork> nets;
    for (auto &n : modemNets) {
      if (n.ssid == ownApSsid) continue;
      nets.push_back({n.ssid, n.rssi, n.open, n.is5GHz});
    }
    WifiCaptivePortal.setNetworks(nets);

    // Register callback so captive portal can connect 5 GHz networks via modem
    WifiCaptivePortal.setModemConnectCallback([](const String &ssid, const String &pass) {
      return g_modem->connectToNetwork(ssid, pass, getWifiClientHostname());
    });

    // Register callback so the captive portal's Refresh button can trigger a fresh modem scan
    WifiCaptivePortal.setModemScanCallback([ownApSsid]() {
      auto modemNets = g_modem->scanNetworks();
      Log_info("Modem re-scan found %d network(s)", modemNets.size());
      std::vector<ExternalNetwork> nets;
      for (auto &n : modemNets) {
        if (n.ssid == ownApSsid) continue;
        nets.push_back({n.ssid, n.rssi, n.open, n.is5GHz});
      }
      return nets;
    });

    String modemMac = g_modem->getMacAddress();
    if (!modemMac.isEmpty()) {
      WifiCaptivePortal.setModemMac(modemMac);
    }
  }
}
#endif // BOARD_TRMNL_X

void wifiSessionConnect(void) {
#ifdef BOARD_TRMNL_X
  wifiSessionPrepareModem();
#endif

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  MSG current_msg = NONE;

// uncomment this to hardcode WiFi credentials (useful for testing wifi errors, etc.)
// #define HARDCODED_WIFI
#ifdef HARDCODED_WIFI
  WifiCredentials hardcodedCreds = {.ssid = "ssid-goes-here", .pswd = "password-goes-here"};
  Log_info("Hardcoded WiFi: connecting to SSID '%s'", hardcodedCreds.ssid.c_str());
  auto connectResult = WifiCaptivePortal.connect(hardcodedCreds);
  Log_info("Hardcoded WiFi: connect result '%s'", wifiStatusStr(connectResult.status));
// goToSleep();
#else

  if (WifiCaptivePortal.isSaved()) {
    // WiFi saved, connection
    Log.info("%s [%d]: WiFi saved\r\n", __FILE__, __LINE__);
    int connection_res = connectWithSavedCredentials() ? 1 : 0;

    Log.info("%s [%d]: Connection result: %d, WiFI Status: %d\r\n", __FILE__, __LINE__, connection_res, WiFi.status());

    // Check if connected
    if (connection_res) {
      String ip = String(WiFi.localIP());
      Log.info("%s [%d]:wifi_connection [DEBUG]: Connected: %s\r\n", __FILE__, __LINE__, ip.c_str());
      preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
    } else {
      if (current_msg != WIFI_FAILED) {
        showMessageWithLogo(WIFI_FAILED);
        current_msg = WIFI_FAILED;
      }

      Log_fatal_submit("Connection failed! WL Status: %d", WiFi.status());

      wifiErrorDeepSleep();
    }
  } else {
    // WiFi credentials are not saved - start captive portal
    Log.info("%s [%d]: WiFi NOT saved\r\n", __FILE__, __LINE__);

    Log_info("FW version %s", Messages::firmware_version().c_str());

    showMessageWithLogo(WIFI_CONNECT, "", false, Messages::firmware_version().c_str(), WifiCaptivePortal.getAPSSID());
#ifdef BOARD_TRMNL_X
    touchbar_init_captive_portal_power_off_hook();
#endif
    WifiCaptivePortal.setResetSettingsCallback(resetDeviceCredentials);
    bool portal_ok = WifiCaptivePortal.startPortal();
    if (!portal_ok) {
      WiFi.disconnect(true);

      showMessageWithLogo(WIFI_FAILED);

      Log_error("Failed to connect or hit timeout");

      // Go to deep sleep
      wifiErrorDeepSleep();
    }
    Log.info("%s [%d]: WiFi connected\r\n", __FILE__, __LINE__);
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
  }

#endif
}
