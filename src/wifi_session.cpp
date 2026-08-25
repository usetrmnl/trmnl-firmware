#include <Arduino.h>
#include <ArduinoLog.h>
#include <WiFi.h>
#include <WifiCaptive.h>
#include <config.h>
#include <display.h>
#include <globals.h>
#include <messages.h>
#include <trmnl_log.h>
#include <wifi_network.h>
#include <wifi_session.h>

#ifdef BOARD_TRMNL_X
#include <IQS323.h>
#include <iqs323_task.h>
#include <modem.h>
#include <vector>
#endif

// --- Helpers still owned by bl.cpp ---
void showMessageWithLogo(MSG message_type);
void showMessageWithLogo(MSG message_type, String friendly_id, bool id, const char *fw_version, String message);
void wifiErrorDeepSleep(void);
void resetDeviceCredentials(void);

#ifdef BOARD_TRMNL_X
/** Portal-tick power-off corners handler (IQS323); implemented in bl.cpp. */
void blWifiPortalTickX(void);
#endif

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
  const bool bModemNeeded = wifiSessionModemNeeded();
  Log.info("%s [%d]: Checking if we need to use the ESP32-C5 modem...\r\n", __FILE__, __LINE__);
  Log.info("%s [%d]: modem needed = %d\n\r", __FILE__, __LINE__, bModemNeeded);

  if (bModemNeeded) {
    modem_reset_target(); // before Modem ctor — expects modem ready
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

  // Only scan when no credentials are saved (captive portal will be shown).
  if (g_modem && !WifiCaptivePortal.isSaved()) {
    // Modem can see this device's SoftAP; exclude it from connectable list.
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

    WifiCaptivePortal.setModemConnectCallback([](const String &ssid, const String &pass) {
      return g_modem->connectToNetwork(ssid, pass, getWifiClientHostname());
    });

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
  Log_info("Hardcoded WiFi: connect result '%s'", wifiStatusStr(connectResult));
// goToSleep();
#else

  if (WifiCaptivePortal.isSaved()) {
    Log.info("%s [%d]: WiFi saved\r\n", __FILE__, __LINE__);
    int connection_res = connectWithSavedCredentials() ? 1 : 0;

    Log.info("%s [%d]: Connection result: %d, WiFI Status: %d\r\n", __FILE__, __LINE__, connection_res, WiFi.status());

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
    Log.info("%s [%d]: WiFi NOT saved\r\n", __FILE__, __LINE__);

    Log_info("FW version %s", Messages::firmware_version().c_str());

    showMessageWithLogo(WIFI_CONNECT, "", false, Messages::firmware_version().c_str(), WifiCaptivePortal.getAPSSID());
#ifdef BOARD_TRMNL_X
    // set TAP mode as default while portal is up
    iqs323_task_i2c_lock();
    iqs323.setGestureConfig(true, STOP);
    iqs323_task_i2c_unlock();
    touchbar_tap_mode = true;

    WifiCaptivePortal.setPortalTickCallback([]() { blWifiPortalTickX(); });
#endif
    WifiCaptivePortal.setResetSettingsCallback(resetDeviceCredentials);
    bool portal_ok = WifiCaptivePortal.startPortal();
    if (!portal_ok) {
      WiFi.disconnect(true);

      showMessageWithLogo(WIFI_FAILED);

      Log_error("Failed to connect or hit timeout");

      wifiErrorDeepSleep();
    }
    Log.info("%s [%d]: WiFi connected\r\n", __FILE__, __LINE__);
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
  }

#endif
}
