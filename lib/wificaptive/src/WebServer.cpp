#include "WebServer.h"

#include <Preferences.h>
#include <WiFi.h>
#include <test.h>
#include <trmnl_log.h>

#include "WifiCaptive.h"

namespace {
  const char *connectionStateName(WifiConnectionState state) {
    switch (state) {
    case WifiConnectionState::Idle:
      return "idle";
    case WifiConnectionState::Connecting:
      return "connecting";
    case WifiConnectionState::Connected:
      return "connected";
    case WifiConnectionState::AuthenticationFailed:
      return "authentication_failed";
    case WifiConnectionState::NetworkNotFound:
      return "network_not_found";
    case WifiConnectionState::Failed:
      return "failed";
    }
    return "failed";
  }

  // The network a /connect body describes
  WifiCredentials credentialsFromJson(JsonObject data) {
    WifiCredentials credentials;
    credentials.ssid = data["ssid"].is<String>() ? data["ssid"].as<String>() : "";
    credentials.pswd = data["pswd"].is<String>() ? data["pswd"].as<String>() : "";
    credentials.isEnterprise = data["isEnterprise"].is<bool>() && data["isEnterprise"].as<bool>();
    credentials.username = data["username"].is<String>() ? data["username"].as<String>() : "";
    credentials.identity = data["identity"].is<String>() ? data["identity"].as<String>() : "";
    // Static IP fields (optional)
    credentials.useStaticIP = data["useStaticIP"].is<bool>() && data["useStaticIP"].as<bool>();
    credentials.staticIP = data["staticIP"].is<String>() ? data["staticIP"].as<String>() : "";
    credentials.gateway = data["gateway"].is<String>() ? data["gateway"].as<String>() : "";
    credentials.subnet = data["subnet"].is<String>() ? data["subnet"].as<String>() : "";
    credentials.dns1 = data["dns1"].is<String>() ? data["dns1"].as<String>() : "";
    credentials.dns2 = data["dns2"].is<String>() ? data["dns2"].as<String>() : "";
    return credentials;
  }

  // Device settings in the "data" namespace: "" goes back to the default
  void saveDeviceSetting(const char *key, const String &value) {
    Preferences prefs;
    prefs.begin("data", false);
    if (value.length() > 0) {
      prefs.putString(key, value);
    } else if (prefs.isKey(key)) {
      prefs.remove(key);
    }
    prefs.end();
  }

  void sendJson(AsyncWebServerRequest *request, int code, JsonDocument &doc) {
    String out;
    serializeJson(doc, out);
    request->send(code, "application/json", out);
  }
} // namespace

void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP, WifiOperationCallbacks callbacks,
                    const String &modemMac) {
    //======================== Webserver ========================
    // WARNING IOS (and maybe macos) WILL NOT POP UP IF IT CONTAINS THE WORD "Success"
  // https://www.esp8266.com/viewtopic.php?f=34&t=4398 SAFARI (IOS) IS STUPID, G-ZIPPED FILES CAN'T END IN .GZ
  // https://github.com/homieiot/homie-esp8266/issues/476 this is fixed by the webserver serve static function. SAFARI
  // (IOS) there is a 128KB limit to the size of the HTML. The HTML can reference external resources/images that bring
  // the total over 128KB SAFARI (IOS) popup browser has some severe limitations (javascript disabled, cookies disabled)

  // Required
  server.on("/connecttest.txt", [](AsyncWebServerRequest *request) {
    request->redirect("http://logout.net");
  }); // windows 11 captive portal workaround
  server.on("/wpad.dat", [](AsyncWebServerRequest *request) {
    request->send(404);
  }); // Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the
      // esp32 :)

  // Background responses: Probably not all are Required, but some are. Others might speed things up?
  // A Tier (commonly used by modern systems)
  server.on("/generate_204",
            [](AsyncWebServerRequest *request) { request->redirect(LocalIPURL); }); // android captive portal redirect
  server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(LocalIPURL); }); // microsoft redirect
  server.on("/hotspot-detect.html",
            [](AsyncWebServerRequest *request) { request->redirect(LocalIPURL); }); // apple call home
  server.on("/canonical.html",
            [](AsyncWebServerRequest *request) { request->redirect(LocalIPURL); }); // firefox captive portal call home
  server.on("/success.txt",
            [](AsyncWebServerRequest *request) { request->send(200); }); // firefox captive portal call home
  server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(LocalIPURL); }); // windows call home

  // return 404 to webpage icon
  server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); }); // webpage icon

  // Serve index.html
  server.on("/", HTTP_ANY, [&](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response =
      request->beginResponse(200, "text/html; charset=utf-8", INDEX_HTML, INDEX_HTML_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response); // redirect to the local IP URL
  });

  server.on("/soft-reset", HTTP_ANY, [callbacks](AsyncWebServerRequest *request) {
    callbacks.resetSettings();
    request->send(200);
  });

  server.on("/advanced", HTTP_GET, [&](AsyncWebServerRequest *request) { request->redirect("/#advanced"); });
  server.on("/run-test", HTTP_GET, [](AsyncWebServerRequest *request) {
    Log_info_serial("Running sensor test from web...");
    String json = testTemperature();
    request->send(200, "application/json", json);
  });

  server.on("/device-settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    Preferences prefs;
    prefs.begin("data", true);
    JsonDocument doc;
    doc["api_url"] = prefs.getString("api_url", "");
    doc["hostname"] = prefs.getString("hostname", "");
    doc["ntp_server"] = prefs.getString("ntp_server", "");
    prefs.end();
    doc["verbose_logging"] = read_verbose_logging();
    sendJson(request, 200, doc);
  });

  // The device-wide settings of the Advanced page, saved right away (a field left out is not touched)
  AsyncCallbackJsonWebHandler *settingsHandler =
    new AsyncCallbackJsonWebHandler("/settings", [](AsyncWebServerRequest *request, JsonVariant &json) {
      JsonObject data = json.as<JsonObject>();
      JsonDocument doc;

      if (data["server"].is<String>()) {
        String server = data["server"].as<String>();
        server.trim();
        while (server.endsWith("/"))
          server.remove(server.length() - 1);
        if (server.length() > 0 && !server.startsWith("http://") && !server.startsWith("https://")) {
          doc["error"] = "server";
          return sendJson(request, 400, doc);
        }
        WifiCaptivePortal.saveApiServer(server); // a different server clears the pairing
        Log_info("WebServer: Saved API server: %s", server.c_str());
      }

      if (data["hostname"].is<String>()) {
        String hostname = data["hostname"].as<String>();
        hostname.trim();
        saveDeviceSetting("hostname", hostname);
        // an emptied hostname goes back to the default on the next start
        if (hostname.length() > 0) WifiCaptivePortal.setHostname(hostname);
        Log_info("WebServer: Saved hostname: %s", hostname.c_str());
      }

      if (data["ntpServer1"].is<String>()) {
        String ntpServer = data["ntpServer1"].as<String>();
        ntpServer.trim();
        saveDeviceSetting("ntp_server", ntpServer);
        Log_info("WebServer: Saved NTP server: %s", ntpServer.c_str());
      }

      doc["saved"] = true;
      sendJson(request, 200, doc);
    });
  server.addHandler(settingsHandler);

  // The saved networks (never their passwords), in the order of their slots
  server.on("/saved-networks", HTTP_GET, [](AsyncWebServerRequest *request) {
    std::vector<WifiCredentials> saved;
    int lastUsed = WifiCaptivePortal.getSavedCredentials(saved);
    JsonDocument doc;
    doc["max"] = WIFI_MAX_SAVED_CREDS;
    JsonArray networks = doc["networks"].to<JsonArray>();
    for (size_t i = 0; i < saved.size(); i++) {
      JsonObject network = networks.add<JsonObject>();
      network["index"] = i;
      network["ssid"] = saved[i].ssid;
      network["band"] = saved[i].is5GHz ? "5GHz" : "2.4GHz";
      network["enterprise"] = saved[i].isEnterprise;
      network["static_ip"] = saved[i].useStaticIP;
      network["last_used"] = (int)i == lastUsed;
    }
    sendJson(request, 200, doc);
  });

  // The SSID comes along with the slot, so a stale page can't forget the wrong network
  AsyncCallbackJsonWebHandler *forgetHandler =
    new AsyncCallbackJsonWebHandler("/forget-network", [](AsyncWebServerRequest *request, JsonVariant &json) {
      JsonObject data = json.as<JsonObject>();
      int index = data["index"].is<int>() ? data["index"].as<int>() : -1;
      String ssid = data["ssid"].is<String>() ? data["ssid"].as<String>() : "";
      JsonDocument doc;
      bool forgotten = WifiCaptivePortal.forgetWifiCredentials(index, ssid);
      doc["forgotten"] = forgotten;
      sendJson(request, forgotten ? 200 : 404, doc);
    });
  server.addHandler(forgetHandler);

  // "Done": the device restarts and joins a saved network
  server.on("/exit", HTTP_POST, [callbacks](AsyncWebServerRequest *request) {
    bool exiting = callbacks.requestExit();
    request->send(exiting ? 200 : 409, "application/json", exiting ? "{\"exiting\":true}" : "{\"exiting\":false}");
  });

  server.on("/connection-status", HTTP_GET, [callbacks](AsyncWebServerRequest *request) {
    String response = "{\"status\":\"";
    response += connectionStateName(callbacks.getConnectionState());
    response += "\"}";
    request->send(200, "application/json", response);
  });

  AsyncCallbackJsonWebHandler *verboseLoggingHandler =
    new AsyncCallbackJsonWebHandler("/verbose-logging", [](AsyncWebServerRequest *request, JsonVariant &json) {
      JsonObject data = json.as<JsonObject>();
      bool enabled = data["enabled"].is<bool>() && data["enabled"].as<bool>();
      write_verbose_logging(enabled); // takes effect immediately

      Log_info("WebServer: Verbose logging set to %s", enabled ? "enabled" : "disabled");
      request->send(200, "application/json", String("{\"verbose_logging\":") + (enabled ? "true" : "false") + "}");
    });
  server.addHandler(verboseLoggingHandler);

  auto scanGET = server.on("/scan", HTTP_GET, [callbacks, modemMac](AsyncWebServerRequest *request) {
    String json = "{\"networks\":[";

    if (request->hasParam("force")) {
      callbacks.forceRescan();
    }

    if (!callbacks.isNetworkListReady()) {
      if (WiFi.scanComplete() == WIFI_SCAN_FAILED) {
        WiFi.scanNetworks(true);
      }
      return request->send(202);
    }

    // Data structure to store the highest RSSI for each SSID
    // Warning: DO NOT USE true on getAnnotatedNetworks in an async context!
    std::vector<WifiNetwork> combinedNetworks = callbacks.getAnnotatedNetworks(false);

    // Generate JSON response
    size_t size = 0;
    for (const auto &network : combinedNetworks) {
      String ssid = network.ssid;
      String rssi = String(network.rssi);

      // Escape invalid characters
      ssid.replace("\\", "\\\\");
      ssid.replace("\"", "\\\"");
      json += "{";
      json += "\"name\":\"" + ssid + "\",";
      json += "\"rssi\":\"" + rssi + "\",";
      json += "\"open\":" + String(network.open ? "true," : "false,");
      json += "\"saved\":" + String(network.saved ? "true," : "false,");
      json += "\"band\":\"" + String(network.is5GHz ? "5GHz" : "2.4GHz") + "\",";
      json += "\"enterprise\":" + String(network.enterprise ? "true" : "false");
      json += "}";

      size += 1;

      if (size != combinedNetworks.size()) {
        json += ",";
      }
    }

    WiFi.scanDelete();
    Log_info_serial("%s", json.c_str());

    if (WiFi.scanComplete() == -2) {
#ifdef CONFIG_IDF_TARGET_ESP32C5
      WiFi.setBandMode(WIFI_BAND_MODE_AUTO);
#endif
      WiFi.scanNetworks(true);
    }

    json += "],";

    // Expose MAC pre-connect for easier setup debugging
    String mac = WiFi.macAddress();
    json += "\"mac\":\"" + mac + "\"";
#ifdef BOARD_TRMNL_X
    if (!modemMac.isEmpty()) {
      json += ",\"mac_5ghz\":\"" + modemMac + "\"";
    }
#endif
    json += "}";
    request->send(200, "application/json", json);
    json = String();
  });

  AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler(
    "/connect", [callbacks, modemMac](AsyncWebServerRequest *request, JsonVariant &json) {
      JsonObject data = json.as<JsonObject>();
      WifiCredentials credentials = credentialsFromJson(data);
      String ssid = credentials.ssid;
      bool useStaticIP = credentials.useStaticIP;
      // The settings page saves the server on its Advanced page (/settings) and leaves it out here
      bool api_server_given = data["server"].is<String>();
      String api_server = api_server_given ? data["server"].as<String>() : "";
      String band = data["band"].is<String>() ? data["band"].as<String>() : "";

      String ntpServer = data["ntpServer1"].is<String>() ? data["ntpServer1"].as<String>() : "";
      if (ntpServer.length() > 0) {
        Preferences prefs;
        prefs.begin("data", false);
        prefs.putString("ntp_server", ntpServer);
        prefs.end();
        Log_info("WebServer: Saved NTP server: %s", ntpServer.c_str());
      }

      String hostname = data["hostname"].is<String>() ? data["hostname"].as<String>() : "";
      if (hostname.length() > 0) {
        Preferences prefs;
        prefs.begin("data", false);
        prefs.putString("hostname", hostname);
        prefs.end();
        WifiCaptivePortal.setHostname(hostname); // keep in-memory hostname up-to-date
        Log_info("WebServer: Saved hostname: %s", hostname.c_str());
      }

      Log_info("WebServer: Received SSID: %s, Static IP: %s", ssid.c_str(), useStaticIP ? "yes" : "no");

      callbacks.setConnectionCredentials(credentials, api_server, api_server_given, band);
      String mac = WiFi.macAddress();
      String message = "{\"ssid\":\"" + ssid + "\",\"mac\":\"" + mac + "\"";
#ifdef BOARD_TRMNL_X
      if (!modemMac.isEmpty()) {
        message += ",\"mac_5ghz\":\"" + modemMac + "\"";
      }
#endif
      message += "}";
      request->send(200, "application/json", message);
    });

  server.addHandler(handler);

  server.onNotFound([](AsyncWebServerRequest *request) { request->redirect(LocalIPURL); });
}
