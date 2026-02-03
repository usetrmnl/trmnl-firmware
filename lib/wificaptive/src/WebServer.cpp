#include "WebServer.h"
#include <WiFi.h>
#include <test.h>
#include <trmnl_log.h>
#include <Preferences.h>

void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP, WifiOperationCallbacks callbacks)
{
    //======================== Webserver ========================
    // WARNING IOS (and maybe macos) WILL NOT POP UP IF IT CONTAINS THE WORD "Success" https://www.esp8266.com/viewtopic.php?f=34&t=4398
    // SAFARI (IOS) IS STUPID, G-ZIPPED FILES CAN'T END IN .GZ https://github.com/homieiot/homie-esp8266/issues/476 this is fixed by the webserver serve static function.
    // SAFARI (IOS) there is a 128KB limit to the size of the HTML. The HTML can reference external resources/images that bring the total over 128KB
    // SAFARI (IOS) popup browser has some severe limitations (javascript disabled, cookies disabled)

    // Required
    server.on("/connecttest.txt", [](AsyncWebServerRequest *request)
              { request->redirect("http://logout.net"); }); // windows 11 captive portal workaround
    server.on("/wpad.dat", [](AsyncWebServerRequest *request)
              { request->send(404); }); // Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :)

    // Background responses: Probably not all are Required, but some are. Others might speed things up?
    // A Tier (commonly used by modern systems)
    server.on("/generate_204", [](AsyncWebServerRequest *request)
              { request->redirect(LocalIPURL); }); // android captive portal redirect
    server.on("/redirect", [](AsyncWebServerRequest *request)
              { request->redirect(LocalIPURL); }); // microsoft redirect
    server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request)
              { request->redirect(LocalIPURL); }); // apple call home
    server.on("/canonical.html", [](AsyncWebServerRequest *request)
              { request->redirect(LocalIPURL); }); // firefox captive portal call home
    server.on("/success.txt", [](AsyncWebServerRequest *request)
              { request->send(200); }); // firefox captive portal call home
    server.on("/ncsi.txt", [](AsyncWebServerRequest *request)
              { request->redirect(LocalIPURL); }); // windows call home

    // return 404 to webpage icon
    server.on("/favicon.ico", [](AsyncWebServerRequest *request)
              { request->send(404); }); // webpage icon

    // Serve index.html
    server.on("/", HTTP_ANY, [&](AsyncWebServerRequest *request)
              {
                  AsyncWebServerResponse *response = request->beginResponse(200, "text/html", INDEX_HTML, INDEX_HTML_LEN);
                  response->addHeader("Content-Encoding", "gzip");
                  request->send(response); // redirect to the local IP URL
              });

    // Servce logo.svg
    server.on("/logo.svg", HTTP_ANY, [&](AsyncWebServerRequest *request)
              {
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", LOGO_SVG, LOGO_SVG_LEN);
		response->addHeader("Content-Encoding", "gzip");
		response->addHeader("Content-Type", "image/svg+xml");
    	request->send(response); });

    server.on("/soft-reset", HTTP_ANY, [callbacks](AsyncWebServerRequest *request)
              {
		callbacks.resetSettings();
		request->send(200); });

    server.on("/advanced", HTTP_GET, [&](AsyncWebServerRequest *request)
              {
                AsyncWebServerResponse *response = request->beginResponse(200, "text/html", ADVANCED_HTML, ADVANCED_HTML_LEN);
                response->addHeader("Content-Encoding", "gzip");
                request->send(response); });
    server.on("/run-test", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                Serial.println("Running sensor test from web...");
                String json = testTemperature();
                request->send(200, "application/json", json); });

    auto scanGET = server.on("/scan", HTTP_GET, [callbacks](AsyncWebServerRequest *request)
                             {
		String json = "{\"networks\":[";
		int n = WiFi.scanComplete();
		if (n == WIFI_SCAN_FAILED) {
			WiFi.scanNetworks(true);
			return request->send(202);
		} else if(n == WIFI_SCAN_RUNNING){
			return request->send(202);
		} else {
			// Data structure to store the highest RSSI for each SSID
            // Warning: DO NOT USE true on this function in an async context!
            std::vector<Network> combinedNetworks = callbacks.getAnnotatedNetworks(false);

            // Generate JSON response
            size_t size = 0;
            for (const auto &network : combinedNetworks)
            {
                String ssid = network.ssid;
				String rssi = String(network.rssi);

				// Escape invalid characters
				ssid.replace("\\","\\\\");
				ssid.replace("\"","\\\"");
				json+= "{";
				json+= "\"name\":\""+ssid+"\",";
				json+= "\"rssi\":\""+rssi+"\",";
				json+= "\"open\":"+String(network.open == WIFI_AUTH_OPEN ? "true,": "false,");
                json+= "\"saved\":"+String(network.saved ? "true,": "false,");
                json+= "\"enterprise\":"+String(network.enterprise ? "true": "false");
				json+= "}";

				size += 1;

				if (size != combinedNetworks.size())
				{
					json+= ",";
				}
            }

            WiFi.scanDelete();
			Serial.println(json);

			if (WiFi.scanComplete() == -2){
				WiFi.scanNetworks(true);
			}
		}
		json += "],";

        // Expose MAC pre-connect for easier setup debugging
        String mac = WiFi.macAddress();
        json+= "\"mac\":\""+mac+"\"";

        json += "}";
        request->send(200, "application/json", json);
		json = String(); });

    AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/connect", [callbacks](AsyncWebServerRequest *request, JsonVariant &json)
                                                                           {
		JsonObject data = json.as<JsonObject>();
		String ssid = data["ssid"];
		String pswd = data["pswd"];
        String api_server = data["server"];

        bool isEnterprise = data["isEnterprise"].is<bool>() && data["isEnterprise"].as<bool>();
        String username = data["username"].is<String>() ? data["username"].as<String>() : "";
        String identity = data["identity"].is<String>() ? data["identity"].as<String>() : "";

        // Static IP fields (optional)
        bool useStaticIP = data["useStaticIP"].is<bool>() && data["useStaticIP"].as<bool>();
        String staticIP = data["staticIP"].is<String>() ? data["staticIP"].as<String>() : "";
        String gateway = data["gateway"].is<String>() ? data["gateway"].as<String>() : "";
        String subnet = data["subnet"].is<String>() ? data["subnet"].as<String>() : "";
        String dns1 = data["dns1"].is<String>() ? data["dns1"].as<String>() : "";
        String dns2 = data["dns2"].is<String>() ? data["dns2"].as<String>() : "";

        WifiCredentials credentials;
        credentials.ssid = ssid;
        credentials.pswd = pswd;
        credentials.isEnterprise = isEnterprise;
        credentials.username = username;
        credentials.identity = identity;
        // Static IP settings
        credentials.useStaticIP = useStaticIP;
        credentials.staticIP = staticIP;
        credentials.gateway = gateway;
        credentials.subnet = subnet;
        credentials.dns1 = dns1;
        credentials.dns2 = dns2;

        String ntpServer = data["ntpServer1"].is<String>() ? data["ntpServer1"].as<String>() : "";
        if (ntpServer.length() > 0) {
            Preferences prefs;
            prefs.begin("data", false);
            prefs.putString("ntp_server", ntpServer);
            prefs.end();
            Log_info("WebServer: Saved NTP server: %s", ntpServer.c_str());
        }

        Log_info("WebServer: Received SSID: %s, Static IP: %s", ssid.c_str(), useStaticIP ? "yes" : "no");

        callbacks.setConnectionCredentials(credentials, api_server);
        String mac = WiFi.macAddress();
        String message = "{\"ssid\":\"" + ssid + "\",\"mac\":\"" + mac + "\"}";
        request->send(200, "application/json", message); });

    server.addHandler(handler);

    server.onNotFound([](AsyncWebServerRequest *request)
                      { request->redirect(LocalIPURL); });
}
