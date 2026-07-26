#include <display_session.h>

#include <bl_bridge.h>
#include <image_pipeline.h>

#include <ArduinoLog.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WifiCaptive.h>
#include <api-client/display.h>
#include <api-client/request_headers.h>
#include <api_response_parsing.h>
#include <bmp.h>
#include <png.h>
#include <bl.h>
#include <config.h>
#include <device_id.h>
#include <filesystem.h>
#include <http_client.h>
#include <power.h>
#include <trmnl_log.h>
#include <wifi_network.h>
#include "logging_parcers.h"
#include "displayed_image.h"

#ifdef BOARD_TRMNL_X
#include <modem.h>
#endif

// Error name table lives with the display session (used by downloadAndShow logging).
const char *szHTTPErrors[] = {
    "HTTPS_NO_ERR",
    "HTTPS_RESET",
    "HTTPS_NO_REGISTER",
    "HTTPS_SUCCESS",
    "HTTPS_CLIENT_FAILED",
    "HTTPS_REQUEST_FAILED",
    "HTTPS_UNABLE_TO_CONNECT",
    "HTTPS_CONNECTION_FAILED",
    "HTTPS_RESPONSE_CODE_INVALID",
    "HTTPS_JSON_PARSING_ERR",
    "HTTPS_WRONG_IMAGE_SIZE",
    "HTTPS_WRONG_IMAGE_FORMAT",
    "HTTPS_IMAGE_FILE_TOO_BIG",
    "HTTPS_PLUGIN_NOT_ATTACHED",
    "HTTPS_BAD_CLIENT",
    "HTTPS_OUT_OF_MEMORY",
    "HTTPS_TIMED_OUT",
};

ApiDisplayInputs loadApiDisplayInputs(Preferences &preferences)
{
  ApiDisplayInputs inputs;

  inputs.baseUrl = preferences.getString(PREFERENCES_API_URL, API_BASE_URL);

  Log.info("%s [%d]: baseUrl from preferences: %s\r\n", __FILE__, __LINE__, inputs.baseUrl.c_str());

  char wakeupReasonString[32] = {0};
  if (parseWakeupReasonToStr(wakeupReasonString, sizeof(wakeupReasonString), (esp_sleep_source_t)wakeup_reason))
  {
    inputs.updateSource = String(wakeupReasonString);
  }
  else
  {
    inputs.updateSource = "unknown";
  }

  if (preferences.isKey(PREFERENCES_API_KEY))
  {
    inputs.apiKey = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY, inputs.apiKey.c_str());
  }
  else
  {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY);
  }

  if (preferences.isKey(PREFERENCES_FRIENDLY_ID))
  {
    inputs.friendlyId = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_FRIENDLY_ID, inputs.friendlyId);
  }
  else
  {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_FRIENDLY_ID);
  }

  inputs.refreshRate = SLEEP_TIME_TO_SLEEP;

  if (preferences.isKey(PREFERENCES_SLEEP_TIME_KEY))
  {
    inputs.refreshRate = preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP);
    Log.info("%s [%d]: %s key exists. Value - %d\r\n", __FILE__, __LINE__, PREFERENCES_SLEEP_TIME_KEY, inputs.refreshRate);
  }
  else
  {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_SLEEP_TIME_KEY);
  }

  inputs.macAddress = device_mac_address();
  WiFiStatus wifi = getWiFiStatus();
  inputs.rssi = wifi.rssi;
  inputs.wifiBand = wifi.band;
  inputs.batteryVoltage = vBatt; //readBatteryVoltage();
  inputs.firmwareVersion = String(FW_VERSION_STRING);
  inputs.firmwareCommit = String(FW_COMMIT);
  inputs.displayWidth = display_width();
  inputs.displayHeight = display_height();
  inputs.model = DEVICE_MODEL;
  inputs.specialFunction = special_function;
  inputs.imageCached = bUsedCachedImage;
  inputs.prevWakeTime = iPrevWakeTime;
  inputs.usbStatus = get_usb_status();
  inputs.chargingStatus = get_charging_status();

#ifdef BOARD_TRMNL_X
  inputs.batteryCount = battery_count;
  if (lipo._initialized) { // only report SoC if battery was detected and BQ27427 initialized successfully
    inputs.stateOfCharge = lipo.soc();
    inputs.stateOfHealth = lipo.soh();
    inputs.batteryCurrent = lipo.current(AVG);
    inputs.batteryTemperature = float((lipo.temperature(BATTERY)) - 2732) / 10.0; // convert from K to C
    inputs.currentBatteryCapacity = lipo.capacity(REMAIN) * lipo.designEnergyScale();
    inputs.maxBatteryCapacity = lipo.capacity(FULL) * lipo.designEnergyScale();
  }
  else {
    inputs.stateOfCharge = -1;
    inputs.stateOfHealth = -1;
    inputs.batteryCurrent = -1;
    inputs.batteryTemperature = -1;
    inputs.currentBatteryCapacity = -1;
    inputs.maxBatteryCapacity = -1;
  }
#endif // BOARD_TRMNL_X

  return inputs;
}

https_request_err_e downloadAndShow()
{
  image_err_e png_res = PNG_DECODE_ERR;
  bmp_err_e bmp_res = BMP_NOT_BMP;
  auto apiDisplayInputs = loadApiDisplayInputs(preferences);

#ifdef BOARD_TRMNL_X
  if (g_modem && WifiCaptivePortal.getLastCredentials().is5GHz)
  {
    Log_info("Fetching /api/display via modem (5 GHz path)");
    String reqHeaders = formatHeaders(buildDisplayHeaders(apiDisplayInputs));

    auto httpRes = g_modem->httpGet(apiDisplayInputs.baseUrl + "/api/display", "", 0, reqHeaders);
    if (!httpRes.ok)
    {
      Log_error_submit("Modem /api/display request failed (%u bytes received)", httpRes.bytesReceived);
      return HTTPS_REQUEST_FAILED;
    }
    auto apiResp = parseResponse_apiDisplay(httpRes.body);
    if (apiResp.outcome == ApiDisplayOutcome::DeserializationError)
    {
      Log_error_submit("Modem /api/display JSON parse error: %s", apiResp.error_detail.c_str());
      return HTTPS_JSON_PARSING_ERR;
    }
    apiDisplayResult = {HTTPS_NO_ERR, apiResp, ""};
  }
  else 
#endif // BOARD_TRMNL_X  
  {
    for (int attempt = 1; attempt <= 5; ++attempt)
    {
      apiDisplayResult = fetchApiDisplay(apiDisplayInputs);
      if (apiDisplayResult.error != HTTPS_UNABLE_TO_CONNECT &&
          apiDisplayResult.error != HTTPS_RESPONSE_CODE_INVALID)
        break;
      Log_error_serial("Connection attempt %d/5 failed: %s", attempt, apiDisplayResult.error_detail.c_str());
      if (attempt < 5) delay(2000);
    }
  }

  if (apiDisplayResult.error != HTTPS_NO_ERR)
  {
    Log_error_submit("Error fetching API display: %d, detail: %s", apiDisplayResult.error, apiDisplayResult.error_detail.c_str());
    return apiDisplayResult.error;
  }

  https_request_err_e result = handleApiDisplayResponse(apiDisplayResult.response);

  if (!status && result == HTTPS_SUCCESS) { // this means we already have this image stored in SPIFFS
      char szTemp[36];
#if BOARD_X_CLASS && !defined(BOARD_SEEED_RETERMINAL_E1003)
      if (DisplayedImage::exists()) {
        ImagePipeline::loadPreviousImageIntoEpd(); // decode the older image into the previous buffer of FastEPD
      }
#endif
      fixFileName(apiDisplayResult.response.filename.c_str(), szTemp);
      if (DisplayedImage::matches(szTemp)) {
        // We just displayed the same image, don't refresh the display
        Log.info("%s [%d]: The image hasn't changed since the last wakeup, don't refresh the display.\r\n", __FILE__, __LINE__);
        buffer = nullptr;
        return result;
      }
      DisplayedImage::remember(szTemp);
      Log.info("%s [%d]: Reading %s from SPIFFS\r\n", __FILE__, __LINE__, szTemp);
      size_t content_size = filesystem_read_and_allocate(szTemp, &buffer);
      if (!buffer || content_size == 0) {
        filesystem_file_delete(szTemp);
        Log_error_submit("Cached image is empty or unreadable: %s", szTemp);
        return HTTPS_WRONG_IMAGE_SIZE;
      }
      Log.info("%s [%d]: Decoding image...\r\n", __FILE__, __LINE__);
      display_show_image(buffer, content_size, true);
      free(buffer);
      buffer = nullptr;
      DisplayedImage::remember(szTemp); // current image becomes the previous image
      // Rotate NVS path keys: last ← current ← szTemp
      ImagePipeline::persistImagePaths(preferences, szTemp);
      return result;
  }

  #ifdef BOARD_TRMNL_X
// Special logic (TRMNL-X only) to download and disply the image if using a 5GHz AP
  if (status && !reset_firmware && WifiCaptivePortal.getLastCredentials().is5GHz && g_modem)
  {
    Log_info("Downloading image via modem (5 GHz path)");

    char szTemp[36];
    fixFileName(apiDisplayResult.response.filename.c_str(), szTemp);
    Log_info("Modem: saving to %s", szTemp);
    filesystem_purge_old_file(szTemp);

    String _prevPath = preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "");
    String _prevLastPath = preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    if (!_prevPath.isEmpty() && (_prevPath != String(szTemp) || _prevLastPath.isEmpty()))
      preferences.putString(PREFERENCES_LAST_PATH_KEY, _prevPath);

    // Include ID and Access Token if the image is hosted on the same server as the API
    String imgHeaders;
    if (strncmp(filename, apiDisplayInputs.baseUrl.c_str(), apiDisplayInputs.baseUrl.length()) == 0)
      imgHeaders = formatHeaders(buildImageHeaders(apiDisplayInputs));

    auto httpRes = g_modem->httpGet(String(filename), szTemp, 0, imgHeaders);
    if (!httpRes.ok)
    {
      Log_error_submit("Modem httpGet failed: %u bytes received", httpRes.bytesReceived);
      return HTTPS_REQUEST_FAILED;
    }

    int fileSize = 0;
    uint8_t* buf = display_read_file(szTemp, &fileSize);
    if (!buf || fileSize == 0)
    {
      filesystem_file_delete(szTemp);
      Log_error_submit("Modem: failed to read downloaded image from %s", szTemp);
      return HTTPS_WRONG_IMAGE_SIZE;
    }

    display_show_image(buf, fileSize, true);
    free(buf);
    DisplayedImage::remember(szTemp); // current image becomes the previous image

    ImagePipeline::persistImagePaths(preferences, szTemp);

//    new_filename = apiDisplayResult.response.filename;
//    saveCurrentFileName(new_filename);

    if (result != HTTPS_PLUGIN_NOT_ATTACHED)
      result = HTTPS_SUCCESS;
    return result;
  }
#endif // BOARD_TRMNL_X

  withHttp(
      filename,
      [&](HTTPClient *httpsp, HttpError error) -> https_request_err_e
      {
        if (error != HttpError::HTTPCLIENT_SUCCESS)
        {

          return HTTPS_UNABLE_TO_CONNECT;
        }

        HTTPClient &https = *httpsp;

        https.setTimeout(15000);
        https.setConnectTimeout(15000);

        https.addHeader("Accept-Encoding", "identity"); // Disable compression for raw image data

        // Include ID and Access Token if the image is hosted on the same server as the API
        if (strncmp(filename, apiDisplayInputs.baseUrl.c_str(), apiDisplayInputs.baseUrl.length()) == 0)
          applyHeaders(https, buildImageHeaders(apiDisplayInputs));

        if (status && !reset_firmware)
        {
          status = false;

          // The timeout will be zero if no value was returned, and in that case we just use the default timeout.
          // Otherwise, we set the requested timeout.
          uint32_t requestedTimeout = apiDisplayResult.response.image_url_timeout;
          if (requestedTimeout > 0)
          {
            // Convert from seconds to milliseconds.
            // A uint32_t should be large enough not to worry about overflow for any reasonable timeout.
            requestedTimeout *= MS_TO_S_FACTOR;
            if (requestedTimeout > UINT16_MAX)
            {
              // To avoid surprising behaviour if the server returned a timeout of more than 65 seconds
              // we will send a log message back to the server and truncate the timeout to the maximum.
              Log_info_submit("Requested image URL timeout too large (%d ms). Using maximum of %d ms.", requestedTimeout, UINT16_MAX);
              https.setTimeout(UINT16_MAX);
            }
            else
            {
              https.setTimeout(uint16_t(requestedTimeout));
            }
          }

          const char *headers[] = {"Content-Type"};
          https.collectHeaders(headers, 1);
          Log_info("GET...");
          Log_info("RSSI: %d", WiFi.RSSI());
          // start connection and send HTTP header
          int httpCode = https.GET();
          int content_size = https.getSize();
          if(httpCode == HTTP_CODE_PERMANENT_REDIRECT ||
            httpCode == HTTP_CODE_TEMPORARY_REDIRECT){
              String location = https.getLocation();
              https.end();
              String redirectUrl;
              if (location.startsWith("http://") || location.startsWith("https://")) {
                redirectUrl = location;
              } else {
                // Extract origin from the original image URL for relative redirects
                String origin = String(filename);
                int schemeEnd = origin.indexOf("://");
                if (schemeEnd != -1) {
                  int pathStart = origin.indexOf('/', schemeEnd + 3);
                  if (pathStart != -1) origin = origin.substring(0, pathStart);
                }
                redirectUrl = origin + location;
              }
              https.begin(redirectUrl);
              Log_info("Redirected to: %s", redirectUrl.c_str());
              https.setReuse(false); 
              https.setTimeout(15000);
              https.setConnectTimeout(15000);
              httpCode = https.GET();
              content_size = https.getSize();
            }
//          uint8_t *buffer_old = nullptr; // Disable partial update for now
//          int file_size_old = 0;

          // httpCode will be negative on error
          if (httpCode < 0)
          {
            Log_error_submit("[HTTPS] GET... failed, error: %d (%s)", httpCode, https.errorToString(httpCode).c_str());

            return HTTPS_REQUEST_FAILED;
          }

          // HTTP header has been send and Server response header has been handled
          Log.info("%s [%d]: [HTTPS] GET... code: %d\r\n", __FILE__, __LINE__, httpCode);
          Log.info("%s [%d]: RSSI: %d\r\n", __FILE__, __LINE__, WiFi.RSSI());
          // file found at server
          if (httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY)
          {
            Log_error_submit("[HTTPS] GET... failed, code: %d (%s)", httpCode, https.errorToString(httpCode).c_str());
            return HTTPS_REQUEST_FAILED;
          }
          
          Log.info("%s [%d]: Content size: %d\r\n", __FILE__, __LINE__, https.getSize());

          uint32_t counter = 0;
          String payload;
          long lStartTime = millis();
          if (content_size <= 0)
          {
            Log.warning("%s [%d]: Content-Length not provided, using getString()\r\n", __FILE__, __LINE__);
          }

          bool isPNG = https.header("Content-Type") == "image/png";
          bool isJPEG = https.header("Content-Type") == "image/jpeg";

          Log.info("%s [%d]: Starting a download at: %d\r\n", __FILE__, __LINE__, getTime());
          heap_caps_check_integrity_all(true);

          buffer = nullptr;
          bool buffer_malloc = false;
          if (content_size <= 0) {
          // getString() handles lack of content size and chunked transfer encoding automatically
            Log.info("%s [%d]: Downloading image with getString\r\n", __FILE__, __LINE__);
            payload = https.getString();
            counter = payload.length();
            buffer = (uint8_t *)payload.c_str();
          } else {
            Log.info("%s [%d]: Downloading image with WifiClient (stream)\r\n", __FILE__, __LINE__);
            counter = https.getSize();
            if (counter && counter <= MAX_IMAGE_SIZE) {
              WiFiClient *stream = https.getStreamPtr();
              int iCount = 0;

              buffer = (uint8_t *)malloc(counter);
              if (buffer) {
                buffer_malloc = true;
                while (iCount < counter && millis() < (lStartTime + API_FIRST_RETRY*1000)) {
                  if (stream->available()) {
                    buffer[iCount++] = stream->read();
                    lStartTime = millis(); // reset start time
                  } else { // 15 seconds with no activity => stop trying
                    vTaskDelay(1); // yield to allow time for the data to arrive
                  }
                }
              } // if buffer
              stream->stop(); // Important! If you don't do this, WiFi will have a memory exception later
              if (millis() > (lStartTime + API_FIRST_RETRY*1000)) { // we timed out
                  Log_error_submit("Receiving failed; download timed out. Image size = %d", counter);
                  return HTTPS_TIMED_OUT;
              }
            }
          } // if payload size is non-zero
          Log.info("%s [%d]: %d bytes received in %d milliseconds\r\n", __FILE__, __LINE__, counter, (int)(millis() - lStartTime));

          if (counter == 0)
          {
            Log_error_submit("Receiving failed. No data received");
            return HTTPS_WRONG_IMAGE_SIZE;
          }

          if (counter > MAX_IMAGE_SIZE)
          {
            Log_error_submit("Receiving failed; file size too big: %d", counter);
            return HTTPS_IMAGE_FILE_TOO_BIG;
          }

          if (buffer == NULL)
          {
            Log_error_submit("Failed to allocate %d bytes for image buffer", counter);
            return HTTPS_OUT_OF_MEMORY;
          }

          //memcpy(buffer, payload.c_str(), counter);
          content_size = counter;

          if (counter >= 2 && buffer[0] == 'B' && buffer[1] == 'M')
          {
            isPNG = false;
            Log.info("BMP file detected");
          }

          submitStoredLogs();

          WiFi.disconnect(true); // no need for WiFi, save power starting here
          Log.info("%s [%d]: Received successfully; WiFi off.\r\n", __FILE__, __LINE__);

          bool image_reverse = false;
          if (isPNG || isJPEG)
          {
            char szTemp[36];
            fixFileName(apiDisplayResult.response.filename.c_str(), szTemp);
            Log.info("%s [%d]: Writing %s to SPIFFS\r\n", __FILE__, __LINE__, szTemp);
            filesystem_purge_old_file(szTemp); // try to delete the old version or older than 24h
            ImagePipeline::writeImageToFile(szTemp, buffer, content_size);
            Log.info("%s [%d]: Decoding %s\r\n", __FILE__, __LINE__, (isPNG) ? "png" : "jpeg");
            display_show_image(buffer, content_size, true);
            DisplayedImage::remember(szTemp); // current image becomes the previous image
            if (buffer_malloc) {
              Log.info("%s [%d]: Freeing the image buffer we allocated\r\n", __FILE__, __LINE__);
              free(buffer);
            }
            buffer = nullptr;
            png_res = PNG_NO_ERR; // DEBUG
            ImagePipeline::persistImagePaths(preferences, szTemp);
          }
          else
          {
            bmp_res = parseBMPHeader(buffer, image_reverse);
            Log.info("%s [%d]: BMP Parsing result: %d\r\n", __FILE__, __LINE__, bmp_res);
          }
          Serial.println();
          String error = "";
         // uint8_t *imagePointer = buffer;
//          uint8_t *imagePointer = (decodedPng == nullptr) ? buffer : decodedPng;
        //  bool lastImageExists = filesystem_file_exists("/last.bmp") || filesystem_file_exists("/last.png");

          switch (png_res)
          {
          case PNG_NO_ERR:
          {

           // Log.info("Free heap at before display - %d", ESP.getMaxAllocHeap());
           // display_show_image(imagePointer, image_reverse, isPNG);

            // Using filename from API response
            new_filename = apiDisplayResult.response.filename;

            // Print the extracted string
            Log.info("%s [%d]: New filename - %s\r\n", __FILE__, __LINE__, new_filename.c_str());

            if (result != HTTPS_PLUGIN_NOT_ATTACHED)
              result = HTTPS_SUCCESS;
          }
          break;
          case PNG_WRONG_FORMAT:
          {
            error = "Wrong image format. Did not pass signature check";
          }
          break;
          case PNG_BAD_SIZE:
          {
            error = "IMAGE width, height or size are invalid";
          }
          break;
          case PNG_DECODE_ERR:
          {
            error = "could not decode png image";
          }
          break;
          case PNG_MALLOC_FAILED:
          {
            error = "could not allocate memory for png image decoder";
          }
          break;
          default:
            break;
          }

          switch (bmp_res)
          {
          case BMP_NO_ERR:
          {
            if (!filesystem_file_exists("/current.png"))
            {
              ImagePipeline::writeImageToFile("/current.bmp", buffer, content_size);
            }
            Log.info("Free heap at before display - %d", ESP.getMaxAllocHeap());
            display_show_image(buffer, content_size, true);
            {
              char szTemp[36];
              fixFileName(apiDisplayResult.response.filename.c_str(), szTemp);
              DisplayedImage::remember(szTemp);
            }

            if (buffer_malloc) {
              Log.info("%s [%d]: Freeing the image buffer we allocated\r\n", __FILE__, __LINE__);
              free(buffer);
            }
            buffer = nullptr;

            // Using filename from API response
            new_filename = apiDisplayResult.response.filename;

            // Print the extracted string
            Log.info("%s [%d]: New filename - %s\r\n", __FILE__, __LINE__, new_filename.c_str());

            if (result != HTTPS_PLUGIN_NOT_ATTACHED)
              result = HTTPS_SUCCESS;
          }
          break;
          case BMP_NOT_BMP:
          {
            error = "First two header bytes are invalid!";
          }
          break;
          case BMP_BAD_SIZE:
          {
            error = "BMP width, height or size are invalid";
          }
          break;
          case BMP_COLOR_SCHEME_FAILED:
          {
            error = "BMP color scheme is invalid";
          }
          break;
          case BMP_INVALID_OFFSET:
          {
            error = "BMP header offset is invalid";
          }
          break;
          default:
            break;
          }

          if (isPNG && png_res != PNG_NO_ERR)
          {
            char szTemp[36];
            fixFileName(apiDisplayResult.response.filename.c_str(), szTemp);
            filesystem_file_delete(szTemp);
            Log_error_submit("error parsing image file - %s", error.c_str());

            return HTTPS_WRONG_IMAGE_FORMAT;
          }
        }

        return result;
      });

  if (result == HTTPS_UNABLE_TO_CONNECT)
  {
    Log_error_submit("unable to connect");
  }

  Log_info("Returned result - %s", szHTTPErrors[result]);

  return result;
}

https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse)
{
  https_request_err_e result = HTTPS_NO_ERR;
  int file_size = 0;

#ifdef BOARD_TRMNL_X
  // Set touchbar mode and persist to NVS
  if (apiResponse.touchbar_mode.length() == 0 || touchbar_tap_mode == (apiResponse.touchbar_mode == "tap")) {
    Log.info("%s [%d]: No need to update touchbar mode\r\n", __FILE__, __LINE__);
  }
  else {
    touchbar_tap_mode = (apiResponse.touchbar_mode == "tap");
    preferences.putBool(PREFERENCES_TOUCHBAR_MODE_KEY, touchbar_tap_mode);
  }
#endif // BOARD_TRMNL_X

  if (special_function == SF_NONE)
  {
    uint64_t request_status = apiResponse.status;
    Log.info("%s [%d]: status: %d\r\n", __FILE__, __LINE__, request_status);
    switch (request_status)
    {
    case 0:
    {
      String image_url = apiResponse.image_url;
      uint64_t rate = apiResponse.refresh_rate;
      reset_firmware = apiResponse.reset_firmware;

      bool sleep_5_seconds = false;

      writeSpecialFunction(apiResponse.special_function);

      if (image_url.length() > 0)
      {
        Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
        Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

        image_url.toCharArray(filename, image_url.length() + 1);
        // check if plugin is applied
        bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
        Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

        if (apiResponse.filename == "empty_state")
        {
          Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
          if (!flag)
          {
            // draw received logo
            status = true;
            // set flag to true
            if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != true) // check the flag to avoid the re-writing
            {
              bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
              if (res)
                Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
              else
                Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
            }
          }
          else
          {
            // don't draw received logo
            status = false;
          }
          // sleep 5 seconds
          sleep_5_seconds = true;
        }
        else
        {
          Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
          if (flag)
          {
            if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != false) // check the flag to avoid the re-writing
            {
              bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
              if (res)
                Log.info("%s [%d]: Flag written false successfully\r\n", __FILE__, __LINE__);
              else
                Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
            }
          }
          // Using filename from API response
          new_filename = apiResponse.filename;

          // Print the extracted string
          Log.info("%s [%d]: New filename - %s\r\n", __FILE__, __LINE__, new_filename.c_str());
          if (!ImagePipeline::cachedFileExists(new_filename))
          {
            Log.info("%s [%d]: New image. Download and show it.\r\n", __FILE__, __LINE__);
            status = true;
            bUsedCachedImage = false;
          }
          else
          {
            Log.info("%s [%d]: Old image. Read from FLASH and show it.\r\n", __FILE__, __LINE__);
            status = false;
            bUsedCachedImage = true;
            result = HTTPS_SUCCESS;
          }
        }
      }
      Log.info("%s [%d]: refresh_rate: %d\r\n", __FILE__, __LINE__, rate);
      if (rate != preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP))
      {
        Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, rate);
        preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, rate);
      }

      if (reset_firmware)
      {
        Log.info("%s [%d]: Reset status is true\r\n", __FILE__, __LINE__);
      }

      if (apiResponse.update_firmware && apiResponse.firmware_url.length() > 0)
        result = HTTPS_SUCCESS;
      if (reset_firmware)
        result = HTTPS_RESET;
      if (sleep_5_seconds)
        result = HTTPS_PLUGIN_NOT_ATTACHED;
      Log.info("%s [%d]: result - %s\r\n", __FILE__, __LINE__, szHTTPErrors[result]);
    }
    break;
    case 202:
    {
      result = HTTPS_NO_REGISTER;
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_NOT_CONNECTED);
      size_t result = preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_NOT_CONNECTED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      status = false;
    }
    break;
    case 500:
    {
      result = HTTPS_RESET;
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_NOT_CONNECTED);
      preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_NOT_CONNECTED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      status = false;
    }
    break;

    default:
      break;
    }
  }
  else if (special_function != SF_NONE)
  {
    uint64_t request_status = apiResponse.status;
    Log.info("%s [%d]: status: %d\r\n", __FILE__, __LINE__, request_status);
    switch (request_status)
    {
    case 0:
    {
      switch (special_function)
      {
      case SF_IDENTIFY:
      {
        String action = apiResponse.action;
        if (action.equals("identify"))
        {
          Log.info("%s [%d]:Identify success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          if (image_url.length() > 0)
          {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state")
            {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag)
              {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              else
              {
                status = false;
              }
            }
            else
            {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag)
              {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != false) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
                  if (res)
                    Log.info("%s [%d]: Flag written false successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              status = true;
            }
          }
        }
        else
        {
          Log.error("%s [%d]: identify failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_SLEEP:
      {
        String action = apiResponse.action;
        if (action.equals("sleep"))
        {
          uint64_t rate = apiResponse.refresh_rate;
          Log.info("%s [%d]: refresh_rate: %d\r\n", __FILE__, __LINE__, rate);
          if (rate != preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP))
          {
            Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, rate);
            preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, rate);
            Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
          }
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: sleep success\r\n", __FILE__, __LINE__);
        }
        else
        {
          Log.error("%s [%d]: sleep failed\r\n", __FILE__, __LINE__);
          // need to add error
        }
      }
      break;
      case SF_ADD_WIFI:
      {
        String action = apiResponse.action;
        if (action.equals("add_wifi"))
        {
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: Add wifi success\r\n", __FILE__, __LINE__);
        }
        else
        {
          Log.error("%s [%d]: Add wifi failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_RESTART_PLAYLIST:
      {
        String action = apiResponse.action;
        if (action.equals("restart_playlist"))
        {
          Log.info("%s [%d]:Restart playlist success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          if (image_url.length() > 0)
          {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state")
            {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag)
              {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              else
              {
                // don't draw received logo
                status = false;
              }
            }
            else
            {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag)
              {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != false) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
                  if (res)
                    Log.info("%s [%d]: Flag written false successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              status = true;
            }
          }
        }
        else
        {
          Log.error("%s [%d]: Restart playlist failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_REWIND:
      {
        String action = apiResponse.action;
        if (action.equals("rewind"))
        {
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: rewind success\r\n", __FILE__, __LINE__);

          bool image_reverse = false;
          bool file_check_bmp = true;
          image_err_e image_proccess_response = PNG_WRONG_FORMAT;
          bmp_err_e bmp_proccess_response = BMP_NOT_BMP;

          // showMessageWithLogo(MSG_FORMAT_ERROR);
          String last_dot_file = filesystem_file_exists("/last.bmp") ? "/last.bmp" : "/last.png";
          if (last_dot_file == "/last.bmp")
          {
            Log.info("Rewind BMP\n\r");
            buffer = (uint8_t *)malloc(DISPLAY_BMP_IMAGE_SIZE);
            file_check_bmp = filesystem_read_from_file(last_dot_file.c_str(), buffer, DISPLAY_BMP_IMAGE_SIZE);
            bmp_proccess_response = parseBMPHeader(buffer, image_reverse);
          }
          else if (last_dot_file == "/last.png")
          {
            Log.info("Rewind PNG\n\r");
            buffer = display_read_file(last_dot_file.c_str(), &file_size);
            image_proccess_response = PNG_NO_ERR; // DEBUG
          }

          if (file_check_bmp)
          {
            switch (image_proccess_response)
            {
            case PNG_NO_ERR:
            {
              Log.info("Showing image\n\r");
              display_show_image(buffer, file_size, true);
              need_to_refresh_display = 1;
            }
            break;
            default:
            {
            }
            break;
            }
            switch (bmp_proccess_response)
            {
            case BMP_NO_ERR:
            {
              Log.info("Showing image\n\r");
              display_show_image(buffer, DISPLAY_BMP_IMAGE_SIZE, true);
              need_to_refresh_display = 1;
            }
            break;
            default:
            {
            }
            break;
            }
          }
          else
          {
            if (buffer) {
              free(buffer);
              buffer = nullptr;
            }
            showMessageWithLogo(MSG_FORMAT_ERROR);
          }
        }
        else
        {
          Log.error("%s [%d]: rewind failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_SEND_TO_ME:
      {
        String action = apiResponse.action;

        if (action.equals("send_to_me"))
        {
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: send_to_me success\r\n", __FILE__, __LINE__);

          bool image_reverse = false;

          if (!filesystem_file_exists("/current.bmp") && !filesystem_file_exists("/current.png"))
          {
            Log.info("%s [%d]: No current image!\r\n", __FILE__, __LINE__);
            if (buffer) {
              free(buffer);
              buffer = nullptr;
            }
            return HTTPS_WRONG_IMAGE_FORMAT;
          }

          if (filesystem_file_exists("/current.bmp"))
          {
            Log.info("%s [%d]: send_to_me BMP\r\n", __FILE__, __LINE__);
            buffer = (uint8_t *)malloc(DISPLAY_BMP_IMAGE_SIZE);

            if (!filesystem_read_from_file("/current.bmp", buffer, DISPLAY_BMP_IMAGE_SIZE))
            {
              free(buffer);
              buffer = nullptr;
              Log_error_submit("Error reading image!");
              return HTTPS_WRONG_IMAGE_FORMAT;
            }

            bmp_err_e bmp_parse_result = parseBMPHeader(buffer, image_reverse);
            if (bmp_parse_result != BMP_NO_ERR)
            {
              free(buffer);
              buffer = nullptr;
              Log_error_submit("Error parsing BMP header, code: %d", bmp_parse_result);
              return HTTPS_WRONG_IMAGE_FORMAT;
            }
          }
          else if (filesystem_file_exists("/current.png"))
          {
            Log.info("%s [%d]: send_to_me PNG\r\n", __FILE__, __LINE__);
            image_err_e png_parse_result = PNG_NO_ERR; // DEBUG
            buffer = display_read_file("/current.png", &file_size);
// Disable partial update for now
//            if (filesystem_file_exists("/last.png")) {
//                buffer_old = display_read_file("/last.png", &file_size_old);
//                Log.info("%s [%d]: loading last PNG for partial update\r\n", __FILE__, __LINE__);
//            }
            if (png_parse_result != PNG_NO_ERR)
            {
              Log_error_submit("Error parsing PNG header, code: %d", png_parse_result);
              if (buffer) {
                free(buffer);
                buffer = nullptr;
              }
              return HTTPS_WRONG_IMAGE_FORMAT;
            }
          }

          Log.info("Showing image\n\r");
          display_show_image(buffer, file_size, true);
          need_to_refresh_display = 1;
          if (buffer) {
            free(buffer);
            buffer = nullptr;
          }
        }
        else
        {
          Log.error("%s [%d]: send_to_me failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_GUEST_MODE:
      {
        String action = apiResponse.action;
        if (action.equals("guest_mode"))
        {
          Log.info("%s [%d]:Guest Mode success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          uint64_t rate = apiResponse.refresh_rate;
          if (image_url.length() > 0)
          {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state")
            {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag)
              {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              else
              {
                // don't draw received logo
                status = false;
              }
            }
            else
            {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag)
              {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != false) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
                  if (res)
                    Log.info("%s [%d]: Flag written false successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              status = true;
            }
          }
          preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, rate);
        }
        else
        {
          Log.error("%s [%d]: Guest Mode failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      default:
        break;
      }
    }
    break;
    case 202:
    {
      result = HTTPS_NO_REGISTER;
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_NOT_CONNECTED);
      preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_NOT_CONNECTED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      status = false;
    }
    break;
    case 500:
    {
      result = HTTPS_RESET;
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_NOT_CONNECTED);
      preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_NOT_CONNECTED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      status = false;
    }
    break;

    default:
      break;
    }
  }
  return result;
}
