#include <Arduino.h>
#include <ArduinoLog.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <api-client/setup.h>
#include <config.h>
#include <device_id.h>
#include <filesystem.h>
#include <globals.h>
#include <http_client.h>
#include <inttypes.h>
#include <preferences_persistence.h>
#include <services/device_setup.h>
#include <trmnl_log.h>
#include <types.h>

DeviceSetupResult DeviceSetup::perform() {
  _result = DeviceSetupResult();
  performApiSetup();
  if (_result.outcome == DeviceSetupOutcome::Success) {
    if (_result.imageUrl.isEmpty()) {
      _result.friendlyId = _persistence.readString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
      _result.showSetupScreen = true;
    } else {
      downloadSetupImage();
    }
  }
  return _result;
}

ApiSetupResult DeviceSetup::callSetupApi(ApiSetupInputs &inputs) { return fetchApiSetup(inputs); }

void DeviceSetup::handleInvalidImage(uint32_t bytesRead) {
  _result.errorScreen = (WiFi.RSSI() > WIFI_CONNECTION_RSSI) ? API_SIZE_ERROR : WIFI_WEAK;
  _result.outcome = DeviceSetupOutcome::ImageInvalidError;
  Log_error_submit("Receiving failed. Read: %" PRIu32, bytesRead);
}

/**
 * @brief Performs API setup call to get credentials and image URL, filling
 *        _result; Success means the image download should proceed
 */
void DeviceSetup::performApiSetup() {
  // Set up the API inputs
  ApiSetupInputs inputs;
  inputs.baseUrl = _persistence.readString(PREFERENCES_API_URL, API_BASE_URL);
  inputs.macAddress = device_mac_address();
  inputs.firmwareVersion = FW_VERSION_STRING;
  inputs.model = String(DEVICE_MODEL);

  Log.info("%s [%d]: [HTTPS] begin /api/setup ...\r\n", __FILE__, __LINE__);
  Log.info("%s [%d]: RSSI: %d\r\n", __FILE__, __LINE__, WiFi.RSSI());
  Log.info("%s [%d]: Device MAC address: %s\r\n", __FILE__, __LINE__, inputs.macAddress.c_str());

  // Call the API client
  ApiSetupResult apiResult = callSetupApi(inputs);

  // Handle connection errors
  if (apiResult.error == HTTPS_UNABLE_TO_CONNECT) {
    _result.errorScreen = WIFI_INTERNAL_ERROR;
    _result.outcome = DeviceSetupOutcome::ConnectionError;
    Log_error_submit("[HTTPS] %s", apiResult.error_detail.c_str());
    return;
  }

  // Handle JSON parsing errors
  if (apiResult.error == HTTPS_JSON_PARSING_ERR) {
    _result.outcome = DeviceSetupOutcome::ParsingError;
    Log.error("%s [%d]: JSON deserialization error.\r\n", __FILE__, __LINE__);
    return;
  }

  // Handle HTTP request errors
  if (apiResult.error != HTTPS_NO_ERR) {
    _result.errorScreen = (WiFi.RSSI() > WIFI_CONNECTION_RSSI) ? API_SETUP_FAILED : WIFI_WEAK;
    _result.outcome = DeviceSetupOutcome::RequestError;
    Log_error_submit("[HTTPS] Request failed: %s", apiResult.error_detail.c_str());
    return;
  }

  // Process the successful response
  auto &apiResponse = apiResult.response;
  uint16_t url_status = apiResponse.status;

  Log.info("%s [%d]: GET... code: %d\r\n", __FILE__, __LINE__, url_status);

  if (url_status == 200) {
    Log.info("%s [%d]: status OK.\r\n", __FILE__, __LINE__);

    String api_key = apiResponse.api_key;
    Log.info("%s [%d]: API key - %s\r\n", __FILE__, __LINE__, api_key.c_str());
    _persistence.writeString(PREFERENCES_API_KEY, api_key.c_str());
    Log.info("%s [%d]: api key saved in the preferences\r\n", __FILE__, __LINE__);

    String friendly_id = apiResponse.friendly_id;
    Log.info("%s [%d]: friendly ID - %s\r\n", __FILE__, __LINE__, friendly_id.c_str());
    _persistence.writeString(PREFERENCES_FRIENDLY_ID, friendly_id.c_str());
    Log.info("%s [%d]: friendly ID saved in the preferences\r\n", __FILE__, __LINE__);

    _result.imageUrl = apiResponse.image_url;
    Log.info("%s [%d]: image_url - %s\r\n", __FILE__, __LINE__, _result.imageUrl.c_str());

    _result.message = apiResponse.message;
    Log.info("%s [%d]: message - %s\r\n", __FILE__, __LINE__, _result.message.c_str());

    _result.outcome = DeviceSetupOutcome::Success;
  } else if (url_status == 404) {
    Log_info("MAC Address is not registered on server");

    _result.apiResponse = apiResponse;
    _result.shouldGoToSleep = true;
    _result.outcome = DeviceSetupOutcome::MacNotRegistered;
  } else {
    Log.info("%s [%d]: status FAIL.\r\n", __FILE__, __LINE__);
    _result.outcome = DeviceSetupOutcome::ApiStatusError;
  }
}

/**
 * @brief Downloads the setup image from the API response, filling _result's
 *        outcome and FRIENDLY_ID screen fields
 */
void DeviceSetup::downloadSetupImage() {
  Log.info("%s [%d]: filename - %s\r\n", __FILE__, __LINE__, _result.imageUrl.c_str());

  _result.outcome = DeviceSetupOutcome::ImageDownloadError;

  withHttp(_result.imageUrl, [&](HTTPClient *https, HttpError error) -> bool {
    if (error != HttpError::HTTPCLIENT_SUCCESS) {
      _result.errorScreen = (WiFi.RSSI() > WIFI_CONNECTION_RSSI) ? API_IMAGE_DOWNLOAD_ERROR : WIFI_WEAK;
      Log_error_submit("[HTTPS] Unable to connect");
      return false;
    }

    https->setTimeout(15000);
    https->setConnectTimeout(15000);

    Log.info("%s [%d]: [HTTPS] Request to %s\r\n", __FILE__, __LINE__, _result.imageUrl.c_str());
    Log.info("%s [%d]: [HTTPS] GET..\r\n", __FILE__, __LINE__);

    int httpCode = https->GET();

    if (httpCode == HTTP_CODE_PERMANENT_REDIRECT || httpCode == HTTP_CODE_TEMPORARY_REDIRECT) {
      https->end();
      https->begin(https->getLocation());
      Log_info("Redirected to: %s", https->getLocation().c_str());
      https->setTimeout(15000);
      https->setConnectTimeout(15000);
      httpCode = https->GET();
    }

    // httpCode will be negative on error
    if (httpCode <= 0) {
      _result.errorScreen = (WiFi.RSSI() > WIFI_CONNECTION_RSSI) ? API_IMAGE_DOWNLOAD_ERROR : WIFI_WEAK;
      Log_error_submit("[HTTPS] GET... failed, error: %s", https->errorToString(httpCode).c_str());
      return false;
    }

    // HTTP header has been send and Server response header has been handled
    Log.info("%s [%d]: [HTTPS] GET... code: %d\r\n", __FILE__, __LINE__, httpCode);

    // file found at server
    if (httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY) {
      _result.errorScreen = (WiFi.RSSI() > WIFI_CONNECTION_RSSI) ? API_IMAGE_DOWNLOAD_ERROR : WIFI_WEAK;
      Log_error_submit("[HTTPS] GET... failed, error: %s", https->errorToString(httpCode).c_str());
      return false;
    }

    Log.info("%s [%d]: Content size: %d\r\n", __FILE__, __LINE__, https->getSize());

    WiFiClient *stream = https->getStreamPtr();

    // Read and save image data to buffer (BMP or PNG)
    uint32_t counter = 0;
    int contentSize = https->getSize();
    uint8_t *imageBuffer = (uint8_t *)malloc(contentSize > 0 ? contentSize : 1);
    if (imageBuffer == nullptr) {
      Log_error_submit("Failed to allocate buffer for setup image (%d bytes)", contentSize);
      return false;
    }
    if (stream->available() && contentSize > 0) {
      counter = downloadStream(stream, contentSize, imageBuffer);
    }

    if (counter == DISPLAY_BMP_IMAGE_SIZE) {
      Log.info("%s [%d]: Received successfully\r\n", __FILE__, __LINE__);

      writeImageToFile("/logo.bmp", imageBuffer, DEFAULT_IMAGE_SIZE);
      free(imageBuffer);

      _result.friendlyId = _persistence.readString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
      _result.showSetupScreen = true;
      _result.outcome = DeviceSetupOutcome::Success;
    } else if (counter == (uint32_t)contentSize && counter >= 4 && imageBuffer[0] == 0x89 && imageBuffer[1] == 'P' &&
               imageBuffer[2] == 'N' && imageBuffer[3] == 'G') {
      Log.info("%s [%d]: Received PNG setup logo (%d bytes)\r\n", __FILE__, __LINE__, counter);
      writeImageToFile("/logo.png", imageBuffer, counter);
      free(imageBuffer);

      _result.friendlyId = _persistence.readString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
      _result.showSetupScreen = true;
      _result.outcome = DeviceSetupOutcome::Success;
    } else {
      free(imageBuffer);
      handleInvalidImage(counter);
    }
    return true;
  });
}

#ifdef INCLUDE_DEVICE_SETUP_X
static DeviceSetupX deviceSetupService(preferencesPersistence, g_modem);
#else
static DeviceSetup deviceSetupService(preferencesPersistence);
#endif

DeviceSetup &deviceSetup() { return deviceSetupService; }
