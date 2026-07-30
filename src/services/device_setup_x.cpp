#include <services/device_setup.h>

#ifdef INCLUDE_DEVICE_SETUP_X

#include <Arduino.h>
#include <WifiCaptive.h>
#include <api-client/request_headers.h>
#include <api_response_parsing.h>
#include <config.h>
#include <filesystem.h>
#include <trmnl_log.h>
#include <types.h>

#include "modem.h"

bool DeviceSetupX::is5Ghz() { return WifiCaptivePortal.getLastCredentials().is5GHz && _modem; }

ApiSetupResult DeviceSetupX::callSetupApi(ApiSetupInputs &inputs) {
  if (!is5Ghz()) {
    return DeviceSetup::callSetupApi(inputs);
  }

  Log_info("API setup via modem (5 GHz path)");
  String reqHeaders = formatHeaders(buildSetupHeaders(inputs));
  auto httpRes = _modem->httpGet(inputs.baseUrl + "/api/setup", "", 0, reqHeaders);
  if (!httpRes.ok) {
    Log_error_submit("[MODEM] /api/setup request failed (%u bytes received)", httpRes.bytesReceived);
    return {HTTPS_RESPONSE_CODE_INVALID, {}, "Modem httpGet failed"};
  }

  auto apiResp = parseResponse_apiSetup(httpRes.body);
  if (apiResp.outcome == ApiSetupOutcome::DeserializationError) {
    return {HTTPS_JSON_PARSING_ERR, {}, "JSON deserialization error"};
  }
  return {HTTPS_NO_ERR, apiResp, ""};
}

void DeviceSetupX::downloadSetupImage() {
  if (!is5Ghz()) {
    DeviceSetup::downloadSetupImage();
    return;
  }

  Log_info("Downloading setup image via modem (5 GHz path)");
  _result.outcome = DeviceSetupOutcome::Success;
  auto httpRes = _modem->httpGet(_result.imageUrl, "/logo.bmp");
  if (!httpRes.ok || httpRes.bytesReceived != DISPLAY_BMP_IMAGE_SIZE) {
    Log_error_submit("Modem logo download failed: ok=%d bytes=%u expected=%u", httpRes.ok, httpRes.bytesReceived,
                     DISPLAY_BMP_IMAGE_SIZE);
    filesystem_file_delete("/logo.bmp");
    _result.outcome = DeviceSetupOutcome::ImageDownloadError;
  }
  // Show the FRIENDLY_ID screen even when the download failed, matching the
  // original modem-path behavior.
  _result.friendlyId = _persistence.readString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
  _result.showSetupScreen = true;
}

void DeviceSetupX::handleInvalidImage(uint32_t bytesRead) {
  _result.outcome = DeviceSetupOutcome::ImageInvalidError;
  Log_error_submit("Setup image: unexpected format or size. Read: %d bytes (expected BMP %d)", bytesRead,
                   DISPLAY_BMP_IMAGE_SIZE);
}

#endif // INCLUDE_DEVICE_SETUP_X
