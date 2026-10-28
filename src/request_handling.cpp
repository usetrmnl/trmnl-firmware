#include <Arduino.h>
#include <ArduinoLog.h>
#include <WiFi.h>
#include <bl.h>
#include <config.h>
#include <display.h>
#include <globals.h>
#include <http_result_handling.h>
#include <request_handling.h>
#include <status_screens.h>
#include <touch_i2c_lock.h>
#include <trmnl_log.h>

void showImageTooBigIfNeeded(https_request_err_e request_result)
{
  if (request_result == HTTPS_IMAGE_FILE_TOO_BIG)
  {
    iqs323_task_i2c_lock();
    showMessageWithLogo(MSG_TOO_BIG);
    iqs323_task_i2c_unlock();
  }
}

void applyRetryPolicy(https_request_err_e request_result, bool should_show_error_now)
{
  if (!preferences.isKey(PREFERENCES_CONNECT_API_RETRY_COUNT))
  {
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, 1);
  }

  uint8_t retries = preferences.getInt(PREFERENCES_CONNECT_API_RETRY_COUNT);
  RequestFailureAction failure_action = requestFailureAction(request_result, should_show_error_now, retries);

  if (failure_action == RequestFailureAction::ResetCount)
  {
    if (request_result == HTTPS_IMAGE_DOWNLOAD_FAILED) // API answered, so no backoff
      Log_info("Image download failed; showing error and sleeping for the API refresh interval (%d s)", refreshInterval.seconds());
    else
      Log_info("Connection done successfully or WiFi failed. Retries counter reset.");
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, 1);
  }
  else if (failure_action == RequestFailureAction::ShowNow)
  {
    Log.info("%s [%d]: Manual wake - showing error now, no retry scheduled\r\n", __FILE__, __LINE__);
  }
  else
  {
    uint32_t retry_sleep = refreshInterval.applyApiRetry(retries);
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, retries + 1);

    if (failure_action == RequestFailureAction::StayQuiet)
    {
      Log.info("%s [%d]: retry: %d - time to sleep: %d\r\n", __FILE__, __LINE__, retries, retry_sleep);
      iqs323_task_i2c_lock();
      display_sleep();
      goToSleep();
    }
    Log.info("%s [%d]: Max retries done. Time to sleep: %d\r\n", __FILE__, __LINE__, retry_sleep);
  }
}

void showFriendlyIdIfUnregistered(https_request_err_e request_result)
{
  if (request_result == HTTPS_NO_REGISTER && need_to_refresh_display == 1)
  {
    // show the image
    String friendly_id = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
    showMessageWithLogo(FRIENDLY_ID, friendly_id, true, "", String(message_buffer));
    need_to_refresh_display = 0;
  }
}

void resetIfRequested(https_request_err_e request_result)
{
  // reset checking
  if (request_result == HTTPS_RESET)
  {
    Log.info("%s [%d]: Device reseting...\r\n", __FILE__, __LINE__);
    resetDeviceCredentials();
  }
}

void updateFirmwareIfDue(void)
{
  // OTA update checking
  if (firmwareUpdateService.isUpdateDue(
          apiDisplayResult.response.update_firmware,
          apiDisplayResult.response.firmware_url))
  {
    showMessageWithLogo(FW_UPDATE);
    FirmwareUpdateResult firmwareUpdateResult = firmwareUpdateService.performUpdate();
    if (firmwareUpdateResult.updated)
    {
      showMessageWithLogo(FW_UPDATE_SUCCESS);
      ESP.restart();
    }
    if (firmwareUpdateResult.failureMessage != NONE)
      showMessageWithLogo(firmwareUpdateResult.failureMessage);
  }
}

void showRequestResult(https_request_err_e request_result)
{
  if (request_result == HTTPS_PLUGIN_NOT_ATTACHED)
    refreshInterval.applyFastPoll();
  MSG error_screen = requestErrorScreen(request_result, WiFi.RSSI() <= WIFI_CONNECTION_RSSI);
  if (error_screen != NONE)
    showMessageWithLogo(error_screen);
}
