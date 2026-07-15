#include <firmware_update.h>
#include <ota_schedule.h>
#include <http_client.h>
#include <trmnl_log.h>
#include <Update.h>
#include <WiFi.h>
#include <wifi_network.h>
#include <HTTPClient.h>
#include "esp_ota_ops.h"

#ifdef BOARD_TRMNL_X
#include <modem.h>
#include <WifiCaptive.h>
extern Modem *g_modem;
#endif

FirmwareUpdateService::FirmwareUpdateService(
    Persistence &persistence,
    GetTimeFn getTime,
    int32_t wifiConnectionRssiThreshold)
    : _persistence(persistence),
      _getTime(std::move(getTime)),
      _wifiConnectionRssiThreshold(wifiConnectionRssiThreshold),
      _firmwareUrl{0}
{
}

bool FirmwareUpdateService::validateFirmwareUpdatePossible(bool update_firmware, const String &firmware_url)
{
  if (!update_firmware)
    return false;

  Log_info("%s [%d]: update firmware. Check URL\r\n", __FILE__, __LINE__);
  if (firmware_url.length() == 0)
  {
    Log_error("%s [%d]: Empty URL\r\n", __FILE__, __LINE__);
    return false;
  }

  firmware_url.toCharArray(_firmwareUrl, sizeof(_firmwareUrl));
  Log_info("%s [%d]: firmware_url: %s\r\n", __FILE__, __LINE__, _firmwareUrl);

  uint32_t now = _getTime();
  if (!otaAttemptDue(now, otaLastAttempt(_persistence)))
  {
    Log_info("%s [%d]: Last OTA attempt was < 24h ago, skipping...\r\n", __FILE__, __LINE__);
    return false;
  }

  Log_info("%s [%d]: Last OTA attempt was > 24h ago, proceeding with download...\r\n", __FILE__, __LINE__);
  return true;
}

bool FirmwareUpdateService::performFirmwareUpdate()
{
#ifdef BOARD_TRMNL_X
  if (g_modem && WifiCaptivePortal.getLastCredentials().is5GHz)
  {
    Log_info("%s [%d]: Starting modem OTA download...\r\n", __FILE__, __LINE__);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);
    if (!update_partition)
    {
      Log_fatal("%s [%d]: No OTA partition available\r\n", __FILE__, __LINE__);
      _failureMessage = FW_UPDATE_FAILED;
      return false;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK)
    {
      Log_fatal("%s [%d]: esp_ota_begin failed: %s\r\n", __FILE__, __LINE__, esp_err_to_name(err));
      _failureMessage = FW_UPDATE_FAILED;
      return false;
    }

    bool write_ok = true;
    auto result = g_modem->httpGet(
        String(_firmwareUrl),
        [&](const uint8_t *data, size_t len) -> bool
        {
          esp_err_t e = esp_ota_write(ota_handle, data, len);
          if (e != ESP_OK)
          {
            Log_fatal("%s [%d]: esp_ota_write failed: %s\r\n", __FILE__, __LINE__, esp_err_to_name(e));
            write_ok = false;
            return false;
          }
          return true;
        },
        0,
        "",
        120000UL);

    if (!result.ok || !write_ok)
    {
      esp_ota_abort(ota_handle);
      Log_fatal("%s [%d]: Modem OTA download failed\r\n", __FILE__, __LINE__);
      _failureMessage = FW_UPDATE_FAILED;
      return false;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
      Log_fatal("%s [%d]: esp_ota_end failed: %s\r\n", __FILE__, __LINE__, esp_err_to_name(err));
      _failureMessage = FW_UPDATE_FAILED;
      return false;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
      Log_fatal("%s [%d]: esp_ota_set_boot_partition failed: %s\r\n", __FILE__, __LINE__, esp_err_to_name(err));
      _failureMessage = FW_UPDATE_FAILED;
      return false;
    }

    Log_info("%s [%d]: Modem OTA successful. Rebooting...\r\n", __FILE__, __LINE__);
    return true;
  }
#endif

  if (!ensureWifiConnected())
  {
    Log_fatal("%s [%d]: Unable to reconnect WiFi for firmware update\r\n", __FILE__, __LINE__);
    _failureMessage = API_FIRMWARE_UPDATE_ERROR;
    return false;
  }

  bool ota_ok = false;
  withHttp(_firmwareUrl, [&](HTTPClient *https, HttpError errorCode) -> bool
           {
             if (errorCode != HttpError::HTTPCLIENT_SUCCESS || !https)
             {
               Log_fatal("%s [%d]: Unable to connect for firmware update\r\n", __FILE__, __LINE__);
               _failureMessage = WiFi.RSSI() > _wifiConnectionRssiThreshold
                                     ? API_FIRMWARE_UPDATE_ERROR
                                     : WIFI_WEAK;
               return false;
             }

             int httpCode = https->GET();
             if (httpCode == HTTP_CODE_OK)
             {
               Log_info("%s [%d]: Downloading .bin file...\r\n", __FILE__, __LINE__);

               size_t contentLength = https->getSize();
               if (Update.begin(contentLength))
               {
                 Log_info("%s [%d]: Firmware update start\r\n", __FILE__, __LINE__);

                 if (Update.writeStream(https->getStream()))
                 {
                   if (Update.end(true))
                   {
                     Log_info("%s [%d]: Firmware update successful. Rebooting...\r\n", __FILE__, __LINE__);
                     ota_ok = true;
                   }
                   else
                   {
                     Log_fatal("%s [%d]: Firmware update failed!\r\n", __FILE__, __LINE__);
                     _failureMessage = FW_UPDATE_FAILED;
                   }
                 }
                 else
                 {
                   Log_fatal("%s [%d]: Write to firmware update stream failed!\r\n", __FILE__, __LINE__);
                   _failureMessage = FW_UPDATE_FAILED;
                 }
               }
               else
               {
                 Log_fatal("%s [%d]: Begin firmware update failed!\r\n", __FILE__, __LINE__);
                 _failureMessage = FW_UPDATE_FAILED;
               }
             }
             else
             {
               Log_fatal("%s [%d]: Firmware GET failed, code: %d\r\n", __FILE__, __LINE__, httpCode);
               _failureMessage = API_FIRMWARE_UPDATE_ERROR;
             }
             return false;
           });
  return ota_ok;
}

FirmwareUpdateResult FirmwareUpdateService::tryUpdate(bool update_firmware, const String &firmware_url)
{
  _failureMessage = NONE;
  FirmwareUpdateResult result;
  Log_info("%s [%d]: update_firmware: %d\r\n", __FILE__, __LINE__, update_firmware);

  if (!validateFirmwareUpdatePossible(update_firmware, firmware_url))
    return result;

  uint32_t now = _getTime();
  if (!performFirmwareUpdate())
  {
    Log_info("%s [%d]: OTA update failed, storing the timestamp to prevent boot looping.\r\n", __FILE__, __LINE__);
    otaRecordAttempt(_persistence, now);
    result.failureMessage = _failureMessage;
    return result;
  }

  result.updated = true;
  return result;
}
