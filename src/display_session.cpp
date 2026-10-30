#include <ArduinoLog.h>
#include <Preferences.h>
#include <WiFi.h>
#include <api-client/header.h>
#include <api-client/request_headers.h>
#include <app_logger.h>
#include <battery.h>
#include <bl.h>
#include <bmp.h>
#include <config.h>
#include <device_id.h>
#include <display.h>
#include <display_session.h>
#include <filesystem.h>
#include <globals.h>
#include <logging_parcers.h>
#include <message_screens.h>
#include <png.h>
#include <power.h>
#include <refresh_interval.h>
#include <services/http_retry_request.h>
#include <special_function.h>
#include <status_screens.h>
#include <trmnl_log.h>
#include <types.h>
#include <wifi_network.h>

#include "displayed_image.h"

#ifdef BOARD_TRMNL_X
#include "touchbar_gesture.h"
#endif

// --- Helpers / tables still owned by bl.cpp ---
void showMessageWithLogo(MSG message_type);

static void writeSpecialFunction(SPECIAL_FUNCTION function) {
  if (preferences.isKey(PREFERENCES_SF_KEY)) {
    Log.info("%s [%d]: SF saved. Reading...\r\n", __FILE__, __LINE__);
    if ((SPECIAL_FUNCTION)preferences.getUInt(PREFERENCES_SF_KEY, 0) == function) {
      Log.info("%s [%d]: No need to re-write\r\n", __FILE__, __LINE__);
    } else {
      Log.info("%s [%d]: Writing new special function\r\n", __FILE__, __LINE__);
      bool res = preferences.putUInt(PREFERENCES_SF_KEY, function);
      if (res)
        Log.info("%s [%d]: Written new special function successfully\r\n", __FILE__, __LINE__);
      else
        Log.error("%s [%d]: Writing new special function failed\r\n", __FILE__, __LINE__);
    }
  } else {
    Log.error("%s [%d]: SF not saved\r\n", __FILE__, __LINE__);
    bool res = preferences.putUInt(PREFERENCES_SF_KEY, function);
    if (res)
      Log.info("%s [%d]: Written new special function successfully\r\n", __FILE__, __LINE__);
    else
      Log.error("%s [%d]: Writing new special function failed\r\n", __FILE__, __LINE__);
  }
}

https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse) {
  https_request_err_e result = HTTPS_NO_ERR;
  int file_size = 0;

  if (special_function == SF_NONE) {
    uint64_t request_status = apiResponse.status;
    Log.info("%s [%d]: status: %d\r\n", __FILE__, __LINE__, request_status);
    switch (request_status) {
    case 0: {
      String image_url = apiResponse.image_url;
      uint64_t rate = apiResponse.refresh_rate;
      reset_firmware = apiResponse.reset_firmware;
#ifdef BYOS_PROTOCOL_V1
      if (apiResponse.ota_wait) {
        // OTA waiting mode: short polls, no drawing; the server flips update_firmware when the binary is ready.
        Log.info("%s [%d]: byos: ota_wait, polling every %d s\r\n", __FILE__, __LINE__, (int)rate);
        if (!preferences.getBool(PREFERENCES_OTA_WAIT_SHOWN, false)) {
          display_show_msg_api(storedLogoOrDefault(0), "Waiting for firmware update...");
          preferences.putBool(PREFERENCES_OTA_WAIT_SHOWN, true);
        }
      } else if (preferences.getBool(PREFERENCES_OTA_WAIT_SHOWN, false)) {
        preferences.putBool(PREFERENCES_OTA_WAIT_SHOWN, false);
        preferences.remove(PREFERENCES_FRAME_ID_KEY); // the panel shows the waiting screen, force a full frame next
      }
      if (apiResponse.v1_action == V1_ACTION_NONE) {
        Log.info("%s [%d]: byos: action none, frame %s stays\r\n", __FILE__, __LINE__, apiResponse.frame_id.c_str());
        refreshInterval.applyServerRate(rate);
        refreshInterval.resetFastPollStreak();
        status = false;
        result = HTTPS_SUCCESS;
        if (reset_firmware) result = HTTPS_RESET;
        break;
      }
      // partial: until windowed regions land, download the full frame; display_show_image() already does a
      // non-flashing REFRESH_PARTIAL against the cached previous image, so only changed pixels move.
      // full + full_mode=full: the server asks for a real full refresh (ghost cleaning) → maximum_compatibility.
      if (apiResponse.v1_action == V1_ACTION_FULL && apiResponse.full_mode == V1_FULL_FULL)
        apiResponse.maximum_compatibility = true;
      else
        apiResponse.maximum_compatibility = false;
#endif

      bool sleep_5_seconds = false;

      writeSpecialFunction(apiResponse.special_function);

      if (image_url.length() > 0) {
        Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
        Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

        image_url.toCharArray(filename, image_url.length() + 1);
        // check if plugin is applied
        bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
        Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

        if (apiResponse.filename == "empty_state") {
          Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
          if (!flag) {
            // draw received logo
            status = true;
            // set flag to true
            if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) !=
                true) // check the flag to avoid the re-writing
            {
              bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
              if (res)
                Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
              else
                Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
            }
          } else {
            // don't draw received logo
            status = false;
          }
          // sleep 5 seconds
          sleep_5_seconds = true;
        } else {
          Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
          refreshInterval.resetFastPollStreak();
          if (flag) {
            if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) !=
                false) // check the flag to avoid the re-writing
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
          if (!filesystem_fixed_file_exists(new_filename)) {
            Log.info("%s [%d]: New image. Download and show it.\r\n", __FILE__, __LINE__);
            status = true;
            bUsedCachedImage = false;
          } else {
            Log.info("%s [%d]: Old image. Read from FLASH and show it.\r\n", __FILE__, __LINE__);
            status = false;
            bUsedCachedImage = true;
            result = HTTPS_SUCCESS;
          }
        }
      }
      Log.info("%s [%d]: refresh_rate: %d\r\n", __FILE__, __LINE__, rate);
      refreshInterval.applyServerRate(rate);

      if (reset_firmware) {
        Log.info("%s [%d]: Reset status is true\r\n", __FILE__, __LINE__);
      }

      if (apiResponse.update_firmware && apiResponse.firmware_url.length() > 0) result = HTTPS_SUCCESS;
      if (reset_firmware) result = HTTPS_RESET;
      if (sleep_5_seconds) result = HTTPS_PLUGIN_NOT_ATTACHED;
      Log.info("%s [%d]: result - %s\r\n", __FILE__, __LINE__, https_request_err_str(result));
    } break;
    case 202: {
      result = HTTPS_NO_REGISTER;
      refreshInterval.applyFastPoll();
      status = false;
    } break;
    case 500: {
      result = HTTPS_RESET;
      refreshInterval.applyFastPoll();
      status = false;
    } break;

    default:
      break;
    }
  } else if (special_function != SF_NONE) {
    uint64_t request_status = apiResponse.status;
    Log.info("%s [%d]: status: %d\r\n", __FILE__, __LINE__, request_status);
    switch (request_status) {
    case 0: {
      switch (special_function) {
      case SF_IDENTIFY: {
        String action = apiResponse.action;
        if (action.equals("identify")) {
          Log.info("%s [%d]:Identify success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          if (image_url.length() > 0) {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state") {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag) {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) !=
                    true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              } else {
                status = false;
              }
            } else {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag) {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) !=
                    false) // check the flag to avoid the re-writing
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
        } else {
          Log.error("%s [%d]: identify failed\r\n", __FILE__, __LINE__);
        }
      } break;
      case SF_SLEEP: {
        String action = apiResponse.action;
        if (action.equals("sleep")) {
          uint64_t rate = apiResponse.refresh_rate;
          Log.info("%s [%d]: refresh_rate: %d\r\n", __FILE__, __LINE__, rate);
          refreshInterval.applyServerRate(rate);
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: sleep success\r\n", __FILE__, __LINE__);
        } else {
          Log.error("%s [%d]: sleep failed\r\n", __FILE__, __LINE__);
          // need to add error
        }
      } break;
      case SF_ADD_WIFI: {
        String action = apiResponse.action;
        if (action.equals("add_wifi")) {
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: Add wifi success\r\n", __FILE__, __LINE__);
        } else {
          Log.error("%s [%d]: Add wifi failed\r\n", __FILE__, __LINE__);
        }
      } break;
      case SF_RESTART_PLAYLIST: {
        String action = apiResponse.action;
        if (action.equals("restart_playlist")) {
          Log.info("%s [%d]:Restart playlist success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          if (image_url.length() > 0) {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state") {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag) {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) !=
                    true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              } else {
                // don't draw received logo
                status = false;
              }
            } else {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag) {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) !=
                    false) // check the flag to avoid the re-writing
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
        } else {
          Log.error("%s [%d]: Restart playlist failed\r\n", __FILE__, __LINE__);
        }
      } break;
      case SF_REWIND: {
        String action = apiResponse.action;
        if (action.equals("rewind")) {
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: rewind success\r\n", __FILE__, __LINE__);

          bool image_reverse = false;
          bool file_check_bmp = true;
          image_err_e image_proccess_response = PNG_WRONG_FORMAT;
          bmp_err_e bmp_proccess_response = BMP_NOT_BMP;

          String last_dot_file = filesystem_file_exists("/last.bmp") ? "/last.bmp" : "/last.png";
          if (last_dot_file == "/last.bmp") {
            Log.info("Rewind BMP\n\r");
            buffer = (uint8_t *)malloc(DISPLAY_BMP_IMAGE_SIZE);
            file_check_bmp = filesystem_read_from_file(last_dot_file.c_str(), buffer, DISPLAY_BMP_IMAGE_SIZE);
            bmp_proccess_response = parseBMPHeader(buffer, image_reverse);
          } else if (last_dot_file == "/last.png") {
            Log.info("Rewind PNG\n\r");
            buffer = display_read_file(last_dot_file.c_str(), &file_size);
            // NULL without a previous image: nothing to show
            image_proccess_response = buffer ? PNG_NO_ERR : PNG_WRONG_FORMAT;
          }

          if (file_check_bmp) {
            switch (image_proccess_response) {
            case PNG_NO_ERR: {
              Log.info("Showing image\n\r");
              display_show_image(buffer, file_size, true);
              need_to_refresh_display = 1;
            } break;
            default: {
            } break;
            }
            switch (bmp_proccess_response) {
            case BMP_NO_ERR: {
              Log.info("Showing image\n\r");
              display_show_image(buffer, DISPLAY_BMP_IMAGE_SIZE, true);
              need_to_refresh_display = 1;
            } break;
            default: {
            } break;
            }
          } else {
            if (buffer) {
              free(buffer);
              buffer = nullptr;
            }
            showMessageWithLogo(MSG_FORMAT_ERROR);
          }
        } else {
          Log.error("%s [%d]: rewind failed\r\n", __FILE__, __LINE__);
        }
      } break;
      case SF_SEND_TO_ME: {
        String action = apiResponse.action;

        if (action.equals("send_to_me")) {
          status = false;
          result = HTTPS_NO_ERR; // already drawn: skips downloadAndShow()'s cached image path
          Log.info("%s [%d]: send_to_me success\r\n", __FILE__, __LINE__);

          // Redraw the last image, which the loading screen (and the TRMNL X's menu) covers. A PNG or
          // JPEG is cached under its name, a BMP only as /current.bmp.
          const char *path = filesystem_file_exists(DisplayedImage::get()) ? DisplayedImage::get() : "/current.bmp";
          buffer = display_read_file(path, &file_size);
          if (buffer && file_size > 0) {
            display_show_image(buffer, file_size, true);
            need_to_refresh_display = 1;
          }
          free(buffer);
          buffer = nullptr;
        } else {
          Log.error("%s [%d]: send_to_me failed\r\n", __FILE__, __LINE__);
        }
      } break;
      case SF_GUEST_MODE: {
        String action = apiResponse.action;
        if (action.equals("guest_mode")) {
          Log.info("%s [%d]:Guest Mode success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          uint64_t rate = apiResponse.refresh_rate;
          if (image_url.length() > 0) {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state") {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag) {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) !=
                    true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: Flag writing failed\r\n", __FILE__, __LINE__);
                }
              } else {
                // don't draw received logo
                status = false;
              }
            } else {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag) {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) !=
                    false) // check the flag to avoid the re-writing
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
          refreshInterval.applyServerRate(rate);
        } else {
          Log.error("%s [%d]: Guest Mode failed\r\n", __FILE__, __LINE__);
        }
      } break;
      default:
        break;
      }
    } break;
    case 202: {
      result = HTTPS_NO_REGISTER;
      refreshInterval.applyFastPoll();
      status = false;
    } break;
    case 500: {
      result = HTTPS_RESET;
      refreshInterval.applyFastPoll();
      status = false;
    } break;

    default:
      break;
    }
  }
  return result;
}

static ApiDisplayInputs loadApiDisplayInputs(Preferences &preferences, float batteryVoltage) {
  ApiDisplayInputs inputs = createApiHeaders();
  inputs.baseUrl = preferences.getString(PREFERENCES_API_URL, API_BASE_URL);
#ifdef BYOS_PROTOCOL_V1
  inputs.frameId = preferences.getString(PREFERENCES_FRAME_ID_KEY, "");
#endif
  inputs.batteryVoltage = batteryVoltage; // read before the display and WiFi were turned on
  last_wifi_rssi = inputs.rssi;
  Log.info("%s [%d]: baseUrl from preferences: %s\r\n", __FILE__, __LINE__, inputs.baseUrl.c_str());
  return inputs;
}

https_request_err_e downloadAndShow(float batteryVoltage) {
  image_err_e png_res = PNG_DECODE_ERR;
  bmp_err_e bmp_res = BMP_NOT_BMP;
  auto apiDisplayInputs = loadApiDisplayInputs(preferences, batteryVoltage);

  // Retries, redirects and the transport (Wi-Fi vs. TRMNL X modem) are handled inside
  // HttpRetryRequest, via fetchApiDisplay(). The result stays in the global for display.cpp
  // and the OTA check.
  apiDisplayResult = fetchApiDisplay(apiDisplayInputs);
  if (apiDisplayResult.error != HTTPS_NO_ERR) return apiDisplayResult.error;

  https_request_err_e result = handleApiDisplayResponse(apiDisplayResult.response);
  if (apiDisplayResult.response.filename == "screen_wiper.png") {
    // Guard against re-fetching forever if the wiper is the only playlist item
    static bool wiped_this_wake = false;
    if (wiped_this_wake) {
      Log_info("Screen wiper returned again; not wiping twice in one wake");
      return result; // leave the wiped (white) screen as-is
    }
    wiped_this_wake = true;
    Log_info("Detected screen wiper filename: %s, clearing display...", apiDisplayResult.response.filename.c_str());
    display_wipe();
    // Wiping leaves the screen blank; fetch and show the next playlist item
    // right away instead of waiting for the next scheduled refresh.
    Log_info("Screen wipe complete, fetching next playlist item");
    return downloadAndShow(batteryVoltage);
  }

  if (!status && result == HTTPS_SUCCESS) { // this means we already have this image stored in SPIFFS
    char szTemp[36];
    filesystem_fix_filename(apiDisplayResult.response.filename.c_str(), szTemp);
    if (DisplayedImage::matches(szTemp)) {
      // We just displayed the same image, don't refresh the display
      Log.info("%s [%d]: The image hasn't changed since the last wakeup, don't refresh the display.\r\n", __FILE__,
               __LINE__);
      buffer = nullptr;
      return result;
    }
    if (!filesystem_file_exists(szTemp)) {
      // BMPs aren't cached under their filename (e.g. after a sleep special function); redraw /current.bmp
      buffer = nullptr;
      size_t current_size = 0;
      if (filesystem_file_exists("/current.bmp")) {
        current_size = filesystem_read_and_allocate("/current.bmp", &buffer);
      }
      if (!buffer || current_size == 0) {
        Log_info("%s isn't cached; keeping the current screen", szTemp);
        if (buffer) free(buffer);
        buffer = nullptr;
        return result;
      }
      Log_info("%s isn't cached; showing /current.bmp", szTemp);
      display_show_image(buffer, current_size, true);
      free(buffer);
      buffer = nullptr;
      DisplayedImage::remember(szTemp);
      return result;
    }
    DisplayedImage::remember(szTemp);
#ifdef BYOS_PROTOCOL_V1
    preferences.putString(PREFERENCES_FRAME_ID_KEY, apiDisplayResult.response.frame_id);
#endif
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
#ifdef BYOS_PROTOCOL_V1
    preferences.putString(PREFERENCES_FRAME_ID_KEY, apiDisplayResult.response.frame_id);
#endif
    // Rotate NVS path keys: last ← current ← szTemp
    String _curPath = preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "");
    String _lastPath = preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    if (!_curPath.isEmpty() && (_curPath != String(szTemp) || _lastPath.isEmpty()))
      preferences.putString(PREFERENCES_LAST_PATH_KEY, _curPath);
    preferences.putString(PREFERENCES_CURRENT_PATH_KEY, String(szTemp));
#ifdef BOARD_TRMNL_X
    update_playlist_order(preferences, szTemp, _curPath.c_str());
#endif
    preferences.putString(PREFERENCES_BROWSE_PATH_KEY, String(szTemp));
    return result;
  }

  if (!(status && !reset_firmware)) return result; // nothing new to download

  status = false;

  // A 202 carries no image_url, so filename is still empty here. HTTPClient::begin() rejects
  // that and the fetch below would report it as an API failure over the friendly ID screen.
  if (filename[0] == '\0') return result;

  HttpRetryRequestConfig imageConfig;
  imageConfig.url = filename;
  imageConfig.apiBaseUrl = apiDisplayInputs.baseUrl; // ID / Access-Token only go to the API server
  imageConfig.headers = buildImageHeaders(apiDisplayInputs);
  imageConfig.readTimeoutSeconds = apiDisplayResult.response.image_url_timeout;

  HttpRetryRequest imageRequest(imageConfig);
  https_request_err_e fetchResult = imageRequest.execute();
  if (fetchResult == HTTPS_RESPONSE_CODE_INVALID)
    fetchResult = HTTPS_IMAGE_DOWNLOAD_FAILED; // dedicated screen: the API answered, the image host did not
  if (fetchResult != HTTPS_NO_ERR) {
    Log_info("Returned result - %s", https_request_err_str(fetchResult));
    return fetchResult;
  }

  // imageRequest owns the body; the global `buffer` only borrows it for the decode tail below and
  // must not outlive this function. The guard nulls it on every exit path.
  buffer = imageRequest.body();
  uint32_t content_size = imageRequest.bodySize();
  struct BufferGuard {
    ~BufferGuard() { buffer = nullptr; }
  } bufferGuard;

  bool isPNG = imageRequest.contentType() == "image/png";
  bool isJPEG = imageRequest.contentType() == "image/jpeg";
  if (content_size >= 2 && buffer[0] == 'B' && buffer[1] == 'M') {
    isPNG = false;
    Log.info("BMP file detected");
  }

  submitStoredLogs();

  WiFi.disconnect(true); // no need for WiFi, save power starting here
  Log.info("%s [%d]: Received successfully; WiFi off.\r\n", __FILE__, __LINE__);

  bool image_reverse = false;
  if (isPNG)
    png_res = parsePNGHeader(buffer, content_size); // the decoder reports nothing back
  else if (isJPEG)
    png_res = PNG_NO_ERR;
  if (png_res == PNG_NO_ERR && (isPNG || isJPEG)) {
    char szTemp[36];
    filesystem_fix_filename(apiDisplayResult.response.filename.c_str(), szTemp);
    Log.info("%s [%d]: Decoding %s\r\n", __FILE__, __LINE__, (isPNG) ? "png" : "jpeg");
    display_show_image(buffer, content_size, true); // PNG and JPEG decoders leave the buffer intact
    Log.info("%s [%d]: Writing %s to SPIFFS\r\n", __FILE__, __LINE__, szTemp);
    filesystem_purge_old_file(szTemp); // try to delete the old version or older than 24h
    writeImageToFile(szTemp, buffer, content_size);
    DisplayedImage::remember(szTemp); // current image becomes the previous image
#ifdef BYOS_PROTOCOL_V1
    preferences.putString(PREFERENCES_FRAME_ID_KEY, apiDisplayResult.response.frame_id);
#endif
    String _curPath = preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "");
    String _lastPath = preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    if (!_curPath.isEmpty() && (_curPath != String(szTemp) || _lastPath.isEmpty()))
      preferences.putString(PREFERENCES_LAST_PATH_KEY, _curPath);
    preferences.putString(PREFERENCES_CURRENT_PATH_KEY, String(szTemp));
#ifdef BOARD_TRMNL_X
    update_playlist_order(preferences, szTemp, _curPath.c_str());
#endif
    preferences.putString(PREFERENCES_BROWSE_PATH_KEY, String(szTemp));
  } else if (!isPNG && !isJPEG) {
    bmp_res = parseBMPHeader(buffer, image_reverse);
    Log.info("%s [%d]: BMP Parsing result: %d\r\n", __FILE__, __LINE__, bmp_res);
  }
  Serial.println();
  String error = "";
  String png_error = ""; // its own string: the bmp switch below always runs too and would overwrite it

  switch (png_res) {
  case PNG_NO_ERR: {
    // Using filename from API response
    new_filename = apiDisplayResult.response.filename;

    // Print the extracted string
    Log.info("%s [%d]: New filename - %s\r\n", __FILE__, __LINE__, new_filename.c_str());

    if (result != HTTPS_PLUGIN_NOT_ATTACHED) result = HTTPS_SUCCESS;
  } break;
  case PNG_WRONG_FORMAT: {
    png_error = "Wrong image format. Did not pass signature check";
  } break;
  case PNG_BAD_SIZE: {
    png_error = "IMAGE width, height or size are invalid";
  } break;
  case PNG_DECODE_ERR: {
    png_error = "could not decode png image";
  } break;
  case PNG_MALLOC_FAILED: {
    png_error = "could not allocate memory for png image decoder";
  } break;
  default:
    break;
  }

  switch (bmp_res) {
  case BMP_NO_ERR: {
    if (!filesystem_file_exists("/current.png")) {
      writeImageToFile("/current.bmp", buffer, content_size);
    }
    Log.info("Free heap at before display - %d", ESP.getMaxAllocHeap());
    display_show_image(buffer, content_size, true);
    {
      char szTemp[36];
      filesystem_fix_filename(apiDisplayResult.response.filename.c_str(), szTemp);
      DisplayedImage::remember(szTemp);
#ifdef BYOS_PROTOCOL_V1
      preferences.putString(PREFERENCES_FRAME_ID_KEY, apiDisplayResult.response.frame_id);
#endif
    }

    // Using filename from API response
    new_filename = apiDisplayResult.response.filename;

    // Print the extracted string
    Log.info("%s [%d]: New filename - %s\r\n", __FILE__, __LINE__, new_filename.c_str());

    if (result != HTTPS_PLUGIN_NOT_ATTACHED) result = HTTPS_SUCCESS;
  } break;
  case BMP_NOT_BMP: {
    error = "First two header bytes are invalid!";
  } break;
  case BMP_BAD_SIZE: {
    error = "BMP width, height or size are invalid";
  } break;
  case BMP_COLOR_SCHEME_FAILED: {
    error = "BMP color scheme is invalid";
  } break;
  case BMP_INVALID_OFFSET: {
    error = "BMP header offset is invalid";
  } break;
  default:
    break;
  }

  if (isPNG && png_res != PNG_NO_ERR) {
    char szTemp[36];
    filesystem_fix_filename(apiDisplayResult.response.filename.c_str(), szTemp);
    filesystem_file_delete(szTemp);
    Log_error_submit("error parsing image file - %s", png_error.c_str());

    return HTTPS_WRONG_IMAGE_FORMAT;
  }

  Log_info("Returned result - %s", https_request_err_str(result));

  return result;
}
