#include <ArduinoLog.h>
#include <Preferences.h>
#include <bmp.h>
#include <config.h>
#include <display.h>
#include <display_session.h>
#include <filesystem.h>
#include <globals.h>
#include <png.h>
#include <trmnl_log.h>
#include <types.h>

// --- Helpers still owned by bl.cpp ---
void writeSpecialFunction(SPECIAL_FUNCTION function);
void showMessageWithLogo(MSG message_type);
extern const char *szHTTPErrors[];

https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse) {
  https_request_err_e result = HTTPS_NO_ERR;
  int file_size = 0;

#ifdef BOARD_TRMNL_X
  // Set touchbar mode and persist to NVS
  if (apiResponse.touchbar_mode.length() == 0 || touchbar_tap_mode == (apiResponse.touchbar_mode == "tap")) {
    Log.info("%s [%d]: No need to update touchbar mode\r\n", __FILE__, __LINE__);
  } else {
    touchbar_tap_mode = (apiResponse.touchbar_mode == "tap");
    preferences.putBool(PREFERENCES_TOUCHBAR_MODE_KEY, touchbar_tap_mode);
  }
#endif // BOARD_TRMNL_X

  if (special_function == SF_NONE) {
    uint64_t request_status = apiResponse.status;
    Log.info("%s [%d]: status: %d\r\n", __FILE__, __LINE__, request_status);
    switch (request_status) {
    case 0: {
      String image_url = apiResponse.image_url;
      uint64_t rate = apiResponse.refresh_rate;
      reset_firmware = apiResponse.reset_firmware;

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
      Log.info("%s [%d]: result - %s\r\n", __FILE__, __LINE__, szHTTPErrors[result]);
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

          // showMessageWithLogo(MSG_FORMAT_ERROR);
          String last_dot_file = filesystem_file_exists("/last.bmp") ? "/last.bmp" : "/last.png";
          if (last_dot_file == "/last.bmp") {
            Log.info("Rewind BMP\n\r");
            buffer = (uint8_t *)malloc(DISPLAY_BMP_IMAGE_SIZE);
            file_check_bmp = filesystem_read_from_file(last_dot_file.c_str(), buffer, DISPLAY_BMP_IMAGE_SIZE);
            bmp_proccess_response = parseBMPHeader(buffer, image_reverse);
          } else if (last_dot_file == "/last.png") {
            Log.info("Rewind PNG\n\r");
            buffer = display_read_file(last_dot_file.c_str(), &file_size);
            image_proccess_response = PNG_NO_ERR; // DEBUG
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
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: send_to_me success\r\n", __FILE__, __LINE__);

          bool image_reverse = false;

          if (!filesystem_file_exists("/current.bmp") && !filesystem_file_exists("/current.png")) {
            Log.info("%s [%d]: No current image!\r\n", __FILE__, __LINE__);
            if (buffer) {
              free(buffer);
              buffer = nullptr;
            }
            return HTTPS_WRONG_IMAGE_FORMAT;
          }

          if (filesystem_file_exists("/current.bmp")) {
            Log.info("%s [%d]: send_to_me BMP\r\n", __FILE__, __LINE__);
            buffer = (uint8_t *)malloc(DISPLAY_BMP_IMAGE_SIZE);

            if (!filesystem_read_from_file("/current.bmp", buffer, DISPLAY_BMP_IMAGE_SIZE)) {
              free(buffer);
              buffer = nullptr;
              Log_error_submit("Error reading image!");
              return HTTPS_WRONG_IMAGE_FORMAT;
            }

            bmp_err_e bmp_parse_result = parseBMPHeader(buffer, image_reverse);
            if (bmp_parse_result != BMP_NO_ERR) {
              free(buffer);
              buffer = nullptr;
              Log_error_submit("Error parsing BMP header, code: %d", bmp_parse_result);
              return HTTPS_WRONG_IMAGE_FORMAT;
            }
          } else if (filesystem_file_exists("/current.png")) {
            Log.info("%s [%d]: send_to_me PNG\r\n", __FILE__, __LINE__);
            image_err_e png_parse_result = PNG_NO_ERR; // DEBUG
            buffer = display_read_file("/current.png", &file_size);
// Disable partial update for now
//            if (filesystem_file_exists("/last.png")) {
//                buffer_old = display_read_file("/last.png", &file_size_old);
//                Log.info("%s [%d]: loading last PNG for partial update\r\n", __FILE__, __LINE__);
//            }
            if (png_parse_result != PNG_NO_ERR) {
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
