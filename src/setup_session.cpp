#include <Arduino.h>
#include <ArduinoLog.h>
#include <WifiCaptive.h>
#include <bl.h>
#include <display.h>
#include <filesystem.h>
#include <globals.h>
#include <message_screens.h>
#include <services/device_setup.h>
#include <setup_session.h>
#include <status_screens.h>
#include <trmnl_log.h>

static void showMacNotRegistered(const DeviceSetupResult &setup)
{
  // The server's image for an unregistered device when it sent one and this run saved it
  // (not a logo left on flash by an earlier setup), else the server's message under the logo.
  bool drawable = setup.imagePath.length() > 0;
#ifdef PARALLEL_EPD
  // FastEPD's display_show_image draws no uncompressed BMP (the X's 5 GHz modem path saves only those)
  drawable = drawable && !setup.imagePath.endsWith(".bmp");
#endif
  int image_size = 0;
  uint8_t *image = drawable ? display_read_file(setup.imagePath.c_str(), &image_size) : nullptr;
  if (image)
  {
    display_show_image(image, image_size, true);
    free(image);
  }
  else
    showMessageWithLogo(MAC_NOT_REGISTERED, setup.apiResponse);
  preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
  refreshInterval.applyDefault();
}

void ensureDeviceSetUp(void)
{
  if (preferences.isKey(PREFERENCES_API_KEY) && preferences.isKey(PREFERENCES_FRIENDLY_ID))
  {
    Log.info("%s [%d]: API key and friendly ID saved\r\n", __FILE__, __LINE__);
  }
  else
  {
    Log.info("%s [%d]: API key or friendly ID not saved\r\n", __FILE__, __LINE__);

    // attempt to get the API key and friendly ID
    DeviceSetupResult setup = deviceSetup().perform();

    // copy results into globals
    setup.imageUrl.toCharArray(filename, sizeof(filename));
    setup.message.toCharArray(message_buffer, sizeof(message_buffer));

    // perform post-setup actions
    if (setup.showSetupScreen)
    {
      display_show_msg(storedLogoOrDefault(0), FRIENDLY_ID, setup.friendlyId, true, "", setup.message);
      need_to_refresh_display = 0;
    }
    else if (setup.errorScreen != NONE)
    {
      showMessageWithLogo(setup.errorScreen);
    }

    if (setup.outcome == DeviceSetupOutcome::MacNotRegistered)
      showMacNotRegistered(setup);

    if (setup.shouldGoToSleep)
    {
      display_sleep();
      goToSleep();
    }
  }
}

/**
 * @brief Function to reset the friendly id, API key, WiFi SSID and password
 * @param url Server URL address
 * @return none
 */
void resetDeviceCredentials(void)
{
  Log.info("%s [%d]: The device will be reset now...\r\n", __FILE__, __LINE__);
  Log.info("%s [%d]: WiFi resetting...\r\n", __FILE__, __LINE__);
  WifiCaptivePortal.resetSettings();
  need_to_refresh_display = 1;
  bool res = preferences.clear();
  if (res)
    Log.info("%s [%d]: The device reset success. Restarting...\r\n", __FILE__, __LINE__);
  else
    Log.error("%s [%d]: The device resetting error. The device will be reset now...\r\n", __FILE__, __LINE__);
  preferences.end();
  ESP.restart();
}
