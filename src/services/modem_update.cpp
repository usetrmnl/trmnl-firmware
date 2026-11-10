#ifdef BOARD_TRMNL_X

#include <Arduino.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <WifiCaptive.h>
#include <battery.h>
#include <config.h>
#include <display.h>
#include <globals.h>
#include <http_client.h>
#include <message_screens.h>
#include <misc/clock.h>
#include <modem.h>
#include <ota_schedule.h>
#include <power.h>
#include <services/modem_update.h>
#include <status_screens.h>
#include <trmnl_log.h>
#include <wifi_network.h>

#include "logo_medium.h"

#define MODEM_IMAGE_URL         MODEM_FW_BASE_URL "/" MODEM_FW_VERSION ".bin"

#define MODEM_IMAGE_SIZE        (4 * 1024 * 1024) // full C5 flash
#define MODEM_FS_MARGIN         (64 * 1024)       // LittleFS overhead on top of the file
#define MODEM_STAGE_TIMEOUT_MS  300000UL
#define MODEM_UPDATE_MIN_SOC    15   // percent, ignored on USB
#define MODEM_UPDATE_COOLDOWN_S 3600 // after a failed download or flash

static File s_stageFile;
static size_t s_stageBytes;
static unsigned long s_stageStart;

// Starts the cooldown
static void updateFailed(void) {
  uint32_t now = systemClock().getTime();
  if (now != 0) preferences.putUInt(PREFERENCES_MODEM_LAST_FAILURE, now);
}

static bool updateCooldownOver(void) {
  uint32_t lastFailure = preferences.getUInt(PREFERENCES_MODEM_LAST_FAILURE, 0);
  if (lastFailure == 0) return true; // skips the clock read, up to 200 ms while unsynced
  return retryDue(systemClock().getTime(), lastFailure, MODEM_UPDATE_COOLDOWN_S);
}

static bool modemUpdateBatteryOk(void) {
  if (power().usbStatus() == UsbStatus::CONNECTED) return true;
  int soc = battery().readSoc();
  Log_info("Modem update: battery SoC %d%% (min %d%%)", soc, MODEM_UPDATE_MIN_SOC);
  if (soc >= MODEM_UPDATE_MIN_SOC) return true;
  Log_info("Modem update: battery too low, postponing");
  return false;
}

// ---------------------------------------------------------------------------
// Staging
// ---------------------------------------------------------------------------

static bool stageChunk(const uint8_t *data, size_t len) {
  if (millis() - s_stageStart > MODEM_STAGE_TIMEOUT_MS) return false; // overall limit, both transports
  if (s_stageBytes + len > MODEM_IMAGE_SIZE) return false;
  if (s_stageFile.write(data, len) != len) return false;
  s_stageBytes += len;
  display_update_progress((int)((uint64_t)s_stageBytes * 100 / MODEM_IMAGE_SIZE));
  return true;
}

static bool stageHttpBody(HTTPClient *https, HttpError error) {
  if (error != HTTPCLIENT_SUCCESS || !https) return false;
  if (https->GET() != HTTP_CODE_OK) return false;
  if (https->getSize() != MODEM_IMAGE_SIZE) return false;

  WiFiClient *stream = https->getStreamPtr();
  uint8_t buf[2048];
  unsigned long lastData = millis();
  while (s_stageBytes < MODEM_IMAGE_SIZE) {
    size_t avail = stream->available();
    if (avail == 0) {
      if (!https->connected() || millis() - lastData > IMAGE_STREAM_INACTIVITY_TIMEOUT_MS) return false;
      delay(1);
      continue;
    }
    size_t got = stream->readBytes(buf, min(avail, sizeof(buf)));
    if (!stageChunk(buf, got)) return false;
    lastData = millis();
  }
  return true;
}

// Empty if the modem doesn't answer (old or missing firmware)
static String modemReadVersion(void) {
  if (g_modem) return g_modem->getFirmwareVersion();
  // 2.4 GHz: the modem is off, power it up just to ask
  modem_reset_target();
  Modem modem(115200);
  String version = modem.isInitialized() ? modem.getFirmwareVersion() : String("");
  modem_power_off();
  return version;
}

void modemUpdateStage(void) {
  // The stored version is a hint, to avoid waking the modem every boot
  if (preferences.getString(PREFERENCES_MODEM_FW_VERSION, "") == MODEM_FW_VERSION) return;
  if (preferences.getString(PREFERENCES_MODEM_PENDING_VERSION, "") == MODEM_FW_VERSION &&
      LittleFS.exists(MODEM_IMAGE_PATH))
    return; // already staged
  if (!updateCooldownOver()) return;
  if (!modemUpdateBatteryOk()) return;

  // Space is checked before waking the modem. An old or partial image goes first, there is no room for two.
  LittleFS.remove(MODEM_IMAGE_PATH);
  if (LittleFS.totalBytes() - LittleFS.usedBytes() < MODEM_IMAGE_SIZE + MODEM_FS_MARGIN) {
    Log_error("Modem update: not enough free space in LittleFS to stage %s", MODEM_FW_VERSION);
    updateFailed();
    return;
  }

  String installed = modemReadVersion();
  preferences.putString(PREFERENCES_MODEM_FW_VERSION, installed);
  if (installed == MODEM_FW_VERSION) return;
  Log_info("Modem update: modem reports \"%s\", latest is \"%s\"", installed.c_str(), MODEM_FW_VERSION);

  preferences.remove(PREFERENCES_MODEM_PENDING_VERSION);
  showMessageWithLogo(MODEM_DOWNLOADING);

  Log_info("Modem update: staging version %s", MODEM_FW_VERSION);
  if (!LittleFS.exists(MODEM_IMAGE_DIR)) LittleFS.mkdir(MODEM_IMAGE_DIR); // open() doesn't create directories
  s_stageFile = LittleFS.open(MODEM_IMAGE_PATH, "w");
  if (!s_stageFile) {
    Log_error("Modem update: cannot create %s", MODEM_IMAGE_PATH);
    updateFailed();
    return;
  }
  s_stageBytes = 0;
  s_stageStart = millis();
  bool downloaded;
  if (g_modem && WifiCaptivePortal.getLastCredentials().is5GHz) {
    // On 5 GHz the modem is the only way online
    downloaded = g_modem->httpGet(MODEM_IMAGE_URL, stageChunk, 0, "", IMAGE_STREAM_INACTIVITY_TIMEOUT_MS).ok;
  } else {
    downloaded = ensureWifiConnected() && withHttp(MODEM_IMAGE_URL, stageHttpBody);
  }
  s_stageFile.close();

  if (!downloaded || s_stageBytes != MODEM_IMAGE_SIZE) {
    Log_error("Modem update: download of %s failed (%u of %u bytes)", MODEM_IMAGE_URL, (unsigned)s_stageBytes,
              (unsigned)MODEM_IMAGE_SIZE);
    LittleFS.remove(MODEM_IMAGE_PATH);
    updateFailed();
    return;
  }

  // Only a pending version marks the image as complete
  preferences.remove(PREFERENCES_MODEM_LAST_FAILURE);
  preferences.putString(PREFERENCES_MODEM_PENDING_VERSION, MODEM_FW_VERSION);
  Log_info("Modem update: version %s staged, restarting to flash it", MODEM_FW_VERSION);
  ESP.restart();
}

// ---------------------------------------------------------------------------
// Applying
// ---------------------------------------------------------------------------

bool modemFlash(const char *path, String &error) {
  Serial.println("[MODEM] Resetting modem...");
  modem_reset_target();
  delay(500); // let modem reach bootloader
  Modem modem(115200);
  return modem.flashFromFile(path, error, display_update_progress);
}

void modemUpdateApplyPending(void) {
  String pending = preferences.getString(PREFERENCES_MODEM_PENDING_VERSION, "");
  if (pending.isEmpty()) return;

  // Staged for an older firmware: an OTA came in while the flash was postponed
  if (pending != MODEM_FW_VERSION) {
    Log_info("Modem update: staged version %s is no longer the target (%s), dropping it", pending.c_str(),
             MODEM_FW_VERSION);
    LittleFS.remove(MODEM_IMAGE_PATH);
    preferences.remove(PREFERENCES_MODEM_PENDING_VERSION);
    return;
  }
  if (!LittleFS.exists(MODEM_IMAGE_PATH)) {
    Log_error("Modem update: %s pending but the image is missing", MODEM_FW_VERSION);
    preferences.remove(PREFERENCES_MODEM_PENDING_VERSION);
    return;
  }
  if (!updateCooldownOver()) {
    // A modem erased by a failed write can't set the clock on 5 GHz, so the cooldown would never end
    if (systemClock().getTime() != 0) return;
    modem_reset_target();
    bool answers;
    {
      Modem modem(115200);
      answers = modem.isInitialized();
    }
    modem_power_off();
    if (answers) return;
    Log_info("Modem update: no clock and the modem doesn't answer, flashing without the cooldown");
  }
  if (!modemUpdateBatteryOk()) return;

  Log_info("Modem update: flashing version %s", MODEM_FW_VERSION);
  showMessageWithLogo(MODEM_FLASHING);

  String error;
  String installed;
  bool ok = false;
  // A failed write keeps the image and pending: the modem may be erased, so it's flashed again from it
  if (modemFlash(MODEM_IMAGE_PATH, error)) {
    // Must come up on the high-speed UART, as wifiSessionPrepareModem() expects
    modem_reset_target();
    Modem modem(115200);
    ok = modem.isInitialized();
    if (ok)
      installed = modem.getFirmwareVersion();
    else
      error = "Modem not responding after flash";
    LittleFS.remove(MODEM_IMAGE_PATH); // done with it; a bad image is downloaded again
    preferences.remove(PREFERENCES_MODEM_PENDING_VERSION);
  }
  modem_power_off(); // wifiSessionConnect() starts it again if needed
  if (ok) {
    preferences.putString(PREFERENCES_MODEM_FW_VERSION, installed);
    Log_info("Modem update: version %s installed (modem reports \"%s\")", MODEM_FW_VERSION, installed.c_str());
    display_show_modem_up_to_date();
    return;
  }

  Log_error("Modem update: flashing %s failed (%s), retrying in an hour", MODEM_FW_VERSION, error.c_str());
  display_show_msg(const_cast<uint8_t *>(logo_medium), MODEM_FLASH_FAILED, error.c_str());
  delay(5000);
  updateFailed();
}

#endif // BOARD_TRMNL_X
