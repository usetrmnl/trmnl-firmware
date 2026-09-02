#include <globals.h>
#include <misc/clock.h>

// --- Persistence & long-lived services ---
Preferences preferences;
PreferencesPersistence preferencesPersistence(preferences);
StoredLogs storedLogs(LOG_MAX_NOTES_NUMBER / 2, LOG_MAX_NOTES_NUMBER / 2, PREFERENCES_LOG_KEY,
                      PREFERENCES_LOG_BUFFER_HEAD_KEY, preferencesPersistence);
RefreshInterval refreshInterval(preferencesPersistence);
FirmwareUpdateService firmwareUpdateService(preferencesPersistence, systemClock(), WIFI_CONNECTION_RSSI);

// --- Image download / API state ---
String new_filename = "";
ApiDisplayResult apiDisplayResult;
uint8_t *buffer = nullptr;
char filename[1024];      // image URL
char message_buffer[128]; // message to show on the screen
bool status = false;      // need to download a new image
bool reset_firmware = false; // need to reset credentials
bool log_retry = false;      // need to log connection retry

// --- Wake / sleep state ---
uint32_t time_since_sleep;
esp_sleep_wakeup_cause_t wakeup_reason = ESP_SLEEP_WAKEUP_UNDEFINED;
SPECIAL_FUNCTION special_function = SF_NONE;
RTC_DATA_ATTR int iPrevWakeTime = 0;
RTC_DATA_ATTR bool bUsedCachedImage = false;
RTC_DATA_ATTR uint8_t need_to_refresh_display = 1;
RTC_DATA_ATTR bool otg_state = false;

// --- Display refresh bookkeeping ---
// Counts the number of partial updates to know when to do a full update
RTC_DATA_ATTR int iUpdateCount = 0;
RTC_DATA_ATTR bool bCanDoPartial = false;
uint32_t iTempProfile;

#ifdef BOARD_TRMNL_X
// --- TRMNL X hardware ---
// Set once during bl_init, used by download helpers and getWiFiStatus()
Modem *g_modem = nullptr;
battery_count_t battery_count = BATTERY_NONE;
bool battery_charging = false;
#endif // BOARD_TRMNL_X
