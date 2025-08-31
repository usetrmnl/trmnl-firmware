#ifndef CONFIG_H
#define CONFIG_H

#define FW_MAJOR_VERSION 1
#define FW_MINOR_VERSION 6
#define FW_PATCH_VERSION 7

// Helper macros for stringification
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#ifndef FW_VERSION_SUFFIX
#define FW_VERSION_SUFFIX ""
#endif

// Compile-time firmware version string
#define FW_VERSION_STRING TOSTRING(FW_MAJOR_VERSION) "." TOSTRING(FW_MINOR_VERSION) "." TOSTRING(FW_PATCH_VERSION) FW_VERSION_SUFFIX

#define LOG_MAX_NOTES_NUMBER 10

#define PREFERENCES_API_KEY "api_key"
#define PREFERENCES_API_KEY_DEFAULT ""
#define PREFERENCES_API_URL "api_url"
#define PREFERENCES_FRIENDLY_ID "friendly_id"
#define PREFERENCES_FRIENDLY_ID_DEFAULT ""
#define PREFERENCES_SLEEP_TIME_KEY "refresh_rate"
#define PREFERENCES_LOG_KEY "log_"
#define PREFERENCES_LOG_BUFFER_HEAD_KEY "log_head"
#define PREFERENCES_LOG_ID_KEY "log_id"
#define PREFERENCES_DEVICE_REGISTERED_KEY "plugin"
#define PREFERENCES_SF_KEY "sf"
#define PREFERENCES_FILENAME_KEY "filename"
#define PREFERENCES_LAST_SLEEP_TIME "last_sleep"
#define PREFERENCES_CONNECT_API_RETRY_COUNT "retry_count"
#define PREFERENCES_CONNECT_WIFI_RETRY_COUNT "wifi_retry"

#define WIFI_CONNECTION_RSSI (-100)

#define DISPLAY_BMP_IMAGE_SIZE 48062 // in bytes - 62 bytes - header; 48000 bytes - bitmap (480*800 1bpp) / 8
#define DEFAULT_IMAGE_SIZE 48000
#define MAX_IMAGE_SIZE 90000 // largest compressed image we can receive
#define SLEEP_uS_TO_S_FACTOR 1000000           /* Conversion factor for micro seconds to seconds */
#define SLEEP_TIME_TO_SLEEP 900                /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIME_WHILE_NOT_CONNECTED 5       /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIME_WHILE_PLUGIN_NOT_ATTACHED 5 /* Time ESP32 will go to sleep (in seconds) */

#define MS_TO_S_FACTOR 1000                    /* Conversion factor for milliseconds to seconds */

enum API_CONNECT_RETRY_TIME // Time to sleep before trying to connect to the API (in seconds)
{
    API_FIRST_RETRY = 15,
    API_SECOND_RETRY = 30,
    API_THIRD_RETRY = 60
};

enum WIFI_CONNECT_RETRY_TIME // Time to sleep before trying to connect to the Wi-Fi (in seconds)
{
    WIFI_FIRST_RETRY = 60,
    WIFI_SECOND_RETRY = 180,
    WIFI_THIRD_RETRY = 300
};

#if defined(BOARD_TRMNL)
#define PIN_INTERRUPT 2
#define DEVICE_MODEL "og"
#elif defined(BOARD_WAVESHARE_ESP32_DRIVER)
#define PIN_INTERRUPT 33
#define DEVICE_MODEL "waveshare"
#define FAKE_BATTERY_VOLTAGE
#elif defined(BOARD_SEEED_XIAO_ESP32C3)
#define DEVICE_MODEL "seeed_esp32c3"
#define PIN_INTERRUPT 9
#define FAKE_BATTERY_VOLTAGE
#elif defined(BOARD_SEEED_XIAO_ESP32S3)
#define DEVICE_MODEL "seeed_esp32s3"
#define PIN_INTERRUPT 0
#define FAKE_BATTERY_VOLTAGE
#elif defined(BOARD_XIAO_EPAPER_DISPLAY)
#define DEVICE_MODEL "xiao_epaper_display"
#define PIN_INTERRUPT 5     //with silkscreen "KEY3"
#define PIN_RESET 0
#define FAKE_BATTERY_VOLTAGE
#endif

#if defined(BOARD_XIAO_EPAPER_DISPLAY)
#define PIN_BATTERY 1
#else
#define PIN_BATTERY 3
#endif

// #define FAKE_BATTERY_VOLTAGE // Uncomment to report 4.2V instead of reading ADC

#define BUTTON_HOLD_TIME 5000
#define BUTTON_MEDIUM_HOLD_TIME 1000
#define BUTTON_SOFT_RESET_TIME 15000

#define SERVER_MAX_RETRIES 3

#define API_BASE_URL "https://trmnl.app"

#endif
