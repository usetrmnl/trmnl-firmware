#ifndef CONFIG_H
#define CONFIG_H

#define FW_MAJOR_VERSION 1
#define FW_MINOR_VERSION 7
#define FW_PATCH_VERSION 4

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
#define PREFERENCES_TEMP_PROFILE "temp_profile"
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
#ifdef BOARD_TRMNL_X
#define MAX_IMAGE_SIZE 750000 // Use PSRAM on the ESP32-S3
#else
#define MAX_IMAGE_SIZE 90000 // largest compressed image we can receive
#endif
#define SLEEP_uS_TO_S_FACTOR 1000000           /* Conversion factor for micro seconds to seconds */
#define SLEEP_TIME_TO_SLEEP 900                /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIME_WHILE_NOT_CONNECTED 5       /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_TIME_WHILE_PLUGIN_NOT_ATTACHED 5 /* Time ESP32 will go to sleep (in seconds) */

// Different display profiles
#define TEMP_PROFILE_DEFAULT 0
#define TEMP_PROFILE_A 1
#define TEMP_PROFILE_B 2
#define TEMP_PROFILE_C 3

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

#if defined(BOARD_TRMNL) || defined(BOARD_TRMNL_4CLR)
#define PIN_INTERRUPT 2
#define DEVICE_MODEL "og"
#elif defined(BOARD_XTEINK_X4)
#define DEVICE_MODEL "XTEINK_X4"
#define PIN_INTERRUPT 3
#elif defined(BOARD_TRMNL_X)
#define PIN_INTERRUPT 0
#define DEVICE_MODEL "x"
#elif defined(BOARD_WAVESHARE_ESP32_DRIVER)
#define PIN_INTERRUPT 33
#define DEVICE_MODEL "waveshare"
#define FAKE_BATTERY_VOLTAGE
#elif defined(BOARD_SEEED_XIAO_ESP32C3)
#define DEVICE_MODEL "seeed_esp32c3"
#define PIN_INTERRUPT 9         //the boot button on the XIAO ESP32-C3, this button can't be used as wakeup  source though
                                //because it's not in the RTC GPIO group. Instead, you can always use the reset button to
                                //wake up the device. Resetting WiFi configuration needs special routine - press reset button
                                //then press the boot button in less than 2 seconds, and hold it for 5 seconds.
#define FAKE_BATTERY_VOLTAGE
#elif defined(BOARD_SEEED_XIAO_ESP32S3)
#define DEVICE_MODEL "seeed_esp32s3"
#define PIN_INTERRUPT 0         //the boot button on the XIAO ESP32-S3, this button works as regular wakeup button
#define FAKE_BATTERY_VOLTAGE
#elif (defined(BOARD_XIAO_EPAPER_DISPLAY) || defined(BOARD_XIAO_EPAPER_DISPLAY_3CLR))
#define DEVICE_MODEL "xiao_epaper_display"
#define PIN_INTERRUPT 5         //with silkscreen "KEY3"
#define PIN_VBAT_SWITCH 6       //load switch enable pin for battery voltage measurement
#define VBAT_SWITCH_LEVEL HIGH  //load switch enable pin active level
#elif defined(BOARD_SEEED_RETERMINAL_E1001)
#define DEVICE_MODEL "reTerminal E1001"
#define PIN_INTERRUPT 3         //the green button
#define PIN_VBAT_SWITCH 21      //load switch enable pin for battery voltage measurement
#define VBAT_SWITCH_LEVEL HIGH  //load switch enable pin active level
#elif defined(BOARD_SEEED_RETERMINAL_E1002)
#define DEVICE_MODEL "reTerminal E1002"
#define PIN_INTERRUPT 3         //the green button
#define PIN_VBAT_SWITCH 21      //load switch enable pin for battery voltage measurement
#define VBAT_SWITCH_LEVEL HIGH  //load switch enable pin active level
#endif

// Board identifier and display color palette
// DISPLAY_MEASURED_COLORS: actual sRGB values the e-ink panel produces,
// derived from Pimoroni's photographed saturated palette and community
// spectrophotometer measurements (reflectance-to-sRGB). Best-effort
// approximations — useful for server-side dithering/quantization.
#if defined(BOARD_TRMNL) && !defined(BOARD_TRMNL_4CLR)
#define DEVICE_BOARD "trmnl_og"
#define DISPLAY_COLORS "#000000,#555555,#AAAAAA,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#6E6E6E,#9C9C9C,#B8B8B0"
#elif defined(BOARD_TRMNL_4CLR)
#define DEVICE_BOARD "trmnl_og_4clr"
#define DISPLAY_COLORS "#000000,#FFFFFF,#FF0000,#FFFF00"
#define DISPLAY_MEASURED_COLORS "#383038,#B8B8B0,#9C484B,#D0BE47"
#elif defined(BOARD_XTEINK_X4)
#define DEVICE_BOARD "xteink_x4"
#define DISPLAY_COLORS "#000000,#555555,#AAAAAA,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#6E6E6E,#9C9C9C,#B8B8B0"
#elif defined(BOARD_TRMNL_X)
#define DEVICE_BOARD "trmnl_x"
#define DISPLAY_COLORS "#000000,#111111,#222222,#333333,#444444,#555555,#666666,#777777,#888888,#999999,#AAAAAA,#BBBBBB,#CCCCCC,#DDDDDD,#EEEEEE,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#434343,#4E4E4E,#595959,#646464,#6E6E6E,#787878,#828282,#8B8B8B,#949494,#9C9C9C,#A3A3A3,#AAAAAA,#B0B0B0,#B5B5B2,#B8B8B0"
#elif defined(BOARD_WAVESHARE_ESP32_DRIVER)
#define DEVICE_BOARD "waveshare"
#define DISPLAY_COLORS "#000000,#555555,#AAAAAA,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#6E6E6E,#9C9C9C,#B8B8B0"
#elif defined(BOARD_SEEED_XIAO_ESP32C3)
#define DEVICE_BOARD "seeed_xiao_c3"
#define DISPLAY_COLORS "#000000,#555555,#AAAAAA,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#6E6E6E,#9C9C9C,#B8B8B0"
#elif defined(BOARD_SEEED_XIAO_ESP32S3)
#define DEVICE_BOARD "seeed_xiao_s3"
#define DISPLAY_COLORS "#000000,#555555,#AAAAAA,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#6E6E6E,#9C9C9C,#B8B8B0"
#elif defined(BOARD_XIAO_EPAPER_DISPLAY)
#define DEVICE_BOARD "xiao_epaper"
#define DISPLAY_COLORS "#000000,#555555,#AAAAAA,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#6E6E6E,#9C9C9C,#B8B8B0"
#elif defined(BOARD_XIAO_EPAPER_DISPLAY_3CLR)
#define DEVICE_BOARD "xiao_epaper_3clr"
#define DISPLAY_COLORS "#000000,#FFFFFF,#FF0000"
#define DISPLAY_MEASURED_COLORS "#383038,#B8B8B0,#9C484B"
#elif defined(BOARD_SEEED_RETERMINAL_E1001)
#define DEVICE_BOARD "reterminal_e1001"
#define DISPLAY_COLORS "#000000,#555555,#AAAAAA,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#6E6E6E,#9C9C9C,#B8B8B0"
#elif defined(BOARD_SEEED_RETERMINAL_E1002)
#define DEVICE_BOARD "reterminal_e1002"
#define DISPLAY_COLORS "#000000,#FFFFFF,#FF0000,#FFFF00,#0000FF,#00FF00"
#define DISPLAY_MEASURED_COLORS "#383038,#B8B8B0,#9C484B,#D0BE47,#3D3B5E,#3A5B46"
#else
#define DEVICE_BOARD "unknown"
#define DISPLAY_COLORS "#000000,#FFFFFF"
#define DISPLAY_MEASURED_COLORS "#383838,#B8B8B0"
#endif

#if defined(BOARD_XIAO_EPAPER_DISPLAY) || defined(BOARD_SEEED_RETERMINAL_E1001) || defined(BOARD_SEEED_RETERMINAL_E1002)
#define PIN_BATTERY 1
#elif defined(BOARD_XTEINK_X4)
#define PIN_BATTERY 0
#else
#define PIN_BATTERY 3
#endif

// #define FAKE_BATTERY_VOLTAGE // Uncomment to report 4.2V instead of reading ADC

#define BUTTON_HOLD_TIME 5000
#define BUTTON_MEDIUM_HOLD_TIME 1000
#define BUTTON_SOFT_RESET_TIME 15000
#define BUTTON_DOUBLE_CLICK_WINDOW 800

#define SERVER_MAX_RETRIES 3
#define API_BASE_URL "https://trmnl.app"

#endif
