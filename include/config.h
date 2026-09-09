#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>

#define FW_MAJOR_VERSION 1
#define FW_MINOR_VERSION 8
#define FW_PATCH_VERSION 16

// Helper macros for stringification
#define STRINGIFY(x)     #x
#define TOSTRING(x)      STRINGIFY(x)

#ifndef FW_COMMIT
#define FW_COMMIT ""
#endif

// Compile-time firmware version string
#define FW_VERSION_STRING    TOSTRING(FW_MAJOR_VERSION) "." TOSTRING(FW_MINOR_VERSION) "." TOSTRING(FW_PATCH_VERSION)

#define LOG_MAX_NOTES_NUMBER 10

#define PREFERENCES_API_KEY  "api_key"
#define PREFERENCES_API_KEY_DEFAULT          ""
#define PREFERENCES_API_URL                  "api_url"
#define PREFERENCES_FRIENDLY_ID              "friendly_id"
#define PREFERENCES_FRIENDLY_ID_DEFAULT      ""
#define PREFERENCES_HOSTNAME                 "hostname"
#define PREFERENCES_TEMP_PROFILE             "temp_profile"
#define PREFERENCES_LOG_KEY                  "log_"
#define PREFERENCES_LOG_BUFFER_HEAD_KEY      "log_head"
#define PREFERENCES_LOG_ID_KEY               "log_id"
#define PREFERENCES_DEVICE_REGISTERED_KEY    "plugin"
#define PREFERENCES_SF_KEY                   "sf"
#define PREFERENCES_FILENAME_KEY             "filename"
#define PREFERENCES_CURRENT_PATH_KEY         "curr_path"
#define PREFERENCES_LAST_PATH_KEY            "last_path"
#define PREFERENCES_PLAYLIST_ORDER_KEY       "playlist_order"
#define PREFERENCES_BROWSE_PATH_KEY          "browse_path"
#define MAX_CACHED_IMAGES                    30
#define PREFERENCES_LAST_SLEEP_TIME          "last_sleep"
#define PREFERENCES_CONNECT_API_RETRY_COUNT  "retry_count"
#define PREFERENCES_CONNECT_WIFI_RETRY_COUNT "wifi_retry"
#define PREFERENCES_TOUCHBAR_MODE_KEY        "touchbar_mode"
#define PREFERENCES_LAST_OTA                 "last_ota"

#define WIFI_CONNECTION_RSSI                 (-100)

#define DISPLAY_BMP_IMAGE_SIZE 48062 // in bytes - 62 bytes - header; 48000 bytes - bitmap (480*800 1bpp) / 8
#define DEFAULT_IMAGE_SIZE     48000
#if defined(BOARD_HAS_PSRAM)
#define MAX_IMAGE_SIZE 750000 // Use PSRAM on the ESP32-S3/C5 (all X-class boards have PSRAM)
#else
#define MAX_IMAGE_SIZE 90000 // largest compressed image we can receive in static RAM
#endif
#define SLEEP_uS_TO_S_FACTOR 1000000           /* Conversion factor for micro seconds to seconds */

// Different display profiles
#define TEMP_PROFILE_DEFAULT 0
#define TEMP_PROFILE_A       1
#define TEMP_PROFILE_B       2
#define TEMP_PROFILE_C       3

#define MS_TO_S_FACTOR       1000 /* Conversion factor for milliseconds to seconds */

#if defined(BOARD_TRMNL_X)
#define PIN_INTERRUPT      3
#define PIN_INTERNAL_SDA   39
#define PIN_INTERNAL_SCL   40
#define PIN_INTERNAL_READY GPIO_NUM_3
// TCA9535 expander pins for the BQ25616 charger (open-drain, LOW = active)
#define TCA9535_PG_PIN     0 // P0_0 — LOW = VBUS OK
#define TCA9535_STAT_PIN   2 // P0_2 — LOW = charging in progress
#elif defined(BOARD_TRMNL_X_EPDIY)
#define PIN_INTERRUPT 0
#define DEVICE_MODEL  "x"
#elif defined(BOARD_TRMNL_X_SENSORIAS3)
#define PIN_INTERRUPT 0
#define DEVICE_MODEL  "sensoria_s3"
#define SENSOR_SDA    39
#define SENSOR_SCL    40
#elif defined(BOARD_ESP32_C5_DEVKITC_1)
#define PIN_INTERRUPT 28
#define DEVICE_MODEL  "gen-2"
#elif defined(BOARD_WAVESHARE_ESP32_DRIVER)
#define PIN_INTERRUPT 33
#define DEVICE_MODEL  "waveshare"
#define FAKE_BATTERY_VOLTAGE
#elif defined(BOARD_SEEED_XIAO_ESP32C3)
#define DEVICE_MODEL "seeed_esp32c3"
#define PIN_INTERRUPT                                                                                                  \
  9 // the boot button on the XIAO ESP32-C3, this button can't be used as wakeup  source though
                        // because it's not in the RTC GPIO group. Instead, you can always use the reset button to
    // wake up the device. Resetting WiFi configuration needs special routine - press reset button
    // then press the boot button in less than 2 seconds, and hold it for 5 seconds.
#define FAKE_BATTERY_VOLTAGE
#elif defined(BOARD_SEEED_XIAO_ESP32S3)
#define DEVICE_MODEL  "seeed_esp32s3"
#define PIN_INTERRUPT 0 // the boot button on the XIAO ESP32-S3, this button works as regular wakeup button
#define FAKE_BATTERY_VOLTAGE
#endif

// DHCP hostname prefix (hyphens instead of spaces).
#if defined(PARALLEL_EPD)
#define WIFI_CLIENT_HOSTNAME_PREFIX "TRMNL-X"
#elif defined(BOARD_TRMNL) || defined(BOARD_TRMNL_GEN2)
#define WIFI_CLIENT_HOSTNAME_PREFIX "TRMNL-OG"
#elif defined(BOARD_TRMNL_4CLR)
#define WIFI_CLIENT_HOSTNAME_PREFIX "TRMNL-BWRY"
#else
#define WIFI_CLIENT_HOSTNAME_PREFIX "TRMNL"
#endif

#if defined(BOARD_XIAO_EPAPER_DISPLAY) || defined(BOARD_SEEED_RETERMINAL_E1001) ||                                     \
  defined(BOARD_SEEED_RETERMINAL_E1002) || defined(BOARD_SEEED_RETERMINAL_E1003)
#define PIN_BATTERY 1
#elif defined(BOARD_XTEINK_X4)
#define PIN_BATTERY 0
#else
#define PIN_BATTERY 3
#endif

// #define FAKE_BATTERY_VOLTAGE // Uncomment to report 4.2V instead of reading ADC

#define BUTTON_HOLD_TIME                   5000
#define BUTTON_MEDIUM_HOLD_TIME            1000
#define BUTTON_SOFT_RESET_TIME             15000
#define BUTTON_DOUBLE_CLICK_WINDOW         800

#define SERVER_MAX_RETRIES                 3
#define API_BASE_URL                       "https://trmnl.app"

// Abort an image download when the stream goes this long with no data.
#define IMAGE_STREAM_INACTIVITY_TIMEOUT_MS 15000

// Battery measurement types
// BATT_NONE = use fake voltage
enum { BATT_NONE = 0, BATT_ADC, BATT_BQ27220, BATT_BQ27427 };

#ifdef PARALLEL_EPD
// TRMNL Device structure (slightly different for parallel eink panels)
typedef struct tag_trmnl_device {
  const char *device_name;
  int iBoardType;
  int iPanelSize; // set to 0 if the board type already includes the panel type (e.g. M5 PaperS3)
  uint8_t sensor_sda;
  uint8_t sensor_scl;
  uint8_t interrupt_pin;
  uint8_t batt_pin;
  uint8_t batt_en_pin;
  uint8_t batt_type; // ADC, BQxxx
} TRMNL_DEVICE;
#else
// TRMNL Device structure - defines the GPIO connections for the display, button and battery
// Only needed on "OG" class devices with SPI ePaper displays
typedef struct tag_trmnl_device {
  const char *device_name;
  uint8_t epd_sck_pin;
  uint8_t epd_mosi_pin;
  uint8_t epd_cs_pin;
  uint8_t epd_rst_pin;
  uint8_t epd_dc_pin;
  uint8_t epd_busy_pin;
  uint8_t sensor_sda;
  uint8_t sensor_scl;
  uint8_t interrupt_pin;
  uint8_t batt_pin;
  uint8_t batt_en_pin;
  uint8_t batt_type; // ADC, BQ27xx
  uint8_t panel_set;
} TRMNL_DEVICE;
#endif // PARALLEL_EPD

// This enum defines sets of display configurations for different SPI panel types
// These sets are what define the temperature profile - 3 per size (default, A, B)
enum {
  EPD_75 = 0,
  EPD_426,
  EPD_397,
  EPD_75_3CLR,
  EPD_75_4CLR,
  EPD_75_6CLR,
  EPD_CROWPANEL,
  EPD_583,
  EPD_PAPER_MONO,
  EPD_PAPER_COLOR,
  EPD_133_COLOR,
};

/**
 * data
 **/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

#if defined(BOARD_SEEED_RETERMINAL_E1003)
#define EPD_SCK_PIN  7
#define EPD_MOSI_PIN 9
#define EPD_MISO_PIN 8
#define EPD_CS_PIN   10
#define EPD_RST_PIN  12
#define EPD_EN_PIN   11
#define EPD_BUSY_PIN 13
#define EPD_VCC_EN   21
#endif

#define GPIO_PIN_SET                    1

/**
 * GPIO read and write
 **/
#define DEV_Digital_Write(_pin, _value) digitalWrite(_pin, _value == 0 ? LOW : HIGH)
#define DEV_Digital_Read(_pin)          digitalRead(_pin)

/**
 * delay x ms
 **/
#define DEV_Delay_ms(__xms)             delay(__xms)

/*------------------------------------------------------------------------------------------------------*/
UBYTE DEV_Module_Init(void);
void DEV_SPI_WriteByte(UBYTE data);

#endif
