#include <Arduino.h>
#include <WiFi.h>
#include <bl.h>
#include <trmnl_log.h>
#include <types.h>
#include <ArduinoLog.h>
#include <WifiCaptive.h>
#include <pins.h>
#include <config.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <display.h>
#include <stdlib.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <cstdint>
#include "png.h"
#include <bmp.h>
#include <Update.h>
#include <math.h>
#include <filesystem.h>
#include "trmnl_log.h"
#include <stored_logs.h>
#include <button.h>
#include "api-client/submit_log.h"
#include <api-client/setup.h>
#include <special_function.h>
#include <api_response_parsing.h>
#include "logging_parcers.h"
#include <SPIFFS.h>
#include "http_client.h"
#include <api-client/display.h>
#include "driver/gpio.h"
#include <nvs.h>
#include <serialize_log.h>
#include <preferences_persistence.h>
#include "logo_small.h"
#include "logo_medium.h"
#include "loading.h"
#include <wifi-helpers.h>

bool pref_clear = false;
String new_filename = "";
ApiDisplayResult apiDisplayResult;
uint8_t *buffer = nullptr;
char filename[1024];      // image URL
char binUrl[1024];        // update URL
char message_buffer[128]; // message to show on the screen
uint32_t time_since_sleep;
image_err_e png_res = PNG_DECODE_ERR;
bmp_err_e bmp_res = BMP_NOT_BMP;
static float vBatt;
bool status = false;          // need to download a new image
bool update_firmware = false; // need to download a new firmware
bool reset_firmware = false;  // need to reset credentials
bool send_log = false;        // need to send logs
bool double_click = false;
bool log_retry = false;                                              // need to log connection retry
esp_sleep_wakeup_cause_t wakeup_reason = ESP_SLEEP_WAKEUP_UNDEFINED; // wake-up reason
MSG current_msg = NONE;
SPECIAL_FUNCTION special_function = SF_NONE;
RTC_DATA_ATTR uint8_t need_to_refresh_display = 1;

Preferences preferences;
PreferencesPersistence preferencesPersistence(preferences);
StoredLogs storedLogs(LOG_MAX_NOTES_NUMBER / 2, LOG_MAX_NOTES_NUMBER / 2, PREFERENCES_LOG_KEY, PREFERENCES_LOG_BUFFER_HEAD_KEY, preferencesPersistence);

static https_request_err_e downloadAndShow(); // download and show the image
static uint32_t downloadStream(WiFiClient *stream, int content_size, uint8_t *buffer);
static https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse);
static void getDeviceCredentials();                  // receiveing API key and Friendly ID
static bool performApiSetup();     // perform API setup call and return success
static void downloadSetupImage();                    // download and display setup image
static void resetDeviceCredentials(void);            // reset device credentials API key, Friendly ID, Wi-Fi SSID and password
static void checkAndPerformFirmwareUpdate(void);     // OTA update
static void goToSleep(void);                         // sleep preparing
static bool setClock(void);                          // clock synchronization
static float readBatteryVoltage(void);               // battery voltage reading
static void submitStoredLogs(void);
static void writeSpecialFunction(SPECIAL_FUNCTION function);
static void writeImageToFile(const char *name, uint8_t *in_buffer, size_t size);
static void showMessageWithLogo(MSG message_type);
static void showMessageWithLogo(MSG message_type, String friendly_id, bool id, const char *fw_version, String message);
static void showMessageWithLogo(MSG message_type, const ApiSetupResponse &apiResponse);
static void wifiErrorDeepSleep();
static uint8_t *storedLogoOrDefault(int iType);
static bool saveCurrentFileName(String &name);
static bool checkCurrentFileName(String &newName);
static DeviceStatusStamp getDeviceStatusStamp();
void log_nvs_usage();

static unsigned long startup_time = 0;

void wait_for_serial() {
#ifdef WAIT_FOR_SERIAL
  int idx = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) {
      if (Serial)
        break;
      delay(100);
      idx++;
    }
  Log_info("## Waited for serial.. %d ms", idx * 100);
#endif
}

#ifdef BOARD_TRMNL_X
// ############################ WAKEUP STUB #############################
#include "rtc_wake_stub_trmnl_x.h"
// ############################ WAKEUP STUB #############################

// ############################ SLIDER #############################
#include "IQS323.h"
IQS323 iqs323;
#define IQS323_I2C_ADDRESS 0x44
// Sensor states
iqs323_ch_states button_states[3];
uint16_t slider_position = 65535;
iqs323_gesture_events slider_event = IQS323_GESTURE_NONE;

bool otg_message = false;

#define SENSOR_SDA_PIN 39
#define SENSOR_SCL_PIN 40
#define SENSOR_READY_PIN GPIO_NUM_3

void check_channel_states(void)
{
  bool otg_turned_on = false;
  /* Loop through all the active channels */
  for (uint8_t i = 0; i < 3; i++) {
    /* Check if the touch state bit is set */
    if (iqs323.channel_touchState((iqs323_channel_e)(i))) {
      if (button_states[i] != IQS323_CH_TOUCH && (slider_event == IQS323_GESTURE_TAP || slider_event == IQS323_GESTURE_HOLD)) {
        printf("CH: %d: Touch\n", i);
        switch (i) {
        case 0:
          Serial.println("Next button pressed");
          break;
        case 1:
          Serial.println("Middle button pressed");
          otg_turned_on = turn_otg();
          Serial.printf("Free heap before drawing message with logo: %u bytes, max heap chunk: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
          if (otg_turned_on) {
            showMessageWithLogo(OTG_TURNED_ON);
          }
          else {
            showMessageWithLogo(OTG_TURNED_OFF);
          }
          otg_message = true;
          Serial.printf("Free heap after drawing message with logo: %u bytes, max heap chunk: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
          break;
        case 2:
          Serial.println("Back button pressed");
          break;
        }
        button_states[i] = IQS323_CH_TOUCH;
      }
    }
  }
}

void read_slider_coordinates(void)
{
  /* read slider coordinates from memory */
  uint16_t buffer = iqs323.sliderCoordinate();

  if(buffer != slider_position)
  {
    slider_position = buffer;
  }
}

/* Function to process Slider gesture events */
void read_gesture_event(void)
{
  /* Read slider bit to check if a slider event occurred */
  bool gesture_event = iqs323.getSliderEvent();
  printf("Gesture event: %d\n", gesture_event);
  if (gesture_event)
  {
    /* returns slider event that occurred (tap, swipe or flick) by reading event bits from MM */
    iqs323_gesture_events gesture_buffer = iqs323.getGestureType();
    printf("Gesture type: %d\n", gesture_buffer);
    if(slider_event != gesture_buffer)
    {
      slider_event = gesture_buffer;
      int file_size = 0;
      switch (slider_event)
      {
        case IQS323_GESTURE_UNKNOWN:
          Serial.println("SLIDER: UNKNOWN (something went wrong?)");
          break;
        case IQS323_GESTURE_TAP:
          Serial.println("SLIDER: Tap");
          break;
        case IQS323_GESTURE_SWIPE_NEGATIVE:
          Serial.println("SLIDER: Swipe ->");
          buffer = display_read_file("/current.png", &file_size);
          if (!buffer || file_size == 0) {
            Serial.println("No previous image found");
            break;
          }
          Serial.printf("Drawing current plugin... File size: %d\n", file_size);
          display_show_image(buffer, file_size, false);
          goToSleep();
          break;
        case IQS323_GESTURE_SWIPE_POSITIVE:
          Serial.println("SLIDER: Swipe <-");
          buffer = display_read_file("/last.png", &file_size);
          if (!buffer || file_size == 0) {
            Serial.println("No previous image found");
            break;
          }
          Serial.printf("Drawing previous plugin... File size: %d\n", file_size);
          display_show_image(buffer, file_size, false);
          goToSleep();
          break;
        case IQS323_GESTURE_FLICK_NEGATIVE:
          Serial.println("SLIDER: Flick ->");
          break;
        case IQS323_GESTURE_FLICK_POSITIVE:
          Serial.println("SLIDER: Flick <-");
          break;
        case IQS323_GESTURE_HOLD:
          Serial.println("SLIDER: Hold");
          break;
        case IQS323_GESTURE_NONE:
          Serial.println("SLIDER: None");
          break;
      }

      /* Clear event if a finger is removed from slider after the event was processed */
      if (slider_position == 65535)
      {
        slider_event = IQS323_GESTURE_NONE;
      }
    }
  }
}
// ############################ SLIDER #############################

// ############################ ACCELEROMETER #############################
#include "bma530_features.h"

#define BMA530_I2C_ADDRESS  0x18

#define TCA9535_INT 38

// Orientation output definitions
#define FACE_UP            0x00
#define FACE_DOWN          0x01
#define PORTRAIT_UP_RIGHT  0x00
#define LANDSCAPE_LEFT     0x01
#define PORTRAIT_UP_DOWN   0x02
#define LANDSCAPE_RIGHT    0x03

// Global device structure
struct bma5_dev bma530_dev;

// Volatile flag for interrupt
volatile bool orientation_interrupt_occurred = false;

/**
 * INT1 interrupt handler
 */
void IRAM_ATTR bma530_int1_handler() {
    orientation_interrupt_occurred = true;
}

/**
 * I2C read function for BMA530
 */
int8_t bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    if (reg_data == NULL) {
        return -1;
    }

    Wire.beginTransmission(BMA530_I2C_ADDRESS);
    Wire.write(reg_addr);

    if (Wire.endTransmission(false) != 0) {
        return -1;
    }

    Wire.requestFrom(BMA530_I2C_ADDRESS, (uint8_t)length);

    for (uint32_t i = 0; i < length; i++) {
        if (Wire.available()) {
            reg_data[i] = Wire.read();
        } else {
            return -1;
        }
    }

    return 0;
}

/**
 * I2C write function for BMA530
 */
int8_t bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    if (reg_data == NULL) {
        return -1;
    }

    Wire.beginTransmission(BMA530_I2C_ADDRESS);
    Wire.write(reg_addr);

    for (uint32_t i = 0; i < length; i++) {
        Wire.write(reg_data[i]);
    }

    if (Wire.endTransmission() != 0) {
        return -1;
    }

    return 0;
}

/**
 * Delay function for BMA530
 */
void bma5_delay_us(uint32_t period_us, void *intf_ptr) {
    delayMicroseconds(period_us);
}

/**
 * Initialize the BMA530 device
 */
int8_t bma530_init_device(struct bma5_dev *dev) {
    int8_t rslt;

    // Configure device structure for I2C interface
    dev->intf = BMA5_I2C_INTF;
    dev->bus_read = bma5_i2c_read;
    dev->bus_write = bma5_i2c_write;
    dev->delay_us = bma5_delay_us;
    dev->intf_ptr = NULL;
    dev->context = BMA5_SMARTPHONE;  // Or BMA5_WEARABLE/BMA5_HEARABLE

    // Initialize the sensor
    rslt = bma530_init(dev);
    if (rslt != BMA5_OK) {
        Serial.printf("BMA530 initialization failed: %d\n", rslt);
        return rslt;
    }

    Serial.println("BMA530 initialized successfully");
    Serial.printf("Chip ID: 0x%02X\n", dev->chip_id);

    return BMA5_OK;
}

/**
 * Configure accelerometer for low power mode
 */
int8_t bma530_configure_low_power_mode(struct bma5_dev *dev) {
    int8_t rslt;
    struct bma5_acc_conf acc_cfg;
    uint8_t sensor_ctrl;

    // Get current accelerometer configuration
    rslt = bma5_get_acc_conf(&acc_cfg, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to get acc config: %d\n", rslt);
        return rslt;
    }

    // Configure for Low Power Mode
    acc_cfg.power_mode = BMA5_POWER_MODE_LPM;        // Low Power Mode (duty cycling)
    acc_cfg.acc_odr = BMA5_ACC_ODR_HZ_25;            // 25 Hz ODR (valid for LPM)
    acc_cfg.acc_bwp = BMA5_ACC_BWP_NORM_AVG4;        // Normal averaging
    acc_cfg.acc_range = BMA5_ACC_RANGE_MAX_4G;       // 4G range
    acc_cfg.noise_mode = BMA5_NOISE_MODE_LOWER_POWER;// Lower power noise mode
    acc_cfg.acc_drdy_int_auto_clear = BMA5_ACC_DRDY_INT_AUTO_CLEAR_ENABLED;

    // Apply configuration
    rslt = bma5_set_acc_conf(&acc_cfg, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to set acc config: %d\n", rslt);
        return rslt;
    }

    // Enable accelerometer
    sensor_ctrl = BMA5_SENSOR_CTRL_ENABLE;
    rslt = bma5_set_acc_conf_0(sensor_ctrl, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to enable accelerometer: %d\n", rslt);
        return rslt;
    }

    Serial.println("Low power mode configured:");
    Serial.println("  Power Mode: Low Power Mode (Duty Cycling)");
    Serial.println("  ODR: 25 Hz");
    Serial.println("  Range: 4G");
    Serial.println("  Noise Mode: Lower Power");

    return BMA5_OK;
}

/**
 * Configure orientation detection
 */
int8_t bma530_configure_orientation(struct bma5_dev *dev) {
    int8_t rslt;
    struct bma530_orient conf;
    struct bma530_feat_eng_gpr_0 gpr_0;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;

    // Get current orientation configuration
    rslt = bma530_get_orient_config(&conf, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to get orientation config: %d\n", rslt);
        return rslt;
    }

    // Configure orientation parameters
    conf.ud_en = 0;             // Enable upside-down (face up/down) detection
    conf.mode = 0;              // Symmetric mode
    conf.blocking = 0;          // No blocking during movement
    conf.theta = 0x27;          // Tilt angle threshold (default: 0x27 = 39)
    conf.hold_time = 0x5;       // Hold time for confirmation (default: 0x5)
    conf.slope_thres = 0xCD;    // Slope threshold to prevent false detection (default: 0xCD = 205)
    conf.hysteresis = 0x20;     // Hysteresis value (default: 0x20 = 32)

    // Apply orientation configuration
    rslt = bma530_set_orient_config(&conf, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to set orientation config: %d\n", rslt);
        return rslt;
    }

    // Enable orientation feature in feature engine
    rslt = bma530_get_feat_eng_gpr_0(&gpr_0, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to get feature engine GPR: %d\n", rslt);
        return rslt;
    }

    // Disable ALL other features - ensure ONLY orientation triggers interrupts
    gpr_0.gen_int1_en = 0x00;
    gpr_0.gen_int2_en = 0x00;
    gpr_0.gen_int3_en = 0x00;
    gpr_0.step_en = 0x00;
    gpr_0.sig_mo_en = 0x00;
    gpr_0.tilt_en = 0x00;
    gpr_0.acc_foc_en = 0x00;

    // Enable ONLY orientation detection
    gpr_0.orient_en = 0x01;

    rslt = bma530_set_feat_eng_gpr_0(&gpr_0, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to enable orientation feature: %d\n", rslt);
        return rslt;
    }

    // Set feature engine control to host
    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to set feature engine control: %d\n", rslt);
        return rslt;
    }

    Serial.println("Orientation detection configured:");
    Serial.println("  Face up/down detection: Enabled");
    Serial.println("  Mode: Symmetric");
    Serial.printf("  Theta: 0x%02X\n", conf.theta);
    Serial.printf("  Hold time: 0x%02X\n", conf.hold_time);
    Serial.printf("  Slope threshold: 0x%02X\n", conf.slope_thres);
    Serial.printf("  Hysteresis: 0x%02X\n", conf.hysteresis);

    return BMA5_OK;
}

/**
 * Configure INT1 pin for orientation interrupts
 */
int8_t bma530_configure_int1(struct bma5_dev *dev) {
    int8_t rslt;
    struct bma530_int_map int_map;
    struct bma5_int_conf_types int_config;

    // Get current interrupt mapping
    rslt = bma530_get_int_map(&int_map, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to get interrupt mapping: %d\n", rslt);
        return rslt;
    }

    // Clear ALL interrupt mappings first to ensure only orientation triggers
    int_map.acc_drdy_int_map = BMA530_ACC_DRDY_INT_MAP_UNMAPPED;
    int_map.fifo_wm_int_map = BMA530_FIFO_WM_INT_MAP_UNMAPPED;
    int_map.fifo_full_int_map = BMA530_FIFO_FULL_INT_MAP_UNMAPPED;
    int_map.gen_int1_int_map = BMA530_GEN_INT1_INT_MAP_UNMAPPED;
    int_map.gen_int2_int_map = BMA530_GEN_INT2_INT_MAP_UNMAPPED;
    int_map.gen_int3_int_map = BMA530_GEN_INT3_INT_MAP_UNMAPPED;
    int_map.step_det_int_map = BMA530_STEP_DET_INT_MAP_UNMAPPED;
    int_map.step_cnt_int_map = BMA530_STEP_CNT_INT_MAP_UNMAPPED;
    int_map.sig_mo_int_map = BMA530_SIG_MO_INT_MAP_UNMAPPED;
    int_map.tilt_int_map = BMA530_TILT_INT_MAP_UNMAPPED;
    int_map.acc_foc_int_map = BMA530_ACC_FOC_INT_MAP_UNMAPPED;
    int_map.feat_eng_err_int_map = BMA530_FEAT_ENG_ERR_INT_MAP_UNMAPPED;

    // Map ONLY orientation interrupt to INT1 pin
    int_map.orient_int_map = BMA530_ORIENT_INT_MAP_INT1;

    rslt = bma530_set_int_map(&int_map, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to set interrupt mapping: %d\n", rslt);
        return rslt;
    }

    // Configure INT1 hardware pin
    int_config.int_src = BMA5_INT_1;  // Configure INT1

    rslt = bma5_get_int_conf(&int_config, 1, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to get INT1 config: %d\n", rslt);
        return rslt;
    }

    // INT1 pin configuration (external pull-up to 3.3V on hardware)
    int_config.int_conf.int_mode = BMA5_INT1_MODE_LATCHED;       // Latched mode (stays low until cleared)
    int_config.int_conf.int_od = BMA5_INT1_OD_OPEN_DRAIN;        // Open-drain (for external pull-up)
    int_config.int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_LOW;      // Active low (pulls to GND)

    rslt = bma5_set_int_conf(&int_config, 1, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to set INT1 config: %d\n", rslt);
        return rslt;
    }

    Serial.println("INT1 configured:");
    Serial.println("  Orientation interrupt mapped to INT1");
    Serial.println("  Mode: Latched");
    Serial.println("  Output: Open-Drain");
    Serial.println("  Level: Active Low (pulls to GND with external pull-up)");

    return BMA5_OK;
}

/**
 * Process orientation change
 */
void bma530_process_orientation(struct bma5_dev *dev) {
    int8_t rslt;
    struct bma530_int_status_types int_status;
    struct bma530_feat_eng_feat_out feat_out;

    // Set interrupt source to INT1
    int_status.int_src = BMA530_INT_STATUS_INT1;

    // Read interrupt status
    rslt = bma530_get_int_status(&int_status, 1, dev);
    if (rslt != BMA5_OK) {
        Serial.printf("Failed to read interrupt status: %d\n", rslt);
        return;
    }

    // Check if orientation interrupt occurred
    if (int_status.int_status.orient_int_status & BMA5_ENABLE) {
        Serial.println("\n*** Orientation change detected! ***");

        // Read orientation output values
        rslt = bma530_get_feat_eng_feature_out(&feat_out, dev);
        if (rslt != BMA5_OK) {
            Serial.printf("Failed to read orientation data: %d\n", rslt);
            return;
        }

        uint8_t portrait_landscape = feat_out.orientation_portrait_landscape;
        uint8_t face_up_down = feat_out.orientation_face_up_down;

        // Print orientation state
        Serial.print("Orientation: ");
        switch (portrait_landscape) {
            case PORTRAIT_UP_RIGHT:
                Serial.print("Portrait Upright");
                break;
            case LANDSCAPE_LEFT:
                Serial.print("Landscape Left");
                break;
            case PORTRAIT_UP_DOWN:
                Serial.print("Portrait Upside Down");
                break;
            case LANDSCAPE_RIGHT:
                Serial.print("Landscape Right");
                break;
            default:
                Serial.print("Unknown");
        }

        Serial.print(" | ");

        switch (face_up_down) {
            case FACE_UP:
                Serial.print("Face Up");
                break;
            case FACE_DOWN:
                Serial.print("Face Down");
                break;
            default:
                Serial.print("Unknown");
        }

        Serial.println();

        // Clear interrupt status
        rslt = bma530_set_int_status(&int_status, 1, dev);
        if (rslt != BMA5_OK) {
            Serial.printf("Failed to clear interrupt status: %d\n", rslt);
        }
    }
}

// ############################ ACCELEROMETER #############################

#endif

/**
 * @brief Function to init business logic module
 * @param none
 * @return none
 */
void bl_init(void)
{
  startup_time = millis();
  Serial.begin(115200);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log_info("BL init success");
  pins_init();
  vBatt = readBatteryVoltage(); // Read the battery voltage BEFORE WiFi is turned on

#if defined(BOARD_SEEED_XIAO_ESP32C3)
  delay(2000);

  if (digitalRead(PIN_INTERRUPT) == LOW) {
    Log_info("Boot button pressed during startup, resetting WiFi credentials...");
    WifiCaptivePortal.resetSettings();
    Log_info("WiFi credentials reset completed");
  }
#endif

  wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO || wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 || wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)
  {
    Log_info("GPIO wakeup detected (%d)", wakeup_reason);
    #ifdef BOARD_TRMNL_X
    iqs323.begin(IQS323_I2C_ADDRESS, SENSOR_SDA_PIN, SENSOR_SCL_PIN, PIN_INTERRUPT, false);
    // iqs323.read_gesture();
    // iqs323.run();
    iqs323.setIQSMemoryMap(wakeup_stub_iqs_status);
    if (iqs323.checkReset()) {
      Serial.println("IQS323 Reset Occurred!\n");
      iqs323.new_data_available = false;
      iqs323.iqs323_state.state = IQS323_STATE_START;
      iqs323.iqs323_state.init_state = IQS323_INIT_VERIFY_PRODUCT;
    }
    else {
      iqs323.new_data_available = 1;
    }
    auto button = 2;
    #else
    auto button = read_button_presses();
    #endif
    // wait_for_serial();
    Log_info("GPIO wakeup (%d) -> button was read (%s)", wakeup_reason, ButtonPressResultNames[button]);
    switch (button)
    {
    case LongPress:
      Log_info("WiFi reset");
      WifiCaptivePortal.resetSettings();
      break;
    case DoubleClick:
      double_click = true;
      break;
    case NoAction:
      break;
    case SoftReset:
      resetDeviceCredentials();
    }
    Log_info("button handling end");
  }
  else
  {
    wait_for_serial();
    Log_info("Non-GPIO wakeup (%d) -> didn't read buttons", wakeup_reason);

    #ifdef BOARD_TRMNL_X

    // If we woke up not from GPIO, need to initialize the IQS323

    iqs323.begin(IQS323_I2C_ADDRESS, SENSOR_SDA_PIN, SENSOR_SCL_PIN, PIN_INTERRUPT, true);
    printf("IQS323 resetting device...\n");
    iqs323.SW_Reset(STOP);
    Serial.println("IQS323 Ready to configure!");

    while (true)
    {
      iqs323.run();
      if (iqs323.iqs323_state.init_state == IQS323_INIT_DONE) {
        iqs323.iqs323_state.state = IQS323_STATE_RUN;
        break;
      }
      else if (iqs323.iqs323_state.init_state == IQS323_INIT_NONE) {
        Serial.println("IQS323 Initialization Error! Reboot device...");
        ESP.restart();
        break;
      }
    }
    #endif
  }

  Log_info("preferences start");
  bool res = preferences.begin("data", false);
  if (res)
  {
    Log_info("preferences init success (%d free entries)", preferences.freeEntries());
    if (pref_clear)
    {
      res = preferences.clear(); // if needed to clear the saved data
      if (res)
        Log_info("preferences cleared success");
      else
        Log_fatal("preferences clearing error");
    }
  }
  else
  {
    Log_fatal("preferences init failed");
    ESP.restart();
  }
  Log_info("preferences end");

  if (double_click)
  { // special function reading
    if (preferences.isKey(PREFERENCES_SF_KEY))
    {
      Log.info("%s [%d]: SF saved. Reading...\r\n", __FILE__, __LINE__);
      special_function = (SPECIAL_FUNCTION)preferences.getUInt(PREFERENCES_SF_KEY, 0);
      Log.info("%s [%d]: Read special function - %d\r\n", __FILE__, __LINE__, special_function);
      switch (special_function)
      {
      case SF_IDENTIFY:
      {
        Log.info("%s [%d]: Identify special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_SLEEP:
      {
        Log.info("%s [%d]: Sleep special function...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_ADD_WIFI:
      {
        Log.info("%s [%d]: Add WiFi function...\r\n", __FILE__, __LINE__);
        WifiCaptivePortal.startPortal();
      }
      break;
      case SF_RESTART_PLAYLIST:
      {
        Log.info("%s [%d]: Restart Playlist special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_REWIND:
      {
        Log.info("%s [%d]: Rewind special function...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_SEND_TO_ME:
      {
        Log.info("%s [%d]: Send to me special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_GUEST_MODE:
      {
        Log.info("%s [%d]: Guest Mode special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
      }
      break;
      default:
        break;
      }
    }
    else
    {
      Log_error("SF not saved");
    }
  }
  Log.info("%s [%d]: Display init\r\n", __FILE__, __LINE__);
  display_init();

  // int8_t rslt;
  // Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
  // Wire.setClock(400000);  // 400kHz I2C clock
  // Serial.printf("I2C initialized (SDA: %d, SCL: %d)\n\n", SENSOR_SDA_PIN, SENSOR_SCL_PIN);

  // rslt = bma530_init_device(&bma530_dev);
  // if (rslt != BMA5_OK) {
  //   while (1) {
  //     Serial.println("Failed to initialize BMA530!");
  //     delay(10000);
  //   }
  // }

  // rslt = bma530_configure_low_power_mode(&bma530_dev);
  // if (rslt != BMA5_OK) {
  //   while (1) {
  //     Serial.println("Failed to configure low power mode!");
  //     delay(10000);
  //   }
  // }

  // rslt = bma530_configure_orientation(&bma530_dev);
  // if (rslt != BMA5_OK) {
  //   while (1) {
  //     Serial.println("Failed to configure orientation!");
  //     delay(10000);
  //   }
  // }

  // // Configure INT1 pin
  // rslt = bma530_configure_int1(&bma530_dev);
  // if (rslt != BMA5_OK) {
  //   while (1) {
  //     Serial.println("Failed to configure INT1!");
  //     delay(10000);
  //   }
  // }

  // config_bma530_interrupt();

  // pinMode(TCA9535_INT, INPUT);

  // while (true) {
  //   // Read pin states (active-low: 0=interrupt active, 1=idle)
  //   uint8_t tca_int = digitalRead(TCA9535_INT);
  //   uint8_t bma_int = pca9535_interrupt_clear();  // Read BMA530_INT1 from P03

  //   // Serial.printf("TCA9535_INT=%d, BMA530_INT1=%d\n", tca_int, bma_int);

  //   // Check if BMA530 interrupt is active (LOW=0)
  //   if (bma_int == 0) {
  //     Serial.println("\n*** BMA530 Interrupt detected! ***");

  //     // Process orientation (this will clear the BMA530 interrupt internally)
  //     bma530_process_orientation(&bma530_dev);

  //     // After clearing, BMA530_INT1 should return to HIGH (1)
  //     Serial.println("Interrupt cleared, waiting for next orientation change...\n");
  //   }

  //   delay(100);
  // }

  filesystem_init();

  #ifdef BOARD_TRMNL_X
  if (iqs323.new_data_available) {
    read_slider_coordinates();
    read_gesture_event();
    check_channel_states();
    iqs323.new_data_available = false;
  }
  #endif
  if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
  {
    Log.info("%s [%d]: Display TRMNL logo start\r\n", __FILE__, __LINE__);

    if (!otg_message) {
      display_show_image(storedLogoOrDefault(1), DEFAULT_IMAGE_SIZE, false);
    }

    need_to_refresh_display = 1;
    preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
    Log.info("%s [%d]: Display TRMNL logo end\r\n", __FILE__, __LINE__);
    preferences.putString(PREFERENCES_FILENAME_KEY, "");
  }

  Log_info("Firmware version %s", FW_VERSION_STRING);
  Log_info("Arduino version %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Log_info("ESP-IDF version %d.%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
  list_files();
  log_nvs_usage();

  // DEBUG - test message display
  // showMessageWithLogo(MSG_FORMAT_ERROR);
  // display_show_msg(storedLogoOrDefault(1), WIFI_CONNECT, "ABCDEF", true, FW_VERSION_STRING, "Hello World!");
  // wifiErrorDeepSleep();

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

// uncomment this to hardcode WiFi credentials (useful for testing wifi errors, etc.)
// #define HARDCODED_WIFI
#ifdef HARDCODED_WIFI
  WifiCredentials hardcodedCreds = {.ssid = "ssid-goes-here", .pswd = "password-goes-here"};
  Log_info("Hardcoded WiFi: connecting to SSID '%s'", hardcodedCreds.ssid.c_str());
  esp_wifi_set_max_tx_power(34);
  auto connectResult = WifiCaptivePortal.connect(hardcodedCreds);
  WiFi.waitForConnectResult(60000);
  Log_info("Hardcoded WiFi: connect result '%s'", wifiStatusStr(connectResult));
// goToSleep();
#else

  if (WifiCaptivePortal.isSaved())
  {
    // WiFi saved, connection
    Log.info("%s [%d]: WiFi saved\r\n", __FILE__, __LINE__);
    int connection_res = WifiCaptivePortal.autoConnect();

    Log.info("%s [%d]: Connection result: %d, WiFI Status: %d\r\n", __FILE__, __LINE__, connection_res, WiFi.status());

    // Check if connected
    if (connection_res)
    {
      String ip = String(WiFi.localIP());
      Log.info("%s [%d]:wifi_connection [DEBUG]: Connected: %s\r\n", __FILE__, __LINE__, ip.c_str());
      preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
    }
    else
    {
      if (current_msg != WIFI_FAILED)
      {
        showMessageWithLogo(WIFI_FAILED);
        current_msg = WIFI_FAILED;
      }

      Log_fatal_submit("Connection failed! WL Status: %d", WiFi.status());

      // Go to deep sleep
      wifiErrorDeepSleep();
    }
  }
  else
  {
    // WiFi credentials are not saved - start captive portal
    Log.info("%s [%d]: WiFi NOT saved\r\n", __FILE__, __LINE__);

    Log_info("FW version %s", FW_VERSION_STRING);

    showMessageWithLogo(WIFI_CONNECT, "", false, FW_VERSION_STRING, "");
    WifiCaptivePortal.setResetSettingsCallback(resetDeviceCredentials);
    res = WifiCaptivePortal.startPortal();
    if (!res)
    {
      WiFi.disconnect(true);

      showMessageWithLogo(WIFI_FAILED);

      Log_error("Failed to connect or hit timeout");

      // Go to deep sleep
      wifiErrorDeepSleep();
    }
    Log.info("%s [%d]: WiFi connected\r\n", __FILE__, __LINE__);
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
  }

#endif

  // clock synchronization
  if (setClock())
  {
    time_since_sleep = preferences.getUInt(PREFERENCES_LAST_SLEEP_TIME, 0);
    time_since_sleep = time_since_sleep ? getTime() - time_since_sleep : 0; // may be can be used even if no sync
  }
  else
  {
    time_since_sleep = 0;
    Log.info("%s [%d]: Time wasn't synced.\r\n", __FILE__, __LINE__);
  }

  Log.info("%s [%d]: Time since last sleep: %d\r\n", __FILE__, __LINE__, time_since_sleep);

  if (!preferences.isKey(PREFERENCES_API_KEY) || !preferences.isKey(PREFERENCES_FRIENDLY_ID))
  {
    Log.info("%s [%d]: API key or friendly ID not saved\r\n", __FILE__, __LINE__);
    // lets get the api key and friendly ID
    getDeviceCredentials();
  }
  else
  {
    Log.info("%s [%d]: API key and friendly ID saved\r\n", __FILE__, __LINE__);
  }

  submitStoredLogs();

  log_retry = true;

  // OTA checking, image checking and drawing
  https_request_err_e request_result = downloadAndShow();
  Log.info("%s [%d]: request result - %d\r\n", __FILE__, __LINE__, request_result);

  if (request_result == HTTPS_IMAGE_FILE_TOO_BIG)
  {
    showMessageWithLogo(MSG_TOO_BIG);
  }

  if (!preferences.isKey(PREFERENCES_CONNECT_API_RETRY_COUNT))
  {
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, 1);
  }

  if (request_result != HTTPS_SUCCESS && request_result != HTTPS_NO_ERR && request_result != HTTPS_NO_REGISTER && request_result != HTTPS_RESET && request_result != HTTPS_PLUGIN_NOT_ATTACHED)
  {
    uint8_t retries = preferences.getInt(PREFERENCES_CONNECT_API_RETRY_COUNT);

    switch (retries)
    {
    case 1:
      Log.info("%s [%d]: retry: %d - time to sleep: %d\r\n", __FILE__, __LINE__, retries, API_CONNECT_RETRY_TIME::API_FIRST_RETRY);
      res = preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, API_CONNECT_RETRY_TIME::API_FIRST_RETRY);
      preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, ++retries);
      display_sleep();
      goToSleep();
      break;

    case 2:
      Log.info("%s [%d]: retry:%d - time to sleep: %d\r\n", __FILE__, __LINE__, retries, API_CONNECT_RETRY_TIME::API_SECOND_RETRY);
      res = preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, API_CONNECT_RETRY_TIME::API_SECOND_RETRY);
      preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, ++retries);
      display_sleep();
      goToSleep();
      break;

    case 3:
      Log.info("%s [%d]: retry:%d - time to sleep: %d\r\n", __FILE__, __LINE__, retries, API_CONNECT_RETRY_TIME::API_THIRD_RETRY);
      res = preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, API_CONNECT_RETRY_TIME::API_THIRD_RETRY);
      preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, ++retries);
      display_sleep();
      goToSleep();
      break;

    default:
      Log.info("%s [%d]: Max retries done. Time to sleep: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_TO_SLEEP);
      preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP);
      preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, ++retries);
      break;
    }
  }

  else
  {
    Log_info("Connection done successfully. Retries counter reset.");
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, 1);
  }

  submitStoredLogs();

  if (request_result == HTTPS_NO_REGISTER && need_to_refresh_display == 1)
  {
    // show the image
    String friendly_id = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
    showMessageWithLogo(FRIENDLY_ID, friendly_id, true, "", String(message_buffer));
    need_to_refresh_display = 0;
  }

  // reset checking
  if (request_result == HTTPS_RESET)
  {
    Log.info("%s [%d]: Device reseting...\r\n", __FILE__, __LINE__);
    resetDeviceCredentials();
  }

  // OTA update checking
  if (update_firmware)
  {
    checkAndPerformFirmwareUpdate();
  }

  // error handling
  switch (request_result)
  {
  case HTTPS_REQUEST_FAILED:
  {
    if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
    {
      showMessageWithLogo(API_REQUEST_FAILED);
    }
    else
    {
      showMessageWithLogo(WIFI_WEAK);
    }
  }
  break;
  case HTTPS_RESPONSE_CODE_INVALID:
  {
    showMessageWithLogo(WIFI_INTERNAL_ERROR);
  }
  break;
  case HTTPS_UNABLE_TO_CONNECT:
  {
    if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
    {
      showMessageWithLogo(API_UNABLE_TO_CONNECT);
    }
    else
    {
      showMessageWithLogo(WIFI_WEAK);
    }
  }
  break;
  case HTTPS_WRONG_IMAGE_FORMAT:
  {
    showMessageWithLogo(MSG_FORMAT_ERROR);
  }
  break;
  case HTTPS_WRONG_IMAGE_SIZE:
  {
    if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
    {
      showMessageWithLogo(API_SIZE_ERROR);
    }
    else
    {
      showMessageWithLogo(WIFI_WEAK);
    }
  }
  break;
  case HTTPS_CLIENT_FAILED:
  {
    showMessageWithLogo(WIFI_INTERNAL_ERROR);
  }
  break;
  case HTTPS_PLUGIN_NOT_ATTACHED:
  {
    if (preferences.getInt(PREFERENCES_SLEEP_TIME_KEY, 0) != SLEEP_TIME_WHILE_PLUGIN_NOT_ATTACHED)
    {
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_PLUGIN_NOT_ATTACHED);
      preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_PLUGIN_NOT_ATTACHED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_PLUGIN_NOT_ATTACHED);
    }
  }
  break;
  default:
    break;
  }

  // display go to sleep
  display_sleep();
  if (!update_firmware)
    goToSleep();
  else
    ESP.restart();
}

/**
 * @brief Function to process business logic module
 * @param none
 * @return none
 */
void bl_process(void)
{
}

ApiDisplayInputs loadApiDisplayInputs(Preferences &preferences)
{
  ApiDisplayInputs inputs;

  inputs.baseUrl = preferences.getString(PREFERENCES_API_URL, API_BASE_URL);

  if (preferences.isKey(PREFERENCES_API_KEY))
  {
    inputs.apiKey = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY, inputs.apiKey.c_str());
  }
  else
  {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY);
  }

  if (preferences.isKey(PREFERENCES_FRIENDLY_ID))
  {
    inputs.friendlyId = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_FRIENDLY_ID, inputs.friendlyId);
  }
  else
  {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_FRIENDLY_ID);
  }

  inputs.refreshRate = SLEEP_TIME_TO_SLEEP;

  if (preferences.isKey(PREFERENCES_SLEEP_TIME_KEY))
  {
    inputs.refreshRate = preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP);
    Log.info("%s [%d]: %s key exists. Value - %d\r\n", __FILE__, __LINE__, PREFERENCES_SLEEP_TIME_KEY, inputs.refreshRate);
  }
  else
  {
    Log.info("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_SLEEP_TIME_KEY);
  }

  inputs.macAddress = WiFi.macAddress();

  inputs.batteryVoltage = vBatt; //readBatteryVoltage();

  inputs.firmwareVersion = String(FW_VERSION_STRING);

  inputs.rssi = WiFi.RSSI();
  inputs.displayWidth = display_width();
  inputs.displayHeight = display_height();
  inputs.model = DEVICE_MODEL;
  inputs.specialFunction = special_function;

  return inputs;
}

/**
 * @brief Function to ping server and download and show the image if all is OK
 * @param url Server URL address
 * @return https_request_err_e error code
 */
static https_request_err_e downloadAndShow()
{
  IPAddress serverIP;
  String apiHostname = preferences.getString(PREFERENCES_API_URL, API_BASE_URL);
  apiHostname.replace("https://", "");
  apiHostname.replace("http://", "");
  apiHostname.replace("/", "");

  int colon = apiHostname.indexOf(':');
  if (colon != -1) {
    apiHostname = apiHostname.substring(0, colon);
  }

  for (int attempt = 1; attempt <= 5; ++attempt)
  {
    if (WiFi.hostByName(apiHostname.c_str(), serverIP) == 1)
    {
      Log.info("%s [%d]: Hostname resolved to %s on attempt %d\r\n", __FILE__, __LINE__, serverIP.toString().c_str(), attempt);
      break;
    }
    else
    {
      Log_error("Failed to resolve hostname on attempt %d", attempt);
      delay(2000);
    }
  }

  auto apiDisplayInputs = loadApiDisplayInputs(preferences);

  apiDisplayResult = fetchApiDisplay(apiDisplayInputs);

  if (apiDisplayResult.error != HTTPS_NO_ERR)
  {
    Log_error_submit("Error fetching API display: %d, detail: %s", apiDisplayResult.error, apiDisplayResult.error_detail.c_str());
    return apiDisplayResult.error;
  }

  https_request_err_e result = handleApiDisplayResponse(apiDisplayResult.response);

  withHttp(
      filename,
      [&](HTTPClient *httpsp, HttpError error) -> https_request_err_e
      {
        if (error != HttpError::HTTPCLIENT_SUCCESS)
        {

          return HTTPS_UNABLE_TO_CONNECT;
        }

        HTTPClient &https = *httpsp;

        https.setTimeout(15000);
        https.setConnectTimeout(15000);

        https.addHeader("Accept-Encoding", "identity"); // Disable compression for raw image data

        // Include ID and Access Token if the image is hosted on the same server as the API
        if (strncmp(filename, apiDisplayInputs.baseUrl.c_str(), apiDisplayInputs.baseUrl.length()) == 0)
        {
          https.addHeader("ID", apiDisplayInputs.macAddress);
          https.addHeader("Access-Token", apiDisplayInputs.apiKey);
        }
        
        if (status && !update_firmware && !reset_firmware)
        {
          status = false;

          // The timeout will be zero if no value was returned, and in that case we just use the default timeout.
          // Otherwise, we set the requested timeout.
          uint32_t requestedTimeout = apiDisplayResult.response.image_url_timeout;
          if (requestedTimeout > 0)
          {
            // Convert from seconds to milliseconds.
            // A uint32_t should be large enough not to worry about overflow for any reasonable timeout.
            requestedTimeout *= MS_TO_S_FACTOR;
            if (requestedTimeout > UINT16_MAX)
            {
              // To avoid surprising behaviour if the server returned a timeout of more than 65 seconds
              // we will send a log message back to the server and truncate the timeout to the maximum.
              Log_info_submit("Requested image URL timeout too large (%d ms). Using maximum of %d ms.", requestedTimeout, UINT16_MAX);
              https.setTimeout(UINT16_MAX);
            }
            else
            {
              https.setTimeout(uint16_t(requestedTimeout));
            }
          }

          const char *headers[] = {"Content-Type"};
          https.collectHeaders(headers, 1);
          Log_info("GET...");
          Log_info("RSSI: %d", WiFi.RSSI());
          // start connection and send HTTP header
          int httpCode = https.GET();
          int content_size = https.getSize();
          if(httpCode == HTTP_CODE_PERMANENT_REDIRECT ||
            httpCode == HTTP_CODE_TEMPORARY_REDIRECT){
              https.end();
              https.begin(API_BASE_URL +https.getLocation());
              Log_info("Redirected to: %s", https.getLocation().c_str());
              https.setTimeout(15000);
              https.setConnectTimeout(15000);
              httpCode = https.GET();
              content_size = https.getSize();
            }
//          uint8_t *buffer_old = nullptr; // Disable partial update for now
//          int file_size_old = 0;

          // httpCode will be negative on error
          if (httpCode < 0)
          {
            Log_error_submit("[HTTPS] GET... failed, error: %d (%s)", httpCode, https.errorToString(httpCode).c_str());

            return HTTPS_REQUEST_FAILED;
          }

          // HTTP header has been send and Server response header has been handled
          Log.error("%s [%d]: [HTTPS] GET... code: %d\r\n", __FILE__, __LINE__, httpCode);
          Log.info("%s [%d]: RSSI: %d\r\n", __FILE__, __LINE__, WiFi.RSSI());
          // file found at server
          if (httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY)
          {
            Log_error_submit("[HTTPS] GET... failed, code: %d (%s)", httpCode, https.errorToString(httpCode).c_str());
            return HTTPS_REQUEST_FAILED;
          }
          
          Log.info("%s [%d]: Content size: %d\r\n", __FILE__, __LINE__, https.getSize());

          uint32_t counter = 0;

          if (content_size <= 0)
          {
            Log.warning("%s [%d]: Content-Length not provided (size: %d)\r\n", __FILE__, __LINE__, content_size);
          }

          bool isPNG = https.header("Content-Type") == "image/png";
          bool isJPEG = https.header("Content-Type") == "image/jpeg";

          Log.info("%s [%d]: Starting a download at: %d\r\n", __FILE__, __LINE__, getTime());
          heap_caps_check_integrity_all(true);

          // getString() handles chunked transfer encoding automatically
          String payload = https.getString();
          counter = payload.length();

          if (counter == 0)
          {
            Log_error_submit("Receiving failed. No data received");
            return HTTPS_WRONG_IMAGE_SIZE;
          }

          if (counter > MAX_IMAGE_SIZE)
          {
            Log_error_submit("Receiving failed; file size too big: %d", counter);
            return HTTPS_IMAGE_FILE_TOO_BIG;
          }

          buffer = (uint8_t *)malloc(counter);

          if (buffer == NULL)
          {
            Log_error_submit("Failed to allocate %d bytes for image buffer", counter);
            return HTTPS_OUT_OF_MEMORY;
          }

          memcpy(buffer, payload.c_str(), counter);
          content_size = counter;

          if (counter >= 2 && buffer[0] == 'B' && buffer[1] == 'M')
          {
            isPNG = false;
            Log.info("BMP file detected");
          }

          submitStoredLogs();

          WiFi.disconnect(true); // no need for WiFi, save power starting here
          Log.info("%s [%d]: Received successfully; WiFi off; WiFi off\r\n", __FILE__, __LINE__);


          if (filesystem_file_exists("/current.bmp") || filesystem_file_exists("/current.png"))
          {
            filesystem_file_delete("/last.bmp");
            filesystem_file_delete("/last.png");
            filesystem_file_rename("/current.png", "/last.png");
            filesystem_file_rename("/current.bmp", "/last.bmp");
// Disable partial update (for now)
//            if (filesystem_file_exists("/last.png")) {
//                buffer_old = display_read_file("/last.png", &file_size_old);
//                Log.info("%s [%d]: Reading last.png to use for partial update, size = %d\r\n", __FILE__, __LINE__, file_size_old);
//            }
          }

          bool image_reverse = false;
          if (isPNG || isJPEG)
          {
            writeImageToFile("/current.png", buffer, content_size);
            Log.info("%s [%d]: Decoding %s\r\n", __FILE__, __LINE__, (isPNG) ? "png" : "jpeg");
            display_show_image(buffer, content_size, true);
//            delay(100);
//            free(buffer);
//            buffer = nullptr;
//            png_res = decodePNG("/current.png", decodedPng);
            png_res = PNG_NO_ERR; // DEBUG
          }
          else
          {
            bmp_res = parseBMPHeader(buffer, image_reverse);
            Log.info("%s [%d]: BMP Parsing result: %d\r\n", __FILE__, __LINE__, bmp_res);
          }
          Serial.println("PNG res: " + String(png_res));
          String error = "";
         // uint8_t *imagePointer = buffer;
//          uint8_t *imagePointer = (decodedPng == nullptr) ? buffer : decodedPng;
        //  bool lastImageExists = filesystem_file_exists("/last.bmp") || filesystem_file_exists("/last.png");

          switch (png_res)
          {
          case PNG_NO_ERR:
          {

           // Log.info("Free heap at before display - %d", ESP.getMaxAllocHeap());
           // display_show_image(imagePointer, image_reverse, isPNG);

            // Using filename from API response
            new_filename = apiDisplayResult.response.filename;

            // Print the extracted string
            Log.info("%s [%d]: New filename - %s\r\n", __FILE__, __LINE__, new_filename.c_str());

            bool res = saveCurrentFileName(new_filename);
            if (res)
              Log.info("%s [%d]: New filename saved\r\n", __FILE__, __LINE__);
            else
              Log.error("%s [%d]: New image name saving error!", __FILE__, __LINE__);

            if (result != HTTPS_PLUGIN_NOT_ATTACHED)
              result = HTTPS_SUCCESS;
          }
          break;
          case PNG_WRONG_FORMAT:
          {
            error = "Wrong image format. Did not pass signature check";
          }
          break;
          case PNG_BAD_SIZE:
          {
            error = "IMAGE width, height or size are invalid";
          }
          break;
          case PNG_DECODE_ERR:
          {
            error = "could not decode png image";
          }
          break;
          case PNG_MALLOC_FAILED:
          {
            error = "could not allocate memory for png image decoder";
          }
          break;
          default:
            break;
          }

          switch (bmp_res)
          {
          case BMP_NO_ERR:
          {
            if (!filesystem_file_exists("/current.png"))
            {
              writeImageToFile("/current.bmp", buffer, content_size);
            }
            Log.info("Free heap at before display - %d", ESP.getMaxAllocHeap());
            display_show_image(buffer, content_size, true);

            // Using filename from API response
            new_filename = apiDisplayResult.response.filename;

            // Print the extracted string
            Log.info("%s [%d]: New filename - %s\r\n", __FILE__, __LINE__, new_filename.c_str());

            bool res = saveCurrentFileName(new_filename);
            if (res)
              Log.info("%s [%d]: New filename saved\r\n", __FILE__, __LINE__);
            else
              Log.error("%s [%d]: New image name saving error!", __FILE__, __LINE__);

            if (result != HTTPS_PLUGIN_NOT_ATTACHED)
              result = HTTPS_SUCCESS;
          }
          break;
          case BMP_NOT_BMP:
          {
            error = "First two header bytes are invalid!";
          }
          break;
          case BMP_BAD_SIZE:
          {
            error = "BMP width, height or size are invalid";
          }
          break;
          case BMP_COLOR_SCHEME_FAILED:
          {
            error = "BMP color scheme is invalid";
          }
          break;
          case BMP_INVALID_OFFSET:
          {
            error = "BMP header offset is invalid";
          }
          break;
          default:
            break;
          }

          if (isPNG && png_res != PNG_NO_ERR)
          {
            filesystem_file_delete("/current.png");
            Log_error_submit("error parsing image file - %s", error.c_str());

            return HTTPS_WRONG_IMAGE_FORMAT;
          }
        }

        return result;
      });

  if (result == HTTPS_UNABLE_TO_CONNECT)
  {
    Log_error_submit("unable to connect");
  }

  if (send_log)
  {
    send_log = false;
  }

  Log_info("Returned result - %d", result);

  return result;
}

uint32_t downloadStream(WiFiClient *stream, int content_size, uint8_t *buffer)
{
  int iteration_counter = 0;
  int counter2 = content_size;
  unsigned long download_start = millis();
  unsigned long last_data_time = millis();
  int counter = 0;

  while (counter < content_size && millis() - download_start < 30000)
  {
    if (stream->available())
    {
      Log.info("%s [%d]: Downloading... Available bytes: %d\r\n", __FILE__, __LINE__, stream->available());
      int bytes_to_read = min(stream->available(), counter2 - counter);
      counter += stream->readBytes(buffer + counter, bytes_to_read);
      iteration_counter++;
      last_data_time = millis();
    }
    else if (!stream->connected() || millis() - last_data_time > 5000)
    {
      break;
    }
    delay(10);
  }

  Log_info("Download end: %d/%d bytes in %d ms (%d iterations)", counter, content_size, millis() - download_start, iteration_counter);
  return counter;
}

https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse)
{
  https_request_err_e result = HTTPS_NO_ERR;
  int file_size = 0;

  if (special_function == SF_NONE)
  {
    uint64_t request_status = apiResponse.status;
    Log.info("%s [%d]: status: %d\r\n", __FILE__, __LINE__, request_status);
    switch (request_status)
    {
    case 0:
    {
      String image_url = apiResponse.image_url;
      update_firmware = apiResponse.update_firmware;
      String firmware_url = apiResponse.firmware_url;
      uint64_t rate = apiResponse.refresh_rate;
      reset_firmware = apiResponse.reset_firmware;

      bool sleep_5_seconds = false;

      writeSpecialFunction(apiResponse.special_function);

      if (update_firmware)
      {
        Log.info("%s [%d]: update firmware. Check URL\r\n", __FILE__, __LINE__);
        if (firmware_url.length() == 0)
        {
          Log.error("%s [%d]: Empty URL\r\n", __FILE__, __LINE__);
          update_firmware = false;
        }
      }
      if (image_url.length() > 0)
      {
        Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
        Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

        image_url.toCharArray(filename, image_url.length() + 1);
        // check if plugin is applied
        bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
        Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

        if (apiResponse.filename == "empty_state")
        {
          Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
          if (!flag)
          {
            // draw received logo
            status = true;
            // set flag to true
            if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != true) // check the flag to avoid the re-writing
            {
              bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
              if (res)
                Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
              else
                Log.error("%s [%d]: FLag writing failed\r\n", __FILE__, __LINE__);
            }
          }
          else
          {
            // don't draw received logo
            status = false;
          }
          // sleep 5 seconds
          sleep_5_seconds = true;
        }
        else
        {
          Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
          if (flag)
          {
            if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != false) // check the flag to avoid the re-writing
            {
              bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
              if (res)
                Log.info("%s [%d]: Flag written false successfully\r\n", __FILE__, __LINE__);
              else
                Log.error("%s [%d]: FLag writing failed\r\n", __FILE__, __LINE__);
            }
          }
          // Using filename from API response
          new_filename = apiResponse.filename;

          // Print the extracted string
          Log.info("%s [%d]: New filename - %s\r\n", __FILE__, __LINE__, new_filename.c_str());
          if (!checkCurrentFileName(new_filename))
          {
            Log.info("%s [%d]: New image. Show it.\r\n", __FILE__, __LINE__);
            status = true;
          }
          else
          {
            Log.info("%s [%d]: Old image. No needed to show it.\r\n", __FILE__, __LINE__);
            status = false;
            result = HTTPS_SUCCESS;
          }
        }
      }
      Log.info("%s [%d]: update_firmware: %d\r\n", __FILE__, __LINE__, update_firmware);
      if (firmware_url.length() > 0)
      {
        Log.info("%s [%d]: firmware_url: %s\r\n", __FILE__, __LINE__, firmware_url.c_str());
        firmware_url.toCharArray(binUrl, firmware_url.length() + 1);
      }
      Log.info("%s [%d]: refresh_rate: %d\r\n", __FILE__, __LINE__, rate);
      if (rate != preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP))
      {
        Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, rate);
        preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, rate);
        Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      }

      if (reset_firmware)
      {
        Log.info("%s [%d]: Reset status is true\r\n", __FILE__, __LINE__);
      }

      if (update_firmware)
        result = HTTPS_SUCCESS;
      if (reset_firmware)
        result = HTTPS_RESET;
      if (sleep_5_seconds)
        result = HTTPS_PLUGIN_NOT_ATTACHED;
      Log.info("%s [%d]: result - %d\r\n", __FILE__, __LINE__, result);
    }
    break;
    case 202:
    {
      result = HTTPS_NO_REGISTER;
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_NOT_CONNECTED);
      size_t result = preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_NOT_CONNECTED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      status = false;
    }
    break;
    case 500:
    {
      result = HTTPS_RESET;
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_NOT_CONNECTED);
      preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_NOT_CONNECTED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      status = false;
    }
    break;

    default:
      break;
    }
  }
  else if (special_function != SF_NONE)
  {
    uint64_t request_status = apiResponse.status;
    Log.info("%s [%d]: status: %d\r\n", __FILE__, __LINE__, request_status);
    switch (request_status)
    {
    case 0:
    {
      switch (special_function)
      {
      case SF_IDENTIFY:
      {
        String action = apiResponse.action;
        if (action.equals("identify"))
        {
          Log.info("%s [%d]:Identify success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          if (image_url.length() > 0)
          {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state")
            {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag)
              {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: FLag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              else
              {
                status = false;
              }
            }
            else
            {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag)
              {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != false) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
                  if (res)
                    Log.info("%s [%d]: Flag written false successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: FLag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              status = true;
            }
          }
        }
        else
        {
          Log.error("%s [%d]: identify failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_SLEEP:
      {
        String action = apiResponse.action;
        if (action.equals("sleep"))
        {
          uint64_t rate = apiResponse.refresh_rate;
          Log.info("%s [%d]: refresh_rate: %d\r\n", __FILE__, __LINE__, rate);
          if (rate != preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP))
          {
            Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, rate);
            preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, rate);
            Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
          }
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: sleep success\r\n", __FILE__, __LINE__);
        }
        else
        {
          Log.error("%s [%d]: sleep failed\r\n", __FILE__, __LINE__);
          // need to add error
        }
      }
      break;
      case SF_ADD_WIFI:
      {
        String action = apiResponse.action;
        if (action.equals("add_wifi"))
        {
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: Add wifi success\r\n", __FILE__, __LINE__);
        }
        else
        {
          Log.error("%s [%d]: Add wifi failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_RESTART_PLAYLIST:
      {
        String action = apiResponse.action;
        if (action.equals("restart_playlist"))
        {
          Log.info("%s [%d]:Restart playlist success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          if (image_url.length() > 0)
          {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state")
            {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag)
              {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: FLag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              else
              {
                // don't draw received logo
                status = false;
              }
            }
            else
            {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag)
              {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != false) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
                  if (res)
                    Log.info("%s [%d]: Flag written false successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: FLag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              status = true;
            }
          }
        }
        else
        {
          Log.error("%s [%d]: Restart playlist failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_REWIND:
      {
        String action = apiResponse.action;
        if (action.equals("rewind"))
        {
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: rewind success\r\n", __FILE__, __LINE__);

          bool image_reverse = false;
          bool file_check_bmp = true;
          image_err_e image_proccess_response = PNG_WRONG_FORMAT;
          bmp_err_e bmp_proccess_response = BMP_NOT_BMP;

          // showMessageWithLogo(MSG_FORMAT_ERROR);
          String last_dot_file = filesystem_file_exists("/last.bmp") ? "/last.bmp" : "/last.png";
          if (last_dot_file == "/last.bmp")
          {
            Log.info("Rewind BMP\n\r");
            buffer = (uint8_t *)malloc(DISPLAY_BMP_IMAGE_SIZE);
            file_check_bmp = filesystem_read_from_file(last_dot_file.c_str(), buffer, DISPLAY_BMP_IMAGE_SIZE);
            bmp_proccess_response = parseBMPHeader(buffer, image_reverse);
          }
          else if (last_dot_file == "/last.png")
          {
            Log.info("Rewind PNG\n\r");
            buffer = display_read_file(last_dot_file.c_str(), &file_size);
            image_proccess_response = PNG_NO_ERR; // DEBUG
          }

          if (file_check_bmp)
          {
            switch (image_proccess_response)
            {
            case PNG_NO_ERR:
            {
              Log.info("Showing image\n\r");
              display_show_image(buffer, file_size, true);
              need_to_refresh_display = 1;
            }
            break;
            default:
            {
            }
            break;
            }
            switch (bmp_proccess_response)
            {
            case BMP_NO_ERR:
            {
              Log.info("Showing image\n\r");
              display_show_image(buffer, DISPLAY_BMP_IMAGE_SIZE, true);
              need_to_refresh_display = 1;
            }
            break;
            default:
            {
            }
            break;
            }
          }
          else
          {
            free(buffer);
            buffer = nullptr;
            showMessageWithLogo(MSG_FORMAT_ERROR);
          }
        }
        else
        {
          Log.error("%s [%d]: rewind failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_SEND_TO_ME:
      {
        String action = apiResponse.action;

        if (action.equals("send_to_me"))
        {
          status = false;
          result = HTTPS_SUCCESS;
          Log.info("%s [%d]: send_to_me success\r\n", __FILE__, __LINE__);

          bool image_reverse = false;

          if (!filesystem_file_exists("/current.bmp") && !filesystem_file_exists("/current.png"))
          {
            Log.info("%s [%d]: No current image!\r\n", __FILE__, __LINE__);
            free(buffer);
            buffer = nullptr;
            return HTTPS_WRONG_IMAGE_FORMAT;
          }

          if (filesystem_file_exists("/current.bmp"))
          {
            Log.info("%s [%d]: send_to_me BMP\r\n", __FILE__, __LINE__);
            buffer = (uint8_t *)malloc(DISPLAY_BMP_IMAGE_SIZE);

            if (!filesystem_read_from_file("/current.bmp", buffer, DISPLAY_BMP_IMAGE_SIZE))
            {
              free(buffer);
              buffer = nullptr;
              Log_error_submit("Error reading image!");
              return HTTPS_WRONG_IMAGE_FORMAT;
            }

            bmp_err_e bmp_parse_result = parseBMPHeader(buffer, image_reverse);
            if (bmp_parse_result != BMP_NO_ERR)
            {
              free(buffer);
              buffer = nullptr;
              Log_error_submit("Error parsing BMP header, code: %d", bmp_parse_result);
              return HTTPS_WRONG_IMAGE_FORMAT;
            }
          }
          else if (filesystem_file_exists("/current.png"))
          {
            Log.info("%s [%d]: send_to_me PNG\r\n", __FILE__, __LINE__);
            image_err_e png_parse_result = PNG_NO_ERR; // DEBUG
            buffer = display_read_file("/current.png", &file_size);
// Disable partial update for now
//            if (filesystem_file_exists("/last.png")) {
//                buffer_old = display_read_file("/last.png", &file_size_old);
//                Log.info("%s [%d]: loading last PNG for partial update\r\n", __FILE__, __LINE__);
//            }
            if (png_parse_result != PNG_NO_ERR)
            {
              Log_error_submit("Error parsing PNG header, code: %d", png_parse_result);
              free(buffer);
              buffer = nullptr;
              return HTTPS_WRONG_IMAGE_FORMAT;
            }
          }

          Log.info("Showing image\n\r");
          display_show_image(buffer, file_size, true);
          need_to_refresh_display = 1;

          free(buffer);
          buffer = nullptr;
        }
        else
        {
          Log.error("%s [%d]: send_to_me failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      case SF_GUEST_MODE:
      {
        String action = apiResponse.action;
        if (action.equals("guest_mode"))
        {
          Log.info("%s [%d]:Guest Mode success\r\n", __FILE__, __LINE__);
          String image_url = apiResponse.image_url;
          uint64_t rate = apiResponse.refresh_rate;
          if (image_url.length() > 0)
          {
            Log.info("%s [%d]: image_url: %s\r\n", __FILE__, __LINE__, image_url.c_str());
            Log.info("%s [%d]: image url end with: %d\r\n", __FILE__, __LINE__, image_url.endsWith("/setup-logo.bmp"));

            image_url.toCharArray(filename, image_url.length() + 1);
            // check if plugin is applied
            bool flag = preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
            Log.info("%s [%d]: flag: %d\r\n", __FILE__, __LINE__, flag);

            if (apiResponse.filename == "empty_state")
            {
              Log.info("%s [%d]: End with empty_state\r\n", __FILE__, __LINE__);
              if (!flag)
              {
                // draw received logo
                status = true;
                // set flag to true
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != true) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, true);
                  if (res)
                    Log.info("%s [%d]: Flag written true successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: FLag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              else
              {
                // don't draw received logo
                status = false;
              }
            }
            else
            {
              Log.info("%s [%d]: End with NO empty_state\r\n", __FILE__, __LINE__);
              if (flag)
              {
                if (preferences.getBool(PREFERENCES_DEVICE_REGISTERED_KEY, false) != false) // check the flag to avoid the re-writing
                {
                  bool res = preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
                  if (res)
                    Log.info("%s [%d]: Flag written false successfully\r\n", __FILE__, __LINE__);
                  else
                    Log.error("%s [%d]: FLag writing failed\r\n", __FILE__, __LINE__);
                }
              }
              status = true;
            }
          }
          preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, rate);
        }
        else
        {
          Log.error("%s [%d]: Guest Mode failed\r\n", __FILE__, __LINE__);
        }
      }
      break;
      default:
        break;
      }
    }
    break;
    case 202:
    {
      result = HTTPS_NO_REGISTER;
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_NOT_CONNECTED);
      preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_NOT_CONNECTED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      status = false;
    }
    break;
    case 500:
    {
      result = HTTPS_RESET;
      Log.info("%s [%d]: write new refresh rate: %d\r\n", __FILE__, __LINE__, SLEEP_TIME_WHILE_NOT_CONNECTED);
      preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_WHILE_NOT_CONNECTED);
      Log.info("%s [%d]: written new refresh rate: %d\r\n", __FILE__, __LINE__, result);
      status = false;
    }
    break;

    default:
      break;
    }
  }
  return result;
}

/**
 * @brief Performs API setup call to get credentials and image URL
 * @return true if should continue to image download, false otherwise
 */
static bool performApiSetup()
{
  // Set up the API inputs
  ApiSetupInputs inputs;
  inputs.baseUrl = preferences.getString(PREFERENCES_API_URL, API_BASE_URL);
  inputs.macAddress = WiFi.macAddress();
  inputs.firmwareVersion = FW_VERSION_STRING;

  Log.info("%s [%d]: [HTTPS] begin /api/setup ...\r\n", __FILE__, __LINE__);
  Log.info("%s [%d]: RSSI: %d\r\n", __FILE__, __LINE__, WiFi.RSSI());
  Log.info("%s [%d]: Device MAC address: %s\r\n", __FILE__, __LINE__, WiFi.macAddress().c_str());

  // Call the API client
  ApiSetupResult result = fetchApiSetup(inputs);

  // Handle connection errors
  if (result.error == HTTPS_UNABLE_TO_CONNECT)
  {
    showMessageWithLogo(WIFI_INTERNAL_ERROR);
    Log_error_submit("[HTTPS] %s", result.error_detail.c_str());
    return false;
  }

  // Handle JSON parsing errors
  if (result.error == HTTPS_JSON_PARSING_ERR)
  {
    Log.error("%s [%d]: JSON deserialization error.\r\n", __FILE__, __LINE__);
    return false;
  }

  // Handle HTTP request errors
  if (result.error != HTTPS_NO_ERR)
  {
    if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
    {
      showMessageWithLogo(API_SETUP_FAILED);
    }
    else
    {
      showMessageWithLogo(WIFI_WEAK);
    }
    Log_error_submit("[HTTPS] Request failed: %s", result.error_detail.c_str());
    return false;
  }

  // Process the successful response
  auto &apiResponse = result.response;
  uint16_t url_status = apiResponse.status;

  Log.info("%s [%d]: GET... code: %d\r\n", __FILE__, __LINE__, url_status);

  if (url_status == 200)
  {
    status = true;
    Log.info("%s [%d]: status OK.\r\n", __FILE__, __LINE__);

    String api_key = apiResponse.api_key;
    Log.info("%s [%d]: API key - %s\r\n", __FILE__, __LINE__, api_key.c_str());
    size_t res = preferences.putString(PREFERENCES_API_KEY, api_key);
    Log.info("%s [%d]: api key saved in the preferences - %d\r\n", __FILE__, __LINE__, res);

    String friendly_id = apiResponse.friendly_id;
    Log.info("%s [%d]: friendly ID - %s\r\n", __FILE__, __LINE__, friendly_id.c_str());
    res = preferences.putString(PREFERENCES_FRIENDLY_ID, friendly_id);
    Log.info("%s [%d]: friendly ID saved in the preferences - %d\r\n", __FILE__, __LINE__, res);

    String image_url = apiResponse.image_url;
    Log.info("%s [%d]: image_url - %s\r\n", __FILE__, __LINE__, image_url.c_str());
    image_url.toCharArray(filename, image_url.length() + 1);

    String message_str = apiResponse.message;
    Log.info("%s [%d]: message - %s\r\n", __FILE__, __LINE__, message_str.c_str());
    message_str.toCharArray(message_buffer, message_str.length() + 1);

    Log.info("%s [%d]: status - %d\r\n", __FILE__, __LINE__, status);
    return true;
  }
  else if (url_status == 404)
  {
    Log_info("MAC Address is not registered on server");

    showMessageWithLogo(MAC_NOT_REGISTERED, apiResponse);

    preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP);

    display_sleep();
    goToSleep();
    return false;
  }
  else
  {
    Log.info("%s [%d]: status FAIL.\r\n", __FILE__, __LINE__);
    status = false;
    return false;
  }
}

/**
 * @brief Downloads and displays the setup image from the API response
 * @return none
 */
static void downloadSetupImage()
{
  status = false;
  Log.info("%s [%d]: filename - %s\r\n", __FILE__, __LINE__, filename);

  withHttp(filename, [&](HTTPClient *https, HttpError error) -> bool
           {
    if (error != HttpError::HTTPCLIENT_SUCCESS)
    {
      if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
      {
        showMessageWithLogo(API_IMAGE_DOWNLOAD_ERROR);
      }
      else
      {
        showMessageWithLogo(WIFI_WEAK);
      }
      Log_error_submit("[HTTPS] Unable to connect");
      return false;
    }

    https->setTimeout(15000);
    https->setConnectTimeout(15000);

    Log.info("%s [%d]: [HTTPS] Request to %s\r\n", __FILE__, __LINE__, filename);
    Log.info("%s [%d]: [HTTPS] GET..\r\n", __FILE__, __LINE__);

    int httpCode = https->GET();

    if(httpCode == HTTP_CODE_PERMANENT_REDIRECT ||httpCode == HTTP_CODE_TEMPORARY_REDIRECT){
              https->end();
              https->begin(https->getLocation());
              Log_info("Redirected to: %s", https->getLocation().c_str());
              https->setTimeout(15000);
              https->setConnectTimeout(15000);
              httpCode = https->GET();
            }

    // httpCode will be negative on error
    if (httpCode <= 0)
    {
      if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
      {
        showMessageWithLogo(API_IMAGE_DOWNLOAD_ERROR);
      }
      else
      {
        showMessageWithLogo(WIFI_WEAK);
      }
      Log_error_submit("[HTTPS] GET... failed, error: %s", https->errorToString(httpCode).c_str());
      return false;
    }

    // HTTP header has been send and Server response header has been handled
    Log.error("%s [%d]: [HTTPS] GET... code: %d\r\n", __FILE__, __LINE__, httpCode);
    
    // file found at server
    if (httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY)
    {
      if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
      {
        showMessageWithLogo(API_IMAGE_DOWNLOAD_ERROR);
      }
      else
      {
        showMessageWithLogo(WIFI_WEAK);
      }
      Log_error_submit("[HTTPS] GET... failed, error: %s", https->errorToString(httpCode).c_str());
      return false;
    }

    Log.info("%s [%d]: Content size: %d\r\n", __FILE__, __LINE__, https->getSize());

    WiFiClient *stream = https->getStreamPtr();

    uint32_t counter = 0;
    // Read and save BMP data to buffer
    buffer = (uint8_t *)malloc(https->getSize());
    if (stream->available() && https->getSize() == DISPLAY_BMP_IMAGE_SIZE)
    {
      counter = downloadStream(stream, DISPLAY_BMP_IMAGE_SIZE, buffer);
    }
    
    if (counter == DISPLAY_BMP_IMAGE_SIZE)
    {
      Log.info("%s [%d]: Received successfully\r\n", __FILE__, __LINE__);

      writeImageToFile("/logo.bmp", buffer, DEFAULT_IMAGE_SIZE);

      // show the image
      String friendly_id = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
      display_show_msg(storedLogoOrDefault(0), FRIENDLY_ID, friendly_id, true, "", String(message_buffer));
      need_to_refresh_display = 0;
    }
    else
    {
      free(buffer);
      buffer = nullptr;
      if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
      {
        showMessageWithLogo(API_SIZE_ERROR);
      }
      else
      {
        showMessageWithLogo(WIFI_WEAK);
      }
      Log_error_submit("Receiving failed. Read: %d", counter);
    }
    
    return true; });
}

/**
 * @brief Function to getting the friendly id and API key
 * @return none
 */
static void getDeviceCredentials()
{
  bool shouldDownloadImage = performApiSetup();

  Log.info("%s [%d]: status - %d\r\n", __FILE__, __LINE__, status);
  if (shouldDownloadImage)
  {
    downloadSetupImage();
  }
}

/**
 * @brief Function to reset the friendly id, API key, WiFi SSID and password
 * @param url Server URL address
 * @return none
 */
static void resetDeviceCredentials(void)
{
  Log.info("%s [%d]: The device will be reset now...\r\n", __FILE__, __LINE__);
  Log.info("%s [%d]: WiFi reseting...\r\n", __FILE__, __LINE__);
  WifiCaptivePortal.resetSettings();
  need_to_refresh_display = 1;
  bool res = preferences.clear();
  if (res)
    Log.info("%s [%d]: The device reset success. Restarting...\r\n", __FILE__, __LINE__);
  else
    Log.error("%s [%d]: The device reseting error. The device will be reset now...\r\n", __FILE__, __LINE__);
  preferences.end();
  ESP.restart();
}

/**
 * @brief Function to check and performing OTA update
 * @param none
 * @return none
 */
static void checkAndPerformFirmwareUpdate(void)
{

  withHttp(binUrl, [&](HTTPClient *https, HttpError errorCode) -> bool
           {
             if (errorCode != HttpError::HTTPCLIENT_SUCCESS || !https)
             {
               Log.fatal("%s [%d]: Unable to connect for firmware update\r\n", __FILE__, __LINE__);
               if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
               {
                 showMessageWithLogo(API_FIRMWARE_UPDATE_ERROR);
               }
               else
               {
                 showMessageWithLogo(WIFI_WEAK);
               }
             }

             int httpCode = https->GET();
             if (httpCode == HTTP_CODE_OK)
             {
               Log.info("%s [%d]: Downloading .bin file...\r\n", __FILE__, __LINE__);

               size_t contentLength = https->getSize();
               // Perform firmware update
               if (Update.begin(contentLength))
               {
                 Log.info("%s [%d]: Firmware update start\r\n", __FILE__, __LINE__);
                 showMessageWithLogo(FW_UPDATE);

                 if (Update.writeStream(https->getStream()))
                 {
                   if (Update.end(true))
                   {
                     Log.info("%s [%d]: Firmware update successful. Rebooting...\r\n", __FILE__, __LINE__);
                     showMessageWithLogo(FW_UPDATE_SUCCESS);
                   }
                   else
                   {
                     Log.fatal("%s [%d]: Firmware update failed!\r\n", __FILE__, __LINE__);
                     showMessageWithLogo(FW_UPDATE_FAILED);
                   }
                 }
                 else
                 {
                   Log.fatal("%s [%d]: Write to firmware update stream failed!\r\n", __FILE__, __LINE__);
                   showMessageWithLogo(FW_UPDATE_FAILED);
                 }
               }
               else
               {
                 Log.fatal("%s [%d]: Begin firmware update failed!\r\n", __FILE__, __LINE__);
                 showMessageWithLogo(FW_UPDATE_FAILED);
               }
             }
             return true; });
}

/**
 * @brief Function to sleep preparing and go to sleep
 * @param none
 * @return none
 */
static void goToSleep(void)
{
#ifdef BOARD_TRMNL_X
  iqs323.run(); // to clear any pending operations before sleep
#endif
  submitStoredLogs();
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  WiFi.mode(WIFI_OFF); 
  filesystem_deinit();
  uint32_t time_to_sleep = SLEEP_TIME_TO_SLEEP;
  if (preferences.isKey(PREFERENCES_SLEEP_TIME_KEY))
    time_to_sleep = preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP);
  Log.info("%s [%d]: total awake time - %d ms\r\n", __FILE__, __LINE__, millis() - startup_time); 
  Log.info("%s [%d]: time to sleep - %d\r\n", __FILE__, __LINE__, time_to_sleep);
  preferences.putUInt(PREFERENCES_LAST_SLEEP_TIME, getTime());
  preferences.end();
  esp_sleep_enable_timer_wakeup((uint64_t)time_to_sleep * SLEEP_uS_TO_S_FACTOR);
#if BOARD_TRMNL_X
  esp_set_deep_sleep_wake_stub(*wakeup_stub);
#endif
  // Configure GPIO pin for wakeup
#if CONFIG_IDF_TARGET_ESP32
  #define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK(PIN_INTERRUPT), ESP_EXT1_WAKEUP_ALL_LOW);
#elif CONFIG_IDF_TARGET_ESP32C3
  esp_deep_sleep_enable_gpio_wakeup(1 << PIN_INTERRUPT, ESP_GPIO_WAKEUP_GPIO_LOW);
#elif CONFIG_IDF_TARGET_ESP32S3
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INTERRUPT, 0);
#elif CONFIG_IDF_TARGET_ESP32C5
  esp_deep_sleep_enable_gpio_wakeup(1 << PIN_INTERRUPT, ESP_GPIO_WAKEUP_GPIO_LOW);

#else
#error "Unsupported ESP32 target for GPIO wakeup configuration"
#endif
  esp_deep_sleep_start();
}

// Not sure if WiFiClientSecure checks the validity date of the certificate.
// Setting clock just to be sure...
/**
 * @brief Function to clock synchronization
 * @param none
 * @return none
 */
static bool setClock()
{
  bool sync_status = false;
  struct tm timeinfo;

  configTime(0, 0, "time.google.com", "time.cloudflare.com");
  Log.info("%s [%d]: Time synchronization...\r\n", __FILE__, __LINE__);

  // Wait for time to be set
  if (getLocalTime(&timeinfo))
  {
    sync_status = true;
    Log.info("%s [%d]: Time synchronization succeed!\r\n", __FILE__, __LINE__);
  }
  else
  {
    Log.info("%s [%d]: Time synchronization failed...\r\n", __FILE__, __LINE__);
  }

  Log.info("%s [%d]: Current time - %s\r\n", __FILE__, __LINE__, asctime(&timeinfo));

  return sync_status;
}

/**
 * @brief Function to read the battery voltage
 * @param none
 * @return float voltage in Volts
 */
static float readBatteryVoltage(void)
{
#ifdef FAKE_BATTERY_VOLTAGE
  Log.warning("%s [%d]: FAKE_BATTERY_VOLTAGE is defined. Returning 4.2V.\r\n", __FILE__, __LINE__);
  return 4.2f;
#else
  #if defined(BOARD_XIAO_EPAPER_DISPLAY) || defined(BOARD_SEEED_RETERMINAL_E1001)
    pinMode(PIN_VBAT_SWITCH, OUTPUT);
    digitalWrite(PIN_VBAT_SWITCH, VBAT_SWITCH_LEVEL);
    delay(10); // Wait for the switch to stabilize
  #endif
    Log.info("%s [%d]: Battery voltage reading...\r\n", __FILE__, __LINE__);
    int32_t adc;
    int32_t sensorValue;

    adc = 0;
    analogRead(PIN_BATTERY); // This is needed to properly initialize the ADC BEFORE calling analogReadMilliVolts()
    for (uint8_t i = 0; i < 8; i++) {
      adc += analogReadMilliVolts(PIN_BATTERY);
    }
  #if defined(BOARD_XIAO_EPAPER_DISPLAY) || defined(BOARD_SEEED_RETERMINAL_E1001)
    digitalWrite(PIN_VBAT_SWITCH, (VBAT_SWITCH_LEVEL == HIGH ? LOW : HIGH));
  #endif
    sensorValue = (adc / 8) * 2;
    Log.info("%s [%d]: Battery sensorValue = %d\r\n", __FILE__, __LINE__, (int)sensorValue);
    float voltage = sensorValue / 1000.0;
    return voltage;
#endif // FAKE_BATTERY_VOLTAGE
}

/**
 * @brief Function to submit a log string to the API
 * @param log_buffer pointer to the buffer that contains log note
 * @return bool true if successful, false if failed
 */
bool submitLogString(const char *log_buffer)
{
  String api_key = "";
  if (preferences.isKey(PREFERENCES_API_KEY))
  {
    api_key = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log_info("%s key exists. Value - %s", PREFERENCES_API_KEY, api_key.c_str());
  }
  else
  {
    Log_info("%s key not exists.", PREFERENCES_API_KEY);
    return false;
  }

  LogApiInput input{api_key, log_buffer};
  return submitLogToApi(input, preferences.getString(PREFERENCES_API_URL, API_BASE_URL).c_str());
}

/**
 * @brief Function to store a log string locally
 * @param log_buffer pointer to the buffer that contains log note
 * @return bool true if successful, false if failed
 */
bool storeLogString(const char *log_buffer)
{
  LogStoreResult store_result = storedLogs.store_log(String(log_buffer));
  if (store_result.status != LogStoreResult::SUCCESS)
  {
    Log_error("Failed to store log: %s", store_result.message);
    return false;
  }
  return true;
}


uint32_t getTime(void)
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 200))
  {
    Log_info("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

static void submitStoredLogs(void)
{
  if (WiFi.isConnected() == false)
  {
    Log_info("WiFi not connected; not submitting stored logs.");
    return;
  }
  String log = storedLogs.gather_stored_logs();

  String api_key = "";
  if (preferences.isKey(PREFERENCES_API_KEY))
  {
    api_key = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY, api_key.c_str());
  }
  else
  {
    Log.error("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY);
  }

  bool submitLogToApiResult = false;
  if (log.length() > 0)
  {
    Log.info("%s [%d]: log string - %s\r\n", __FILE__, __LINE__, log.c_str());
    Log.info("%s [%d]: need to send the log\r\n", __FILE__, __LINE__);

    LogApiInput input{api_key, log.c_str()};
    submitLogToApiResult = submitLogToApi(input, preferences.getString(PREFERENCES_API_URL, API_BASE_URL).c_str());
  }
  else
  {
    Log.info("%s [%d]: no needed to send the log\r\n", __FILE__, __LINE__);
  }
  if (submitLogToApiResult == true)
  {
    storedLogs.clear_stored_logs();
  }
}

static void writeImageToFile(const char *name, uint8_t *in_buffer, size_t size)
{
  size_t res = filesystem_write_to_file(name, in_buffer, size);
  if (res != size)
  {
    Log_error_submit("File writing ERROR. Result - %d", res);
  }
  else
  {
    Log.info("%s [%d]: file %s writing success - %d bytes\r\n", __FILE__, __LINE__, name, res);
  }
}

static void writeSpecialFunction(SPECIAL_FUNCTION function)
{
  if (preferences.isKey(PREFERENCES_SF_KEY))
  {
    Log.info("%s [%d]: SF saved. Reading...\r\n", __FILE__, __LINE__);
    if ((SPECIAL_FUNCTION)preferences.getUInt(PREFERENCES_SF_KEY, 0) == function)
    {
      Log.info("%s [%d]: No needed to re-write\r\n", __FILE__, __LINE__);
    }
    else
    {
      Log.info("%s [%d]: Writing new special function\r\n", __FILE__, __LINE__);
      bool res = preferences.putUInt(PREFERENCES_SF_KEY, function);
      if (res)
        Log.info("%s [%d]: Written new special function successfully\r\n", __FILE__, __LINE__);
      else
        Log.error("%s [%d]: Writing new special function failed\r\n", __FILE__, __LINE__);
    }
  }
  else
  {
    Log.error("%s [%d]: SF not saved\r\n", __FILE__, __LINE__);
    bool res = preferences.putUInt(PREFERENCES_SF_KEY, function);
    if (res)
      Log.info("%s [%d]: Written new special function successfully\r\n", __FILE__, __LINE__);
    else
      Log.error("%s [%d]: Writing new special function failed\r\n", __FILE__, __LINE__);
  }
}

static void showMessageWithLogo(MSG message_type, String friendly_id, bool id, const char *fw_version, String message)
{
  display_show_msg(storedLogoOrDefault(0), message_type, friendly_id, id, fw_version, message);
  need_to_refresh_display = 1;
  preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
}

static void showMessageWithLogo(MSG message_type)
{
  display_show_msg(storedLogoOrDefault(0), message_type);
}

/**
 * @brief Show a message with the logo using data from API setup response
 * @param message_type Type of message to display
 * @param apiResponse The API setup response containing the message
 * @return none
 */
static void showMessageWithLogo(MSG message_type, const ApiSetupResponse &apiResponse)
{
  display_show_msg(storedLogoOrDefault(0), message_type, "", false, "", apiResponse.message);
  need_to_refresh_display = 1;
  preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
}

// 0 = larger glyph, centered for message screens
// 1 = small glyph, set in lower-right corner for loading screen
static uint8_t *storedLogoOrDefault(int iType)
{
//  if (filesystem_read_from_file("/logo.bmp", buffer, DEFAULT_IMAGE_SIZE))
//  {
//    return buffer;
//  }
#ifdef BOARD_TRMNL_X
    return const_cast<uint8_t *>(logo_medium);
#else
  if (iType == 0) {
    return const_cast<uint8_t *>(logo_small);
  } else {
    // Force the loading screen to always use the slower update method because
    // we don't know (yet) if the panel can handle the faster update modes
    apiDisplayResult.response.maximum_compatibility = true;
    return const_cast<uint8_t *>(loading);
  }
#endif
}

static bool saveCurrentFileName(String &name)
{
  if (!preferences.getString(PREFERENCES_FILENAME_KEY, "").equals(name))
  {
    Log.info("%s [%d]: New filename:  - %s\r\n", __FILE__, __LINE__, name.c_str());
    size_t res = preferences.putString(PREFERENCES_FILENAME_KEY, name);
    if (res > 0)
    {
      Log.info("%s [%d]: New filename saved in the preferences - %d\r\n", __FILE__, __LINE__, res);
      return true;
    }
    else
    {
      Log.error("%s [%d]: New filename saving error!\r\n", __FILE__, __LINE__);
      return false;
    }
  }
  else
  {
    Log.info("%s [%d]: No needed to re-write\r\n", __FILE__, __LINE__);
    return true;
  }
}

static bool checkCurrentFileName(String &newName)
{
  String currentFilename = preferences.getString(PREFERENCES_FILENAME_KEY, "");

  Log.error("%s [%d]: Current filename: %s\r\n", __FILE__, __LINE__, currentFilename);

  if (currentFilename.equals(newName))
  {
    Log.info("%s [%d]: Current filename equals to the new filename\r\n", __FILE__, __LINE__);
    return true;
  }
  else
  {
    Log.error("%s [%d]: Current filename doesn't equal to the new filename\r\n", __FILE__, __LINE__);
    return false;
  }
}

static void wifiErrorDeepSleep()
{
  if (!preferences.isKey(PREFERENCES_CONNECT_WIFI_RETRY_COUNT))
  {
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
  }

  uint8_t retry_count = preferences.getInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT);

  Log_info("WIFI connection failed! Retry count: %d \n", retry_count);

  switch (retry_count)
  {
  case 1:
    preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, WIFI_CONNECT_RETRY_TIME::WIFI_FIRST_RETRY);
    break;

  case 2:
    preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, WIFI_CONNECT_RETRY_TIME::WIFI_SECOND_RETRY);
    break;

  case 3:
    preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, WIFI_CONNECT_RETRY_TIME::WIFI_THIRD_RETRY);
    break;

  default:
    preferences.putUInt(PREFERENCES_SLEEP_TIME_KEY, SLEEP_TIME_TO_SLEEP);
    break;
  }
  retry_count++;
  preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, retry_count);

  display_sleep();
  goToSleep();
}

DeviceStatusStamp getDeviceStatusStamp()
{
  DeviceStatusStamp deviceStatus = {};

  deviceStatus.wifi_rssi_level = WiFi.RSSI();
  strncpy(deviceStatus.wifi_status, wifiStatusStr(WiFi.status()), sizeof(deviceStatus.wifi_status) - 1);
  deviceStatus.refresh_rate = preferences.getUInt(PREFERENCES_SLEEP_TIME_KEY);
  deviceStatus.time_since_last_sleep = time_since_sleep;
  snprintf(deviceStatus.current_fw_version, sizeof(deviceStatus.current_fw_version), "%s", FW_VERSION_STRING);
  parseSpecialFunctionToStr(deviceStatus.special_function, sizeof(deviceStatus.special_function), special_function);
  deviceStatus.battery_voltage = vBatt; //readBatteryVoltage()
  parseWakeupReasonToStr(deviceStatus.wakeup_reason, sizeof(deviceStatus.wakeup_reason), esp_sleep_get_wakeup_cause());
  deviceStatus.free_heap_size = ESP.getFreeHeap();
  deviceStatus.max_alloc_size = ESP.getMaxAllocHeap();

  return deviceStatus;
}

void logWithAction(LogAction action, const char *message, time_t time, int line, const char *file)
{
  uint32_t log_id = preferences.getUInt(PREFERENCES_LOG_ID_KEY, 1);

  LogWithDetails input = {
      .deviceStatusStamp = getDeviceStatusStamp(),
      .timestamp = time,
      .codeline = line,
      .sourceFile = file,
      .logMessage = message,
      .logId = log_id,
      .filenameCurrent = preferences.getString(PREFERENCES_FILENAME_KEY, ""),
      .filenameNew = new_filename,
      .logRetry = log_retry,
      .retryAttempt = log_retry ? preferences.getInt(PREFERENCES_CONNECT_API_RETRY_COUNT) : 0};

  String json_string = serialize_log(input);

  switch (action)
  {
    case LOG_ACTION_STORE:
      storeLogString(json_string.c_str());
      break;
    case LOG_ACTION_SUBMIT:
      submitLogString(json_string.c_str());
      break;
    case LOG_ACTION_SUBMIT_OR_STORE:
      if (!submitLogString(json_string.c_str()))
      {
        Log_info("Was unable to send log to API; saving locally for later.");
        storeLogString(json_string.c_str());
      }
      break;
  }

  preferences.putUInt(PREFERENCES_LOG_ID_KEY, ++log_id);
}

void log_nvs_usage()
{
  nvs_stats_t nv;
  esp_err_t ret = nvs_get_stats(NULL, &nv);
  if (ret == ESP_OK)
  {
    float percent = (float)nv.used_entries / (float)nv.total_entries * 100.0f;
    char percent_str[16];
    dtostrf(percent, 0, 2, percent_str); // 2 decimal places
    Log_info("NVS Usage: %d/%d entries (%s%%)", nv.used_entries, nv.total_entries, percent_str);
  }
  else
  {
    Log_error("Failed to get NVS stats: %s", esp_err_to_name(ret));
  }
}

void Test_new_screens(void){
    showMessageWithLogo(API_ERROR);
    delay(000);
    showMessageWithLogo(API_REQUEST_FAILED);
    delay(2000);
    showMessageWithLogo(API_IMAGE_DOWNLOAD_ERROR);
    delay(2000);
    showMessageWithLogo(API_FIRMWARE_UPDATE_ERROR);
    delay(2000);
    showMessageWithLogo(API_SETUP_FAILED);
    delay(2000);
    showMessageWithLogo(API_UNABLE_TO_CONNECT);
};
