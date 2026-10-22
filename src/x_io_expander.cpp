#include <x_io_expander.h>

#ifdef BOARD_TRMNL_X

#include <Arduino.h>
#include <FastEPD.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <trmnl_log.h>

extern FASTEPD bbep; // display.cpp

#define TCA9535_INT 38

void BQ27427_reset() {
  bbep.ioPinMode(10, OUTPUT);

  bbep.ioWrite(10, LOW);
  delay(10);

  bbep.ioWrite(10, HIGH);
  Serial.println("BQ27427 reset performed");
}

void config_tca95535_pins_for_lp() {
  bbep.ioPinMode(0, INPUT);

    // Pin 1 (OTG) is configured separately in init_otg()

  bbep.ioPinMode(2, INPUT);

  bbep.ioPinMode(3, INPUT);

  bbep.ioPinMode(4, INPUT);

  bbep.ioPinMode(5, INPUT);

  bbep.ioPinMode(6, OUTPUT);
  bbep.ioWrite(6, LOW);

  bbep.ioPinMode(7, INPUT);

  bbep.ioPinMode(8, OUTPUT);
  bbep.ioWrite(8, LOW);

  bbep.ioPinMode(9, OUTPUT);
  bbep.ioWrite(9, LOW);

  bbep.ioPinMode(10, INPUT);

  bbep.ioPinMode(11, OUTPUT);
  bbep.ioWrite(11, LOW);

  bbep.ioPinMode(12, OUTPUT);
  bbep.ioWrite(12, LOW);

  bbep.ioPinMode(13, OUTPUT);
  bbep.ioWrite(13, LOW);

  bbep.ioPinMode(14, INPUT);

  bbep.ioPinMode(15, INPUT);
}

void otg_turn_on() {
  bbep.ioPinMode(1, OUTPUT);
  bbep.ioWrite(1, HIGH);
  Log_info("OTG turned on");
}

void otg_turn_off() {
  bbep.ioPinMode(1, OUTPUT);
  bbep.ioWrite(1, LOW);
  Log_info("OTG turned off");
}

#define BAT_DET_PIN      7    // TCA9535 P0_7 in bbep pin numbering
#define BAT_CHARGE_MS    2    // drive HIGH for 2 ms to charge RC network
#define BAT_TIMEOUT_US   6000 // no battery if pin is still HIGH after this
#define BAT_THRESHOLD_US 750 // >750 µs → 1 cell; ≤750 µs → 2 cells

static battery_count_t measure_battery_once() {
    // -- Charge RC network: drive P0_7 HIGH for 2 ms --
  bbep.ioPinMode(BAT_DET_PIN, OUTPUT);
  bbep.ioWrite(BAT_DET_PIN, HIGH);
  delay(BAT_CHARGE_MS);

    // -- Release: switch to input and start timing --
  bbep.ioPinMode(BAT_DET_PIN, INPUT);

  unsigned long start = micros();
  unsigned long now = start;
  bool timeout = false;

  while (true) {
    bool still_high = (bbep.ioRead(BAT_DET_PIN) != 0);
    now = micros();

    if (!still_high) break;               // pin went LOW → discharged

    if ((now - start) >= BAT_TIMEOUT_US) {
      timeout = true;
      break;
    }
  }

  if (timeout) return BATTERY_NONE;
  if ((now - start) > BAT_THRESHOLD_US) return BATTERY_ONE;
  return BATTERY_TWO;
}

battery_count_t detect_battery_count() {
  battery_count_t last = (battery_count_t)-1;

  for (int attempt = 0; attempt < 20; attempt++) {
    battery_count_t result = measure_battery_once();

    if (result == last) {
            // Two identical results in a row — confident reading
      Log_info("Battery detection: %s (confirmed after %d attempt(s))",
               result == BATTERY_NONE  ? "none"
               : result == BATTERY_ONE ? "1 cell"
                                       : "2 cells",
               attempt + 1);
      return result;
    }

    last = result;
    delay(10);
  }

    // Fallback: return last reading if we never got two in a row
  Log_error("Battery detection: unstable reading — defaulting to last result");
  return last;
}

void enter_shipment_sleep() {
  config_tca95535_pins_for_lp();
  bbep.ioPinMode(0, INPUT);  // Pin 0 = charger detect

    // Clear any pending TCA9535 interrupts
  for (uint8_t pin = 0; pin < 16; pin++) {
    bbep.ioPinMode(pin, INPUT);
    bbep.ioRead(pin);
  }
  delay(50);

  pinMode(TCA9535_INT, INPUT_PULLUP);

    // Check initial GPIO38 state
  int gpio38_state = digitalRead(TCA9535_INT);

    // display_sleep() leaves its light-sleep timer armed, which would wake us every second.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);

    // Enable GPIO wakeup for light sleep
  esp_err_t err = esp_sleep_enable_gpio_wakeup();
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to enable GPIO wakeup: %d\n", err);
    return;
  }

  err = gpio_wakeup_enable((gpio_num_t)TCA9535_INT, GPIO_INTR_LOW_LEVEL);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to configure GPIO38 wakeup: %d\n", err);
    return;
  }

  Serial.println("=== Entering shipment mode light sleep loop ===");
  delay(100);

  uint32_t sleep_count = 0;

    // Main sleep loop
  while (true) {
    sleep_count++;

    esp_light_sleep_start();

        // Re-initialize Serial after light sleep (USB may have been disabled)
    delay(100);
    Serial.begin(115200);
    delay(100);

    Serial.println("\n=== WAKEUP from light sleep ===");
    Serial.printf("Sleep cycle: %lu\n", sleep_count);

        // Check wakeup cause
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.printf("Wakeup cause: %d (GPIO=%d)\n", wakeup_reason, ESP_SLEEP_WAKEUP_GPIO);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
      Serial.println("GPIO wakeup detected");

            // Check GPIO38 state
      gpio38_state = digitalRead(TCA9535_INT);
      Serial.printf("GPIO38 state after wakeup: %d\n", gpio38_state);

            // Check if charger connected (TCA9535 pin 0 LOW)
      bbep.ioPinMode(0, INPUT);
      delay(10);
      uint8_t pin0_state = bbep.ioRead(0);
      Serial.printf("TCA9535 pin 0 (charger detect): %d (0=charger present)\n", pin0_state);

      if (pin0_state == 0) {
        Serial.println("*** CHARGER DETECTED - Exiting shipment mode ***");
        Serial.flush();
        delay(100);
        break;  // Exit shipment mode
      }

            // False wakeup - clear interrupt and continue
      for (uint8_t pin = 0; pin < 16; pin++) {
        bbep.ioPinMode(pin, INPUT);
        bbep.ioRead(pin);
      }
      delay(50);

      gpio38_state = digitalRead(TCA9535_INT);
      Serial.printf("GPIO38 after clear: %d\n", gpio38_state);

    } else {
      Serial.printf("Unexpected wakeup cause: %d\n", wakeup_reason);
    }

    Serial.println("Returning to sleep...\n");
    Serial.flush();
    delay(100);
  }

  gpio_wakeup_disable((gpio_num_t)TCA9535_INT);
  gpio_set_intr_type((gpio_num_t)TCA9535_INT, GPIO_INTR_DISABLE);
  Serial.println("=== Exited shipment mode successfully ===");
  Serial.flush();
}

#define PIN_ESP32C5_SPI_BOOT 4
#define PIN_ESP32C5_USB_BOOT 5
#define PIN_ESP32C5_EN       6

void modem_enter_bootloader(void) {
  // Disable target and wait for full power down
  bbep.ioWrite(PIN_ESP32C5_EN, 0);
  delay(100);

  // Configure boot pins for bootloader mode
  bbep.ioPinMode(PIN_ESP32C5_SPI_BOOT, OUTPUT);
  bbep.ioPinMode(PIN_ESP32C5_USB_BOOT, OUTPUT);
  bbep.ioWrite(PIN_ESP32C5_SPI_BOOT, 0);
  bbep.ioWrite(PIN_ESP32C5_USB_BOOT, 1);
  delay(50);

  // Enable target in bootloader mode
  bbep.ioWrite(PIN_ESP32C5_EN, 1);
  delay(300);
}

void modem_reset_target(void) {
  // Disable target
  bbep.ioWrite(PIN_ESP32C5_EN, 0);
  delay(50);

  // Release boot pins (set to input for normal boot)
  bbep.ioPinMode(PIN_ESP32C5_SPI_BOOT, INPUT);
  bbep.ioPinMode(PIN_ESP32C5_USB_BOOT, INPUT);
  delay(50);

  // Enable target (normal boot)
  bbep.ioWrite(PIN_ESP32C5_EN, 1);
}

#endif // BOARD_TRMNL_X
