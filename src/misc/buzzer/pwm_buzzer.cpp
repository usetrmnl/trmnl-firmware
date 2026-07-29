#include <Arduino.h>
#include <misc/buzzer.h>

#include "trmnl_log.h"

#define BUZZER_CHANNEL    0
#define BUZZER_RESOLUTION 8
#define BUZZER_DUTY       128 // 50% duty cycle

PWMBuzzer::PWMBuzzer(uint8_t pin, uint32_t freq) : _pin(pin), _freq(freq) {}

void PWMBuzzer::init() {
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  Log_info("Buzzer initialized on GPIO %d", _pin);
}

// Breaking API change in Arduino-ESP32 core 3.x: LEDC channels are managed per-pin
#if ESP_ARDUINO_VERSION_MAJOR >= 3

void PWMBuzzer::on() {
  ledcAttach(_pin, _freq, BUZZER_RESOLUTION);
  ledcWrite(_pin, BUZZER_DUTY);
}

void PWMBuzzer::off() {
  ledcWrite(_pin, 0);
  ledcDetach(_pin);
  digitalWrite(_pin, LOW);
}

#else // => ESP_ARDUINO_VERSION_MAJOR < 3

void PWMBuzzer::on() {
  ledcSetup(BUZZER_CHANNEL, _freq, BUZZER_RESOLUTION);
  ledcAttachPin(_pin, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, BUZZER_DUTY);
}

void PWMBuzzer::off() {
  ledcWrite(BUZZER_CHANNEL, 0);
  ledcDetachPin(_pin);
  digitalWrite(_pin, LOW);
}

#endif // ESP_ARDUINO_VERSION_MAJOR >= 3

void PWMBuzzer::beep(unsigned long duration_ms) {
  on();
  delay(duration_ms);
  off();
}

void PWMBuzzer::beepPattern(int count, unsigned long on_ms, unsigned long off_ms) {
  for (int i = 0; i < count; i++) {
    on();
    delay(on_ms);
    off();
    if (i < count - 1) {
      delay(off_ms);
    }
  }
}