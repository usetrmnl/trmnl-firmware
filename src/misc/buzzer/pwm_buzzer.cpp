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