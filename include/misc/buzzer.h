#pragma once

#include <config.h>
#include <stdint.h>

/// @brief Buzzer interface. The base class silently does nothing, for boards
///        with no buzzer hardware.
class BaseBuzzer {
public:
  virtual ~BaseBuzzer() = default;

  virtual void init() {}
  virtual void beep(unsigned long duration_ms) { (void)duration_ms; }
  virtual void beepPattern(int count, unsigned long on_ms, unsigned long off_ms) {
    (void)count;
    (void)on_ms;
    (void)off_ms;
  }

private:
  virtual void on() {}
  virtual void off() {}
};

/// @brief Drives a passive buzzer with a PWM tone.
class PWMBuzzer : public BaseBuzzer {
public:
  PWMBuzzer(uint8_t pin, uint32_t freq);
  void init() override;
  void beep(unsigned long duration_ms) override;
  void beepPattern(int count, unsigned long on_ms, unsigned long off_ms) override;

private:
  void on() override;
  void off() override;

  uint8_t _pin;
  uint32_t _freq;
};

/// @brief Buzzer instance for the running board, selected at compile time.
BaseBuzzer &buzzer();
