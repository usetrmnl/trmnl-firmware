#include <Arduino.h>
#include <board_config.h>
#include <button.h>
#include <globals.h>
#include <http_result_handling.h>
#include <keep_wifi_on.h>
#include <sleep_session.h>

static volatile bool button_pressed_while_awake = false;
static void IRAM_ATTR latch_button_press(void) { button_pressed_while_awake = true; }

// The check-in after the wait reads as a timer wake, or as a button wake after a press, even one made during a check-in
static void wait_awake_for_next_check_in(void) {
  const ApiDisplayResponse &answer = apiDisplayResult.response;
  // the floors stop a 0 from hammering the API
  const uint32_t wait_ms =
    answer.refresh_rate_ms ? max<uint32_t>(answer.refresh_rate_ms, 250) : max<uint32_t>(answer.refresh_rate, 1) * 1000;
  wakeup_reason = ESP_SLEEP_WAKEUP_TIMER;
  while (millis() - apiDisplayResult.received_ms < wait_ms) {
    if (button_pressed_while_awake) {
      button_pressed_while_awake = false;
      wakeup_reason = button_wakeup_cause();
      // read_button_presses() waits for a new press when the button is already up
      ButtonPressResult press = digitalRead(pDevice->interrupt_pin) == LOW ? read_button_presses() : ShortPress;
      if (button_wake_act(press)) button_wake_run_special_function();
      return;
    }
    delay(20);
  }
}

static constexpr int CHECK_IN_TRIES_AFTER_A_FAILURE = 3; // 2 s apart; only the last one draws its error screen

void stay_awake_while_server_asks(https_request_err_e (*show_screen)(bool draw_error_screen_on_failure)) {
  if (!apiDisplayResult.response.keep_wifi_on) return;
  attachInterrupt(digitalPinToInterrupt(pDevice->interrupt_pin), latch_button_press, FALLING);
  while (apiDisplayResult.response.keep_wifi_on) { // a check-in that keeps failing clears it, so errors sleep as usual
    awake_time_limit_restart(); // the limit catches a device stuck awake, not one the server keeps checking in
    wait_awake_for_next_check_in();
    https_request_err_e result = show_screen(false);
    for (int attempt = 1; requestIsFailure(result) && attempt <= CHECK_IN_TRIES_AFTER_A_FAILURE; attempt++) {
      delay(2000);
      result = show_screen(attempt == CHECK_IN_TRIES_AFTER_A_FAILURE);
    }
  }
  detachInterrupt(digitalPinToInterrupt(pDevice->interrupt_pin));
}
