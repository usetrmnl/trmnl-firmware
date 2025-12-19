#include "rtc_wake_stub_trmnl_x.h"

#include <inttypes.h>
#include "esp_sleep.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "esp_wake_stub.h"
#include "sdkconfig.h"

#include "rtc_wake_stub_i2c.h"

// counter value, stored in RTC memory
static uint32_t s_count = 0;
static const uint32_t s_max_count = 20;

// wakeup_cause stored in RTC memory
static uint32_t wakeup_cause;

// wakeup_time from CPU start to wake stub
static uint32_t wakeup_time;

iqs323_system_status_t wakeup_stub_iqs_status = {0};

void wakeup_stub(void) {
  // Get wakeup time.
  wakeup_time = esp_cpu_get_cycle_count() / esp_rom_get_cpu_ticks_per_us();
  // Get wakeup cause.
  wakeup_cause = esp_wake_stub_get_wakeup_cause();
  // Increment the counter.
  s_count++;

  ESP_RTC_LOGI("Before i2c init");

  wake_stub_i2c_init(39, 40);

  ESP_RTC_LOGI("before i2c read");

  uint8_t buf[17];
  wake_stub_i2c_read(IQS323_ADDR, IQS323_MM_SYSTEM_STATUS, buf, 18);

  ESP_RTC_LOGI("After i2c read");

  wakeup_stub_iqs_status.status[0] = buf[0];
  wakeup_stub_iqs_status.status[1] = buf[1];

  wakeup_stub_iqs_status.gestures[0] = buf[2];
  wakeup_stub_iqs_status.gestures[1] = buf[3];

  wakeup_stub_iqs_status.slider_cords[0] = buf[4];
  wakeup_stub_iqs_status.slider_cords[1] = buf[5];

  wakeup_stub_iqs_status.ch0_cnts[0] = buf[6];
  wakeup_stub_iqs_status.ch0_cnts[1] = buf[7];
  wakeup_stub_iqs_status.ch0_cnts[2] = buf[8];
  wakeup_stub_iqs_status.ch0_cnts[3] = buf[9];

  wakeup_stub_iqs_status.ch1_cnts[0] = buf[10];
  wakeup_stub_iqs_status.ch1_cnts[1] = buf[11];
  wakeup_stub_iqs_status.ch1_cnts[2] = buf[12];
  wakeup_stub_iqs_status.ch1_cnts[3] = buf[13];

  wakeup_stub_iqs_status.ch2_cnts[0] = buf[14];
  wakeup_stub_iqs_status.ch2_cnts[1] = buf[15];
  wakeup_stub_iqs_status.ch2_cnts[2] = buf[16];
  wakeup_stub_iqs_status.ch2_cnts[3] = buf[17];
  

  // Print the counter value and wakeup cause.
  ESP_RTC_LOGI("wake stub: wakeup count is %d, wakeup cause is %d, wakeup cost %ld us", s_count, wakeup_cause, wakeup_time);
  ESP_RTC_LOGI("wakeup_stub_iqs values %x, %x", buf[0], buf[1]);

  esp_default_wake_deep_sleep();

  // Return from the wake stub function to continue
  // booting the firmware.
  return;
}