#include <string.h>

typedef enum {
    ESP_SLEEP_WAKEUP_UNDEFINED,    //!< In case of deep sleep, reset was not caused by exit from deep sleep
    ESP_SLEEP_WAKEUP_ALL,          //!< Not a wakeup cause, used to disable all wakeup sources with esp_sleep_disable_wakeup_source
    ESP_SLEEP_WAKEUP_EXT0,         //!< Wakeup caused by external signal using RTC_IO
    ESP_SLEEP_WAKEUP_EXT1,         //!< Wakeup caused by external signal using RTC_CNTL
    ESP_SLEEP_WAKEUP_TIMER,        //!< Wakeup caused by timer
    ESP_SLEEP_WAKEUP_TOUCHPAD,     //!< Wakeup caused by touchpad
    ESP_SLEEP_WAKEUP_ULP,          //!< Wakeup caused by ULP program
    ESP_SLEEP_WAKEUP_GPIO,         //!< Wakeup caused by GPIO (light sleep only on ESP32, S2 and S3)
    ESP_SLEEP_WAKEUP_UART,         //!< Wakeup caused by UART (light sleep only)
    ESP_SLEEP_WAKEUP_WIFI,              //!< Wakeup caused by WIFI (light sleep only)
    ESP_SLEEP_WAKEUP_COCPU,             //!< Wakeup caused by COCPU int
    ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG,   //!< Wakeup caused by COCPU crash
    ESP_SLEEP_WAKEUP_BT,           //!< Wakeup caused by BT (light sleep only)
} esp_sleep_source_t;

struct WakeupReasonNode
{
  const char *name;
  esp_sleep_source_t value;
};

static const WakeupReasonNode wakeupReasonMap[] = {
    {"powercycle", ESP_SLEEP_WAKEUP_UNDEFINED},
    {"all", ESP_SLEEP_WAKEUP_ALL},
    {"EXT0", ESP_SLEEP_WAKEUP_EXT0},
    {"EXT1", ESP_SLEEP_WAKEUP_EXT1},
    {"timer", ESP_SLEEP_WAKEUP_TIMER},
    {"touchpad", ESP_SLEEP_WAKEUP_TOUCHPAD},
    {"ulp", ESP_SLEEP_WAKEUP_ULP},
    {"button", ESP_SLEEP_WAKEUP_GPIO},
    {"uart", ESP_SLEEP_WAKEUP_UART},
    {"wifi", ESP_SLEEP_WAKEUP_WIFI},
    {"cocpu", ESP_SLEEP_WAKEUP_COCPU},
    {"cocpu_trap_trig", ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG},
    {"bt", ESP_SLEEP_WAKEUP_BT},
};

bool parseWakeupReasonToStr(char *buffer, size_t buffer_size, esp_sleep_source_t wakeup_reason)
{
  for (const WakeupReasonNode &entry : wakeupReasonMap)
  {
    if (wakeup_reason == entry.value)
    {
      strncpy(buffer, entry.name, buffer_size);
      return true;
    }
  }
  return false;
}