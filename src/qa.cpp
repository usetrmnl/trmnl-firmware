#include <Arduino.h>
#include <display.h>
#include "button.h"
#include "pins.h"
#include "config.h"
#include <ArduinoLog.h>
#include <Preferences.h>
#include "WifiCaptive.h"
#include "logo_small.h"

extern "C" {
  #include "esp_timer.h"   // esp_timer_get_time()
}

Preferences preferencesQA;

const int adcPin = 3; // ADC pin for voltage measurement
const int samples = 1000;
const int intervalMs = 7000; // 7s
const int sample_interval = 1; //1 ms
const int temperature_threshold = 35; // 35 by Celsium
bool result = false;
float initialTemp = 0;
static bool radioOn = false;

float tempDiff = 0;
float voltageDiff = 0;

volatile bool stopRequested = false;

void IRAM_ATTR onBtnPress() {
  stopRequested = true;
}


bool checkIfAlreadyPassed(){
  preferencesQA.begin("qa", true);
  bool testPassed = preferencesQA.getBool("testPassed", false);
  preferencesQA.end();
  return testPassed;
}

void savePassedTest(){
  preferencesQA.begin("qa", false);
  preferencesQA.putBool("testPassed", true);
  preferencesQA.end();
}

float measureTemperatureAverage() {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += temperatureRead();
    delay(sample_interval);
  }
  return sum / samples;
}

float measureVoltageAverage() {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(adcPin);
    delay(sample_interval);
  }
  return (sum / samples) * (3.3 / 4095.0) * 2; 
}


static inline void startRadioRX() {
  if (radioOn) return;
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);                
  WiFi.scanNetworks(true, true, true, 120, 0);  
  radioOn = true;
}

static inline void pumpRadioRX() {
  if (!radioOn) return;
  if (WiFi.scanComplete() != WIFI_SCAN_RUNNING) {
    WiFi.scanNetworks(true, true, true, 120, 0);
  }
}

static inline void stopRadioRX() {
  if (!radioOn) return;
  WiFi.mode(WIFI_OFF);
  radioOn = false;
}

static void loadCPUAndRadio(uint32_t ms) {
  static uint32_t scratch[128];
  for (int i = 0; i < 128; ++i) scratch[i] = i * 2654435761u;

  volatile uint32_t a = 0x1234567u, b = 0x89ABCDEFu;
  volatile float fx = 1.2345f, fy = 3.14159f;
  const uint64_t deadline = esp_timer_get_time() + (uint64_t)ms * 1000ULL;

  uint32_t iters = 0;
  while (!stopRequested && esp_timer_get_time() < deadline) {
    
    a ^= b; b += a; a = (a << 5) | (a >> (27)); b = (b << 7) ^ (b >> 3);

    
    fx = fx * 1.000123f + 0.000987f;
    fy = fy * 0.999771f - 0.000321f;

    
    scratch[(a ^ b) & 127] ^= a + b;
    scratch[(a + b) & 127] += (uint32_t)(fx * 1000.0f);

    
    if ((++iters & 0x3FF) == 0) { 
      pumpRadioRX();
      yield();
    }
  }
}

bool startQA(){
  
  bool wifiSaved = checkForSavedCredentials();

  if(wifiSaved){

    Serial.print("WiFi credentials found, skipping QA\n");
    return true;
  }

  while(!stopRequested){
  
  Serial.begin(115200);
  
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  pins_init();
  Log.info("QA Test started\n");
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), onBtnPress, FALLING);
  
  uint8_t *buffer = (uint8_t *)malloc(48000);
  memset(buffer, 255, 48000);
  display_init();
  
  display_show_msg(const_cast<uint8_t *>(logo_small),QA_START);
  
  Log.info("QA Test started\n");

  float initial_temp = measureTemperatureAverage();/*
  if (initial_temp > temperature_threshold){
    return false;
  }*/
  float initial_voltage = measureVoltageAverage();
  /*if (initial_voltage > temperature_threshold){
    return false;
  }*/

  Log.info("Stress test started\n");
  startRadioRX();
  loadCPUAndRadio(intervalMs);
  stopRadioRX();
  Log.info("Stress test ended\n");
  if(stopRequested){
    break;
  }
  
  float last_temp = measureTemperatureAverage();
  float last_voltage = measureVoltageAverage();

  Log.info("QA test ended\n");
  
  tempDiff = last_temp - initial_temp;
  voltageDiff = last_voltage - initial_voltage;

  float temperature[3] = {initial_temp, last_temp, tempDiff};
  float voltage[3] = {initial_voltage, last_voltage, voltageDiff};

  result = last_temp - initial_temp < 3;

  Log.info("Displaying results\n");
  display_init();
  display_show_msg_qa(buffer,voltage,temperature,result);
  break;

  }

  if(!stopRequested){
    savePassedTest();
    while (1){
      Serial.println("QA Test Passed. Long press the button to continue...");
      auto button = read_long_press();

      if(button == LongPress){
        Serial.println("Long press detected, painting screen white");
        display_show_msg(NULL, FILL_WHITE);
        delay(10000);
        result = true;
        break;
      }

    }
  }
  return result;
}

void testLoadScreen(){
  display_init();
  uint8_t *buffer = (uint8_t *)malloc(48000);
  memset(buffer, 255, 48000);
  display_show_msg(const_cast<uint8_t *>(logo_small),QA_START);
}

void testResultScreen(bool result){
  display_init();
  uint8_t *buffer = (uint8_t *)malloc(48000);
  memset(buffer, 255, 48000);
  float temperature[3] = {0,0,0};
  float voltage[3] = {0,0,0};
  display_show_msg_qa(buffer,voltage,temperature,result);
}




