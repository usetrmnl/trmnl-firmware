#include <Arduino.h>
#include <WebServer.h>
#include <test.h>

extern AsyncWebServer* _server;


const int adcPin = 3; // ADC pin for voltage measurement
const int samples = 1000;
const int intervalMs = 7000; // 7s
const int sample_interval = 1; //1 ms
const int temperature_threshold = 55; // 35 by Celsium

float initialTemp = 0;


void setupTestEndpoint() {
  _server->on("/run-test", HTTP_GET, [](AsyncWebServerRequest* request){
    
  });
}

float measureTemperatureAverage() {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += temperatureRead();
    delay(sample_interval);
  }
  return sum / samples;
}

String Main_Test() {
  float initial_temp = measureTemperatureAverage();
  float last_temp;

  bool passed = false;

  if (initial_temp <= temperature_threshold) {
    delay(intervalMs);
    last_temp = measureTemperatureAverage();
    float diff = last_temp - initial_temp;
    passed = diff < 2;
  } else {
    String json = "{";
    json += "\"Threshold\":" + String("true");
    json += "}";
    return json;
  }

  float diff = last_temp - initial_temp;

  String json = "{";
  json += "\"testPassed\":" + String(passed ? "true" : "false") + ",";
  json += "\"initialTemp\":" + String(initial_temp, 2) + ",";
  json += "\"finalTemp\":" + String(last_temp, 2) + ",";
  json += "\"tempDifference\":" + String(diff, 2) + ",";
  json += "\"Threshold\":" + String("false");
  json += "}";

  return json;
}
