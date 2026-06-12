#include <api-client/request_headers.h>
#include <trmnl_log.h>

HttpHeaderList buildDisplayHeaders(const ApiDisplayInputs &inputs)
{
  HttpHeaderList headers;
  headers.push_back({"ID", inputs.macAddress});
  headers.push_back({"Content-Type", "application/json"});
  headers.push_back({"Update-Source", inputs.updateSource});
  headers.push_back({"Access-Token", inputs.apiKey});
  headers.push_back({"Refresh-Rate", String(inputs.refreshRate)});
  headers.push_back({"Battery-Voltage", String(inputs.batteryVoltage)});

  if (inputs.chargingStatus != ChargingStatus::UNKNOWN)
    headers.push_back({"Battery-Charging", String(inputs.chargingStatus == ChargingStatus::CHARGING ? "1" : "0")});

  if (inputs.usbStatus != UsbStatus::UNKNOWN)
    headers.push_back({"USB-Connected", inputs.usbStatus == UsbStatus::CONNECTED ? "true" : "false"});

#ifdef BOARD_TRMNL_X
  headers.push_back({"Battery-Count", String(inputs.batteryCount)});
  headers.push_back({"Percent-Charged", String(inputs.stateOfCharge)});
  headers.push_back({"Battery-Health", String(inputs.stateOfHealth)});
  headers.push_back({"Battery-Current", String(inputs.batteryCurrent)});
  headers.push_back({"Battery-Temp", String(inputs.batteryTemperature)});
  headers.push_back({"Battery-Capacity", String(inputs.currentBatteryCapacity) + "/" + String(inputs.maxBatteryCapacity)});
#endif // BOARD_TRMNL_X
  headers.push_back({"FW-Version", inputs.firmwareVersion});
  headers.push_back({"Model", String(inputs.model)});
  headers.push_back({"Image-Cached", inputs.imageCached ? "true" : "false"});
  headers.push_back({"Wake-Time", String(inputs.prevWakeTime)});
  headers.push_back({"RSSI", String(inputs.rssi)});
  if (inputs.wifiBand.length() > 0)
    headers.push_back({"WiFi-Band", inputs.wifiBand});
  headers.push_back({"Temperature-Profile", "true"});
  headers.push_back({"Width", String(inputs.displayWidth)});
  headers.push_back({"Height", String(inputs.displayHeight)});

  if (inputs.specialFunction != SF_NONE)
    headers.push_back({"special_function", "true"});

  return headers;
}

HttpHeaderList buildSetupHeaders(const ApiSetupInputs &inputs)
{
  HttpHeaderList headers;
  headers.push_back({"ID", inputs.macAddress});
  headers.push_back({"Content-Type", "application/json"});
  headers.push_back({"FW-Version", inputs.firmwareVersion});
  headers.push_back({"Model", inputs.model});
  return headers;
}

HttpHeaderList buildLogHeaders(const ApiLogInputs &inputs)
{
  HttpHeaderList headers;
  headers.push_back({"ID", inputs.macAddress});
  headers.push_back({"Accept", "application/json, */*"});
  headers.push_back({"Access-Token", inputs.apiKey});
  headers.push_back({"Content-Type", "application/json"});
  return headers;
}

String formatHeaders(const HttpHeaderList &headers)
{
  String out;
  for (size_t i = 0; i < headers.size(); ++i)
  {
    if (i > 0)
      out += "\n";
    out += headers[i].first + ": " + headers[i].second;
  }
  return out;
}

void logHeaders(const HttpHeaderList &headers)
{
  Log_info("Added headers:\n%s", formatHeaders(headers).c_str());
}
