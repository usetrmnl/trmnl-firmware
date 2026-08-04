#pragma once

#include "api_types.h"

/**
 * @brief Function to serialize log data into JSON format for API submission
 * @param input ApiLogInput struct containing all log data
 * @return String JSON formatted log data
 */
String serialize_log(const LogWithDetails &input);

/**
 * @brief Serialize a log entry without the device status stamp, which is most
 *        of an entry's size. Used while collecting verbose logs, where the
 *        stamp is written once per wake by serialize_device_stamp instead.
 * @param input LogWithDetails struct containing all log data
 * @return String JSON formatted log data
 */
String serialize_log_lean(const LogWithDetails &input);

/**
 * @brief Serialize the device status stamp on its own, so the entries that
 *        follow it can be read against the device state of their own wake.
 * @param stamp device status at the time of the wake
 * @param timestamp epoch seconds
 * @return String JSON formatted stamp record, tagged `"type":"stamp"`
 */
String serialize_device_stamp(const DeviceStatusStamp &stamp, time_t timestamp);