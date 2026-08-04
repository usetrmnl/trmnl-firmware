#pragma once

#include "api_types.h"

/**
 * @brief Function to serialize log data into JSON format for API submission
 * @param input ApiLogInput struct containing all log data
 * @return String JSON formatted log data
 */
String serialize_log(const LogWithDetails &input);

/**
 * @brief Serialize a log entry without the device status stamp, for debug
 *        capture where the stamp is sent once per wake cycle instead of on
 *        every entry. Cuts a verbose cycle's payload by roughly five times.
 * @param input LogWithDetails struct containing all log data
 * @return String JSON formatted log data
 */
String serialize_log_lean(const LogWithDetails &input);

/**
 * @brief Serialize the device status stamp as a standalone record, written
 *        once per wake cycle so that lean entries can be attributed to the
 *        cycle they came from.
 * @param stamp device status at the time of the wake
 * @param timestamp epoch seconds
 * @return String JSON formatted stamp record, tagged `"type":"stamp"`
 */
String serialize_device_stamp(const DeviceStatusStamp &stamp, time_t timestamp);