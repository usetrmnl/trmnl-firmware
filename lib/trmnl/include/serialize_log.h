#pragma once

#include "api_types.h"

/**
 * @brief Function to serialize log data into JSON format for API submission
 * @param input ApiLogInput struct containing all log data
 * @return String JSON formatted log data
 */
String serialize_log(const LogWithDetails &input);