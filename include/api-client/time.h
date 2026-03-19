#pragma once

#include <Arduino.h>

/**
 * Result from fetching server time
 */
struct TimeApiResult {
  bool success;
  uint64_t timestamp_ms;  // Server timestamp in milliseconds
  String error;
};

/**
 * Fetch current time from server
 *
 * Makes a GET request to /api/time and parses the JSON response.
 * Used to get a timestamp for Ed25519 authentication signatures.
 *
 * @param baseUrl Base URL of the API server
 * @return TimeApiResult with success status and timestamp or error message
 */
TimeApiResult fetchServerTime(const String &baseUrl);
