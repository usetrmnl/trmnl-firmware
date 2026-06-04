// Integration tests for the production /api/setup client.
//
// Talks to the real backend at TEST_BACKEND_URL (https://trmnl.app). The MAC
// in TEST_MAC_ADDRESS must be registered to a test account on that backend
// for the success case to return 200.

#include <Arduino.h>
#include <unity.h>
#include <api-client/setup.h>
#include <config.h> // FW_VERSION_STRING, DEVICE_MODEL
#include <test_helpers.h>
#include "tests.h"

static void test_setup_succeeds_with_registered_mac(void)
{
  ApiSetupInputs inputs = {
      .baseUrl = TEST_BACKEND_URL,
      .macAddress = TEST_MAC_ADDRESS,
      .firmwareVersion = FW_VERSION_STRING,
      .model = DEVICE_MODEL,
  };

  Serial.printf("calling /api/setup with MAC=%s url=%s\n",
                inputs.macAddress.c_str(), inputs.baseUrl.c_str());

  ApiSetupResult result = fetchApiSetup(inputs);

  TEST_ASSERT_EQUAL_MESSAGE(HTTPS_NO_ERR, result.error,
                            "fetchApiSetup returned a transport error — backend unreachable?");
  TEST_ASSERT_EQUAL_MESSAGE((int)ApiSetupOutcome::Ok, (int)result.response.outcome,
                            "Expected Ok outcome — is the device's MAC registered as a test account?");
  TEST_ASSERT_FALSE_MESSAGE(result.response.api_key.isEmpty(),
                            "Response should carry a non-empty api_key");
  TEST_ASSERT_FALSE_MESSAGE(result.response.friendly_id.isEmpty(),
                            "Response should carry a non-empty friendly_id");
}

static void test_setup_returns_status_error_for_unknown_mac(void)
{
  ApiSetupInputs inputs = {
      .baseUrl = TEST_BACKEND_URL,
      .macAddress = "00:00:00:00:00:01",
      .firmwareVersion = FW_VERSION_STRING,
      .model = DEVICE_MODEL,
  };

  ApiSetupResult result = fetchApiSetup(inputs);

  TEST_ASSERT_EQUAL_MESSAGE(HTTPS_NO_ERR, result.error,
                            "Transport should succeed even when MAC is unregistered");
  TEST_ASSERT_EQUAL_MESSAGE((int)ApiSetupOutcome::StatusError, (int)result.response.outcome,
                            "Expected StatusError outcome for an unregistered MAC");
  TEST_ASSERT_NOT_EQUAL_MESSAGE(200, result.response.status,
                                "Unregistered MAC should not return 200");
}

static void test_setup_fails_with_unreachable_host(void)
{
  ApiSetupInputs inputs = {
      .baseUrl = "https://trmnl-nonexistent-host-xyz.invalid",
      .macAddress = TEST_MAC_ADDRESS,
      .firmwareVersion = FW_VERSION_STRING,
      .model = DEVICE_MODEL,
  };

  ApiSetupResult result = fetchApiSetup(inputs);

  TEST_ASSERT_NOT_EQUAL_MESSAGE(HTTPS_NO_ERR, result.error,
                                "Unreachable host should produce a transport error, not HTTPS_NO_ERR");
}

void test_api_setup(void)
{
  // Tests
  RUN_TEST(test_setup_succeeds_with_registered_mac);
  RUN_TEST(test_setup_returns_status_error_for_unknown_mac);
  RUN_TEST(test_setup_fails_with_unreachable_host);
}
