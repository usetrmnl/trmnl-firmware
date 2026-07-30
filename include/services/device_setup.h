#pragma once

#include <api-client/setup.h>
#include <display.h>
#include <persistence_interface.h>
#include <stdint.h>

//
// derive device-specific setup flags
//
#if defined(BOARD_TRMNL_X)
#define INCLUDE_DEVICE_SETUP_X
#endif

/// @brief Outcome of the device setup flow.
enum class DeviceSetupOutcome {
  Success,            // credentials stored and setup image downloaded
  ConnectionError,    // could not reach the server
  ParsingError,       // malformed /api/setup response
  RequestError,       // /api/setup request failed
  MacNotRegistered,   // server does not know this device
  ApiStatusError,     // /api/setup returned an unexpected status code
  ImageDownloadError, // setup image download failed
  ImageInvalidError,  // setup image had an unexpected size or format
};

/// @brief Everything the caller needs to finish the setup flow: which screen
///        to draw, the /api/setup payload fields, and whether to sleep.
struct DeviceSetupResult {
  DeviceSetupOutcome outcome = DeviceSetupOutcome::RequestError;
  String imageUrl;               // setup image URL from /api/setup
  String message;                // user-facing message from /api/setup
  String friendlyId;             // for the FRIENDLY_ID screen
  ApiSetupResponse apiResponse;  // for the MAC_NOT_REGISTERED screen (404)
  MSG errorScreen = NONE;        // error screen to show, NONE for no screen
  bool showSetupScreen = false;  // draw the FRIENDLY_ID screen
  bool shouldGoToSleep = false;  // 404 path: put the device to sleep
};

/// @brief Registers the device with the server: fetches credentials (API key,
///        friendly ID) from /api/setup, then downloads the setup image.
///        Display and sleep side effects are returned to the caller via
///        DeviceSetupResult rather than performed here.
class DeviceSetup {
public:
  explicit DeviceSetup(Persistence &persistence) : _persistence(persistence) {}
  virtual ~DeviceSetup() = default;

  /// @brief Run the full setup flow.
  DeviceSetupResult perform();

protected:
  /// @brief Call the /api/setup endpoint.
  virtual ApiSetupResult callSetupApi(ApiSetupInputs &inputs);

  /// @brief Download the setup image, filling _result's outcome and
  ///        FRIENDLY_ID screen fields.
  virtual void downloadSetupImage();

  /// @brief Handle a downloaded setup image with an unexpected size or format.
  virtual void handleInvalidImage(uint32_t bytesRead);

  Persistence &_persistence;
  DeviceSetupResult _result;

private:
  void performApiSetup();
};

#ifdef INCLUDE_DEVICE_SETUP_X
class Modem;

/// @brief TRMNL X setup flow: routes API and image traffic through the 5 GHz
///        modem when connected to a 5 GHz network.
class DeviceSetupX : public DeviceSetup {
public:
  /// @brief `modem` is a reference to the owner's pointer because the modem is
  ///        initialized during bl_init, after static construction.
  DeviceSetupX(Persistence &persistence, Modem *&modem) : DeviceSetup(persistence), _modem(modem) {}

protected:
  ApiSetupResult callSetupApi(ApiSetupInputs &inputs) override;
  void downloadSetupImage() override;
  void handleInvalidImage(uint32_t bytesRead) override;

private:
  bool is5Ghz();
  Modem *&_modem;
};
#endif // INCLUDE_DEVICE_SETUP_X

/// @brief Device setup service for the running board, selected at compile time.
DeviceSetup &deviceSetup();
