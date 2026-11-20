#include <http_result_handling.h>

bool requestIsFailure(https_request_err_e result) {
  switch (result) {
  case HTTPS_SUCCESS:
  case HTTPS_NO_ERR:
  case HTTPS_NO_REGISTER:
  case HTTPS_RESET:
  case HTTPS_PLUGIN_NOT_ATTACHED:
  case HTTPS_IMAGE_DOWNLOAD_FAILED:
    return false;
  default:
    return true;
  }
}

RequestFailureAction requestFailureAction(https_request_err_e result, bool showErrorNow, uint8_t failedWakes) {
  if (!requestIsFailure(result)) return RequestFailureAction::ResetCount;
  if (showErrorNow) return RequestFailureAction::ShowNow;
  return failedWakes < MAX_QUIET_SLOW_RETRIES ? RequestFailureAction::StayQuiet : RequestFailureAction::ShowError;
}

MSG requestErrorScreen(https_request_err_e result, bool weakSignal) {
  switch (result) {
  case HTTPS_RESPONSE_CODE_INVALID:
  case HTTPS_CLIENT_FAILED:
    return WIFI_INTERNAL_ERROR;
  case HTTPS_IMAGE_DOWNLOAD_FAILED:
    return IMAGE_DOWNLOAD_FAILED;
  case HTTPS_UNABLE_TO_CONNECT:
    return weakSignal ? WIFI_WEAK : API_UNABLE_TO_CONNECT;
  case HTTPS_WRONG_IMAGE_FORMAT:
    return MSG_FORMAT_ERROR;
  case HTTPS_WRONG_IMAGE_SIZE:
    return weakSignal ? WIFI_WEAK : API_SIZE_ERROR;
  case HTTPS_TIMED_OUT:
    return WIFI_IMAGE_TIMEOUT;
  default:
    return NONE;
  }
}
