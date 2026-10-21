#include <request_result.h>

bool requestFailed(https_request_err_e result) {
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
