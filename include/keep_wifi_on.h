#pragma once

#include <types.h>

// While the server answers keep_wifi_on, waits refresh_rate awake with Wi-Fi on and calls show_screen again
void stay_awake_while_server_asks(https_request_err_e (*show_screen)(bool draw_error_screen_on_failure));
