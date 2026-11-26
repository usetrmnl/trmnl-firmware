#pragma once

// While the server answers keep_wifi_on, waits refresh_rate awake with Wi-Fi on and calls show_screen again
void stay_awake_while_server_asks(void (*show_screen)(void));
