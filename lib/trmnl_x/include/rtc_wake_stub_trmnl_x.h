#pragma once

#include <stdint.h>
#include <stddef.h>

#define IQS323_MM_SYSTEM_STATUS  0x10

#define IQS323_ADDR 0x44

typedef struct {
    uint8_t status[2];
    uint8_t gestures[2];
    uint8_t slider_cords[2];

    uint8_t ch0_cnts[4];
    uint8_t ch1_cnts[4];
    uint8_t ch2_cnts[4];
} iqs323_system_status_t;

extern iqs323_system_status_t wakeup_stub_iqs_status;

void wakeup_stub(void);