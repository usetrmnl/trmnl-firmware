#pragma once

// The TRMNL X touch bar task shares the I2C bus with the battery gauge and the IO expander; these
// lock it around other I2C use. Other boards get no-op stubs, defined in wake_init.cpp.
#ifdef BOARD_TRMNL_X
#include <iqs323_task.h>
#else
void iqs323_task_i2c_lock(void);
void iqs323_task_i2c_unlock(void);
#endif // BOARD_TRMNL_X
