#pragma once
#include <stdint.h>

#define GEN2_PMIC_ADDR      0x48

#define SET_VPOS_VENG2  2
#define SET_VPOS_VENG3  3
#define SET_VCOMDC      4

// Calibration data written to IST9201 PMIC registers (28 bytes)
extern unsigned char initPmicData[28];

#ifdef __cplusplus
extern "C" {
#endif

unsigned char setPmic(void);
unsigned char sendPmicData(unsigned char *buf, unsigned int len);
unsigned char readPmicRegister(unsigned char reg, unsigned char *buf, unsigned int len);
unsigned char enablePmic(void);
void          disablePmic(void);
void          powerSwitchEnable(void);
void          powerSwitchDisable(void);
unsigned int  voltageToRegisterData(unsigned int voltage, unsigned char select);

#ifdef __cplusplus
}
#endif
