//=================================================================================================
//                   PMIC for 31.5" Control Board
//
// File Name : pmic.h
// Author : Electronic Design Dept. II
// Data : 2023.12.15
// Version : 1.1
// Copyright : E Ink Holdings Inc.
//=================================================================================================


#ifndef __PMIC_H__
#define __PMIC_H__

#define PMIC_ADDR	0x48

#define SET_VPOS_VENG2 2
#define SET_VPOS_VENG3 3
#define SET_VCOMDC 4

#endif //#ifndef __PMIC_H__

#define __PMIC_EXTERN__ extern

__PMIC_EXTERN__ unsigned char initPmicData[28];

#ifdef __cplusplus
extern "C" {
#endif

extern unsigned char setPmic(void);
extern unsigned char sendPmicData(unsigned char *dataBuffer, unsigned int dataLength);
extern unsigned char readPmicRegister(unsigned char registerAddress, unsigned char *dataBuffer, unsigned int dataLength);
extern unsigned char enablePmic(void);
extern void disablePmic(void);
extern void powerSwitchEnable(void);
extern void powerSwitchDisable(void);
extern unsigned int voltageToRegisterData(unsigned int voltageData, unsigned char setSelect);

#ifdef __cplusplus
}
#endif
