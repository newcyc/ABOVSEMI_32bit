/**********************************************************************
* @file		A34M41x_pwr.h
* @brief	Contains all macro definitions and function prototypes
* 			support for PCU firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_PWR_H_
#define _A34M41x_PWR_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* LVI Definition ------------------------------------------------------------- */
#define LVI_DISABLE				(0x0uL<<7)
#define LVI_ENABLE					(0x1uL<<7)

#define LVI_INTDIS					(0x0uL<<5)
#define LVI_INTEN					(0x1uL<<5)
	
#define LVI_DEEPDIS				(0x0uL<<4)
#define LVI_DEEPON				(0x1uL<<4)
	
#define LVI_LVL_1_60V				(0x0uL<<0)
#define LVI_LVL_1_69V				(0x1uL<<0)
#define LVI_LVL_1_78V				(0x2uL<<0)
#define LVI_LVL_1_90V				(0x3uL<<0)
#define LVI_LVL_1_99V				(0x4uL<<0)
#define LVI_LVL_2_12V				(0x5uL<<0)
#define LVI_LVL_2_30V				(0x6uL<<0)
#define LVI_LVL_2_47V				(0x7uL<<0)
#define LVI_LVL_2_67V				(0x8uL<<0)
#define LVI_LVL_3_04V				(0x9uL<<0)
#define LVI_LVL_3_18V				(0xAuL<<0)
#define LVI_LVL_3_59V				(0xBuL<<0)
#define LVI_LVL_3_72V				(0xCuL<<0)
#define LVI_LVL_4_03V				(0xDuL<<0)
#define LVI_LVL_4_20V				(0xEuL<<0)
#define LVI_LVL_4_48V				(0xFuL<<0)

/* LVR Definition ------------------------------------------------------------- */
#define LVR_DISABLE				(0xAAuL<<8)
#define LVR_ENABLE				(0x00uL<<8)
	
#define LVR_DEEPDIS				(0x0uL<<4)
#define LVR_DEEPON				(0x1uL<<4)
	
#define LVR_1_60V				(0x0uL<<0)
#define LVR_1_69V				(0x1uL<<0)
#define LVR_1_78V				(0x2uL<<0)
#define LVR_1_90V				(0x3uL<<0)
#define LVR_1_99V				(0x4uL<<0)
#define LVR_2_12V				(0x5uL<<0)
#define LVR_2_30V				(0x6uL<<0)
#define LVR_2_47V				(0x7uL<<0)
#define LVR_2_67V				(0x8uL<<0)
#define LVR_3_04V				(0x9uL<<0)
#define LVR_3_18V				(0xAuL<<0)
#define LVR_3_59V				(0xBuL<<0)
#define LVR_3_72V				(0xCuL<<0)
#define LVR_4_03V				(0xDuL<<0)
#define LVR_4_20V				(0xEuL<<0)
#define LVR_4_48V				(0xFuL<<0)

typedef struct {
	uint32_t	EnSel;
	uint32_t	IntSel;
	uint32_t	DeepSel;
	uint32_t	LvlSel;
} LVI_CFG_Type;

typedef struct {
	uint32_t	EnSel;
	uint32_t	DeepSel;
	uint32_t	LvlSel;
} LVR_CFG_Type;

/* Public Functions ----------------------------------------------------------- */
void HAL_PWR_EnterSleepMode(void);
void HAL_PWR_EnterPowerDownMode(void);
void HAL_LVI_Init(LVI_CFG_Type *LVI_ConfigStruct);
void HAL_LVR_Init(LVR_CFG_Type *LVR_ConfigStruct);
#ifdef __cplusplus
}
#endif


#endif /* end _A34M41x_PWR_H_ */

/* --------------------------------- End Of File ------------------------------ */
