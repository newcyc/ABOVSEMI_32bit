/*************************************************************************************
* @file		A34M41x_frt.h
* @brief	Contains all macro definitions and function prototypes
* 			support for FRT firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
*************************************************************************************/

#ifndef _A34M41x_FRT_H_
#define _A34M41x_FRT_H_

/* Includes ----------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief FRT CTRL
 */
#define	FRT_CTRL_OVFIE			(1<<9)			/*!< FRT Overflow Interrupt Enable bit */	
#define	FRT_CTRL_MATCHIE		(1<<8)			/*!< FRT Match Interrupt Enable bit */

#define FRT_CTRL_MODE_FREE_RUN	(0<<1)
#define FRT_CTRL_MODE_MATCH		(1<<1)

#define FRT_CTRL_FRT_ON			(1<<0)


/**
 * @brief FRT Flags
 */
#define	FRT_STAT_OVFI			(1<<9)			/*!< FRT Overflow Interrupt flag bit */	
#define	FRT_STAT_MATCHI			(1<<8)			/*!< FRT Match Interrupt flag bit */

/* Public Functions ----------------------------------------------------------- */
HAL_Status_Type HAL_FRT_Init(FRT_Type *FRTx, uint8_t MODE_v);

HAL_Status_Type HAL_FRT_ClockSource(FRT_Type *FRTx, uint8_t ClkSource, uint8_t ClkDivider);
uint8_t HAL_FRT_GetClockSource(FRT_Type *FRTx);

HAL_Status_Type HAL_FRT_Run(FRT_Type *FRTx, uint8_t isset);

HAL_Status_Type HAL_FRT_MatchInterrupt(FRT_Type *FRTx, uint8_t isset);
HAL_Status_Type HAL_FRT_OverflowInterrupt(FRT_Type *FRTx, uint8_t isset);

HAL_Status_Type HAL_FRT_ClearStatus(FRT_Type *FRTx, uint16_t StatusType);
uint16_t HAL_FRT_GetStatus(FRT_Type *FRTx);

uint32_t HAL_FRT_GetCounterVal(FRT_Type *FRTx);
HAL_Status_Type HAL_FRT_ClearCounter(FRT_Type *FRTx);

HAL_Status_Type HAL_FRT_SetMatchCounter(FRT_Type *FRTx, uint32_t MCNT_v);
uint32_t HAL_FRT_GetMatchCounter(FRT_Type *FRTx);
HAL_Status_Type HAL_FRT_ModeSelection(FRT_Type *FRTx, uint8_t MODE_v);


#ifdef __cplusplus
}
#endif


#endif /* _A34M41x_FRT_H_ */

/* --------------------------------- End Of File ------------------------------ */

