/***************************************************************************//**
* @file     A31G22x_wt.h
* @brief    Contains all macro definitions and function prototypes support
*           for watch timer driver on A31G22x
* @author   AE Team, ABOV Semiconductor Co., Ltd.
* @version  V0.0.1
* @date     30. Jul. 2018
*
* Copyright(C) 2018, ABOV Semiconductor
* All rights reserved.
*
*
********************************************************************************
* DISCLAIMER 
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

#ifndef _A31G22x_WT_H_
#define _A31G22x_WT_H_

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
* Included File
*******************************************************************************/
#include "A31G22x.h"
#include "aa_types.h"


/*******************************************************************************
* Public Macro
*******************************************************************************/
#define	WT_INTERRUPT_FLAG		(0x1UL)

/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief  watch timer clock divider selection enumerated definition
 */
typedef enum {
	WT_DIVIDER_2_7 				= 0x00UL, /*!< WT Clock / (2 ^ 7) */
	WT_DIVIDER_2_13 			= 0x01UL, /*!< WT Clock / (2 ^ 13) */
	WT_DIVIDER_2_14 			= 0x02UL, /*!< WT Clock / (2 ^ 14) */
	WT_DIVIDER_2_14_MUL_WTDR 	= 0x03UL  /*!< WT Clock / ((2 ^ 14) * (WTDR + 1)) */
} WT_DIVIDER_Type;

/**
 * @brief  watch timer configuration structure definition
 */
typedef struct {
	WT_DIVIDER_Type ClockDivider; 	/*!< watch timer clock divider selection */
	uint32_t MatchData;				/*!< Data of watch timer, 0x001 ~ 0xFFF */
} WT_CFG_Type;


/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void WT_Init(WT_CFG_Type *pConfig);
void WT_DeInit(void);
void WT_ConfigureInterrupt(FunctionalState Interrupt);
uint32_t WT_GetCurrentCount(void);
FlagStatus WT_GetInterruptStatus(void);
void WT_ClearInterruptStatus(void);
void WT_Start(void);
void WT_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_WT_H_ */
/* --------------------------------- End Of File ------------------------------ */
