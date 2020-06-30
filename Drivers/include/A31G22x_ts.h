/***************************************************************************//**
* @file     A31G22x_ts.h
* @brief    Contains all macro definitions and function prototypes support
*           for TS driver on A31G22x
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

#ifndef _A31G22x_TS_H_
#define _A31G22x_TS_H_

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


/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief  TS configuration structure definition
 */
typedef struct {
	/* CR */
	FunctionalState MatchInterrupt; /*!< Match Interrupt */
	/* REFPERIOD */
	uint32_t ReferencePeriod; /*!< Reference Period, 0x00000 ~ 0xFFFFF */
} TS_CFG_Type;

/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void TS_Init(TS_CFG_Type *pConfig);
void TS_DeInit(void);
FlagStatus TS_GetStatus(void);
void TS_ClearStatus(void);
uint32_t TS_GetSensingCount(void);
void TS_Start(void);
void TS_Stop(void);


#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_TS_H_ */
/* --------------------------------- End Of File ------------------------------ */
