/***************************************************************************//**
* @file     A31G22x_timer1n.h
* @brief    Contains all macro definitions and function prototypes support
*           for 16-bit timer1n driver on A31G22x
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

#ifndef _A31G22x_TIMER1N_H_
#define _A31G22x_TIMER1N_H_

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
 * @brief  Timer1n mode selection enumerated definition
 */
typedef enum
{
	TIMER1n_MODE_PERIODIC = 0x00UL, /*!< Periodic(timer/counter) mode */
	TIMER1n_MODE_CAPTURE = 0x01UL, /*!< Capture mode */
	TIMER1n_MODE_ONESHOT = 0x02UL, /*!< One-shot mode */
	TIMER1n_MODE_REPEAT = 0x03UL /*!< PWM(repeat) mode */
} TIMER1n_MODE_Type;

/**
 * @brief  External clock edge selection enumerated definition
 */
typedef enum
{
	TIMER1n_EXT_CLOCK_EDGE_FALLING = 0x00UL, /*!< Falling edge */
	TIMER1n_EXT_CLOCK_EDGE_RISING = 0x01UL /*!< Rising edge */
} TIMER1n_EXT_CLOCK_EDGE_Type;

/**
 * @brief  Output polarity selection enumerated definition
 */
typedef enum
{
	TIMER1n_OUTPUT_POLARITY_HIGH = 0x00UL, /*!< Start High level (Output is low level at disable) */
	TIMER1n_OUTPUT_POLARITY_LOW = 0x01UL /*!< Start Low level (Output is High level at disable) */
} TIMER1n_OUTPUT_POLARITY_Type;

/**
 * @brief  Capture polarity selection enumerated definition
 */
typedef enum
{
	TIMER1n_CAPTURE_POLARITY_FALLING = 0x00UL, /*!< Capture on falling edge */
	TIMER1n_CAPTURE_POLARITY_RISING = 0x01UL, /*!< Capture on rising edge */
	TIMER1n_CAPTURE_POLARITY_BOTH = 0x02UL /*!< Capture on both of falling and rising edge */
} TIMER1n_CAPTURE_POLARITY_Type;

/**
 * @brief  Timer1n status enumerated definition
 */
typedef enum {
	TIMER1n_STATUS_CAPTURE_INT = (0x01UL << TIMER1n_CR_CIFLAG_Pos), /*!< Capture interrupt status */
	TIMER1n_STATUS_MATCH_INT = (0x01UL << TIMER1n_CR_MIFLAG_Pos) /*!< Match interrupt status */
} TIMER1n_STATUS_Type;

/**
 * @brief  Timer1n IP index enumerated definition
 */
typedef enum {
	TIMER1n_IP_INDEX_TIMER10,
	TIMER1n_IP_INDEX_TIMER11,
	TIMER1n_IP_INDEX_TIMER12,
	TIMER1n_IP_INDEX_TIMER13,
	TIMER1n_IP_INDEX_TIMER14,
	TIMER1n_IP_INDEX_TIMER15,
	TIMER1n_IP_INDEX_TIMER16,
	TIMER1n_IP_INDEX_MAX
} TIMER1n_IP_INDEX_Type;

/**
 * @brief  Timer1n configuration structure definition
 */
typedef struct {
	/* CR */
	FunctionalState StartSync; /*!< Synchronized start counter with TIMER30 */
	FunctionalState ClearSync; /*!< Synchronized clear counter with TIMER30 */
	FunctionalState ExtClock; /*!< External clock */
	TIMER1n_MODE_Type Mode; /*!< Timer1n mode selection */
	TIMER1n_EXT_CLOCK_EDGE_Type ExtClockEdge; /*!< External clock edge selection */
	TIMER1n_OUTPUT_POLARITY_Type OutputPolarity; /*!< Output polarity selection */
	TIMER1n_CAPTURE_POLARITY_Type CapturePolarity; /*!< Capture polarity selection */
	FunctionalState MatchInterrupt; /*!< Match interrupt */
	FunctionalState CaptureInterrupt; /*!< Capture interrupt */

	/* ADR */
	uint32_t AData; /*!< A data bit. The range is 0x0002 to 0xFFFF */
	/* BDR */
	uint32_t BData; /*!< B data bit. The range is 0x0000 to 0xFFFF */
	/* PREDR */
	uint32_t PrescalerData; /*!< Prescaler data bit. The range is 0x0000 to 0x0FFF. Clock = Fclk / (PRED + 1) */
} TIMER1n_CFG_Type;


/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void TIMER1n_Init(TIMER1n_Type *pTIMER1x, TIMER1n_CFG_Type *pConfig);
void TIMER1n_DeInit(TIMER1n_Type *pTIMER1x);
uint32_t TIMER1n_GetAData(TIMER1n_Type *pTIMER1x);
void TIMER1n_SetAData(TIMER1n_Type *pTIMER1x, uint32_t Data);
uint32_t TIMER1n_GetBData(TIMER1n_Type *pTIMER1x);
void TIMER1n_SetBData(TIMER1n_Type *pTIMER1x, uint32_t Data);
uint32_t TIMER1n_GetCaptureData(TIMER1n_Type *pTIMER1x);
uint32_t TIMER1n_GetPrescalerData(TIMER1n_Type *pTIMER1x);
void TIMER1n_SetPrescalerData(TIMER1n_Type *pTIMER1x, uint32_t Data);
uint32_t TIMER1n_GetCounter(TIMER1n_Type *pTIMER1x);
void TIMER1n_ClearCounter(TIMER1n_Type *pTIMER1x);
TIMER1n_STATUS_Type TIMER1n_GetStatus(TIMER1n_Type *pTIMER1x);
void TIMER1n_ClearStatus(TIMER1n_Type *pTIMER1x, TIMER1n_STATUS_Type Status);
void TIMER1n_Pause(TIMER1n_Type *pTIMER1x, FunctionalState Pause);
void TIMER1n_Start(TIMER1n_Type *pTIMER1x);
void TIMER1n_Stop(TIMER1n_Type *pTIMER1x);


#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_TIMER1N_H_ */
/* --------------------------------- End Of File ------------------------------ */
