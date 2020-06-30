/***************************************************************************//**
* @file     A31G22x_timer2n.h
* @brief    Contains all macro definitions and function prototypes support
*           for 32-bit timer2n driver on A31G22x
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

#ifndef _A31G22x_TIMER2N_H_
#define _A31G22x_TIMER2N_H_

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
 * @brief  Timer2n mode selection enumerated definition
 */
typedef enum
{
	TIMER2n_MODE_PERIODIC = 0x00UL, /*!< Periodic(timer/counter) mode */
	TIMER2n_MODE_CAPTURE = 0x01UL, /*!< Capture mode */
	TIMER2n_MODE_ONESHOT = 0x02UL, /*!< One-shot mode */
	TIMER2n_MODE_PWM = 0x03UL /*!< PWM(repeat) mode */
} TIMER2n_MODE_Type;

/**
 * @brief  External clock edge selection enumerated definition
 */
typedef enum
{
	TIMER2n_EXT_CLOCK_EDGE_FALLING = 0x00UL, /*!< Falling edge */
	TIMER2n_EXT_CLOCK_EDGE_RISING = 0x01UL /*!< Rising edge */
} TIMER2n_EXT_CLOCK_EDGE_Type;

/**
 * @brief  Timer2n Capture signal(only timer20) selection enumerated definition
 */
typedef enum
{
	TIMER2n_CAPTURE_SIGNAL_EXTERNAL = 0x00UL, /*!< External capture input signal */
	TIMER2n_CAPTURE_SIGNAL_LSE = 0x01UL, /*!< LSE signal */
	TIMER2n_CAPTURE_SIGNAL_WDTRC = 0x02UL /*!< Watch-dog timer RC oscillator signal */
} TIMER2n_CAPTURE_SIGNAL_Type;

/**
 * @brief  Output polarity selection enumerated definition
 */
typedef enum
{
	TIMER2n_OUTPUT_POLARITY_HIGH = 0x00UL, /*!< Start High level (Output is low level at disable) */
	TIMER2n_OUTPUT_POLARITY_LOW = 0x01UL /*!< Start Low level (Output is High level at disable) */
} TIMER2n_OUTPUT_POLARITY_Type;

/**
 * @brief  Capture polarity selection enumerated definition
 */
typedef enum
{
	TIMER2n_CAPTURE_POLARITY_FALLING = 0x00UL, /*!< Capture on falling edge */
	TIMER2n_CAPTURE_POLARITY_RISING = 0x01UL, /*!< Capture on rising edge */
	TIMER2n_CAPTURE_POLARITY_BOTH = 0x02UL /*!< Capture on both of falling and rising edge */
} TIMER2n_CAPTURE_POLARITY_Type;

/**
 * @brief  Timer2n status enumerated definition
 */
typedef enum {
	TIMER2n_STATUS_CAPTURE_INT = (0x01UL << TIMER2n_CR_CIFLAG_Pos), /*!< Capture interrupt status */
	TIMER2n_STATUS_MATCH_INT = (0x01UL << TIMER2n_CR_MIFLAG_Pos) /*!< Match interrupt status */
} TIMER2n_STATUS_Type;

/**
 * @brief  Timer2n IP index enumerated definition
 */
typedef enum {
	TIMER2n_IP_INDEX_TIMER20,
	TIMER2n_IP_INDEX_TIMER21,
	TIMER2n_IP_INDEX_MAX
} TIMER2n_IP_INDEX_Type;

/**
 * @brief  Timer2n configuration structure definition
 */
typedef struct {
	/* CR */
	FunctionalState ExtClock; /*!< External clock */
	TIMER2n_MODE_Type Mode; /*!< Timer2n mode selection */
	TIMER2n_EXT_CLOCK_EDGE_Type ExtClockEdge; /*!< External clock edge selection */
	TIMER2n_CAPTURE_SIGNAL_Type CaptureSignal; /*!< Capture signal selection (only timer20)*/
	TIMER2n_OUTPUT_POLARITY_Type OutputPolarity; /*!< Output polarity selection */
	TIMER2n_CAPTURE_POLARITY_Type CapturePolarity; /*!< Capture polarity selection */
	FunctionalState MatchInterrupt; /*!< Match interrupt */
	FunctionalState CaptureInterrupt; /*!< Capture interrupt */

	/* ADR */
	uint32_t AData; /*!< A data bit. The range is 0x0002 to 0xFFFF FFFF */
	/* BDR */
	uint32_t BData; /*!< B data bit. The range is 0x0000 to 0xFFFF FFFF */
	/* PREDR */
	uint32_t PrescalerData; /*!< Prescaler data bit. The range is 0x0000 to 0x0FFF. Clock = Fclk / (PRED + 1) */
} TIMER2n_CFG_Type;


/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void TIMER2n_Init(TIMER2n_Type *pTIMER2x, TIMER2n_CFG_Type *pConfig);
void TIMER2n_DeInit(TIMER2n_Type *pTIMER2x);
uint32_t TIMER2n_GetAData(TIMER2n_Type *pTIMER2x);
void TIMER2n_SetAData(TIMER2n_Type *pTIMER2x, uint32_t Data);
uint32_t TIMER2n_GetBData(TIMER2n_Type *pTIMER2x);
void TIMER2n_SetBData(TIMER2n_Type *pTIMER2x, uint32_t Data);
uint32_t TIMER2n_GetCaptureData(TIMER2n_Type *pTIMER2x);
uint32_t TIMER2n_GetPrescalerData(TIMER2n_Type *pTIMER2x);
void TIMER2n_SetPrescalerData(TIMER2n_Type *pTIMER2x, uint32_t Data);
uint32_t TIMER2n_GetCounter(TIMER2n_Type *pTIMER2x);
void TIMER2n_ClearCounter(TIMER2n_Type *pTIMER2x);
TIMER2n_STATUS_Type TIMER2n_GetStatus(TIMER2n_Type *pTIMER2x);
void TIMER2n_ClearStatus(TIMER2n_Type *pTIMER2x, TIMER2n_STATUS_Type Status);
void TIMER2n_Pause(TIMER2n_Type *pTIMER2x, FunctionalState Pause);
void TIMER2n_Start(TIMER2n_Type *pTIMER2x);
void TIMER2n_Stop(TIMER2n_Type *pTIMER2x);


#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_TIMER2N_H_ */
/* --------------------------------- End Of File ------------------------------ */
