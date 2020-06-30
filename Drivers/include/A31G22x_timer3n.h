/***************************************************************************//**
* @file     A31G22x_timer3n.h
* @brief    Contains all macro definitions and function prototypes support
*           for 32-bit timer3n driver on A31G22x
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

#ifndef _A31G22x_TIMER3N_H_
#define _A31G22x_TIMER3N_H_

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
#define TIMER3n_INTFLAG_STATUS_CLEAR_MASK		(TIMER3n_STATUS_HIZ_INT | TIMER3n_STATUS_CAPTURE_INT \
												| TIMER3n_STATUS_BOTTOM_INT | TIMER3n_STATUS_PMATCH_INT \
												| TIMER3n_STATUS_AMATCH_INT | TIMER3n_STATUS_BMATCH_INT \
												| TIMER3n_STATUS_CMATCH_INT)


#define TIMER3n_HIZCR_MASK						(TIMER3n_HIZCR_HIZEN_Msk | TIMER3n_HIZCR_HEDGE_Msk)
#define TIMER3n_HIZCR_SET_HIZ					(0x01UL << TIMER3n_HIZCR_HIZSW_Pos) // automatically clear bit
#define TIMER3n_HIZCR_RESET_HIZ					(0x01UL << TIMER3n_HIZCR_HIZCLR_Pos) // automatically clear bit

/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief  Timer3n mode selection enumerated definition
 */
typedef enum
{
	TIMER3n_MODE_INTERVAL = 0x00UL, /*!< Interval mode (All match interrupts can occur) */
	TIMER3n_MODE_CAPTURE = 0x01UL, /*!< Capture mode (The Period-match interrupt can occur) */
	TIMER3n_MODE_BACKTOBACK = 0x02UL /*!< Back-to-back mode (All interrupts can occur) */
} TIMER3n_MODE_Type;

/**
 * @brief  External clock edge selection enumerated definition
 */
typedef enum
{
	TIMER3n_EXT_CLOCK_EDGE_FALLING = 0x00UL, /*!< Falling edge */
	TIMER3n_EXT_CLOCK_EDGE_RISING = 0x01UL /*!< Rising edge */
} TIMER3n_EXT_CLOCK_EDGE_Type;

/**
 * @brief  Timer3n Output mode selection enumerated definition
 */
typedef enum
{
	TIMER3n_OUTPUT_MODE_6_CHANNEL = 0x00UL, /*!< 6-channel mode */
	TIMER3n_OUTPUT_MODE_FORCE_A_CHANNEL = 0x01UL /*!< Force A-channel mode */
} TIMER3n_OUTPUT_MODE_Type;

/**
 * @brief  Delay time position selection enumerated definition
 */
typedef enum
{
	TIMER3n_DELAY_POSITION_FRONT = 0x00UL, /*!< Insert at front of PWM30xA and at back of PWM30xB pin */
	TIMER3n_DELAY_POSITION_BACK = 0x001L /*!< Insert at back of PWM30xA and at front of PWM30xB pin */
} TIMER3n_DELAY_POSITION_Type;

/**
 * @brief  Capture polarity selection enumerated definition
 */
typedef enum
{
	TIMER3n_CAPTURE_POLARITY_FALLING = 0x00UL, /*!< Capture on falling edge */
	TIMER3n_CAPTURE_POLARITY_RISING = 0x01UL, /*!< Capture on rising edge */
	TIMER3n_CAPTURE_POLARITY_BOTH = 0x02UL /*!< Capture on both of falling and rising edge */
} TIMER3n_CAPTURE_POLARITY_Type;

/**
 * @brief  Data reload time selection enumerated definition
 */
typedef enum
{
	TIMER3n_DATA_RELOAD_TIME_WRITING = 0x00UL, /*!< Update data to buffer at the time of writing */
	TIMER3n_DATA_RELOAD_TIME_MATCH = 0x01UL, /*!< Update data to buffer at period match */
	TIMER3n_DATA_RELOAD_TIME_BOTTOM = 0x02UL /*!< Update data to buffer at bottom */
} TIMER3n_DATA_RELOAD_TIME_Type;

/**
 * @brief  Period match interrupt interval selection enumerated definition
 */
typedef enum
{
	TIMER3n_MATCH_INT_INTERVAL_1 = 0x00UL, /*!< Once every 1 period match */
	TIMER3n_MATCH_INT_INTERVAL_2 = 0x01UL, /*!< Once every 2 period match */
	TIMER3n_MATCH_INT_INTERVAL_3 = 0x02UL, /*!< Once every 3 period match */
	TIMER3n_MATCH_INT_INTERVAL_4 = 0x03UL, /*!< Once every 4 period match */
	TIMER3n_MATCH_INT_INTERVAL_5 = 0x04UL, /*!< Once every 5 period match */
	TIMER3n_MATCH_INT_INTERVAL_6 = 0x05UL, /*!< Once every 6 period match */
	TIMER3n_MATCH_INT_INTERVAL_7 = 0x06UL, /*!< Once every 7 period match */
	TIMER3n_MATCH_INT_INTERVAL_8 = 0x07UL /*!< Once every 8 period match */
} TIMER3n_MATCH_INT_INTERVAL_Type;

/**
 * @brief  Output Control selection enumerated definition
 */
typedef enum
{
	TIMER3n_OUTPUT_CTRL_PWM30xA = 0x01UL, /*!< PWM30xA output selection */
	TIMER3n_OUTPUT_CTRL_PWM30xB = 0x02UL, /*!< PWM30xB output selection */
	TIMER3n_OUTPUT_CTRL_ALL = (TIMER3n_OUTPUT_CTRL_PWM30xA | TIMER3n_OUTPUT_CTRL_PWM30xB) /*!< All selection */
} TIMER3n_OUTPUT_CTRL_Type;

/**
 * @brief  Output level selection enumerated definition
 */
typedef enum
{
	TIMER3n_LEVEL_LOW = 0x00UL, /*!< Low level */
	TIMER3n_LEVEL_HIGH = 0x01UL /*!< High level */
} TIMER3n_LEVEL_Type;

/**
 * @brief  Output level selection enumerated definition
 */
typedef enum
{
	TIMER3n_BLNK_EDGE_FALLING = 0x00UL, /*!< Falling edge of the BLNK pin */
	TIMER3n_BLNK_EDGE_RISING = 0x01UL /*!< Rising edge of the BLNK pin */
} TIMER3n_BLNK_EDGE_Type;

/**
 * @brief  Interrupt source enumerated definition
 */
typedef enum {
	TIMER3n_INTERRUPT_CMATCH_INT = (0x01UL << TIMER3n_INTFLAG_CMIFLAG_Pos), /*!< C-ch match interrupt */
	TIMER3n_INTERRUPT_BMATCH_INT = (0x01UL << TIMER3n_INTFLAG_BMIFLAG_Pos), /*!< B-ch match interrupt */
	TIMER3n_INTERRUPT_AMATCH_INT = (0x01UL << TIMER3n_INTFLAG_AMIFLAG_Pos), /*!< A-ch match interrupt */
	TIMER3n_INTERRUPT_PMATCH_INT = (0x01UL << TIMER3n_INTFLAG_PMIFLAG_Pos), /*!< Period match interrupt */
	TIMER3n_INTERRUPT_BOTTOM_INT = (0x01UL << TIMER3n_INTFLAG_BTIFLAG_Pos), /*!< Bottom interrupt */
	TIMER3n_INTERRUPT_CAPTURE_INT = (0x01UL << TIMER3n_INTFLAG_CIFLAG_Pos), /*!< Capture interrupt */
	TIMER3n_INTERRUPT_HIZ_INT = (0x01UL << TIMER3n_INTFLAG_HIZIFLAG_Pos) /*!< High impedance interrupt */
} TIMER3n_INTERRUPT_Type;

/**
 * @brief  Timer3n status enumerated definition
 */
typedef enum {
	TIMER3n_STATUS_CMATCH_INT = (0x01UL << TIMER3n_INTFLAG_CMIFLAG_Pos), /*!< C-ch match interrupt status */
	TIMER3n_STATUS_BMATCH_INT = (0x01UL << TIMER3n_INTFLAG_BMIFLAG_Pos), /*!< B-ch match interrupt status */
	TIMER3n_STATUS_AMATCH_INT = (0x01UL << TIMER3n_INTFLAG_AMIFLAG_Pos), /*!< A-ch match interrupt status */
	TIMER3n_STATUS_PMATCH_INT = (0x01UL << TIMER3n_INTFLAG_PMIFLAG_Pos), /*!< Period match interrupt status */
	TIMER3n_STATUS_BOTTOM_INT = (0x01UL << TIMER3n_INTFLAG_BTIFLAG_Pos), /*!< Bottom interrupt status */
	TIMER3n_STATUS_CAPTURE_INT = (0x01UL << TIMER3n_INTFLAG_CIFLAG_Pos), /*!< Capture interrupt status */
	TIMER3n_STATUS_HIZ_INT = (0x01UL << TIMER3n_INTFLAG_HIZIFLAG_Pos) /*!< High impedance interrupt status */
} TIMER3n_STATUS_Type;

/**
 * @brief  Timer3n configuration structure definition
 */
typedef struct {
	/* CR */
	FunctionalState ExtClock; /*!< External clock */
	TIMER3n_MODE_Type Mode; /*!< Timer3n mode selection */
	TIMER3n_EXT_CLOCK_EDGE_Type ExtClockEdge; /*!< External clock edge selection */
	TIMER3n_OUTPUT_MODE_Type OutputMode; /*!<  Timer3n output mode selection */
	FunctionalState DelayInsertion; /*!< Delay time insertion */
	TIMER3n_DELAY_POSITION_Type DelayPosition; /*!< Delay Time Insertion Position */
	TIMER3n_CAPTURE_POLARITY_Type CapturePolarity; /*!< Capture polarity selection */
	TIMER3n_DATA_RELOAD_TIME_Type ReloadTime; /*!< Data reload time selection */
	TIMER3n_MATCH_INT_INTERVAL_Type MatchIntInterval; /*!< Period match interrupt interval selection */

	/* PDR */
	uint32_t PData; /*!< Period data bit. The range is 0x0002 to 0xFFFF */

	/* ADR */
	uint32_t AData; /*!< A data bit. The range is 0x0000 to 0xFFFF */

	/* BDR */
	uint32_t BData; /*!< B data bit. The range is 0x0000 to 0xFFFF */

	/* CDR */
	uint32_t CData; /*!< C data bit. The range is 0x0000 to 0xFFFF */

	/* PREDR */
	uint32_t PrescalerData; /*!< Prescaler data bit. The range is 0x0000 to 0x0FFF. Clock = Fclk / (PRED + 1) */

	/* DLY */
	uint32_t DelayData; /*!< Delay data bit. The range is 0x0000 to 0x03FF. Delay time = (DLY + 1) / fT30 */
} TIMER3n_CFG_Type;

/**
 * @brief  Timer3n output configuration structure definition
 */
typedef struct {
	/* OUTCR */
	TIMER3n_LEVEL_Type StartLevel; /*!< Start level selection */
	FunctionalState Output_PWM30Ax; /*!< PWM30Ax Output */
	FunctionalState Output_PWM30Bx; /*!< PWM30Bx Output */
	FunctionalState Output_PWM30Cx; /*!< PWM30Ax Output */

	TIMER3n_LEVEL_Type DisableLevel_PWM30Ax; /*!< PWM30Ax output level when disable */
	TIMER3n_LEVEL_Type DisableLevel_PWM30Bx; /*!< PWM30Bx output level when disable */
	TIMER3n_LEVEL_Type DisableLevel_PWM30Cx; /*!< PWM30Cx output level when disable */
} TIMER3n_OUTPUT_CFG_Type;

/**
 * @brief  Timer3n high-impedance configuration structure definition
 */
typedef struct {
	/* HIZCR */
	FunctionalState HiZ; /*!< High-impedance */
	TIMER3n_BLNK_EDGE_Type BLNK_Edge; /*!< BLNK pin edge selection */
} TIMER3n_HiZ_CFG_Type;

/**
 * @brief  Timer3n ADC trigger configuration structure definition
 */
typedef struct {
	/* ADTCR */
	FunctionalState Trigger_Bottom; /*!< Bottom trigger */
	FunctionalState Trigger_PMatch; /*!< Period match trigger */
	FunctionalState Trigger_AMatch; /*!< A-ch match trigger */
	FunctionalState Trigger_BMatch; /*!< B-ch match trigger */
	FunctionalState Trigger_CMatch; /*!< C-ch match trigger */
} TIMER3n_ADC_TRIGGER_CFG_Type;


/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void TIMER3n_Init(TIMER3n_Type *pTIMER3x, TIMER3n_CFG_Type *pConfig);
void TIMER3n_DeInit(TIMER3n_Type *pTIMER3x);
void TIMER3n_ConfigureOutput(TIMER3n_Type *pTIMER3x, TIMER3n_OUTPUT_CTRL_Type Output, TIMER3n_OUTPUT_CFG_Type *pPWM30xA, TIMER3n_OUTPUT_CFG_Type *pPWM30xB);
void TIMER3n_ConfigureInterrupt(TIMER3n_Type *pTIMER3x, TIMER3n_INTERRUPT_Type InterruptSource, FunctionalState InterruptEnable);
void TIMER3n_ConfigureHiZ(TIMER3n_Type *pTIMER3x, TIMER3n_HiZ_CFG_Type *pConfig);
void TIMER3n_ConfigureADCTrigger(TIMER3n_Type *pTIMER3x, TIMER3n_ADC_TRIGGER_CFG_Type *pConfig);
uint32_t TIMER3n_GetPData(TIMER3n_Type *pTIMER3x);
void TIMER3n_SetPData(TIMER3n_Type *pTIMER3x, uint32_t Data);
uint32_t TIMER3n_GetAData(TIMER3n_Type *pTIMER3x);
void TIMER3n_SetAData(TIMER3n_Type *pTIMER3x, uint32_t Data);
uint32_t TIMER3n_GetBData(TIMER3n_Type *pTIMER3x);
void TIMER3n_SetBData(TIMER3n_Type *pTIMER3x, uint32_t Data);
uint32_t TIMER3n_GetCData(TIMER3n_Type *pTIMER3x);
void TIMER3n_SetCData(TIMER3n_Type *pTIMER3x, uint32_t Data);
uint32_t TIMER3n_GetCaptureData(TIMER3n_Type *pTIMER3x);
uint32_t TIMER3n_GetPrescalerData(TIMER3n_Type *pTIMER3x);
void TIMER3n_SetPrescalerData(TIMER3n_Type *pTIMER3x, uint32_t Data);
uint32_t TIMER3n_GetCounter(TIMER3n_Type *pTIMER3x);
void TIMER3n_ClearCounter(TIMER3n_Type *pTIMER3x);
uint32_t TIMER3n_GetDelayData(TIMER3n_Type *pTIMER3x);
void TIMER3n_SetDelayData(TIMER3n_Type *pTIMER3x, uint32_t Data);
TIMER3n_STATUS_Type TIMER3n_GetStatus(TIMER3n_Type *pTIMER3x);
void TIMER3n_ClearStatus(TIMER3n_Type *pTIMER3x, TIMER3n_STATUS_Type Status);
SetState TIMER3n_GetHiZStatus(TIMER3n_Type *pTIMER3x);
void TIMER3n_SetHiZOutput(TIMER3n_Type *pTIMER3x, SetState Output);
uint32_t TIMER3n_GetADCTriggerData(TIMER3n_Type *pTIMER3x);
void TIMER3n_SetADCTriggerData(TIMER3n_Type *pTIMER3x, uint32_t Data);
void TIMER3n_Start(TIMER3n_Type *pTIMER3x);
void TIMER3n_Stop(TIMER3n_Type *pTIMER3x);


#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_TIMER3N_H_ */
/* --------------------------------- End Of File ------------------------------ */
