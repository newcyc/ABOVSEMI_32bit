/***************************************************************************//**
* @file     A31G22x_cmp.h
* @brief    Contains all macro definitions and function prototypes support
*           for comparator driver on A31G22x
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

#ifndef _A31G22x_CMP_H_
#define _A31G22x_CMP_H_

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
#define CMP_OUTPUT_MONITORING (1 << CMP_IEN_COUTMON_Pos)
#define CMP0_OUTPUT_LEVEL (1 << CMP_IST_C0COUT_Pos)
#define CMP1_OUTPUT_LEVEL (1 << CMP_IST_C1COUT_Pos)
#define CMP0_OUTPUT_RISING_EDGE (1 << CMP_IST_C0EPR_Pos)
#define CMP0_OUTPUT_FALLING_EDGE (1 << CMP_IST_C0EPF_Pos)
#define CMP1_OUTPUT_RISING_EDGE (1 << CMP_IST_C1EPR_Pos)
#define CMP1_OUTPUT_FALLING_EDGE (1 << CMP_IST_C1EPF_Pos)

/*******************************************************************************
* Public Typedef
*******************************************************************************/
/** 
 * @brief  Comparator block number
 */
typedef enum {
	CMP_0, /*!< Comarator 0 block */
	CMP_1 /*!< Comarator 1 block */
} CMP_NUMBER_Type;


/** 
 * @brief  Comparator hysterisis enumerated definition
 */
typedef enum {
	CMP_HYSSEL_5_MILLIVOLTS = 0x00UL, /*!< 5mV */
	CMP_HYSSEL_20_MILLIVOLTS = 0x01UL /*!< 20mV */
} CMP_HYSSEL_Type;

/** 
 * @brief  Comparator source (input +) enumerated definition
 */
typedef enum {
	CMP_SOURCE_0 = 0x00UL, /*!< CMP_0 -> CP0(PA2), CMP_1 -> CP1A(PA5) */
	CMP_SOURCE_1 = 0x01UL, /*!< CMP_0 -> GND,      CMP_1 -> CP1B(PA4) */
	CMP_SOURCE_2 = 0x02UL /*!< CMP_0 -> GND,      CMP_1 -> CP1C(PA3) */
} CMP_SOURCE_Type;

/** 
 * @brief  Comparator reference (input -) enumerated definition
 */
typedef enum {
	CMP_REFERENCE_0 = 0x00UL, /*!< CMP_0 -> CREF0(PA7), CMP_1 -> CREF1(PA6) */
	CMP_REFERENCE_1 = 0x01UL, /*!< BGR */
	CMP_REFERENCE_2 = 0x02UL /*!< DAC Output */
} CMP_REFERENCE_Type;

/** 
 * @brief  Comparator trigger polarity enumerated definition
 */
typedef enum {
	CMP_TRIGGER_POL_NORMAL = 0x00UL, /*!< Output normal (comparator out high activates trigger)*/
	CMP_TRIGGER_POL_INVERTED = 0x01UL /*!< Output inverted (XOR) */
} CMP_TRIGGER_POL_Type;

/** 
 * @brief  Comparator interrupt polarity (level mode) enumerated definition
 */
typedef enum {
	CMP_INTERRUPT_POL_HIGH = 0x00UL, /*!< Interrupt at comparator out high */
	CMP_INTERRUPT_POL_LOW = 0x01UL /*!< Interrupt at comparator out low */
} CMP_INTERRUPT_POL_Type;

/** 
 * @brief  Comparator interrupt mode enumerated definition
 */
typedef enum {
	CMP_INTERRUPT_MODE_LEVEL = 0x00UL, /*!< Level interrupt */
	CMP_INTERRUPT_MODE_RISING_EDGE = 0x01UL, /*!< Rising edge interrupt */
	CMP_INTERRUPT_MODE_FALLING_EDGE = 0x02UL, /*!< Falling edge interrupt */
	CMP_INTERRUPT_MODE_BOTH_EDGE = 0x03UL /*!< Both edge interrupt */
} CMP_INTERRUPT_MODE_Type;

/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void CMP_Init(void);
void CMP_DeInit(void);
FlagStatus CMP_GetInterruptStatus(CMP_NUMBER_Type Number);
void CMP_ClearInterruptStatus(CMP_NUMBER_Type Number);
void CMP_ConfigureHysterisis(CMP_NUMBER_Type Number, CMP_HYSSEL_Type HysterisisLevel, FunctionalState HysterisisEnable);
void CMP_ConfigureInput(CMP_NUMBER_Type Number, CMP_SOURCE_Type Source, CMP_REFERENCE_Type Reference);
void CMP_ConfigureDBNCCounter(uint32_t Counter);
void CMP_ConfigureDBNCNumber(CMP_NUMBER_Type Number, uint32_t ShiftNumber);
void CMP_ConfigureInterrupt(CMP_NUMBER_Type Number, CMP_TRIGGER_POL_Type TriggerPolarity, CMP_INTERRUPT_POL_Type Polarity, CMP_INTERRUPT_MODE_Type Mode);
void CMP_EnableInterrupt(CMP_NUMBER_Type Number, FunctionalState Interrupt);
void CMP_Start(CMP_NUMBER_Type Number);
void CMP_Stop(CMP_NUMBER_Type Number);
void CMP_SetOutputMonitoring(FunctionalState OutMonMode);
void CMP_ClearMonitoringEdgeStatus(void);
uint32_t CMP_GetMonitoringEdgeStatus(void);
FlagStatus CMP_GetMonitoringStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_CMP_H_ */

/* --------------------------------- End Of File ------------------------------ */
