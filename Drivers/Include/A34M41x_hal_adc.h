/**********************************************************************
 * @file		A34M41x_adc.h
 * @brief	Contains all macro definitions and function prototypes
 * 			support for ADC firmware library on A34M41x
 * @version	1.0
 * @date		
 * @author ABOV Application2 team
 *
 * Copyright(C)  2017, ABOV Semiconductor
 * All rights reserved.
 *
 **********************************************************************/

#ifndef _A34M41x_ADC_H_
#define _A34M41x_ADC_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Private macros ------------------------------------------------------------- */
/* -------------------------- BIT DEFINITIONS ----------------------------------- */
/**********************************************************************
 * Macro defines for ADC  control register
 **********************************************************************/
/**  ADC convert in power down mode */
#define ADC_CCR_PDN			((1UL<<7))

/**********************************************************************
 * Macro defines for ADC control2 register
 **********************************************************************/
/**  Start mask bits */
#define ADC_CR_START_MASK	((1UL<<0))
/**  Stop mask bits */
#define ADC_CR_STOP_MASK	((1UL<<7))

/**********************************************************************
 * Macro defines for ADC Status register
 **********************************************************************/
#define ADC_STAT_COMPIFLG						((1UL<<8))	// Compare Interrupt Flag bit
#define ADC_STAT_END							((1UL<<7))
#define ADC_STAT_BUSY						((1UL<<6))
#define ADC_STAT_DMAOVERRUN			((1UL<<5))			// DMA Overrun Flag (Not an interrupt)

#define ADC_STAT_DMA							((1UL<<4))		// DMA Done Received Flag (DMA transfer is completed)
#define ADC_STAT_TRIGGER					((1UL<<3))		// ADC Trigger Interrupt Flag
#define ADC_STAT_SEQ							((1UL<<2))		// Sequence End Interrupt Flag
#define ADC_STAT_SINGLE						((1UL<<0))		// Sequence Conversion End Interrupt Flag

/**********************************************************************
 * Macro defines for ADC Interrupt register
 **********************************************************************/
/** These bits allow control over which A/D channels generate
 * interrupts for conversion completion */
#define ADC_INTEN_DMA					((1UL<<4))
#define ADC_INTEN_TRIGGER			((1UL<<3))
#define ADC_INTEN_SEQ					((1UL<<2))
#define ADC_INTEN_SINGLE				((1UL<<0))

/**********************************************************************
 * Macro defines for ADC Data register
 **********************************************************************/
/** When DONE is 1, this field contains result value of ADC conversion */
//#define ADC_DR_RESULT(n) 		((n)&0xFFF0)
#define ADC_DR_RESULT(n) 		((n)&0x0FFF) // Modified (2019.11.20)
/** When DONE is 1, this field contains result channel value of ADC conversion */
#define ADC_GET_CH_RESULT(n)	((n)&0xF)

/* Public Types --------------------------------------------------------------- */
#define ADC_SINGLE_MODE					0  /*!< adc single conversion mode */
#define ADC_BURST_MODE					1  /*!< adc burst conversion mode */
#define ADC_MULTI_MODE					2  /*!< adc multiple conversion mode */

#define ADC_INTERNAL_CLK                  0  /*!< adc single conversion mode */
#define ADC_EXTERNAL_CLK                 1  /*!< adc continuous conversion mode */

#define ADC_TRIGGER_DISABLE           0  /*!< Event Trigger Disabled/Soft-Trigger Only */
#define ADC_TRIGGER_TIMER               1  /*!< Timer Event Trigger */
#define ADC_TRIGGER_MPWM0            2  /*!< MPWM0 Event Trigger */
#define ADC_TRIGGER_MPWM1            3  /*!< MPWM1 Event Trigger */

typedef struct {
	/* MR */
	uint32_t Mode; /**< Mode, should be:
	 - ADC_SINGLE_MODE
	 - ADC_CONTINUOUS_MODE
	 - ADC_BURST_MODE */
	uint32_t DmaOpt; /**< DmaOpt = ENABLE or DISABLE */
	uint32_t SamplingTime;
	uint32_t SeqCnt; /**< SeqCnt = count value 2~8	*/
	uint32_t RestartEn; /**< RestartEn = ENABLE or DISABLE */
	uint32_t TrgSel;

	/* CR1 */
	uint32_t UseClk;
	uint32_t InClkDiv;
} ADC_CFG_Type;

/* Public Functions ----------------------------------------------------------- */
/* Init/DeInit ADC peripheral ----------------*/
void HAL_ISTOD_EX(void);
HAL_Status_Type HAL_ADC_Init(ADC_Type *ADCx, ADC_CFG_Type *ADC_ConfigStruct);
HAL_Status_Type HAL_ADC_DeInit(ADC_Type *ADCx);

/* Enable/Disable ADC functions --------------*/
HAL_Status_Type HAL_ADC_EnterPowerdownMode(ADC_Type *ADCx, FunctionalState NewState);

HAL_Status_Type HAL_ADC_ChannelSel(ADC_Type *ADCx, uint32_t Channel);
HAL_Status_Type HAL_ADC_ChannelSel_1(ADC_Type *ADCx, uint32_t Channel);
HAL_Status_Type HAL_ADC_ChannelSel_2(ADC_Type *ADCx, uint32_t Channel);
HAL_Status_Type HAL_ADC_TriggerlSel(ADC_Type *ADCx, uint32_t Trigger);

HAL_Status_Type HAL_ADC_Start(ADC_Type *ADCx);
HAL_Status_Type HAL_ADC_Stop(ADC_Type *ADCx);

/* Configure ADC functions -------------------*/
HAL_Status_Type HAL_ADC_EdgeStartConfig(ADC_Type *ADCx, uint8_t EdgeOption);
HAL_Status_Type HAL_ADC_ConfigInterrupt(ADC_Type *ADCx, uint8_t IntType, FunctionalState NewState);

/* Get ADC information functions -------------------*/
uint16_t HAL_ADC_GetData(ADC_Type *ADCx);

/* Get/Clear ADC status  -------------------*/
uint32_t HAL_ADC_GetStatus(ADC_Type *ADCx);
HAL_Status_Type HAL_ADC_ClearStatus(ADC_Type *ADCx, uint32_t status);

HAL_Status_Type CSP_ADC_SetMR(ADC_Type *ADCx, uint32_t mode);
#ifdef __cplusplus
}
#endif

#endif
/* --------------------------------- End Of File ------------------------------ */
