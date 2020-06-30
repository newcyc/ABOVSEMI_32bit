/***************************************************************************//**
* @file     A31G22x_adc.h
* @brief    Contains all macro definitions and function prototypes support
*           for ADC driver on A31G22x
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

#ifndef _A31G22x_ADC_H_
#define _A31G22x_ADC_H_

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
#define ADC_MAX_SEQUENCE_NUM				(8)

#define ADC_SR_STATUS_GET_MASK				(ADC_STATUS_COMPARE_INT | ADC_STATUS_DMA_OVER \
											| ADC_STATUS_DMA_DONE | ADC_STATUS_TRIGGER_INT \
											| ADC_STATUS_SEQUENCE_INT | ADC_STATUS_SINGLE_INT)
#define ADC_SR_STATUS_CLEAR_MASK			(ADC_STATUS_COMPARE_INT | ADC_STATUS_DMA_OVER \
											| ADC_STATUS_DMA_DONE | ADC_STATUS_TRIGGER_INT \
											| ADC_STATUS_SEQUENCE_INT | ADC_STATUS_SINGLE_INT)
/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief  ADC mode selection enumerated definition
 */
typedef enum {
	ADC_MODE_SINGLE, /*!< Single Mode */
	ADC_MODE_SEQUENTIAL, /*!< Sequential */
	ADC_MODE_BURST, /*!< Burst Mode */
	ADC_MODE_MULTIPLE /*!< Multiple */
} ADC_MODE_Type;

/**
 * @brief  Conversion data selection enumerated definition
 */
typedef enum {
	ADC_DATA_SEL_LAST, /*!< Last conversion data, (ADC_DDR register) */
	ADC_DATA_SEL_SEQ0, /*!< Sequence 0 conversion data, (ADC_DR0 register) */
	ADC_DATA_SEL_SEQ1, /*!< Sequence 1 conversion data, (ADC_DR1 register) */
	ADC_DATA_SEL_SEQ2, /*!< Sequence 2 conversion data, (ADC_DR2 register) */
	ADC_DATA_SEL_SEQ3, /*!< Sequence 3 conversion data, (ADC_DR3 register) */
	ADC_DATA_SEL_SEQ4, /*!< Sequence 4 conversion data, (ADC_DR4 register) */
	ADC_DATA_SEL_SEQ5, /*!< Sequence 5 conversion data, (ADC_DR5 register) */
	ADC_DATA_SEL_SEQ6, /*!< Sequence 6 conversion data, (ADC_DR6 register) */
	ADC_DATA_SEL_SEQ7 /*!< Sequence 7 conversion data, (ADC_DR7 register) */
} ADC_DATA_SEL_Type;

/**
 * @brief  Trigger selection enumerated definition
 */
typedef enum {
	ADC_TRIGGER_SEL_OLNY_SOFT, /*!< Disables event triggers and enables the soft trigger only */
	ADC_TRIGGER_SEL_TIMER1n, /*!< Enables the timer event trigger and soft trigger */
	ADC_TRIGGER_SEL_TIMER3n, /*!< Enables the MPWM0 event trigger and soft trigger */
} ADC_TRIGGER_SEL_Type;

/**
 * @brief  Trigger information enumerated definition
 */
typedef enum {
	ADC_TRIGGER_0 = 0x01UL, /*!< Trigger information of sequence 0 */
	ADC_TRIGGER_1 = 0x02UL, /*!< Trigger information of sequence 1 */
	ADC_TRIGGER_2 = 0x04UL, /*!< Trigger information of sequence 2 */
	ADC_TRIGGER_3 = 0x08UL, /*!< Trigger information of sequence 3 */
	ADC_TRIGGER_4 = 0x10UL, /*!< Trigger information of sequence 4 */
	ADC_TRIGGER_5 = 0x20UL, /*!< Trigger information of sequence 5 */
	ADC_TRIGGER_6 = 0x40UL, /*!< Trigger information of sequence 6 */
	ADC_TRIGGER_7 = 0x80UL /*!< Trigger information of sequence 7 */
} ADC_TRIGGER_INFO_Type;

/**
 * @brief  Trigger selection enumerated definition
 */
typedef enum {
	ADC_TRIGGER_SOURCE_0 = 0, /*!< Trigger Source 0 */
	ADC_TRIGGER_SOURCE_1 = 1, /*!< Trigger Source 1 */
	ADC_TRIGGER_SOURCE_2 = 2, /*!< Trigger Source 2 */
	ADC_TRIGGER_SOURCE_3 = 3, /*!< Trigger Source 3 */
	ADC_TRIGGER_SOURCE_4 = 4, /*!< Trigger Source 4 */
	ADC_TRIGGER_SOURCE_5 = 5, /*!< Trigger Source 5 */
	ADC_TRIGGER_SOURCE_6 = 6, /*!< Trigger Source 6 */
	ADC_TRIGGER_SOURCE_7 = 7, /*!< Trigger Source 7 */
	ADC_TRIGGER_SOURCE_8 = 8, /*!< Trigger Source 8 */
	ADC_TRIGGER_SOURCE_9 = 9, /*!< Trigger Source 9 */
	ADC_TRIGGER_SOURCE_10 = 10, /*!< Trigger Source 10 */
	ADC_TRIGGER_SOURCE_11 = 11, /*!< Trigger Source 11 */
	ADC_TRIGGER_SOURCE_15 = 15 /*!< Trigger Source 15 */
} ADC_TRIGGER_SOURCE_Type;

/**
 * @brief  Interrupt source enumerated definition
 */
typedef enum {
	ADC_INTERRUPT_SINGLE = ADC_IER_EOCIE_Msk, /*!< ADC single conversion interrupt (EOCIE) */
	ADC_INTERRUPT_SEQUENCE = ADC_IER_EOSIE_Msk, /*!< ADC sequence conversion interrupt (EOSIE) */
	ADC_INTERRUPT_TRIGGER = ADC_IER_TRGIE_Msk, /*!< the ADC trigger conversion interrupt (TRGIE) */
	ADC_INTERRUPT_DMA = ADC_IER_DMAIE_Msk /*!< DMA done interrupt.(DMAIE) */
} ADC_INTERRUPT_Type;

/**
 * @brief  Sequence number enumerated definition
 */
typedef enum {
	ADC_SEQ_NUM_1, /*!< Sequence Single sample per sequence */
	ADC_SEQ_NUM_2, /*!< Sequence Two samples per sequence */
	ADC_SEQ_NUM_3, /*!< Sequence Three samples per sequence */
	ADC_SEQ_NUM_4, /*!< Sequence Four samples per sequence */
	ADC_SEQ_NUM_5, /*!< Sequence Five samples per sequence */
	ADC_SEQ_NUM_6, /*!< Sequence Six samples per sequence */
	ADC_SEQ_NUM_7, /*!< Sequence Seven samples per sequence */
	ADC_SEQ_NUM_8  /*!< Sequence Eight samples per sequence */
} ADC_SEQUENCE_NUM_Type;


/**
 * @brief  Sequence index enumerated definition
 */
typedef enum {
	ADC_SEQUENCE_0, /*!< Sequence 0 */
	ADC_SEQUENCE_1, /*!< Sequence 1 */
	ADC_SEQUENCE_2, /*!< Sequence 2 */
	ADC_SEQUENCE_3, /*!< Sequence 3 */
	ADC_SEQUENCE_4, /*!< Sequence 4 */
	ADC_SEQUENCE_5, /*!< Sequence 5 */
	ADC_SEQUENCE_6, /*!< Sequence 6 */
	ADC_SEQUENCE_7 /*!< Sequence 7 */
} ADC_SEQUENCE_Type;

/**
 * @brief  ADC channel index enumerated definition
 */
typedef enum {
	ADC_CHANNEL_0, /*!< ADC Channel 0 */
	ADC_CHANNEL_1, /*!< ADC Channel 1 */
	ADC_CHANNEL_2, /*!< ADC Channel 2 */
	ADC_CHANNEL_3, /*!< ADC Channel 3 */
	ADC_CHANNEL_4, /*!< ADC Channel 4 */
	ADC_CHANNEL_5, /*!< ADC Channel 5 */
	ADC_CHANNEL_6, /*!< ADC Channel 6 */
	ADC_CHANNEL_7, /*!< ADC Channel 7 */
	ADC_CHANNEL_8, /*!< ADC Channel 8 */
	ADC_CHANNEL_9, /*!< ADC Channel 9 */
	ADC_CHANNEL_10, /*!< ADC Channel 10 */
	ADC_CHANNEL_11, /*!< ADC Channel 11 */
	ADC_CHANNEL_12, /*!< ADC Channel 12 */
	ADC_CHANNEL_13, /*!< ADC Channel 13 */
	ADC_CHANNEL_14, /*!< ADC Channel 14 */
	ADC_CHANNEL_15, /*!< ADC Channel 15 */
	ADC_CHANNEL_16, /*!< ADC Channel 16 */
	ADC_CHANNEL_17, /*!< ADC Channel 17 */
	ADC_CHANNEL_18, /*!< ADC Channel 18 */
	ADC_CHANNEL_19, /*!< ADC Channel 19 */
	ADC_CHANNEL_20, /*!< ADC Channel 20 */	
} ADC_CHANNEL_Type;

/**
 * @brief  ADC Status enumerated definition
 */
typedef enum {
	ADC_STATUS_SINGLE_INT = (0x01UL << ADC_SR_EOCIF_Pos), /*!< Conversion end interrupt Status */
	ADC_STATUS_SEQUENCE_INT = (0x01UL << ADC_SR_EOSIF_Pos), /*!< Sequence end interrupt Status */
	ADC_STATUS_TRIGGER_INT = (0x01UL << ADC_SR_TRGIF_Pos), /*!< ADC trigger interrupt Status */
	ADC_STATUS_DMA_DONE = (0x01UL << ADC_SR_DMAIF_Pos), /*!< DMA done received Status (DMA transfer is completed) */
	ADC_STATUS_DMA_OVER = (0x01UL << ADC_SR_DOVRUN_Pos), /*!< DMA overrun Status (not an interrupt) */
	ADC_STATUS_COMPARE_INT = (0x01UL << ADC_SR_CMPIFLG_Pos) /*!< Compare interrupt Status */
} ADC_STATUS_Type;

/**
 * @brief  ADC IER enumerated definition
 */
typedef enum {
	ADC_INTR_DMAIE = (0x1UL << ADC_IER_DMAIE_Pos),
	ADC_INTR_TRGIE = (0x1UL << ADC_IER_TRGIE_Pos), 
	ADC_INTR_EOSIE = (0x1UL << ADC_IER_EOSIE_Pos), 
	ADC_INTR_EOCIE = (0x1UL << ADC_IER_EOCIE_Pos),	
} ADC_INTR_Type;


/**
 * @brief  ADC Buffering Off Time enumerated definition
 */
typedef enum {
	ADC_BUFTIME_1MCLK = (0x0UL << ADC_BCR_TBUFSEL_Pos),
	ADC_BUFTIME_2MCLK = (0x1UL << ADC_BCR_TBUFSEL_Pos),
	ADC_BUFTIME_3MCLK = (0x2UL << ADC_BCR_TBUFSEL_Pos),
	ADC_BUFTIME_4MCLK = (0x3UL << ADC_BCR_TBUFSEL_Pos),
	ADC_BUFTIME_5MCLK = (0x4UL << ADC_BCR_TBUFSEL_Pos),
	ADC_BUFTIME_6MCLK = (0x5UL << ADC_BCR_TBUFSEL_Pos),
	ADC_BUFTIME_7MCLK = (0x6UL << ADC_BCR_TBUFSEL_Pos),
	ADC_BUFTIME_8MCLK = (0x7UL << ADC_BCR_TBUFSEL_Pos),
} ADC_BUFTIME_Type;

/**
 * @brief  ADC Input Buffer Operating Mode enumerated definition
 */
typedef enum {
	ADC_BUFMODE_POWERDOWN = 	(0x0UL << ADC_BCR_BUFEN_Pos),
	ADC_BUFMODE_OPERATE = (0x1UL << ADC_BCR_BUFEN_Pos),
} ADC_BUFMODE_Type;


/**
 * @brief  ADC Input type enumerated definition
 */
typedef enum {
	ADC_BUFINPUT_BUFFER = (0x0UL << ADC_BCR_BYPSEL_Pos),
	ADC_BUFINPUT_BYPASS = (0x1UL << ADC_BCR_BYPSEL_Pos),
} ADC_BUFINPUT_Type;


/**
 * @brief  ADC configuration structure definition
 */
typedef struct {
	/* MR */
	FunctionalState TriggerInfo; /*!< Trigger information */
	FunctionalState ChannelInfo; /*!< Channel information */
	FunctionalState Dma; /*!< DMA Enable/Disable */	
	FunctionalState Restart; /*!< Restart ADC */

	ADC_MODE_Type Mode; /*!< ADC Mode selection */
	ADC_TRIGGER_SEL_Type TriggerSelection; /*!< Trigger selection */

	uint32_t SamplingTime; /*!< samling time selection */
	uint32_t SequenceCounter; /*!< Number of conversions in a sequence, 0 ~ 7 */

	/* CR1 */
	FunctionalState ExternalClock; /*!< External clock */

	uint32_t InternalClockDivider; /*!< ADC internal clock divider (This value is valid when EnableExternalClock is FALSE),  
							- CLKDIV = 0  ADC clock = ADC input clock (bypass)  
							- CLKDIV = 1  ADC clock = clock stop  
							- CLKDIV ≥ 2  ADC clock = ADC input clock/CLKDIV  
							you must make sure that the ADC clock’s frequency does not exceed 25 MHz */
} ADC_CFG_Type;

/**
 * @brief  ADC comparator configuration structure definition
 */
typedef struct {
	/* CMPR */
	FunctionalState Interrupt; /*!< Compare interrupt */
	FunctionalState Operation; /*!< Compare operation */
	FunctionalState TriggerSmall; /*!< When ADC value is smaller than the CVAL value, occurred interrupt */

	ADC_CHANNEL_Type Channel; /*!< ADC channel to compare*/

	uint32_t CompareValue; /*!< Compare value, valid from 0x000 to 0xFFF */
} ADC_CMP_CFG_Type;


/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void ADC_Init(ADC_Type *pADCx, ADC_CFG_Type *pConfig);
void ADC_DeInit(ADC_Type *pADCx);
void ADC_ConfigureInterrupt(ADC_Type *pADCx, ADC_INTERRUPT_Type InterruptSource, FunctionalState InterruptEnable);
void ADC_ConfigureOneTrigger(ADC_Type *pADCx, ADC_SEQUENCE_Type SeqIndex, ADC_TRIGGER_SOURCE_Type TriSource);
void ADC_ConfigureAllTrigger(ADC_Type *pADCx, uint32_t MaxSequenceNum, ADC_TRIGGER_SOURCE_Type *pTriSource);
void ADC_ConfigureOneSequence(ADC_Type *pADCx, ADC_SEQUENCE_Type SeqIndex, ADC_CHANNEL_Type ADCIndex);
void ADC_ConfigureAllSequence(ADC_Type *pADCx, uint32_t MaxSequenceNum, ADC_CHANNEL_Type *pADCChannel);
void ADC_ConfigureCompare(ADC_Type *pADCx, ADC_CMP_CFG_Type *pCMPConfig);
void ADC_CompareStart(ADC_Type *pADCx);
void ADC_CompareStop(ADC_Type *pADCx);
uint16_t ADC_GetReferenceData(ADC_Type *pADCx);
uint32_t ADC_GetConversionData(ADC_Type *pADCx, ADC_DATA_SEL_Type Selection);
ADC_TRIGGER_INFO_Type ADC_GetTriggerInfo(ADC_Type *pADCx, ADC_DATA_SEL_Type Selection);
uint32_t ADC_GetChannelInfo(ADC_Type *pADCx, ADC_DATA_SEL_Type Selection);
ADC_STATUS_Type ADC_GetStatus(ADC_Type *pADCx);
ADC_INTR_Type ADC_GetInterruptEnableStatus(ADC_Type *pADCx);
void ADC_ClearStatus(ADC_Type *pADCx, ADC_STATUS_Type Status);
void ADC_EnterDeepSleep(ADC_Type *pADCx, FunctionalState DeepSleep);
void ADC_Start(ADC_Type *pADCx);
void ADC_Stop(ADC_Type *pADCx);
void ADC_DmaEnable(ADC_Type *pADCx);
void ADC_DmaDisable(ADC_Type *pADCx);
void ADC_SetBuffer(ADC_Type *pADCx, ADC_BUFINPUT_Type Type, ADC_BUFMODE_Type Mode, ADC_BUFTIME_Type TBufSel);
#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_ADC_H_ */
/* --------------------------------- End Of File ------------------------------ */
