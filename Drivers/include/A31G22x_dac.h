/***************************************************************************//**
* @file     A31G22x_dac.h
* @brief    Contains all macro definitions and function prototypes support
*           for DAC driver on A31G22x
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

#ifndef _A31G22x_DAC_H_
#define _A31G22x_DAC_H_

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
#define DAC_CR_DAC1_OUT_EN_Pos 	12
#define DAC_CR_DAC1_OUT_EN_Msk	(0x01UL << DAC_CR_DAC1_OUT_EN_Pos)

/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief  DAC status enumerated definition
 */
typedef enum {
	DAC_STATUS_DMA_DONE_INT = (0x01UL << DAC_ICR_DMAIF_Pos), /*!< DMA done occurred */
	DAC_STATUS_DMA_UNDERRUN_INT = (0x01UL << DAC_ICR_DUDRUNF_Pos) /*!< DMA under-run occurred */
} DAC_STATUS_Type;

/**
 * @brief  DAC DMA Interrupt Enable status enumerated definition
 */
typedef enum {
	DAC_STATUS_DMA_DONE_ENABLE = (0x01UL << DAC_ICR_DAMIE_Pos), /*!< DMA done Interrupt Enable */
	DAC_STATUS_DMA_UNDERRUN_ENABLE = (0x01UL << DAC_ICR_DUDRUNE_Pos) /*!< DMA under-run Interrupt Enable */
} DAC_INTR_ENABLE_Type;


/**
 * @brief  DAC reload signal enumerated definition
 */
typedef enum {
	DAC_RELOAD_SIGNAL_ALWAYS = 0x00UL, /*!< always */
	DAC_RELOAD_SIGNAL_TIMER10 = 0x02UL, /*!< Timer 10 match signal */
	DAC_RELOAD_SIGNAL_TIMER11 = 0x03UL /*!< Timer 11 match signal */
} DAC_RELOAD_SIGNAL_Type;

/**
 * @brief  DAC programmable gain enumerated definition
 */
typedef enum {
	DAC_GAIN_M_30DB = 0x00UL, /*!< - 30dB */
	DAC_GAIN_M_24DB = 0x01UL, /*!< - 24dB */
	DAC_GAIN_M_18DB = 0x02UL, /*!< - 18dB */
	DAC_GAIN_M_12DB = 0x03UL, /*!< - 12dB */
	DAC_GAIN_M_6DB = 0x04UL,  /*!< -  6dB */
	DAC_GAIN_0DB = 0x05UL,    /*!<    0dB */
	DAC_GAIN_P_6DB = 0x06UL,  /*!< +  6dB */
	DAC_GAIN_P_12DB = 0x07UL, /*!< + 12dB */
	DAC_GAIN_P_18DB = 0x08UL, /*!< + 18dB */
	DAC_GAIN_P_24DB = 0x09UL, /*!< + 24dB */
	DAC_GAIN_P_30DB = 0x0AUL /*!< + 30dB */
} DAC_GAIN_Type;

/**
 * @brief  DAC offset direction enumerated definition
 */
typedef enum {
	DAC_OFFSET_DIRECTION_MINUS = 0x00UL,
	DAC_OFFSET_DIRECTION_PLUS = 0x01UL
} DAC_OFFSET_DIRECTION_Type;

/**
 * @brief  DAC Offset value enumerated definition
 */
typedef enum {
	DAC_OFFSET_VALUE_0 = 0x00UL,
	DAC_OFFSET_VALUE_1 = 0x01UL,
	DAC_OFFSET_VALUE_2 = 0x02UL,
	DAC_OFFSET_VALUE_3 = 0x03UL,
	DAC_OFFSET_VALUE_4 = 0x04UL,
	DAC_OFFSET_VALUE_5 = 0x05UL,
	DAC_OFFSET_VALUE_6 = 0x06UL,
	DAC_OFFSET_VALUE_7 = 0x07UL,
	DAC_OFFSET_VALUE_8 = 0x08UL,
	DAC_OFFSET_VALUE_9 = 0x09UL,	
	DAC_OFFSET_VALUE_10 = 0x0AUL,
	DAC_OFFSET_VALUE_11 = 0x0BUL,
	DAC_OFFSET_VALUE_12 = 0x0CUL,
	DAC_OFFSET_VALUE_13 = 0x0DUL,
	DAC_OFFSET_VALUE_14 = 0x0EUL,
	DAC_OFFSET_VALUE_15 = 0x0FUL
} DAC_OFFSET_VALUE_Type;

/**
 * @brief  DAC Offset value enumerated definition
 */
typedef enum {
	DAC_DMA_UNDERRUN_INTR_DISABLE,
	DAC_DMA_UNDERRUN_INTR_ENABLE,
} DAC_DMA_UNDERRUN_Type;

/**
 * @brief  DAC Offset value enumerated definition
 */
typedef enum {
	DAC_DMA_RX_INTR_DISABLE,
	DAC_DMA_RX_INTR_ENABLE,
} DAC_DMA_RX_Type;


/**
 * @brief  DAC configuration structure definition
 */
typedef struct {
	/* CR */
	FunctionalState Stanby;	  /*!< DAC Mode (normal/Standby) */
	FunctionalState DAC_Out2; /*!< DAC Out2 (ADC ch20 In) */	
	FunctionalState DAC_Out1; /*!< DAC Out1 (Comparator In) */
	FunctionalState DAC_Out0; /*!< DAC Out0 (DAO/PA6) */
	FunctionalState OutBuffer; /*!< DAC Buffer. If this is disabled, buffer is bypassed */
	DAC_RELOAD_SIGNAL_Type ReloadSignal; /*!< Select DAC reload signal */
} DAC_CFG_Type;

/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void DAC_Init(DAC_Type *pDACx, DAC_CFG_Type *pConfig);
void DAC_DeInit(DAC_Type *pDACx);
void DAC_SetGain(DAC_Type *pDACx, DAC_GAIN_Type Gain);
void DAC_SetOffset(DAC_Type *pDACx, FunctionalState OffsetEnable, DAC_OFFSET_DIRECTION_Type Direction, DAC_OFFSET_VALUE_Type OffsetValue);
DAC_STATUS_Type DAC_GetStatus(DAC_Type *pDACx);
void DAC_ClearStatus(DAC_Type *pDACx, DAC_STATUS_Type Status);
uint16_t DAC_GetData(DAC_Type *pDACx);
void DAC_SetData(DAC_Type *pDACx, uint16_t Data);
uint16_t DAC_GetBuffer(DAC_Type *pDACx);
void DAC_ClearBuffer(DAC_Type *pDACx);
void DAC_Start(DAC_Type *pDACx);
void DAC_Stop(DAC_Type *pDACx);
void DAC_DmaInterrupt(DAC_Type *pDACx, DAC_DMA_UNDERRUN_Type DmaUnderrun, DAC_DMA_RX_Type DmaRx);
DAC_INTR_ENABLE_Type DAC_GetIntrEn(DAC_Type *pDACx);

#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_DAC_H_ */
/* --------------------------------- End Of File ------------------------------ */
