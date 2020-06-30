/***************************************************************************//**
* @file     A31G22x_dac.c
* @brief    Contains all functions support for DAC dirver on A31G22x
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

/*******************************************************************************
* Included File
*******************************************************************************/
#include "A31G22x_scu.h"
#include "A31G22x_dac.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER2_DAC_ENABLE_PERI			(0x01UL << SCU_PER2_DAC_Pos)
#define SCU_PCER2_DAC_ENABLE_CLOCK			(0x01UL << SCU_PCER2_DAC_Pos)

#define DAC_CR_DACBC_CLEAR_BUFFER			(0x01UL << DAC_CR_DACBC_Pos)
#define DAC_CR_DACEN_ENABLE_DAC				(0x01UL << DAC_CR_DACEN_Pos)

#define DAC_ICR_STATUS_GET_MASK				(DAC_STATUS_DMA_UNDERRUN_INT | DAC_STATUS_DMA_DONE_INT)
#define DAC_ICR_STATUS_CLEAR_MASK			(DAC_STATUS_DMA_UNDERRUN_INT | DAC_STATUS_DMA_DONE_INT)

#define DAC_ICR_INTR_ENABLE_MASK			(DAC_STATUS_DMA_DONE_ENABLE | DAC_STATUS_DMA_UNDERRUN_ENABLE)

/*******************************************************************************
* Private Typedef
*******************************************************************************/


/*******************************************************************************
* Private Variable
*******************************************************************************/


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/


/*******************************************************************************
* Public Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Initialize DAC(Digital/Analog Convertor) peripheral
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @param      pConfig : Pointer contains configuration of DAC
* @return     None
*******************************************************************************/
void DAC_Init(DAC_Type *pDACx, DAC_CFG_Type *pConfig)
{
	volatile uint32_t Reg32;

	// Disable ADC peripheral & clock
	SCU->PER2 &= ~SCU_PER2_DAC_Msk;
	SCU->PCER2 &= ~SCU_PCER2_DAC_Msk;

	// Enable ADC peripheral & clock
	SCU->PER2 |= SCU_PER2_DAC_ENABLE_PERI;
	SCU->PCER2 |= SCU_PCER2_DAC_ENABLE_CLOCK;

	// DAC Control Setting
	Reg32 = 0
	| ((pConfig->DAC_Out2 << DAC_CR_DAC2_OUT_EN_Pos) & DAC_CR_DAC2_OUT_EN_Msk)
	| ((pConfig->DAC_Out1 << DAC_CR_DAC1_OUT_EN_Pos) & DAC_CR_DAC1_OUT_EN_Msk)
	| ((pConfig->DAC_Out0 << DAC_CR_DAC0_OUT_EN_Pos) & DAC_CR_DAC0_OUT_EN_Msk)
	| ((pConfig->OutBuffer << DAC_CR_DACOUTBUFEN_Pos) & DAC_CR_DACOUTBUFEN_Msk)
	| (0x01UL << DAC_CR_DACBC_Pos)
	| ((pConfig->ReloadSignal << DAC_CR_DACRLDS_Pos) & DAC_CR_DACRLDS_Msk)
	;
	pDACx->CR = Reg32;
}

/***************************************************************************//**
* @brief      De-Initialize DAC peripheral
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @return     None
*******************************************************************************/
void DAC_DeInit(DAC_Type *pDACx)
{
	DAC_Stop(pDACx);

	SCU->PER2 &= ~SCU_PER2_DAC_Msk;
	SCU->PCER2 &= ~SCU_PCER2_DAC_Msk;
}

/***************************************************************************//**
* @brief      Set offset of DAC output
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @param      Gain : programmable gain, should be (-30dB ~ +30dB)
*              - DAC_GAIN_M_30DB : - 30dB
*              - DAC_GAIN_0DB    :    0dB
*              - DAC_GAIN_P_30DB : + 30dB
* @return     None
*******************************************************************************/
void DAC_SetGain(DAC_Type *pDACx, DAC_GAIN_Type Gain)
{
	pDACx->PGSR = Gain;
}

/***************************************************************************//**
* @brief      Set offset of DAC output
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @param      OffsetEnable : Enable offset, should be
*              - ENABLE : Enable offset
*              - DISABLE : Disable offset
* @param      Direction : Offset direction, should be
*              - DAC_OFFSET_DIRECTION_MINUS : DAC buffer data are subtracted by (Offset value + 1)
*              - DAC_OFFSET_DIRECTION_PLUS : DAC buffer data are added by (Offset value + 1)
* @param      OffsetValue : Offset value (4-bit)
* @return     None
*******************************************************************************/
void DAC_SetOffset(DAC_Type *pDACx, FunctionalState OffsetEnable, DAC_OFFSET_DIRECTION_Type Direction, DAC_OFFSET_VALUE_Type OffsetValue)
{
	volatile uint32_t Reg32;

	Reg32 = 0
	| (OffsetEnable << DAC_OFSCR_OFSEN_Pos)
	| (Direction << DAC_OFSCR_OFSDIR_Pos)
	| ((OffsetValue & 0x0FUL) << DAC_OFSCR_OFS_Pos)
	;

	pDACx->OFSCR = Reg32;
}

/***************************************************************************//**
* @brief      Get DAC status
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @return     DAC_STATUS_Type
*              - DAC_STATUS_DMA_DONE_INT : DMA done occurred
*              - DAC_STATUS_DMA_UNDERRUN_INT : DMA under-run occurred
*******************************************************************************/
DAC_STATUS_Type DAC_GetStatus(DAC_Type *pDACx)
{
	return (DAC_STATUS_Type)(pDACx->ICR & DAC_ICR_STATUS_GET_MASK);
}

/***************************************************************************//**
* @brief      Clear DAC status
* @param      Status : Status selected, should be
*              - DAC_STATUS_DMA_DONE_INT : DMA done occurred
*              - DAC_STATUS_DMA_UNDERRUN_INT : DMA under-run occurred
* @return     None
*******************************************************************************/
void DAC_ClearStatus(DAC_Type *pDACx, DAC_STATUS_Type Status)
{
	static DAC_INTR_ENABLE_Type IntrEn;

	IntrEn = DAC_GetIntrEn(DAC);		// Get DAC Interrupt Enable

	// DAC DMA Rx Done Interrupt Flag
	if ((Status&DAC_STATUS_DMA_DONE_INT) == DAC_STATUS_DMA_DONE_INT) {
		DAC->ICR = (Status & DAC_STATUS_DMA_DONE_INT) | IntrEn;
	}
	// DAC DMA Underrun Interrupt Flag
	if ((Status&DAC_STATUS_DMA_UNDERRUN_INT) == DAC_STATUS_DMA_UNDERRUN_INT) {
		DAC->ICR = (Status & DAC_STATUS_DMA_UNDERRUN_INT) | IntrEn;
	}
}

/***************************************************************************//**
* @brief      Get DAC data
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @return     DAC Data (16-bit)
*******************************************************************************/
uint16_t DAC_GetData(DAC_Type *pDACx)
{
	return (uint16_t)(pDACx->DR >> 4);
}

/***************************************************************************//**
* @brief      Set DAC data
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @param      Data : DAC Data (16-bit)
* @return     None
*******************************************************************************/
void DAC_SetData(DAC_Type *pDACx, uint16_t Data)
{
	pDACx->DR = Data;
}

/***************************************************************************//**
* @brief      Get DAC buffer
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @return     DAC buffer (16-bit)
*******************************************************************************/
uint16_t DAC_GetBuffer(DAC_Type *pDACx)
{
	return (uint16_t)pDACx->BR;
}

/***************************************************************************//**
* @brief      Clear DAC buffer
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @return     None
*******************************************************************************/
void DAC_ClearBuffer(DAC_Type *pDACx)
{
	pDACx->CR |= DAC_CR_DACBC_CLEAR_BUFFER;
}

/***************************************************************************//**
* @brief      Start D/A conversion
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @return     None
*******************************************************************************/
void DAC_Start(DAC_Type *pDACx)
{
	volatile uint32_t Reg32;

	Reg32 = pDACx->CR;
	Reg32 &= ~DAC_CR_DACEN_Msk;
	Reg32 |= DAC_CR_DACEN_ENABLE_DAC;
	pDACx->CR = Reg32;
}

/***************************************************************************//**
* @brief      Start D/A conversion
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @return     None
*******************************************************************************/
void DAC_Stop(DAC_Type *pDACx)
{
	pDACx->CR &= ~DAC_CR_DACEN_Msk;
}

/***************************************************************************//**
* @brief      DAC DMA Interrupt Setting
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @param      DmaUnderrun :  Setting DAC DMA Underrun Interrupt
*              - DAC_DMA_UNDERRUN_INTR_DISABLE : Disable DMA Underrun Interrupt
*              - DAC_DMA_UNDERRUN_INTR_ENABLE : Enable DMA Underrun Interrupt
* @param      DmaRx :  Setting DAC DMA Rx Interrupt
*              - DAC_DMA_RX_INTR_DISABLE : Disable DMA Rx Interrupt
*              - DAC_DMA_RX_INTR_ENABLE : Enable DMA Rx Interrupt
* @return     None
*******************************************************************************/
void DAC_DmaInterrupt(DAC_Type *pDACx, DAC_DMA_UNDERRUN_Type DmaUnderrun, DAC_DMA_RX_Type DmaRx)
{
	volatile uint32_t Reg32;

	Reg32 = pDACx->ICR;	
	Reg32 &= ~(DAC_ICR_DUDRUNE_Msk | DAC_ICR_DAMIE_Msk);
	Reg32 |= (DmaUnderrun << DAC_ICR_DUDRUNE_Pos) | (DmaRx << DAC_ICR_DAMIE_Pos);
	pDACx->ICR = Reg32;
}

/***************************************************************************//**
* @brief      Get DAC DMA Interrupt Enable Information
* @param      pDACx : DAC peripheral selected, should be
*              - DAC : DAC peripheral
* @return     DAC_INTR_ENABLE_Type
*              - DAC_STATUS_DMA_DONE_ENABLE : DMA done Interrupt Enable
*              - DAC_STATUS_DMA_UNDERRUN_ENABLE : DMA under-run Interrupt Enable
*******************************************************************************/
DAC_INTR_ENABLE_Type DAC_GetIntrEn(DAC_Type *pDACx)
{	
	return (DAC_INTR_ENABLE_Type)(pDACx->ICR & DAC_ICR_INTR_ENABLE_MASK);
}

/* --------------------------------- End Of File ------------------------------ */
