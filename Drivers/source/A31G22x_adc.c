/***************************************************************************//**
* @file     A31G22x_adc.c
* @brief    Contains all functions support for ADC dirver on A31G22x
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
#include "A31G22x_adc.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER2_ADC_ENABLE_PERI			(0x01UL << SCU_PER2_ADC_Pos)
#define SCU_PCER2_ADC_ENABLE_CLOCK			(0x01UL << SCU_PCER2_ADC_Pos)

// SEQTRG0 ~ SEQTRG7 must be same structure
#define ADC_TRG_MASK_VALUE					(ADC_TRG_SEQTRG0_BSTTRG_Msk)
#define ADC_TRG_SHIFT_VALUE					(ADC_TRG_SEQTRG1_Pos)

// SEQ0CH ~ SEQ7CH must be same structure
#define ADC_SCSR_CHANNEL_PER_REGISTER		(4)
#define ADC_SCSR_MASK_VALUE					(ADC_SCSR1_SEQ0CH_Msk)
#define ADC_SCSR_SHIFT_VALUE				(ADC_SCSR1_SEQ1CH_Pos)

#define ADC_CR_ASTOP_STOP_CONVERSION		(0x01UL << ADC_CR_ASTOP_Pos)
#define ADC_CR_ASTART_START_CONVERSION		(0x01UL << ADC_CR_ASTART_Pos)

#define ADC_IER_INTERRUPT_ENABLE_MASK		(ADC_INTR_DMAIE | ADC_INTR_TRGIE \
											| ADC_INTR_EOSIE | ADC_INTR_EOCIE)

#define ADC_IER_MASK_VALUE					(0x01UL)

#define ADC_DDR_TRGINF_Pos					(ADC_DDR_TRGINFO0_Pos)
#define ADC_DDR_TRGINF_Msk					(ADC_DDR_TRGINFO0_Msk | ADC_DDR_TRGINFO1_Msk \
											| ADC_DDR_TRGINFO2_Msk | ADC_DDR_TRGINFO3_Msk \
											| ADC_DDR_TRGINFO4_Msk | ADC_DDR_TRGINFO5_Msk \
											| ADC_DDR_TRGINFO6_Msk | ADC_DDR_TRGINFO7_Msk)

#define ADC_CMPR_STOP_COMPARE				(0x00UL << ADC_CMPR_CMPIEN_Pos)
#define ADC_CMPR_START_COMPARE				(0x01UL << ADC_CMPR_CMPIEN_Pos)

/*******************************************************************************
* Private Typedef
*******************************************************************************/


/*******************************************************************************
* Private Variable
*******************************************************************************/


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/
static uint32_t ADC_GetDataRegister(ADC_Type *pADCx, ADC_DATA_SEL_Type Selection);


/*******************************************************************************
* Public Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Initialize ADC(Analog/Digital Convertor) peripheral
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      pConfig : Pointer contains configuration of ADC
* @return     None
*******************************************************************************/
void ADC_Init(ADC_Type *pADCx, ADC_CFG_Type *pConfig)
{
	volatile uint32_t Reg32;

	// Disable ADC peripheral & clock
	SCU->PER2 &= ~SCU_PER2_ADC_Msk;
	SCU->PCER2 &= ~SCU_PCER2_ADC_Msk;

	// Enable ADC peripheral & clock
	SCU->PER2 |= SCU_PER2_ADC_ENABLE_PERI;
	SCU->PCER2 |= SCU_PCER2_ADC_ENABLE_CLOCK;
	
	// Configure mode register
	Reg32 = 0x00UL
		| ((pConfig->TriggerInfo << ADC_MR_TRGINFO_Pos) & ADC_MR_TRGINFO_Msk)
		| ((pConfig->ChannelInfo << ADC_MR_CHINFO_Pos) & ADC_MR_CHINFO_Msk)
		| ((pConfig->Dma << ADC_MR_DMAEN_Pos) & ADC_MR_DMAEN_Msk)
		| ((pConfig->SamplingTime << ADC_MR_STSEL_Pos) & ADC_MR_STSEL_Msk)
		| ((pConfig->SequenceCounter << ADC_MR_SEQCNT_Pos) & ADC_MR_SEQCNT_Msk)
		| (0x01UL << ADC_MR_ADEN_Pos)
		| ((pConfig->Restart << ADC_MR_ARST_Pos) & ADC_MR_ARST_Msk)
		| ((pConfig->TriggerSelection << ADC_MR_TRGSEL_Pos) & ADC_MR_TRGSEL_Msk)
		;

	switch (pConfig->Mode) {
		case ADC_MODE_SINGLE :
		case ADC_MODE_SEQUENTIAL :
		default :
			Reg32 |= (0x00UL << ADC_MR_ADMOD_Pos);
			break;

		case ADC_MODE_BURST :
			Reg32 |= (0x01UL << ADC_MR_ADMOD_Pos);
			break;

		case ADC_MODE_MULTIPLE :
			Reg32 |= (0x02UL << ADC_MR_ADMOD_Pos);
			break;
	}
	pADCx->MR = Reg32;

	// Configure clock control register
	Reg32 = 0x00UL
		| ((pConfig->InternalClockDivider << ADC_CCR_CLKDIV_Pos) & ADC_CCR_CLKDIV_Msk)
		| (0x00UL << ADC_CCR_ADCPD_Pos)
		| ((pConfig->ExternalClock << ADC_CCR_EXTCLK_Pos) & ADC_CCR_EXTCLK_Msk)
		| (0x00UL << ADC_CCR_CLKDIV_Pos)
		;
	pADCx->CCR = Reg32;

	// Stop ADC
	pADCx->CR = ADC_CR_ASTOP_STOP_CONVERSION;

	// Clear current sequence number
	pADCx->CSCR = 0x00UL;
}

/***************************************************************************//**
* @brief      De-Initialize ADC peripheral
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @return     None
*******************************************************************************/
void ADC_DeInit(ADC_Type *pADCx)
{
	ADC_Stop(pADCx);

	// Disable ADC peripheral & clock
	SCU->PER2 &= ~SCU_PER2_ADC_ENABLE_PERI;
	SCU->PCER2 &= ~SCU_PCER2_ADC_ENABLE_CLOCK;
}

/***************************************************************************//**
* @brief      Configure ADC interrupt
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      InterruptSource : Interrupt source selected, should be
*              - ADC_INTERRUPT_SINGLE : ADC single conversion interrupt
*              - ADC_INTERRUPT_SEQUENCE : ADC sequence conversion interrupt
*              - ADC_INTERRUPT_TRIGGER : ADC trigger conversion interrupt
*              - ADC_INTERRUPT_DMA : DMA done interrupt
* @param      InterruptEnable :
*              - ENABLE : Enable interrupt
*              - DISABLE : Disable interrupt
* @return     None
*******************************************************************************/
void ADC_ConfigureInterrupt(ADC_Type *pADCx, ADC_INTERRUPT_Type InterruptSource, FunctionalState InterruptEnable)
{
	volatile uint32_t Reg32;

	Reg32 = pADCx->IER;
	Reg32 &= ~(InterruptSource);
	Reg32 |= (InterruptSource);
	pADCx->IER = Reg32;
}

/***************************************************************************//**
* @brief      Configure n'th sequence about trigger source
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      SeqIndex : Sequence index selected, should be
*              - ADC_SEQUENCE_0 ~ 7 : Sequence 0 ~ 7
* @param      TriSource : Trigger source selected, should be
*              - ADC_TRIGGER_SOURCE_0 ~ 11, 15 : Trigger source 0 ~ 11, 15
* @return     None
*******************************************************************************/
void ADC_ConfigureOneTrigger(ADC_Type *pADCx, ADC_SEQUENCE_Type SeqIndex, ADC_TRIGGER_SOURCE_Type TriSource)
{
	uint32_t Shift;
	volatile uint32_t Reg32;

	Shift = SeqIndex * ADC_TRG_SHIFT_VALUE;

	Reg32 = pADCx->TRG;
	Reg32 &= ~(ADC_TRG_MASK_VALUE << Shift);
	Reg32 |= (TriSource << Shift);
	pADCx->TRG = Reg32;
}

/***************************************************************************//**
* @brief      Configure all sequence about trigger source
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      MaxSequenceNum : Max sequence number (1 ~ 8)
* @param      pADCChannel : Pointer contains trigger source of n'th sequence
* @return     None
*******************************************************************************/
void ADC_ConfigureAllTrigger(ADC_Type *pADCx, uint32_t MaxSequenceNum, ADC_TRIGGER_SOURCE_Type *pTriSource)
{
	uint32_t i;
	uint32_t Reg32;
	uint32_t Shift;
	uint32_t Source;

	Reg32 = 0;
	for (i = 0; i < MaxSequenceNum; i++) {
		Shift = i * ADC_TRG_SHIFT_VALUE;
		Source = *pTriSource++;
		Reg32 |= (Source << Shift);
	}
	pADCx->TRG = Reg32;
}

/***************************************************************************//**
* @brief      Configure n'th sequence about ADC channel index
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      SeqIndex : Sequence index selected, should be
*              - ADC_SEQUENCE_0 ~ 7 : Sequence 0 ~ 7
* @param      ADCChannel : ADC channel index of n'th sequence, should be 
*              - ADC_CHANNEL_0 ~ 17 : ADC channel 0 ~ 17
* @return     None
*******************************************************************************/
void ADC_ConfigureOneSequence(ADC_Type *pADCx, ADC_SEQUENCE_Type SeqIndex, ADC_CHANNEL_Type ADCIndex)
{
	uint32_t Reg32;
	uint32_t Shift;

	if (SeqIndex < ADC_SCSR_CHANNEL_PER_REGISTER) {
		Shift = SeqIndex * ADC_SCSR_SHIFT_VALUE;

		Reg32 = pADCx->SCSR1;
		Reg32 &= ~(ADC_SCSR_MASK_VALUE << Shift);
		Reg32 |= (ADCIndex << Shift);
		pADCx->SCSR1 = Reg32;
	} else {
		SeqIndex -= ADC_SCSR_CHANNEL_PER_REGISTER;
		Shift = SeqIndex * ADC_SCSR_SHIFT_VALUE;

		Reg32 = pADCx->SCSR2;
		Reg32 &= ~(ADC_SCSR_MASK_VALUE << Shift);
		Reg32 |= (ADCIndex << Shift);
		pADCx->SCSR2 = Reg32;
	}
}

/***************************************************************************//**
* @brief      Configure all sequence about ADC channel index
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      MaxSequenceNum : Max sequence number (1 ~ 8)
* @param      pADCChannel : Pointer contains ADC channel index of n'th sequence
* @return     None
*******************************************************************************/
void ADC_ConfigureAllSequence(ADC_Type *pADCx, uint32_t MaxSequenceNum, ADC_CHANNEL_Type *pADCChannel)
{
	uint32_t i;
	uint32_t Num;
	uint32_t Reg32;
	uint32_t Shift;
	uint32_t ChannelNum;

	Num = MaxSequenceNum;
	if (Num > ADC_SCSR_CHANNEL_PER_REGISTER) {
		Num  = ADC_SCSR_CHANNEL_PER_REGISTER;
		MaxSequenceNum -= ADC_SCSR_CHANNEL_PER_REGISTER;
	} else {
		MaxSequenceNum = 0;
	}

	Reg32 = 0;
	for (i = 0; i < Num; i++) {
		Shift = i * ADC_SCSR_SHIFT_VALUE;
		ChannelNum = *pADCChannel++;
		Reg32 |= (ChannelNum << Shift);
	}
	pADCx->SCSR1 = Reg32;

	Num = MaxSequenceNum;
	Reg32 = 0;
	for (i = 0; i < Num; i++) {
		Shift = i * ADC_SCSR_SHIFT_VALUE;
		ChannelNum = *pADCChannel++;
		Reg32 |= (ChannelNum << Shift);
	}
	pADCx->SCSR2 = Reg32;
}

/***************************************************************************//**
* @brief      Configure ADC compare
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      pConfig : Pointer contains configuration of ADC compare
* @return     None
*******************************************************************************/
void ADC_ConfigureCompare(ADC_Type *pADCx, ADC_CMP_CFG_Type *pCMPConfig)
{
	uint32_t Reg32;

	// Configure mode register
	Reg32 = 0x00UL
		| (pCMPConfig->Interrupt << ADC_CMPR_CMPIEN_Pos)
		| (pCMPConfig->Operation << ADC_CMPR_CMPEN_Pos)
		| (pCMPConfig->TriggerSmall << ADC_CMPR_LTE_Pos)
		| (pCMPConfig->Channel << ADC_CMPR_CCH_Pos)
		| ((pCMPConfig->CompareValue << ADC_CMPR_CVAL_Pos) & ADC_CMPR_CVAL_Msk)
		;
	pADCx->CMPR = Reg32;
}

/***************************************************************************//**
* @brief      Start comparison of A/D conversion value with reference value
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @return     None
*******************************************************************************/
void ADC_CompareStart(ADC_Type *pADCx)
{
	pADCx->CMPR |= ADC_CMPR_CMPEN_Msk;
}

/***************************************************************************//**
* @brief      Start comparison
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @return     None
*******************************************************************************/
void ADC_CompareStop(ADC_Type *pADCx)
{
	pADCx->CMPR &= ~ADC_CMPR_CMPEN_Msk;
}

/***************************************************************************//**
* @brief      Start comparison
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @return     None
*******************************************************************************/
uint16_t ADC_GetReferenceData(ADC_Type *pADCx) 
{	
	return ((ADC->CMPR & ADC_CMPR_CVAL_Msk) >> 4);
}


/***************************************************************************//**
* @brief      Get conversion data from ADC data register
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      Selection : conversion data selected, should be
*              - ADC_DATA_SEL_LAST : Last conversion data
*              - ADC_DATA_SEL_SEQ0 ~ 7 : Sequence 0 ~ 7 conversion data
* @return     Conversion data (12-bit)
*******************************************************************************/
uint32_t ADC_GetConversionData(ADC_Type *pADCx, ADC_DATA_SEL_Type Selection)
{
	uint32_t Reg32;

	Reg32 = ADC_GetDataRegister(pADCx, Selection);
	Reg32 = (Reg32 & ADC_DDR_ADDMAR_Msk) >> ADC_DDR_ADDMAR_Pos;
	return Reg32;
}

/***************************************************************************//**
* @brief      Get trigger informaiotn from ADC data register
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      Selection : conversion data selected, should be
*              - ADC_DATA_SEL_LAST : Last conversion data
*              - ADC_DATA_SEL_SEQ0 ~ 7 : Sequence 0 ~ 7 conversion data
* @return     Trigger Information (ADC_TRIGGER_INFO_Type, 0x01 ~ 0x80)
*******************************************************************************/
ADC_TRIGGER_INFO_Type ADC_GetTriggerInfo(ADC_Type *pADCx, ADC_DATA_SEL_Type Selection)
{
	uint32_t Reg32;

	Reg32 = ADC_GetDataRegister(pADCx, Selection);
	Reg32 = (Reg32 & ADC_DDR_TRGINF_Msk) >> ADC_DDR_TRGINF_Pos;
	return (ADC_TRIGGER_INFO_Type)Reg32;
}

/***************************************************************************//**
* @brief      Get channel information from ADC data register
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      Selection : conversion data selected, should be
*              - ADC_DATA_SEL_LAST : Last conversion data
*              - ADC_DATA_SEL_SEQ0 ~ 7 : Sequence 0 ~ 7 conversion data
* @return     channel information (0x00 ~ 0x11)
*              - 0x00 : ADC channel 0
*              - 0x11 : ADC channel 17
*******************************************************************************/
uint32_t ADC_GetChannelInfo(ADC_Type *pADCx, ADC_DATA_SEL_Type Selection)
{
	uint32_t Reg32;

	Reg32 = ADC_GetDataRegister(pADCx, Selection);
	Reg32 = (Reg32 & ADC_DDR_ADMACH_Msk) >> ADC_DDR_ADMACH_Pos;
	return Reg32;
}

/***************************************************************************//**
* @brief      Get ADC status
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @return     ADC_STATUS_Type value
*              - ADC_STATUS_SINGLE_INT : Conversion end interrupt status
*              - ADC_STATUS_SEQUENCE_INT : Sequence end interrupt status
*              - ADC_STATUS_TRIGGER_INT : ADC trigger interrupt status
*              - ADC_STATUS_DMA_DONE : DMA done received status
*              - ADC_STATUS_DMA_OVER : DMA overrun status
*              - ADC_STATUS_COMPARE_INT : Compare interrupt status
*******************************************************************************/
ADC_STATUS_Type ADC_GetStatus(ADC_Type *pADCx)
{
	return (ADC_STATUS_Type)(pADCx->SR & ADC_SR_STATUS_GET_MASK);
}

/***************************************************************************//**
* @brief      Get ADC Interrupt Enable status
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @return     ADC_STATUS_Type value
*              - ADC_INTR_DMAIE : ADC DMA Done Interrupt Enable
*              - ADC_INTR_TRGIE : ADC Trigger Conversion Interrupt Enable
*              - ADC_INTR_EOSIE : ADC Sequence Conversion Interrupt Enable
*              - ADC_INTR_EOCIE : ADC Single Conversion Interrupt Enable
*******************************************************************************/
ADC_INTR_Type ADC_GetInterruptEnableStatus(ADC_Type *pADCx)
{
	return (ADC_INTR_Type)(pADCx->IER & ADC_IER_INTERRUPT_ENABLE_MASK);
}


/***************************************************************************//**
* @brief      Clear ADC status
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      Status : Status selected, should be
*              - ADC_STATUS_SINGLE_INT : Conversion end interrupt status
*              - ADC_STATUS_SEQUENCE_INT : Sequence end interrupt status
*              - ADC_STATUS_TRIGGER_INT : ADC trigger interrupt status
*              - ADC_STATUS_COMPARE_INT : Compare interrupt status
*              - ADC_STATUS_DMA_DONE : DMA done received status
*              - ADC_STATUS_DMA_OVER : DMA overrun status
* @return      None
*******************************************************************************/
void ADC_ClearStatus(ADC_Type *pADCx, ADC_STATUS_Type Status)
{
	pADCx->SR = (Status & ADC_SR_STATUS_CLEAR_MASK);
}

/***************************************************************************//**
* @brief      Enter deep sleep mode
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      DeepSleep :
*              - ENABLE : Enter deep sleep mode
*              - DISABLE : Enter normal mode
* @return 		None
*******************************************************************************/
void ADC_EnterDeepSleep(ADC_Type *pADCx, FunctionalState DeepSleep)
{
	volatile uint32_t Reg32;

	Reg32 = pADCx->CCR;
	Reg32 &= ~(ADC_CCR_ADCPD_Msk);
	Reg32 |= (DeepSleep << ADC_CCR_ADCPD_Pos);
	pADCx->CCR = Reg32;
}

/***************************************************************************//**
* @brief      Start A/D conversion
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @return     None
*******************************************************************************/
void ADC_Start(ADC_Type *pADCx)
{
	pADCx->CR = ADC_CR_ASTART_START_CONVERSION;
}

/***************************************************************************//**
* @brief      Stop A/D conversion  
*             If this function called after a conversion cycle starts,  
*             the current conversion is completed
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @return     None
*******************************************************************************/
void ADC_Stop(ADC_Type *pADCx)
{
	pADCx->CR = ADC_CR_ASTOP_STOP_CONVERSION;
}


/*******************************************************************************
* Private Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Get ADC data register value
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC peripheral
* @param      Selection : conversion data selected, should be
*              - ADC_DATA_SEL_LAST : Last conversion data
*              - ADC_DATA_SEL_SEQ0 ~ 7 : Sequence 0 ~ 7 conversion data
* @return     data register value
*******************************************************************************/
static uint32_t ADC_GetDataRegister(ADC_Type *pADCx, ADC_DATA_SEL_Type Selection)
{
	uint32_t Reg32;

	switch (Selection) {
		case ADC_DATA_SEL_LAST :
		default :
			Reg32 = pADCx->DDR;
			break;

		case ADC_DATA_SEL_SEQ0 :
			Reg32 = pADCx->DR0;
			break;

		case ADC_DATA_SEL_SEQ1 :
			Reg32 = pADCx->DR1;
			break;

		case ADC_DATA_SEL_SEQ2 :
			Reg32 = pADCx->DR2;
			break;

		case ADC_DATA_SEL_SEQ3 :
			Reg32 = pADCx->DR3;
			break;

		case ADC_DATA_SEL_SEQ4 :
			Reg32 = pADCx->DR4;
			break;

		case ADC_DATA_SEL_SEQ5 :
			Reg32 = pADCx->DR5;
			break;

		case ADC_DATA_SEL_SEQ6 :
			Reg32 = pADCx->DR6;
			break;

		case ADC_DATA_SEL_SEQ7 :
			Reg32 = pADCx->DR7;
			break;
	}

	return Reg32;
}

/***************************************************************************//**
* @brief      Enable ADC DMA function
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC  peripheral
* @return     None
*******************************************************************************/
void ADC_DmaEnable(ADC_Type *pADCx)
{
	pADCx->MR |= ADC_MR_DMAEN_Msk;
}

/***************************************************************************//**
* @brief      Stop ADC DMA function
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC  peripheral
* @return     None
*******************************************************************************/
void ADC_DmaDisable(ADC_Type *pADCx)
{
	pADCx->MR &= ~ADC_MR_DMAEN_Msk;
}

/***************************************************************************//**
* @brief      Set ADC Input Buffer
* @param      pADCx : ADC peripheral selected, should be
*              - ADC : ADC  peripheral
* @param      BufType : ADC input type, should be
*              - ADC_BUFINPUT_BUFFER
*              - ADC_BUFINPUT_BYPASS
* @param      BufType : ADC input buffer operating mode, should be
*              - ADC_BUFMODE_POWERDOWN
*              - ADC_BUFMODE_OPERATE
* @param      TBufSel : ADC Buffering Off Time Selection, Should be
*              - ADC_BUFTIME_1MCLK
*              - ADC_BUFTIME_2MCLK
*              - ADC_BUFTIME_3MCLK
*              - ADC_BUFTIME_4MCLK
*              - ADC_BUFTIME_5MCLK
*              - ADC_BUFTIME_6MCLK
*              - ADC_BUFTIME_7MCLK
*              - ADC_BUFTIME_8MCLK
* @return     None
*******************************************************************************/
void ADC_SetBuffer(ADC_Type *pADCx, ADC_BUFINPUT_Type BufType, ADC_BUFMODE_Type Mode, ADC_BUFTIME_Type TBufSel)
{
	uint32_t Reg32;
	
	Reg32 = pADCx->BCR;
	
	Reg32 &= ~(ADC_BCR_BYPSEL_Msk);
	Reg32 |= BufType;

	Reg32 &= ~(ADC_BCR_BUFEN_Msk);
	Reg32 |= Mode;
	
	Reg32 &= ~(ADC_BCR_TBUFSEL_Msk);
	Reg32 |= TBufSel;
	
	pADCx->BCR = Reg32;
}

/* --------------------------------- End Of File ------------------------------ */
