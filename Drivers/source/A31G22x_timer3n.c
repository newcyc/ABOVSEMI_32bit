/***************************************************************************//**
* @file     A31G22x_timer3n.c
* @brief    Contains all functions support for 32-bit timer3n driver on A31G22x
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
#include "A31G22x_timer3n.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER1_TIMER30_ENABLE_PERI			(0x01UL << SCU_PER1_TIMER30_Pos)
#define SCU_PCER1_TIMER30_ENABLE_CLOCK			(0x01UL << SCU_PCER1_TIMER30_Pos)

#define TIMER3n_CR_EN_ENABLE_TIMER				(0x01UL << TIMER3n_CR_EN_Pos)
#define TIMER3n_CR_EN_CLEAR_COUNTER				(0x01UL << TIMER3n_CR_CLR_Pos)

#define TIMER3n_OUTCR_WTIDKY_KEY				(0xE06CUL)
#define TIMER3n_OUTCR_PWM30xA_MASK				(TIMER3n_OUTCR_POLA_Msk | TIMER3n_OUTCR_PAAOE_Msk \
												| TIMER3n_OUTCR_PBAOE_Msk | TIMER3n_OUTCR_PCAOE_Msk \
												| TIMER3n_OUTCR_LVLAA_Msk | TIMER3n_OUTCR_LVLBA_Msk \
												| TIMER3n_OUTCR_LVLCA_Msk)
#define TIMER3n_OUTCR_PWM30xB_MASK				(TIMER3n_OUTCR_POLB_Msk | TIMER3n_OUTCR_PABOE_Msk \
												| TIMER3n_OUTCR_PBBOE_Msk | TIMER3n_OUTCR_PCBOE_Msk \
												| TIMER3n_OUTCR_LVLAB_Msk | TIMER3n_OUTCR_LVLBB_Msk \
												| TIMER3n_OUTCR_LVLCB_Msk)

#define TIMER3n_INTFLAG_STATUS_GET_MASK			(TIMER3n_STATUS_HIZ_INT | TIMER3n_STATUS_CAPTURE_INT \
												| TIMER3n_STATUS_BOTTOM_INT | TIMER3n_STATUS_PMATCH_INT \
												| TIMER3n_STATUS_AMATCH_INT | TIMER3n_STATUS_BMATCH_INT \
												| TIMER3n_STATUS_CMATCH_INT)

#define TIMER3n_INTCR_MASK						(0x01UL)

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
* @brief      Initializes Timer3n peripheral according to the specified
*               parameters.
* @param      pTIMER3x : Timer3n peripheral selected, should be:
*              - TIMER30 :Timer30 peripheral
* @param      pConfig : Pointer contains configuration of Timer3n
* @return     None
*******************************************************************************/
void TIMER3n_Init(TIMER3n_Type *pTIMER3x, TIMER3n_CFG_Type *pConfig)
{
	SCU->PER1 &= ~SCU_PER1_TIMER30_ENABLE_PERI;
	SCU->PCER1 &= ~SCU_PCER1_TIMER30_ENABLE_CLOCK;

	SCU->PER1 |= SCU_PER1_TIMER30_ENABLE_PERI;
	SCU->PCER1 |= SCU_PCER1_TIMER30_ENABLE_CLOCK;

	// Control register
	pTIMER3x->CR = 0x00UL
		| ((pConfig->ExtClock << TIMER3n_CR_CLK_Pos) & TIMER3n_CR_CLK_Msk)
		| ((pConfig->Mode << TIMER3n_CR_MS_Pos) & TIMER3n_CR_MS_Msk)
		| ((pConfig->ExtClockEdge << TIMER3n_CR_ECE_Pos) & TIMER3n_CR_ECE_Msk)
		| ((pConfig->OutputMode << TIMER3n_CR_FORCA_Pos) & TIMER3n_CR_FORCA_Msk)
		| ((pConfig->DelayInsertion << TIMER3n_CR_DLYEN_Pos) & TIMER3n_CR_DLYEN_Msk)
		| ((pConfig->DelayPosition << TIMER3n_CR_DLYPOS_Pos) & TIMER3n_CR_DLYPOS_Msk)
		| ((pConfig->CapturePolarity << TIMER3n_CR_CPOL_Pos) & TIMER3n_CR_CPOL_Msk)
		| ((pConfig->ReloadTime << TIMER3n_CR_UPDT_Pos) & TIMER3n_CR_UPDT_Msk)
		| ((pConfig->MatchIntInterval << TIMER3n_CR_PMOC_Pos) & TIMER3n_CR_PMOC_Msk)
		| (0x01UL << TIMER3n_CR_CLR_Pos)
		;

	// P Data register
	TIMER3n_SetPData(pTIMER3x, pConfig->PData);

	// A Data register
	TIMER3n_SetAData(pTIMER3x, pConfig->AData);

	// B Data register
	TIMER3n_SetBData(pTIMER3x, pConfig->BData);

	// C Data register
	TIMER3n_SetCData(pTIMER3x, pConfig->CData);

	// Prescaler Data register
	TIMER3n_SetPrescalerData(pTIMER3x, pConfig->PrescalerData);

	// Delay Data register
	TIMER3n_SetDelayData(pTIMER3x, pConfig->DelayData);
}

/***************************************************************************//**
* @brief      De-Initialize Timer3n peripheral
* @param      pTIMER3x : Timer3n peripheral selected, should be:
*              - TIMER30 :Timer30 peripheral
* @return     None
*******************************************************************************/
void TIMER3n_DeInit(TIMER3n_Type *pTIMER3x)
{
	TIMER3n_Stop(pTIMER3x);

	SCU->PER1 &= ~SCU_PER1_TIMER30_ENABLE_PERI;
	SCU->PCER1 &= ~SCU_PCER1_TIMER30_ENABLE_CLOCK;
}

/***************************************************************************//**
* @brief      Configure Timer3n Output control
* @param      pTIMER3x : Timer3n peripheral selected, should be:
*              - TIMER30 :Timer30 peripheral
* @param      Output : Output control selected, should be
*              - TIMER3n_OUTPUT_CTRL_PWM30xA : PWM30xA output selection
*              - TIMER3n_OUTPUT_CTRL_PWM30xB : PWM30xb output selection
*              - TIMER3n_OUTPUT_CTRL_ALL : All selection
* @param      pPWM30xA : Pointer contains configuration of PWM30xA output
* @param      pPWM30xB : Pointer contains configuration of PWM30xB output
* @return     None
*******************************************************************************/
void TIMER3n_ConfigureOutput(TIMER3n_Type *pTIMER3x, TIMER3n_OUTPUT_CTRL_Type Output, TIMER3n_OUTPUT_CFG_Type *pPWM30xA, TIMER3n_OUTPUT_CFG_Type *pPWM30xB)
{
	volatile uint32_t Reg32;

	Reg32 = pTIMER3x->OUTCR;

	if (Output & TIMER3n_OUTPUT_CTRL_PWM30xA) {
		Reg32 &= ~(TIMER3n_OUTCR_PWM30xA_MASK);
		Reg32 |=  0x00UL
			| ((pPWM30xA->StartLevel << TIMER3n_OUTCR_POLA_Pos) & TIMER3n_OUTCR_POLA_Msk)
			| ((pPWM30xA->Output_PWM30Ax << TIMER3n_OUTCR_PAAOE_Pos) & TIMER3n_OUTCR_PAAOE_Msk)
			| ((pPWM30xA->Output_PWM30Bx << TIMER3n_OUTCR_PBAOE_Pos) & TIMER3n_OUTCR_PBAOE_Msk)
			| ((pPWM30xA->Output_PWM30Cx << TIMER3n_OUTCR_PCAOE_Pos) & TIMER3n_OUTCR_PCAOE_Msk)
			| ((pPWM30xA->DisableLevel_PWM30Ax << TIMER3n_OUTCR_LVLAA_Pos) & TIMER3n_OUTCR_LVLAA_Msk)
			| ((pPWM30xA->DisableLevel_PWM30Bx << TIMER3n_OUTCR_LVLBA_Pos) & TIMER3n_OUTCR_LVLBA_Msk)
			| ((pPWM30xA->DisableLevel_PWM30Cx << TIMER3n_OUTCR_LVLCA_Pos) & TIMER3n_OUTCR_LVLCA_Msk)
			;
	}

	if (Output & TIMER3n_OUTPUT_CTRL_PWM30xB) {
		Reg32 &= ~(TIMER3n_OUTCR_PWM30xB_MASK);
		Reg32 |=  0x00UL
			| ((pPWM30xB->StartLevel << TIMER3n_OUTCR_POLB_Pos) & TIMER3n_OUTCR_POLB_Msk)
			| ((pPWM30xB->Output_PWM30Ax << TIMER3n_OUTCR_PABOE_Pos) & TIMER3n_OUTCR_PABOE_Msk)
			| ((pPWM30xB->Output_PWM30Bx << TIMER3n_OUTCR_PBBOE_Pos) & TIMER3n_OUTCR_PBBOE_Msk)
			| ((pPWM30xB->Output_PWM30Cx << TIMER3n_OUTCR_PCBOE_Pos) & TIMER3n_OUTCR_PCBOE_Msk)
			| ((pPWM30xB->DisableLevel_PWM30Ax << TIMER3n_OUTCR_LVLAB_Pos) & TIMER3n_OUTCR_LVLAB_Msk)
			| ((pPWM30xB->DisableLevel_PWM30Bx << TIMER3n_OUTCR_LVLBB_Pos) & TIMER3n_OUTCR_LVLBB_Msk)
			| ((pPWM30xB->DisableLevel_PWM30Cx << TIMER3n_OUTCR_LVLCB_Pos) & TIMER3n_OUTCR_LVLCB_Msk)
			;
	}

	pTIMER3x->OUTCR = (TIMER3n_OUTCR_WTIDKY_KEY << TIMER3n_OUTCR_WTIDKY_Pos) | Reg32;
}

/***************************************************************************//**
* @brief      Configure Timer3n interrupt
* @param      pTIMER3x : Timer3n peripheral selected, should be:
*              - TIMER30 :Timer30 peripheral
* @param      InterruptSource : Interrupt source selected, should be
*              - TIMER3n_INTERRUPT_CMATCH_INT : C-ch match interrupt
*              - TIMER3n_INTERRUPT_BMATCH_INT : B-ch match interrupt
*              - TIMER3n_INTERRUPT_AMATCH_INT : A-ch match interrupt
*              - TIMER3n_INTERRUPT_PMATCH_INT : Period match interrupt
*              - TIMER3n_INTERRUPT_BOTTOM_INT : Bottom interrupt
*              - TIMER3n_INTERRUPT_CAPTURE_INT : Capture interrupt
*              - TIMER3n_INTERRUPT_HIZ_INT : High impedance interrupt
* @param      InterruptEnable :
*              - ENABLE : Enable interrupt
*              - DISABLE : Disable interrupt
* @return     None
*******************************************************************************/
void TIMER3n_ConfigureInterrupt(TIMER3n_Type *pTIMER3x, TIMER3n_INTERRUPT_Type InterruptSource, FunctionalState InterruptEnable)
{
	volatile uint32_t Reg32;

	Reg32 = pTIMER3x->INTCR;
	Reg32 &= ~(TIMER3n_INTFLAG_STATUS_CLEAR_MASK);
	Reg32 |= (InterruptSource);
	pTIMER3x->INTCR = Reg32;
}

/***************************************************************************//**
* @brief      Configure Timer3n high-impedance
* @param      pTIMER3x : Timer3n peripheral selected, should be:
*              - TIMER30 :Timer30 peripheral
* @param      pConfig : Pointer contains configuration of Timer3n high-impedance
* @return     None
*******************************************************************************/
void TIMER3n_ConfigureHiZ(TIMER3n_Type *pTIMER3x, TIMER3n_HiZ_CFG_Type *pConfig)
{
	volatile uint32_t Reg32;

	Reg32 = pTIMER3x->HIZCR;
	Reg32 &= ~(TIMER3n_HIZCR_MASK);
	Reg32 |=  0x00UL
		| ((pConfig->HiZ << TIMER3n_HIZCR_HIZEN_Pos) & TIMER3n_HIZCR_HIZEN_Msk)
		| ((pConfig->BLNK_Edge << TIMER3n_HIZCR_HEDGE_Pos) & TIMER3n_HIZCR_HEDGE_Msk)
		;

	pTIMER3x->HIZCR = Reg32;
}

/***************************************************************************//**
* @brief      Configure ADC trigger of Timer3n
* @param      pTIMER3x : Timer3n peripheral selected, should be:
*              - TIMER30 :Timer30 peripheral
* @param      pConfig : Pointer contains configuration of ADC trigger
* @return     None
*******************************************************************************/
void TIMER3n_ConfigureADCTrigger(TIMER3n_Type *pTIMER3x, TIMER3n_ADC_TRIGGER_CFG_Type *pConfig)
{
	pTIMER3x->ADTCR |=  0x00UL
		| ((pConfig->Trigger_Bottom << TIMER3n_ADTCR_BTTG_Pos) & TIMER3n_ADTCR_BTTG_Msk)
		| ((pConfig->Trigger_PMatch << TIMER3n_ADTCR_PMTG_Pos) & TIMER3n_ADTCR_PMTG_Msk)
		| ((pConfig->Trigger_AMatch << TIMER3n_ADTCR_AMTG_Pos) & TIMER3n_ADTCR_AMTG_Msk)
		| ((pConfig->Trigger_BMatch << TIMER3n_ADTCR_BMTG_Pos) & TIMER3n_ADTCR_BMTG_Msk)
		| ((pConfig->Trigger_CMatch << TIMER3n_ADTCR_CMTG_Pos) & TIMER3n_ADTCR_CMTG_Msk)
		;
}

/***************************************************************************//**
* @brief      Get P data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     P data : 0x0002 to 0xFFFF
*******************************************************************************/
uint32_t TIMER3n_GetPData(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->PDR & TIMER3n_PDR_PDATA_Msk;
}

/***************************************************************************//**
* @brief      Set P data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Data : P data, should be
*              - 0x0002 to 0xFFFF
* @return     None
*******************************************************************************/
void TIMER3n_SetPData(TIMER3n_Type *pTIMER3x, uint32_t Data)
{
	pTIMER3x->PDR = Data & TIMER3n_PDR_PDATA_Msk;
}

/***************************************************************************//**
* @brief      Get A data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     A data : 0x0000 to 0xFFFF
*******************************************************************************/
uint32_t TIMER3n_GetAData(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->ADR & TIMER3n_ADR_ADATA_Msk;
}

/***************************************************************************//**
* @brief      Set A data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Data : A data, should be
*              - 0x0000 to 0xFFFF
* @return     None
*******************************************************************************/
void TIMER3n_SetAData(TIMER3n_Type *pTIMER3x, uint32_t Data)
{
	pTIMER3x->ADR = Data & TIMER3n_ADR_ADATA_Msk;
}

/***************************************************************************//**
* @brief      Get B data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     B data : 0x0000 to 0xFFFF
*******************************************************************************/
uint32_t TIMER3n_GetBData(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->BDR & TIMER3n_BDR_BDATA_Msk;
}

/***************************************************************************//**
* @brief      Set B data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Data : B data, should be
*              - 0x0000 to 0xFFFF
* @return     None
*******************************************************************************/
void TIMER3n_SetBData(TIMER3n_Type *pTIMER3x, uint32_t Data)
{
	pTIMER3x->BDR = Data & TIMER3n_BDR_BDATA_Msk;
}

/***************************************************************************//**
* @brief      Get C data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     C data : 0x0000 to 0xFFFF
*******************************************************************************/
uint32_t TIMER3n_GetCData(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->CDR & TIMER3n_CDR_CDATA_Msk;
}

/***************************************************************************//**
* @brief      Set C data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Data : C data, should be
*              - 0x0000 to 0xFFFF
* @return     None
*******************************************************************************/
void TIMER3n_SetCData(TIMER3n_Type *pTIMER3x, uint32_t Data)
{
	pTIMER3x->CDR = Data & TIMER3n_CDR_CDATA_Msk;
}

/***************************************************************************//**
* @brief      Get capture data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     Capture data : 0x0000 ~ 0xFFFF
*******************************************************************************/
uint32_t TIMER3n_GetCaptureData(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->CAPDR & TIMER3n_CAPDR_CAPD_Msk;
}

/***************************************************************************//**
* @brief      Get prescaler data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     Prescaler data : 0x0000 to 0x0FFF
*******************************************************************************/
uint32_t TIMER3n_GetPrescalerData(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->PREDR & TIMER3n_PREDR_PRED_Msk;
}

/***************************************************************************//**
* @brief      Set prescaler data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Data : Prescaler data, should be
*              - 0x0000 to 0x0FFF
* @return     None
*******************************************************************************/
void TIMER3n_SetPrescalerData(TIMER3n_Type *pTIMER3x, uint32_t Data)
{
	pTIMER3x->PREDR = Data & TIMER3n_PREDR_PRED_Msk;
}

/***************************************************************************//**
* @brief      Get Counter data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     Counter data : 0x0000 ~ 0xFFFF
*******************************************************************************/
uint32_t TIMER3n_GetCounter(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->CNT & TIMER3n_CNT_CNT_Msk;
}

/***************************************************************************//**
* @brief      Clear counter and prescaler  
*             Automatically cleared to ?b?after operation
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     None
*******************************************************************************/
void TIMER3n_ClearCounter(TIMER3n_Type *pTIMER3x)
{
	pTIMER3x->CR |= TIMER3n_CR_EN_CLEAR_COUNTER;
}

/***************************************************************************//**
* @brief      Get delay data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     delay data : 0x0000 to 0x03FF
*******************************************************************************/
uint32_t TIMER3n_GetDelayData(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->DLY & TIMER3n_DLY_DLY_Msk;
}

/***************************************************************************//**
* @brief      Set delay data  
*             Dela time = (DLY + 1) / fT30
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Data : Delay data, should be
*              - 0x0000 to 0x03FF
* @return     None
*******************************************************************************/
void TIMER3n_SetDelayData(TIMER3n_Type *pTIMER3x, uint32_t Data)
{
	pTIMER3x->DLY = Data & TIMER3n_DLY_DLY_Msk;
}

/***************************************************************************//**
* @brief      Get Timer3n status
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     TIMER3n_STATUS_Type value
*              - TIMER3n_STATUS_CMATCH_INT : C-ch match interrupt status
*              - TIMER3n_STATUS_BMATCH_INT : B-ch match interrupt status
*              - TIMER3n_STATUS_AMATCH_INT : A-ch match interrupt status
*              - TIMER3n_STATUS_PMATCH_INT : Period match interrupt status
*              - TIMER3n_STATUS_BOTTOM_INT : Bottom interrupt status
*              - TIMER3n_STATUS_CAPTURE_INT : Capture interrupt status
*              - TIMER3n_STATUS_HIZ_INT : High impedance interrupt status
*******************************************************************************/
TIMER3n_STATUS_Type TIMER3n_GetStatus(TIMER3n_Type *pTIMER3x)
{
	return (TIMER3n_STATUS_Type)(pTIMER3x->INTFLAG & TIMER3n_INTFLAG_STATUS_GET_MASK);
}

/***************************************************************************//**
* @brief      Clear Timer3n status
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Status : Status selected, should be
*              - TIMER3n_STATUS_CMATCH_INT : C-ch match interrupt status
*              - TIMER3n_STATUS_BMATCH_INT : B-ch match interrupt status
*              - TIMER3n_STATUS_AMATCH_INT : A-ch match interrupt status
*              - TIMER3n_STATUS_PMATCH_INT : Period match interrupt status
*              - TIMER3n_STATUS_BOTTOM_INT : Bottom interrupt status
*              - TIMER3n_STATUS_CAPTURE_INT : Capture interrupt status
*              - TIMER3n_STATUS_HIZ_INT : High impedance interrupt status
* @return      None
*******************************************************************************/
void TIMER3n_ClearStatus(TIMER3n_Type *pTIMER3x, TIMER3n_STATUS_Type Status)
{
	pTIMER3x->INTFLAG = (Status & TIMER3n_INTFLAG_STATUS_CLEAR_MASK);
}

/***************************************************************************//**
* @brief      Get high impedance status
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     SET : High impedance, RESET : Normal operation
*******************************************************************************/
SetState TIMER3n_GetHiZStatus(TIMER3n_Type *pTIMER3x)
{
	return (pTIMER3x->HIZCR & TIMER3n_HIZCR_HIZSTA_Msk) ? SET : RESET;
}

/***************************************************************************//**
* @brief      Set high impedance output
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Output : high impedance output, should be
*              - SET : High impedance
*              - RESET : Normal operation
* @return     None
*******************************************************************************/
void TIMER3n_SetHiZOutput(TIMER3n_Type *pTIMER3x, SetState Output)
{
	if (Output == SET) {
		pTIMER3x->HIZCR |= TIMER3n_HIZCR_SET_HIZ;
	} else {
		pTIMER3x->HIZCR |= TIMER3n_HIZCR_RESET_HIZ;
	}
}

/***************************************************************************//**
* @brief      Get ADC trigger generator data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     delay data : 0x0000 to 0x3FFF
*******************************************************************************/
uint32_t TIMER3n_GetADCTriggerData(TIMER3n_Type *pTIMER3x)
{
	return pTIMER3x->ADTDR & TIMER3n_ADTDR_ADTDATA_Msk;
}

/***************************************************************************//**
* @brief      Set ADC trigger generator data
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @param      Data : ADC trigger generator data, should be
*              - 0x0000 to 0x3FFF
* @return     None
*******************************************************************************/
void TIMER3n_SetADCTriggerData(TIMER3n_Type *pTIMER3x, uint32_t Data)
{
	pTIMER3x->ADTDR = Data & TIMER3n_ADTDR_ADTDATA_Msk;
}

/***************************************************************************//**
* @brief      Start Timer3n
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     None
*******************************************************************************/
void TIMER3n_Start(TIMER3n_Type *pTIMER3x)
{
	volatile uint32_t Reg32;

	Reg32 = pTIMER3x->CR;
	Reg32 &= ~TIMER3n_CR_EN_Msk;
	Reg32 |= TIMER3n_CR_EN_ENABLE_TIMER;
	pTIMER3x->CR = Reg32;
}

/***************************************************************************//**
* @brief      Stop Timer3n
* @param      pTIMER3x : Timer3n peripheral selected, should be
*              - TIMER30 : Timer 30 peripheral
* @return     None
*******************************************************************************/
void TIMER3n_Stop(TIMER3n_Type *pTIMER3x)
{
	pTIMER3x->CR &= ~TIMER3n_CR_EN_Msk;
}

/* --------------------------------- End Of File ------------------------------ */
