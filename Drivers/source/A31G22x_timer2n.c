/***************************************************************************//**
* @file     A31G22x_timer2n.c
* @brief    Contains all functions support for 32-bit timer2n driver on A31G22x
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
#include "A31G22x_timer2n.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER1_TIMER20_ENABLE_PERI			(0x01UL << SCU_PER1_TIMER20_Pos)
#define SCU_PCER1_TIMER20_ENABLE_CLOCK			(0x01UL << SCU_PCER1_TIMER20_Pos)

#define SCU_PER1_TIMER21_ENABLE_PERI			(0x01UL << SCU_PER1_TIMER21_Pos)
#define SCU_PCER1_TIMER21_ENABLE_CLOCK			(0x01UL << SCU_PCER1_TIMER21_Pos)

#define TIMER2n_CR_EN_ENABLE_TIMER				(0x01UL << TIMER2n_CR_EN_Pos)
#define TIMER2n_CR_EN_CLEAR_COUNTER				(0x01UL << TIMER2n_CR_CLR_Pos)

#define TIMER2n_CR_STATUS_GET_MASK				(TIMER2n_STATUS_MATCH_INT | TIMER2n_STATUS_CAPTURE_INT)
#define TIMER2n_CR_STATUS_CLEAR_MASK			(TIMER2n_STATUS_MATCH_INT | TIMER2n_STATUS_CAPTURE_INT)


/*******************************************************************************
* Private Typedef
*******************************************************************************/


/*******************************************************************************
* Private Variable
*******************************************************************************/


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/

/***************************************************************************//**
* @brief      Initialize Timer2n peripheral
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @param      pConfig : Pointer contains configuration of Timer2n
* @return     None
*******************************************************************************/
void TIMER2n_Init(TIMER2n_Type *pTIMER2x, TIMER2n_CFG_Type *pConfig)
{
	volatile uint32_t Reg32;

	if (pTIMER2x == TIMER20) {
		SCU->PER1 &= ~SCU_PER1_TIMER20_Msk;
		SCU->PCER1 &= ~SCU_PCER1_TIMER20_Msk;
		
		SCU->PER1 |= SCU_PER1_TIMER20_ENABLE_PERI;
		SCU->PCER1 |= SCU_PCER1_TIMER20_ENABLE_CLOCK;
	} else if (pTIMER2x == TIMER21) {
		SCU->PER1 &= ~SCU_PER1_TIMER21_Msk;
		SCU->PCER1 &= ~SCU_PCER1_TIMER21_Msk;
		
		SCU->PER1 |= SCU_PER1_TIMER21_ENABLE_PERI;
		SCU->PCER1 |= SCU_PCER1_TIMER21_ENABLE_CLOCK;
	}

	// Control register
	Reg32 = 0x00UL
		| ((pConfig->ExtClock << TIMER2n_CR_CLK_Pos) & TIMER2n_CR_CLK_Msk)
		| ((pConfig->Mode << TIMER2n_CR_MS_Pos) & TIMER2n_CR_MS_Msk)
		| ((pConfig->ExtClockEdge << TIMER2n_CR_ECE_Pos) & TIMER2n_CR_ECE_Msk)
		| ((pConfig->OutputPolarity << TIMER2n_CR_OPOL_Pos) & TIMER2n_CR_OPOL_Msk)
		| ((pConfig->CapturePolarity << TIMER2n_CR_CPOL_Pos) & TIMER2n_CR_CPOL_Msk)
		| ((pConfig->MatchInterrupt << TIMER2n_CR_MIEN_Pos) & TIMER2n_CR_MIEN_Msk)
		| ((pConfig->CaptureInterrupt << TIMER2n_CR_CIEN_Pos) & TIMER2n_CR_CIEN_Msk)
		| (0x01UL << TIMER2n_CR_CLR_Pos)
		;
	if (pTIMER2x == TIMER20) {
		Reg32 |= ((pConfig->CaptureSignal << TIMER2n_CR_CAPSEL_Pos) & TIMER2n_CR_CAPSEL_Msk);
	}

	pTIMER2x->CR = Reg32;

	// A Data register
	TIMER2n_SetAData(pTIMER2x, pConfig->AData);

	// B Data register
	TIMER2n_SetBData(pTIMER2x, pConfig->BData);

	// Prescaler Data register
	TIMER2n_SetPrescalerData(pTIMER2x, pConfig->PrescalerData);
}

/***************************************************************************//**
* @brief      De-Initialize Timer2n peripheral
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     None
*******************************************************************************/
void TIMER2n_DeInit(TIMER2n_Type *pTIMER2x)
{
	TIMER2n_Stop(pTIMER2x);

	if (pTIMER2x== TIMER20) {
		SCU->PER1 &= ~SCU_PER1_TIMER20_Msk;
		SCU->PCER1 &= ~SCU_PCER1_TIMER20_Msk;
	} else if (pTIMER2x== TIMER21) {
		SCU->PER1 &= ~SCU_PER1_TIMER21_Msk;
		SCU->PCER1 &= ~SCU_PCER1_TIMER21_Msk;
	}
}

/***************************************************************************//**
* @brief      Get A data
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     A data : 0x0000 0002 to 0xFFFF FFFF
*******************************************************************************/
uint32_t TIMER2n_GetAData(TIMER2n_Type *pTIMER2x)
{
	return pTIMER2x->ADR & TIMER2n_ADR_ADATA_Msk;
}

/***************************************************************************//**
* @brief      Set A data
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @param      Data : A data, should be
*              - 0x0000 0002 to 0xFFFF FFFF
* @return     None
*******************************************************************************/
void TIMER2n_SetAData(TIMER2n_Type *pTIMER2x, uint32_t Data)
{
	pTIMER2x->ADR = Data & TIMER2n_ADR_ADATA_Msk;
}

/***************************************************************************//**
* @brief      Get B data
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     B data : 0x0000 0000 to 0xFFFF FFFF
*******************************************************************************/
uint32_t TIMER2n_GetBData(TIMER2n_Type *pTIMER2x)
{
	return pTIMER2x->BDR & TIMER2n_BDR_BDATA_Msk;
}

/***************************************************************************//**
* @brief      Set B data
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @param      Data : B Data, should be
*              - 0x0000 0000 to 0xFFFF FFFF
* @return     None
*******************************************************************************/
void TIMER2n_SetBData(TIMER2n_Type *pTIMER2x, uint32_t Data)
{
	pTIMER2x->BDR = Data & TIMER2n_BDR_BDATA_Msk;
}

/***************************************************************************//**
* @brief      Get capture data
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     Capture data : 0x0000 0000 ~ 0xFFFF FFFF
*******************************************************************************/
uint32_t TIMER2n_GetCaptureData(TIMER2n_Type *pTIMER2x)
{
	return pTIMER2x->CAPDR & TIMER2n_CAPDR_CAPD_Msk;
}

/***************************************************************************//**
* @brief      Get prescaler data
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     Prescaler data : 0x0000 to 0x0FFF
*******************************************************************************/
uint32_t TIMER2n_GetPrescalerData(TIMER2n_Type *pTIMER2x)
{
	return pTIMER2x->PREDR & TIMER2n_PREDR_PRED_Msk;
}

/***************************************************************************//**
* @brief      Set prescaler data
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @param      Data : Prescaler data, should be
*              - 0x0000 to 0x0FFF
* @return     None
*******************************************************************************/
void TIMER2n_SetPrescalerData(TIMER2n_Type *pTIMER2x, uint32_t Data)
{
	pTIMER2x->PREDR = Data & TIMER2n_PREDR_PRED_Msk;
}

/***************************************************************************//**
* @brief      Get Counter data
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     Counter data : 0x0000 0000 ~ 0xFFFF FFFF
*******************************************************************************/
uint32_t TIMER2n_GetCounter(TIMER2n_Type *pTIMER2x)
{
	return pTIMER2x->CNT & TIMER2n_CNT_CNT_Msk;
}

/***************************************************************************//**
* @brief      Clear counter and prescaler  
*             Automatically cleared to “0b” after operation
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     None
*******************************************************************************/
void TIMER2n_ClearCounter(TIMER2n_Type *pTIMER2x)
{
	pTIMER2x->CR |= TIMER2n_CR_EN_CLEAR_COUNTER;
}

/***************************************************************************//**
* @brief      Get Timer2n status
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     TIMER2n_STATUS_Type value
*              - TIMER2n_STATUS_CAPTURE_INT : Capture interrupt status
*              - TIMER2n_STATUS_MATCH_INT : Match interrupt status
*******************************************************************************/
TIMER2n_STATUS_Type TIMER2n_GetStatus(TIMER2n_Type *pTIMER2x)
{
	return (TIMER2n_STATUS_Type)(pTIMER2x->CR & TIMER2n_CR_STATUS_GET_MASK);
}

/***************************************************************************//**
* @brief      Clear Timer2n status
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @param      Status : Status selected, should be
*              - TIMER2n_STATUS_CAPTURE_INT : Capture interrupt status
*              - TIMER2n_STATUS_MATCH_INT : Match interrupt status
* @return      None
*******************************************************************************/
void TIMER2n_ClearStatus(TIMER2n_Type *pTIMER2x, TIMER2n_STATUS_Type Status)
{
	pTIMER2x->CR |= (Status & TIMER2n_CR_STATUS_CLEAR_MASK);
}

/***************************************************************************//**
* @brief      Pause Timer2n
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @param      Pause :
*              - ENABLE : Pause timer2n
*              - DISABLE : Continue timer2n
* @return     None
*******************************************************************************/
void TIMER2n_Pause(TIMER2n_Type *pTIMER2x, FunctionalState Pause)
{
	volatile uint32_t Reg32;

	Reg32 = pTIMER2x->CR;
	Reg32 &= ~TIMER2n_CR_EN_Msk;
	Reg32 |= (Pause << TIMER2n_CR_PAU_Pos);
	pTIMER2x->CR = Reg32;
}

/***************************************************************************//**
* @brief      Start Timer2n
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     None
*******************************************************************************/
void TIMER2n_Start(TIMER2n_Type *pTIMER2x)
{
	volatile uint32_t Reg32;

	Reg32 = pTIMER2x->CR;
	Reg32 &= ~TIMER2n_CR_EN_Msk;
	Reg32 |= TIMER2n_CR_EN_ENABLE_TIMER;
	pTIMER2x->CR = Reg32;
}

/***************************************************************************//**
* @brief      Stop Timer2n
* @param      pTIMER2x : Timer2n peripheral selected, should be
*              - TIMER20 ~ TIMER21 : Timer 20 ~ 21 peripheral
* @return     None
*******************************************************************************/
void TIMER2n_Stop(TIMER2n_Type *pTIMER2x)
{
	pTIMER2x->CR &= ~TIMER2n_CR_EN_Msk;
}

/* --------------------------------- End Of File ------------------------------ */
