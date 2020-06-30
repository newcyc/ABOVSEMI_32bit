/***************************************************************************//**
* @file     A31G22x_crc.c
* @brief    Contains all functions support for CRC dirver on A31G22x
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
#include "A31G22x_crc.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER2_CRC_ENABLE_PERI			(0x01UL << SCU_PER2_CRC_Pos)
#define SCU_PCER2_CRC_ENABLE_CLOCK			(0x01UL << SCU_PCER2_CRC_Pos)

#define CRC_CR_INIT_EN_APPLY				(0x01UL << CRC_CR_INIT_EN_Pos)
#define CRC_SR_DMADINT_INTERRUPT_STATUS		(0x01UL << CRC_SR_DMADINT_Pos)


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
* @brief      Initialize CRC peripheral
* @param      pConfig : Pointer contains configuration of CRC
* @return     None
*******************************************************************************/
void CRC_Init(CRC_CFG_Type *pConfig)
{
	// Disable CRC peripheral & clock
	SCU->PER2 &= ~SCU_PER2_CRC_Msk;
	SCU->PCER2 &= ~SCU_PCER2_CRC_Msk;

	// Ensable CRC peripheral & clock
	SCU->PER2 |= SCU_PER2_CRC_ENABLE_PERI;
	SCU->PCER2|= SCU_PCER2_CRC_ENABLE_CLOCK;

	CRC->CR = 0x00UL
		| ((pConfig->InverseOutput << CRC_CR_OUT_INV_Pos) & CRC_CR_OUT_INV_Msk)
		| ((pConfig->ReverseOutput << CRC_CR_OUT_REV_Pos) & CRC_CR_OUT_REV_Msk)
		| ((pConfig->FirstBitInput << CRC_CR_IN_REV_Pos) & CRC_CR_IN_REV_Msk)
		| ((pConfig->DMADoneInterrupt<< CRC_CR_DMADINT_Pos) & CRC_CR_DMADINT_Msk)
		| ((pConfig->Polynomial << CRC_CR_POLY_Pos) & CRC_CR_POLY_Msk)
		;
}

/***************************************************************************//**
* @brief      De-Initialize CRC peripheral
* @return     None
*******************************************************************************/
void CRC_DeInit(void)
{
	// Disable CRC peripheral & clock
	SCU->PER2 &= ~SCU_PER2_CRC_Msk;
	SCU->PCER2 &= ~SCU_PCER2_CRC_Msk;
}

/***************************************************************************//**
* @brief      Set initial data of CRC
* @param      Init : Inital data (32-bit)
* @return     None
*******************************************************************************/
void CRC_SetInitVal(uint32_t Init)
{
	CRC->INIT = Init;
}

/***************************************************************************//**
* @brief      Apply CRC Init value
* @param      None
* @return     None
*******************************************************************************/
void CRC_ApplyInitVal(void)
{
	CRC->CR |= CRC_CR_INIT_EN_APPLY;
}

/***************************************************************************//**
* @brief      Set CRC Input Data
* @param      Data : CRC Input data (8-bit)
* @return     None
*******************************************************************************/
void CRC_SetInputData8(uint8_t Data)
{
	CRC->IDR = Data;
}

/***************************************************************************//**
* @brief      Get CRC Output Data
* @param      None
* @return     CRC Output Data (32-bit)
*******************************************************************************/
uint32_t CRC_GetOutputData(void)
{
	return CRC->ODR;
}

/***************************************************************************//**
* @brief      Get interrupt(DMA) status
* @param      none
* @return     TRUE : The DMA transfer is done, FALSE : The DMA transfer is not done
*******************************************************************************/
FlagStatus CRC_GetInterruptStatus(void)
{
	return (CRC->SR & CRC_SR_DMADINT_INTERRUPT_STATUS) ? SET : RESET;
}

/***************************************************************************//**
* @brief      Clear interrupt(DMA) status
* @param      None
* @return     None
*******************************************************************************/
void CRC_ClearInterruptStatus(void)
{
	CRC->SR |= CRC_SR_DMADINT_INTERRUPT_STATUS;
}


/* --------------------------------- End Of File ------------------------------ */
