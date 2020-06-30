/***************************************************************************//**
* @file     A31G22x_dmac.c
* @brief    Contains all functions support for DMA Controller dirver on A31G22x
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
#include "A31G22x_dmac.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER1_DMA_ENABLE_PERI			(0x01UL << SCU_PER1_DMA_Pos)
#define SCU_PCER1_DMA_ENABLE_CLOCK			(0x01UL << SCU_PCER1_DMA_Pos)

#define DMAC_SR_EOT_TRANSFER_STATUS			(0x01UL << DMAC_SR_EOT_Pos)
#define DMAC_SR_DMAEN_ENABLE_DMA			(0x01UL << DMAC_SR_DMAEN_Pos)


/*******************************************************************************
* Private Typedef
*******************************************************************************/
/**
 * @brief  DMAC peripheral number enumerated definition
 */
typedef enum {
	DMAC_PERI_NUMBER_NONE  = 0x00, /*!< Peripheral number of NONE */
	DMAC_PERI_NUMBER_DMAC0 = 0x01, /*!< Peripheral number of DMAC0 */
	DMAC_PERI_NUMBER_DMAC1 = 0x02, /*!< Peripheral number of DMAC1 */
	DMAC_PERI_NUMBER_DMAC2 = 0x04, /*!< Peripheral number of DMAC2 */
	DMAC_PERI_NUMBER_DMAC3 = 0x08, /*!< Peripheral number of DMAC3 */
	DMAC_PERI_NUMBER_DMAC4 = 0x10, /*!< Peripheral number of DMAC4 */
	DMAC_PERI_NUMBER_DMAC5 = 0x20, /*!< Peripheral number of DMAC5 */
	DMAC_PERI_NUMBER_DMAC6 = 0x40, /*!< Peripheral number of DMAC6 */
	DMAC_PERI_NUMBER_DMAC7 = 0x80 /*!< Peripheral number of DMAC7 */
} DMAC_PERI_NUMBER_Type;


/*******************************************************************************
* Private Variable
*******************************************************************************/
static DMAC_PERI_NUMBER_Type DMAC_ActivityFlag = DMAC_PERI_NUMBER_NONE;


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/
static DMAC_PERI_NUMBER_Type DMAC_GetPeriNum(DMAC_Type *pDMACx);


/*******************************************************************************
* Public Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Initialize DMAC(Direct Memory Access Controller) peripheral
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @param      pConfig : Pointer contains configuration of DMAC
* @return     None
*******************************************************************************/
void DMAC_Init(DMAC_Type *pDMACx, DMAC_CFG_Type *pConfig)
{
	DMAC_PERI_NUMBER_Type Number;

	if (DMAC_ActivityFlag == DMAC_PERI_NUMBER_NONE) {
		// Disable DMAC peripheral & clock
		SCU->PER1 &= ~SCU_PER1_DMA_Msk;
		SCU->PCER1 &= ~SCU_PCER1_DMA_Msk;

		// Enable DMAC peripheral & clock
		SCU->PER1 |= SCU_PER1_DMA_ENABLE_PERI;
		SCU->PCER1 |= SCU_PCER1_DMA_ENABLE_CLOCK;
	}

	Number = DMAC_GetPeriNum(pDMACx);
	DMAC_ActivityFlag |= Number;

	pDMACx->CR = 0x00UL
		| ((0x00UL << DMAC_CR_TRANSCNT_Pos) & DMAC_CR_TRANSCNT_Msk)
		| ((pConfig->Peripheral << DMAC_CR_PERISEL_Pos) & DMAC_CR_PERISEL_Msk)
		| ((pConfig->Width << DMAC_CR_SIZE_Pos) & DMAC_CR_SIZE_Msk)
		| ((pConfig->Direction << DMAC_CR_DIR_Pos) & DMAC_CR_DIR_Msk)
		;
	pDMACx->PAR = pConfig->PeripheralAddress;
}

/***************************************************************************//**
* @brief      De-Initialize DMAC peripheral
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @return     None
*******************************************************************************/
void DMAC_DeInit(DMAC_Type *pDMACx)
{
	DMAC_PERI_NUMBER_Type Number;

	DMAC_Stop(pDMACx);

	Number = DMAC_GetPeriNum(pDMACx);
	DMAC_ActivityFlag &= ~Number;

	if (DMAC_ActivityFlag == DMAC_PERI_NUMBER_NONE) {
		// Disable DMAC peripheral & clock
		SCU->PER1 &= ~SCU_PER1_DMA_Msk;
		SCU->PCER1 &= ~SCU_PCER1_DMA_Msk;
	}
}

/***************************************************************************//**
* @brief      Get remained transfer Count
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @return     Remained transfer count (12-bit)
*******************************************************************************/
uint32_t DMAC_GetTransferCount(DMAC_Type *pDMACx)
{
	return (pDMACx->CR & DMAC_CR_TRANSCNT_Msk) >> DMAC_CR_TRANSCNT_Pos;
}

/***************************************************************************//**
* @brief      Set transfer Count
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @param      Count : Transfer count (12-bit), should be
*              - 0x000 ~ 0xFFF
* @return     None
*******************************************************************************/
void DMAC_SetTransferCount(DMAC_Type *pDMACx, uint32_t Count)
{
	volatile uint32_t Reg32;

	Reg32 = pDMACx->CR;
	Reg32 &= ~DMAC_CR_TRANSCNT_Msk;
	Reg32 |= ((Count << DMAC_CR_TRANSCNT_Pos) & DMAC_CR_TRANSCNT_Msk);
	pDMACx->CR = Reg32;
}

/***************************************************************************//**
* @brief      Get memory address
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @return     Memory full address (32-bit)
*******************************************************************************/
uint32_t DMAC_GetMemoryAddress(DMAC_Type *pDMACx)
{
	return pDMACx->MAR;
}

/***************************************************************************//**
* @brief      Set memory address
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @param      Address : Memory full address (32-bit)
* @return     None
*******************************************************************************/
void DMAC_SetMemoryAddress(DMAC_Type *pDMACx, uint32_t Address)
{
	pDMACx->MAR = (uint32_t)Address;
}

/***************************************************************************//**
* @brief      Get DMAC status
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @return     SET : All data is transferred, RESET : Data to be transferred is existing
*******************************************************************************/
FlagStatus DMAC_GetStatus(DMAC_Type *pDMACx)
{
	return (pDMACx->SR & DMAC_SR_EOT_TRANSFER_STATUS) ? SET : RESET;
}

/***************************************************************************//**
* @brief      Start DMA
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @return     None
*******************************************************************************/
void DMAC_Start(DMAC_Type *pDMACx)
{
	volatile uint32_t Reg32;

	Reg32 = pDMACx->SR;
	Reg32 &= ~DMAC_SR_DMAEN_Msk;
	Reg32 |= DMAC_SR_DMAEN_ENABLE_DMA;
	pDMACx->SR = Reg32;
}

/***************************************************************************//**
* @brief      Stop DMA
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @return     None
*******************************************************************************/
void DMAC_Stop(DMAC_Type *pDMACx)
{
	pDMACx->SR &= ~DMAC_SR_DMAEN_Msk;
}

/***************************************************************************//**
* @brief      Clear DMA Buffer
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @return     None
*******************************************************************************/
void DMAC_ClearBuffer(DMAC_Type *pDMACx)
{
	pDMACx->SR |= DMAC_SR_DMARC_Msk;	// Clear DMA Buffer
}

/*******************************************************************************
* Private Function
*******************************************************************************/

/***************************************************************************//**
* @brief      DMAC_GetPeriNum
* @param      pDMACx : DMAC peripheral selected, should be
*              - DMAC0 ~ DMAC7 : DMAC0 ~ DMAC7  peripheral
* @return     DMAC_PERI_NUMBER_Type
*******************************************************************************/
static DMAC_PERI_NUMBER_Type DMAC_GetPeriNum(DMAC_Type *pDMACx)
{
	DMAC_PERI_NUMBER_Type Number;

	switch((uint32_t)pDMACx)
	{
		case DMAC0_BASE :
			Number = DMAC_PERI_NUMBER_DMAC0;
			break;

		case DMAC1_BASE :
			Number = DMAC_PERI_NUMBER_DMAC1;
			break;

		case DMAC2_BASE :
			Number = DMAC_PERI_NUMBER_DMAC2;
			break;

		case DMAC3_BASE :
			Number = DMAC_PERI_NUMBER_DMAC3;
			break;

		case DMAC4_BASE :
			Number = DMAC_PERI_NUMBER_DMAC4;
			break;

		case DMAC5_BASE :
			Number = DMAC_PERI_NUMBER_DMAC5;
			break;

		case DMAC6_BASE :
			Number = DMAC_PERI_NUMBER_DMAC6;
			break;

		case DMAC7_BASE :
			Number = DMAC_PERI_NUMBER_DMAC7;
			break;

		default :
			Number = DMAC_PERI_NUMBER_NONE;
			break;
	}

	return Number;
}


/* --------------------------------- End Of File ------------------------------ */
