/***************************************************************************//**
* @file     A31G22x_crc.c
* @brief    Contains all functions support for TS(Temperature Sensor) dirver on A31G22x
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
#include "A31G22x_ts.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER2_TS_ENABLE_PERI				(0x01UL << SCU_PER2_TS_Pos)
#define SCU_PCER2_TS_ENABLE_CLOCK			(0x01UL << SCU_PCER2_TS_Pos)

#define TS_CR_SS_START_SENSING				(0x01UL << TS_CR_SS_Pos)
#define TS_CR_MF_CLEAR_FLAG					(0x01UL << TS_CR_MF_Pos)


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
* @brief      Initialize TS(temperature sensor) peripheral
* @param      pConfig : Pointer contains configuration of TS
* @return     None
*******************************************************************************/
void TS_Init(TS_CFG_Type *pConfig)
{
	// Disable CRC peripheral & clock
	SCU->PER2 &= ~SCU_PER2_TS_Msk;
	SCU->PCER2 &= ~SCU_PCER2_TS_Msk;

	// Ensable CRC peripheral & clock
	SCU->PER2 |= SCU_PER2_TS_ENABLE_PERI;
	SCU->PCER2|= SCU_PCER2_TS_ENABLE_CLOCK;

	TS->CR = 0x00UL
		| ((pConfig->MatchInterrupt << TS_CR_IEN_Pos) & TS_CR_IEN_Msk)
		| (0x01UL << TS_CR_MF_Pos)
		| (0x00UL << TS_CR_SS_Pos)
		| (0x01UL << TS_CR_EN_Pos)
		;

	TS->REFPERIOD = pConfig->ReferencePeriod & TS_REFPERIOD_REFPERIOD_Msk;
}

/***************************************************************************//**
* @brief      De-Initialize TS peripheral
* @return     None
*******************************************************************************/
void TS_DeInit(void)
{
	TS_Stop();

	// Disable CRC peripheral & clock
	SCU->PER2 &= ~SCU_PER2_TS_Msk;
	SCU->PCER2 &= ~SCU_PCER2_TS_Msk;
}

/***************************************************************************//**
* @brief      Get match flag of TS
* @param      None
* @return     SET : Occurred, RESET : Not occurred
*******************************************************************************/
FlagStatus TS_GetStatus(void)
{
	return (TS->CR & TS_CR_MF_Msk) ? SET : RESET;
}

/***************************************************************************//**
* @brief      Clear match flag of TS
* @param      None
* @return     None
*******************************************************************************/
void TS_ClearStatus(void)
{
	TS->CR |= TS_CR_MF_CLEAR_FLAG;
}

/***************************************************************************//**
* @brief      Get sensing count of TS
* @param      None
* @return     Sensing Count : 0x0000 ~ 0xFFFF
*******************************************************************************/
uint32_t TS_GetSensingCount(void)
{
	return TS->SENSECNT & TS_SENSECNT_SENSECNT_Msk;
}

/***************************************************************************//**
* @brief      Start temperature sensing
* @param      None
* @return     None
*******************************************************************************/
void TS_Start(void)
{
	volatile uint32_t Reg32;

	Reg32 = TS->CR;
	Reg32 &= ~TS_CR_SS_Msk;
	Reg32 |= TS_CR_SS_START_SENSING;
	TS->CR = Reg32;
}

/***************************************************************************//**
* @brief      Stop temperature sensing
* @param      None
* @return     None
*******************************************************************************/
void TS_Stop(void)
{
	TS->CR &= ~TS_CR_SS_Msk;
}

/* --------------------------------- End Of File ------------------------------ */
