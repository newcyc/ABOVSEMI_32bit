/**********************************************************************
* @file		A34M41x_pwr.c
* @brief	Contains all functions support for PCU firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_pwr.h"
#include "A34M41x_hal_scu.h"

/*********************************************************************//**
 * @brief 		Enter Sleep mode with co-operated instruction by the Cortex-M4F.
 * @param[in]	None
 * @return		None
 **********************************************************************/
void HAL_PWR_EnterSleepMode(void)
{
	/* Sleep Mode*/
	SCB->SCR = 0;
	__DSB();
	__WFI();
	__NOP();
	__NOP();
	__NOP();
	__NOP();	
}

/*********************************************************************//**
 * @brief 		Enter Power Down mode with co-operated instruction by the Cortex-M4F.
 * @param[in]	None
 * @return		None
 **********************************************************************/
void HAL_PWR_EnterPowerDownMode(void)
{
    /* Deep-Sleep Mode, set SLEEPDEEP bit */
	SCB->SCR = 0x4;
	/* Power Down Mode*/
	__WFI();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

/*********************************************************************//**
 * @brief 		Low Voltage Indicator Control
 * @param[in]	None
 * @return		None
 **********************************************************************/

void HAL_LVI_Init(LVI_CFG_Type *LVI_ConfigStruct)
{
	uint32_t	tempreg;
	
	SYST_ACCESS_EN();
	
	tempreg = SCU->LVICR&0xFF;
	
	tempreg = 0
	| ((LVI_ConfigStruct->EnSel)<<7)
	| ((LVI_ConfigStruct->IntSel)<<5)
	| ((LVI_ConfigStruct->DeepSel)<<4)
	| ((LVI_ConfigStruct->LvlSel)<<0)
	;
	
	SCU->LVICR = tempreg;
	
	SYST_ACCESS_DIS();
}

/*********************************************************************//**
 * @brief 		Low Voltage Indicator Control
 * @param[in]	None
 * @return		None
 **********************************************************************/

void HAL_LVR_Init(LVR_CFG_Type *LVR_ConfigStruct)
{
	uint32_t	tempreg;
	
	SYST_ACCESS_EN();
	
	tempreg = SCU->LVRCR&0xFF1F;
	
	tempreg = 0
	| (LVR_ConfigStruct->EnSel)
	| (LVR_ConfigStruct->DeepSel)
	| (LVR_ConfigStruct->LvlSel)
	;
	
	SCU->LVRCR = tempreg;
	
	SYST_ACCESS_DIS();
}

/* --------------------------------- End Of File ------------------------------ */
