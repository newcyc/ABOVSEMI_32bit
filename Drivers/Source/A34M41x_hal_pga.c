/**********************************************************************
* @file		A34M41x_pga.c
* @brief	Contains all functions support for ADC firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV M team
*
* Copyright(C) 2015, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_pga.h"
#include "A34M41x_hal_scu.h"
#include "A34M41x_debug_frmwrk.h"

/***********************************************************************************************
* @brief		Configure PGA peripheral and clock reference/input voltage
* @param[in]	NewState		should be :
* 						- DISABLE= 0
* 						- ENABLE	= 1							
* @return	HAL Status
 ***********************************************************************************************/
void HAL_PGA_ClockInit(FunctionalState NewState)
{
	//PGA peripheral & Clock Enable
	SYST_ACCESS_EN();
	
	SCU->PER2 &= ~(1<<24);//PGA disble
	SCU->PCER2 &= ~(1<<24); //PGA clock disable 
	
	if(NewState == ENABLE)
	{
		SCU->PER2 |= (1<<24);//PGA enable
		SCU->PCER2 |= (1<<24); //PGA clock enable 
	}
	
	SYST_ACCESS_DIS();
}

/***********************************************************************************************
* @brief		Configure comparator reference/input voltage
* @param[in]	PGAx		PGA peripheral selected, should be :
						- PGA : PGA0~2 peripheral
* @param[in]	AMPISel		PGA Current Setting bit, should be :
						- PGA_CR_AMPISEL_0	
						- PGA_CR_AMPISEL_17	
						- PGA_CR_AMPISEL_21	
						- PGA_CR_AMPISEL_12	
* @param[in]	GainSel		Gain Selecting bit, should be :
						- PGA_GAINSEL_1_200				
						- PGA_GAINSEL_1_304				
						- PGA_GAINSEL_1_404				
						- PGA_GAINSEL_1_500				
						- PGA_GAINSEL_1_600				
						- PGA_GAINSEL_1_702				
						- PGA_GAINSEL_1_805				
						- PGA_GAINSEL_1_905				
						- PGA_GAINSEL_2_000				
						- PGA_GAINSEL_2_182				
						- PGA_GAINSEL_2_330				
						- PGA_GAINSEL_2_500				
						- PGA_GAINSEL_2_667				
						- PGA_GAINSEL_2_927				
						- PGA_GAINSEL_3_000				
						- PGA_GAINSEL_3_158				
						- PGA_GAINSEL_3_478				
						- PGA_GAINSEL_3_871				
						- PGA_GAINSEL_4_000				
						- PGA_GAINSEL_4_364				
						- PGA_GAINSEL_5_000				
						- PGA_GAINSEL_5_854				
						- PGA_GAINSEL_6_000				
						- PGA_GAINSEL_6_857				
						- PGA_GAINSEL_8_000				
						- PGA_GAINSEL_8_571				
						- PGA_GAINSEL_10_00				
						- PGA_GAINSEL_12_00				
						- PGA_GAINSEL_15_00				
						- PGA_GAINSEL_16_00				
* @param[in]	UGainSel		Unit Gain Enable bit, should be :
* 						- DISABLE= 0
* 						- ENABLE	= 1	
* @param[in]	AMPEnSel	PGA Enable bit, should be :
* 						- DISABLE= 0
* 						- ENABLE	= 1
* @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_PGA_Init(PGA_Type *PGAx, PGA_CFG_Type *PGA_ConfigStruct)
{
	uint32_t	tempreg;
	
	/* Check PGA  handle */
         if(PGAx == NULL)
        {
            return HAL_ERROR;
        }	
	tempreg = PGAx->CR&~(0x3FFFF);
	
	tempreg = 0
	| (PGA_ConfigStruct->AMPISel)
	| ((PGA_ConfigStruct->GainSel)<<8)
	| ((PGA_ConfigStruct->UGainEnSel)<<1)
	| (PGA_ConfigStruct->AMPEnSel)
	;
	
	PGAx->CR = tempreg;
	return HAL_OK;
	
}
/* --------------------------------- End Of File ------------------------------ */

