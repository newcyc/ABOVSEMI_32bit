/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_pmc.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : August, 2017
*
* @ Description 
*   ABOV Semiconductor is supplying this software for use with A33G52x
*   processor. This software contains the confidential and proprietary information
*   of ABOV Semiconductor Co., Ltd ("Confidential Information").
*
*
**************************************************************************************
* DISCLAIMER 
*
* 	THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS  
* 	WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE  
* 	TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* 	DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING  
* 	FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE  
* 	CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.  
*
**************************************************************************************
*/


#include <stdint.h>
#include "A33G52x.h"
#include "a33g52x_pcu.h"
#include "a33G52x_nvic.h"

/**
************************************************************************************
* @ Name : 
*
* @ Parameter
*		pmc		= PCU_A ~ PCU_F
*		pin_no	= PIN_0 ~ PIN_15
*		func		
*
*
*
************************************************************************************
*/
void PCU_ConfigureFunction (PCU_Type * PCUx, uint32_t pin_no, uint32_t func)
{

	uint32_t	pin_offset; 
	uint32_t reg_val; 



	//------------------------------------------------------------------------------
	// pin_offset = pin_no * 2
	//------------------------------------------------------------------------------
	pin_offset = (pin_no << 1); 


	//------------------------------------------------------------------------------
	// PnMR
	//------------------------------------------------------------------------------
	reg_val = PCUx->MR;
	reg_val &= ~(PnMR_FUNC_MASK << pin_offset); 
	reg_val |= func; 

	PCUx->MR = reg_val;
}


/**
************************************************************************************
* @ Name : 
*
* @ Parameter
*		pmc		= PCU_A ~ PCU_F
*		pin_no	= PIN_0 ~ PIN_15
*		dir_type	= PnCR_OUTPUT_PUSH_PULL, PnCR_OUTPUT_OPEN_DRAIN, 
*				   PnCR_INPUT_LOGIC, PnCR_INPUT_ANALOG
*
*
************************************************************************************
*/
void PCU_Set_Direction_Type (PCU_Type * PCUx, uint32_t pin_no, uint32_t dir_type)
{


	uint32_t	pin_offset; 
	uint32_t 	reg_val; 


	//------------------------------------------------------------------------------
	// pin_offset = pin_no * 2
	//------------------------------------------------------------------------------
	pin_offset = (pin_no << 1); 


	//------------------------------------------------------------------------------
	// PnCR
	//------------------------------------------------------------------------------
	reg_val = PCUx->CR;
	reg_val &= ~(PnCR_MASK << pin_offset); 
	reg_val |= ((dir_type & PnCR_MASK) << pin_offset); 

	PCUx->CR = reg_val;
}


/**
************************************************************************************
* @ Name : PCU_ConfigurePullup
*
* @ Parameter
*		pmc		= PCU_A ~ PCU_F
*		pin_no	= PIN_0 ~ PIN_15
*		pullup	= PnPCR_PULLUP_DISABLE, PnPCR_PULLUP_ENABLE
*
*
************************************************************************************
*/
void PCU_ConfigurePullup (PCU_Type * PCUx, uint32_t pin_no, uint32_t pullup)
{

	uint32_t	reg_val; 

	reg_val = PCUx->PCR;
	reg_val &= ~(0x01<<pin_no); 

	if (pullup == PnPCR_PULLUP_ENABLE)
		reg_val |= (0x01<<pin_no); 

	PCUx->PCR = reg_val;
}



/**
************************************************************************************
* @ Name : PCU_ConfigurePullup_Pulldown
*
* @ Parameter
*		pmc		= PCU_A ~ PCU_F
*		pin_no	= PIN_0 ~ PIN_15
*		updown	= PnPCR_PULLUPDOWN_UP, PnPCR_PULLUPDOWN_DOWN
*		enable	= PnPCR_PULLUPDOWN_DISABLE, PnPCR_PULLUPDOWN_ENABLE
*
*
************************************************************************************
*/
void PCU_ConfigurePullup_Pulldown (PCU_Type * PCUx, uint32_t pin_no, uint32_t updown, uint32_t enable)
{

	uint32_t	reg_val; 

	reg_val = PCUx->PCR;
	reg_val &= ~(0x00010001UL<<pin_no); 

	if (enable == PnPCR_PULLUPDOWN_ENABLE && updown == PnPCR_PULLUPDOWN_UP)
		reg_val |= (0x00000001UL<<pin_no);
	else if (enable == PnPCR_PULLUPDOWN_ENABLE && updown == PnPCR_PULLUPDOWN_DOWN)
		reg_val |= (0x00010001UL<<pin_no);

	PCUx->PCR = reg_val;
}


/**
************************************************************************************
* @ Name : PCU_ConfigureInterrupt
*
* @ Parameter
*		pmc			= PCU_A ~ PCU_F
*		pin_no		= PIN_0 ~ PIN_15
*
*		intr_mask	= PCU_NO_INTR, 
*				  	   PCU_LOW_LEVEL_INTR, PCU_HIGH_LEVEL_INTR, 
*					   PCU_FALLING_EDGE_INTR, PCU_RISING_EDGE_INTR, PCU_BOTH_FALLING_RISING_EDGE_INTR
*					
*		enable		= INTR_ENABLE, INTR_DISABLE
*
*
************************************************************************************
*/
void PCU_ConfigureInterrupt (PCU_Type * PCUx, uint32_t pin_no, uint32_t intr_mask, uint32_t enable)
{

	uint32_t		pin_offset; 
	uint32_t		ier, icr; 
	uint32_t		reg_val; 

	
	//------------------------------------------------------------------------------
	// pin_offset = pin_no * 2
	//------------------------------------------------------------------------------
	pin_offset = (pin_no << 1); 



	//------------------------------------------------------------------------------
	// disable interrupt
	//------------------------------------------------------------------------------
	ier = PCUx->IER;
	icr = PCUx->ICR;

	ier &= ~(0x0003 << pin_offset); 
	icr &= ~(0x0003 << pin_offset); 
	
	PCUx->IER = ier;
	PCUx->ICR = icr;

	//------------------------------------------------------------------------------
	// clear interrupt flag
	//------------------------------------------------------------------------------	
	reg_val = PCUx->ISR;
	reg_val &= (PnISR_MASK << pin_offset); 
	PCUx->ISR = reg_val;

	//------------------------------------------------------------------------------
	// enable interrupt
	//------------------------------------------------------------------------------
//	if (enable == 1)
	if (enable == INTR_ENABLE)
	{
		ier |= ((intr_mask & 0x03) << pin_offset); 
		icr |= (((intr_mask & 0x30)>>4) << pin_offset); 
	
		PCUx->ICR = icr;	
		PCUx->IER = ier;
	
	}
}

/**
************************************************************************************
* @ Name : PCU_Debounce
*
* @ Parameter
*		pmc			= PCA ~ PCF
*		pin_no		= PIN_0 ~ PIN_15
*
*		dpr			= 0 ~ 31
*					
*		enable		= 1(Enable), 0 (Disable)
*
*
************************************************************************************
*/

void PCU_Debounce(PCU_Type * PCUx, uint32_t pin_no, uint8_t dpr, uint32_t enable)
{
	uint32_t 	reg_val;
	
	if(enable == 1)
	{
		PCUx->DPR = dpr;
		PCUx->DER |= (1 << pin_no);
	}
	else
	{
		reg_val = PCUx->DER;
		reg_val &= ~(1 << pin_no);
		PCUx->DER = reg_val;
	}
}
