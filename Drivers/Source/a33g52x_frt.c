/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_frt.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
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
#include "a33g52x_frt.h"
#include "a33g52x_nvic.h"


/**
*********************************************************************************************************
* @ Name : FRT_ConfigureInterrupt 
*
* @ Parameters
*		- frt : FRT
*		- intr_mask : FRTCON_FMIE, FRTCON_FOIE
*		- enable : INTR_ENABLE, INTR_DISABLE
*
*
*********************************************************************************************************
*/
void FRT_ConfigureInterrupt (FRT_Type * const frt, uint32_t intr_mask, uint32_t enable)
{
	uint32_t			reg_val; 
	


	//------------------------------------------------------------------------------
	// disable interrupt 
	//
	//				FRTCON			@ address = 0x4000_0508
	//
	//------------------------------------------------------------------------------
	reg_val = frt->CON;
	reg_val &= ~(FRTCON_FMIE|FRTCON_FOIE); 
	frt->CON = reg_val;
	

	//------------------------------------------------------------------------------
	// clear interrupt flag 
	//
	//				FRTCON			@ address = 0x4000_0508
	//
	//------------------------------------------------------------------------------
	reg_val &= ~(FRTCON_FMF|FRTCON_FOF); 
	frt->CON = reg_val;

	

	//------------------------------------------------------------------------------
	// enable interrupt 
	//
	//				FRTCON			@ address = 0x4000_0508
	//
	//------------------------------------------------------------------------------
	if (enable == INTR_ENABLE)
	{
		reg_val |= intr_mask; 
		frt->CON = reg_val;
	}

}


