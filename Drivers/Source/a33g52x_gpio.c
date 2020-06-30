/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_gpio.c
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
#include "a33g52x_gpio.h"



/**
************************************************************************************
* @ Name : GPIO_OutputHigh
*
* @ Parameter
*		- gpio		= GPIO_A ~ GPIO_F
*		- pin_no		= PIN_0 ~ PIN_15
*
*
************************************************************************************
*/
void GPIO_OutputHigh (GPIO_Type * const Px, uint32_t pin_no)
{

	uint32_t 	reg_val; 

	//---------------------------------------------------------------------------
	// bit set 
	//
	// 					PASRR		@ address = 0x4000_0308
	// 					PBSRR		@ address = 0x4000_0318
	// 					PCSRR		@ address = 0x4000_0328
	// 					PDSRR		@ address = 0x4000_0338
	// 					PESRR		@ address = 0x4000_0348
	// 					PFSRR		@ address = 0x4000_0358
	//
	//---------------------------------------------------------------------------
	reg_val = (0x0001<<pin_no); 
	Px->SRR = reg_val;

}




/**
************************************************************************************
* @ Name : GPIO_OutputLow 
*
* @ Parameter
*		- gpio		= GPIO_A ~ GPIO_F
*		- pin_no		= PIN_0 ~ PIN_15
*
*
************************************************************************************
*/
void GPIO_OutputLow (GPIO_Type * const Px, uint32_t pin_no)
{

	uint32_t 	reg_val; 


	//---------------------------------------------------------------------------
	// bit clear 
	//
	// 					PASRR		@ address = 0x4000_0308
	// 					PBSRR		@ address = 0x4000_0318
	// 					PCSRR		@ address = 0x4000_0328
	// 					PDSRR		@ address = 0x4000_0338
	// 					PESRR		@ address = 0x4000_0348
	// 					PFSRR		@ address = 0x4000_0358
	//
	//---------------------------------------------------------------------------
	reg_val = (0x0001UL<<(pin_no+16)); 
	Px->SRR = reg_val;


}

/**
************************************************************************************
* @ Name : GPIO_OutputLow 
*
* @ Parameter
*		- gpio		= GPIO_A ~ GPIO_F
*		- pin_no		= PIN_0 ~ PIN_15
*
*
************************************************************************************
*/
uint16_t GPIO_ReadValue (GPIO_Type * const Px)
{
	//---------------------------------------------------------------------------
	// Input Data Register
	//
	// 					PAIDR		@ address = 0x4000_0304
	// 					PBIDR		@ address = 0x4000_0314
	// 					PCIDR		@ address = 0x4000_0324
	// 					PDIDR		@ address = 0x4000_0334
	// 					PEIDR		@ address = 0x4000_0344
	// 					PFIDR		@ address = 0x4000_0354
	//
	//---------------------------------------------------------------------------
	return (Px->IDR); 
}

