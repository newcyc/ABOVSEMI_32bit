/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_flash.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
*
* @ Description 
*   ABOV Semiconductor is supplying this software for use with HART-m310
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
#include "a33g52x.h"
#include "a33g52x_flash.h"

//#define FLASH_UART_DEBUGGING
#define FLASH_GPIO_DEBUGGING


#ifdef FLASH_UART_DEBUGGING
#include "console.h"
#elif defined(FLASH_GPIO_DEBUGGING)
//#include "hal.h"
#endif 



/**
************************************************************************************
* @ Name : FLASH_Init 
*
* @ Parameter
*		- flash : FMC
*
*
************************************************************************************
*/
void FLASH_Init (FMC_Type * const flash)
{

	return; 

}




/**
************************************************************************************
* @ Name : FLASH_EraseChip 
*
* @ Parameter
*		- flash : FMC
*		- addr : CFLASH_BASE_ADDRESS, DFLASH_BASE_ADDRESS
*
* @ Return
*		0 		success
*		1 		fail 		
*
*
************************************************************************************
*/
int FLASH_EraseChip (FMC_Type * const flash, uint32_t addr)
{

	uint32_t		limit = 100000; 


	limit = FLASH_ExecuteFlashOpteration (FMC, FMCON_MASE, addr, 0, limit); 


#ifdef FLASH_UART_DEBUGGING
	cprintf("limit=%d\r\n", limit); 
#endif 

	if (limit) return (0); 		// success
	else return (1); 			// fail 


}



/**
************************************************************************************
* @ Name : FLASH_EraseSector 
*
* @ Parameter
*		- flash : FMC
*		- addr : 0x0F00_0000, 0x0F00_0400, 0x0F00_0800, ...0x0F00_7C00
*
* @ Return
*		0 		success
*		1 		fail 		
*
*
************************************************************************************
* Comments
*
*		This function erases the flasy by 1KB 
************************************************************************************
*/
int FLASH_EraseSector (FMC_Type * const flash, uint32_t addr)
{

	uint32_t		limit = 10000; 

	limit = FLASH_ExecuteFlashOpteration (FMC, FMCON_SERA, addr, 0, limit); 


#ifdef FLASH_UART_DEBUGGING
	cprintf("limit=%d\r\n", limit); 
#endif 

	if (limit) return (0); 		// success
	else return (1); 			// fail 
}




/**
************************************************************************************
* @ Name : FLASH_Erase512Page 
*
* @ Parameter
*		- flash : FMC
*		- addr : 0x0F00_0000, 0x0F00_0400, 0x0F00_0800, ...0x0F00_7C00
*
* @ Return
*		0 		success
*		1 		fail 		
*
*
************************************************************************************
* Comments
*
*		This function erases the flash by 512B 
************************************************************************************
*/
int FLASH_Erase512Page (FMC_Type * const flash, uint32_t addr)
{

	uint32_t		limit = 10000; 

	limit = FLASH_ExecuteFlashOpteration (FMC, FMCON_PAGE, addr, 0, limit); 

#ifdef FLASH_UART_DEBUGGING
	cprintf("limit=%d\r\n", limit); 
#endif 

	if (limit) return (0); 		// success
	else return (1); 			// fail 
}




/**
************************************************************************************
* @ Name : FLASH_ProgramByte 
*
* @ Parameter
*		- flash : FMC
*		- addr : 0x0F00_0000, 0x0F00_0400, 0x0F00_0800, ...0x0F00_7C00
*
* @ Return
*		0 		success
*		1 		fail 		
*
*
************************************************************************************
* Comments
*
*		If the target flash is the data flash, the size of program-unit is a byte.
************************************************************************************
*/
int FLASH_ProgramByte (FMC_Type * const flash, uint32_t addr, uint8_t data)
{

	uint32_t 		limit = 100000; 

	limit = FLASH_ExecuteFlashOpteration(FMC, FMCON_PROG, addr, data, limit); 

#ifdef FLASH_UART_DEBUGGING
	cprintf("limit=%d\r\n", limit); 
#endif 

	if (limit) return (0); 		// success
	else return (1); 			// fail 
}



/**
************************************************************************************
* @ Name : FLASH_ExecuteFlashOpteration 
*
* @ Parameter
*		- flash : FMC
*		- cmd : FMCON_MASE, "FMCON_MASE+FMCON_BBLK," FMCON_SERA, FMCON_PROG, FMCON_S4K, FMCON_PAGE
*		- addr
*		- data
*		- limit 
*
* @ return 
*		0 			fail
*		non-zero		The value is the left limit value which is decreased in FMC operation	
*
*
************************************************************************************
*/
uint32_t FLASH_ExecuteFlashOpteration (FMC_Type * const flash, uint32_t cmd, uint32_t addr, uint32_t data, uint32_t limit)
{

	volatile uint32_t	delay; 
	uint32_t			status; 
	uint32_t			code_data_sel, busy_check; 


	//-----------------------------------------------------------------------------------
	// examine address
	//
	//						data flash address 		0x0F00_0000 ~ 0x0F00_7FFF
	//-----------------------------------------------------------------------------------
	if (addr < (CFLASH_BASE_ADDRESS+CFLASH_SIZE))
	{
		code_data_sel = FMCON_CSEL; 
		busy_check = FMCON_CTBIT; 
	}
	else if (addr >= DFLASH_BASE_ADDRESS || addr < (DFLASH_BASE_ADDRESS+DFLASH_SIZE))
	{
		code_data_sel = FMCON_DSEL; 
		busy_check = FMCON_DTBIT; 

		addr &= DFLASH_ADDR_OFFSET_MASK; 			
		
	}
	else
	{
		return (0); 		/* FAIL */
	}




	//-----------------------------------------------------------------------------------
	// CSEL OR DSEL
	//-----------------------------------------------------------------------------------	
	FMC->CON = 0;
	FMC->CON = code_data_sel;
	

	//-----------------------------------------------------------------------------------
	// address 
	//-----------------------------------------------------------------------------------
	FMC->AR = addr;
	

	//-----------------------------------------------------------------------------------
	// CS, command 
	//-----------------------------------------------------------------------------------
	FMC->CON = (code_data_sel|FMCON_CS|cmd);


	//-----------------------------------------------------------------------------------
	// data 
	//-----------------------------------------------------------------------------------
	FMC->ODR = data;


	//-----------------------------------------------------------------------------------
	// AE 
	//-----------------------------------------------------------------------------------	
	FMC->CON = (code_data_sel|FMCON_CS|cmd|FMCON_AE);


	//-----------------------------------------------------------------------------------
	// NVSTR 
	//-----------------------------------------------------------------------------------	
	FMC->CON = (code_data_sel|FMCON_CS|cmd|FMCON_AE|FMCON_NVSTR);


	//-----------------------------------------------------------------------------------
	// time delay 
	//-----------------------------------------------------------------------------------
	for (delay=0; delay<10; delay++); 


	//-----------------------------------------------------------------------------------
	// ~command 
	//-----------------------------------------------------------------------------------
	FMC->CON = (code_data_sel|FMCON_CS|FMCON_AE|FMCON_NVSTR);



	//-----------------------------------------------------------------------------------
	// busy check 
	//-----------------------------------------------------------------------------------	
	while (--limit)
	{
		status = FMC->CON;
		if (!(status & busy_check)) break; 
	}



	//-----------------------------------------------------------------------------------
	// ~AE, ~NVSTR
	//-----------------------------------------------------------------------------------
	FMC->CON = (code_data_sel|FMCON_CS);


	//-----------------------------------------------------------------------------------
	// ~CSEL OR ~DSEL, ~CS
	//-----------------------------------------------------------------------------------
	FMC->CON = 0;




	return (limit); 
}







/**
************************************************************************************
* @ Name : FLASH_EnableProtection 
*
* @ Parameter
*		- flash : FMC
*		- mask : FMPROTECT_DP3
*				FMPROTECT_DP2
*				FMPROTECT_DP1
*				FMPROTECT_DP0
*				FMPROTECT_DPx_ALL
*
* @ return 
*		0 			fail
*		non-zero		The value is the left limit value which is decreased in FMC operation	
*
*
************************************************************************************
*/
void FLASH_EnableProtection (FMC_Type * const flash, uint32_t mask)
{

	uint32_t		reg_val; 

	reg_val = FMC->PROTECT;
	reg_val &= ~(mask & FMPROTECT_DP_MASK); 
	FMC->PROTECT = reg_val;


}



/**
************************************************************************************
* @ Name : FLASH_DisableProtection 
*
* @ Parameter
*		- flash : FMC
*		- mask : FMPROTECT_DP3
*				FMPROTECT_DP2
*				FMPROTECT_DP1
*				FMPROTECT_DP0
*				FMPROTECT_DPx_ALL
*
* @ return 
*		0 			fail
*		non-zero		The value is the left limit value which is decreased in FMC operation	
*
*
************************************************************************************
*/
void FLASH_DisableProtection (FMC_Type * const flash, uint32_t mask)
{

	uint32_t		reg_val; 

	reg_val = FMC->PROTECT;
	reg_val |= (mask & FMPROTECT_DP_MASK); 
	FMC->PROTECT = reg_val;

}



/**
************************************************************************************
* @ Name : FLASH_Self_EraseSector 
*
* @ Parameter
*		- flash : FMC
*		- addr : 0, 0x400, ...
*
*
* @ return 
*		0 			fail
*		non-zero		The value is the left limit value which is decreased in FMC operation	
*
*
************************************************************************************
*
* @ Location of the operating code : Code Flash
*
************************************************************************************
*/
int FLASH_Self_EraseSector (FMC_Type * const flash, uint32_t addr)
{

	volatile int		delay; 


	FMC->CON = (FMCON_SELF|0x06000000UL|FMCON_SERA);

	*((volatile uint32_t *) addr) = 0; 

	for (delay=0; delay<1000; delay++); 

	return (0); 

}



/**
************************************************************************************
* @ Name : FLASH_Self_ProgramWORD
*
* @ Parameter
*		- flash : FMC
*		- addr : 0, 0x400, ...
*		- data 
*
*
* @ return 
*		0 			fail
*		non-zero		The value is the left limit value which is decreased in FMC operation	
*
*
************************************************************************************
*
* @ Location of the operating code : Code Flash
*
************************************************************************************
*/
int FLASH_Self_ProgramWORD (FMC_Type * const flash, uint32_t addr, uint32_t data)
{

	volatile int		delay; 


	FMC->CON = (FMCON_SELF|0x06000000UL|FMCON_PROG);

	*((volatile uint32_t *) addr) = data; 

	for (delay=0; delay<1000; delay++); 

	return (0); 

}


