/***************************************************************************//**
* @file     flash.c
* @brief    Contains all functions support for flash on A31G22x
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

/* Includes ------------------------------------------------------------------- */
#include "flash.h"


#define	FMCFG_ENTRY				(0x78580300UL)
#define	FMCFG_EXIT				(0x78580300UL)

#define	BV_LOCKSEL				(0x01UL << 24)
#define	BV_SELFPGM				(0x01UL << 23)
#define	BV_IFEN					(0x01UL << 12)
#define	BV_BBLOCK				(0x01UL << 8)
#define	BV_MAS					(0x01UL << 7)
#define	BV_4KEN					(0x01UL << 6)
#define	BV_1KEN					(0x01UL << 5)
#define	BV_PMOD					(0x01UL << 4)
#define	BV_WADCK				(0x01UL << 3)
#define	BV_PGM					(0x01UL << 2)
#define	BV_ERS					(0x01UL << 1)
#define	BV_HVEN					(0x01UL << 0)

#define	FMMR_BUSY				(CFMC->BUSY & 0x01)

#define	FMCR_LOCKSEL_SET		(CFMC->CR |=  BV_LOCKSEL)
#define	FMCR_LOCKSEL_CLR		(CFMC->CR &=~ BV_LOCKSEL)
#define	FMCR_SELFPGM_SET		(CFMC->CR |=  BV_SELFPGM)
#define	FMCR_SELFPGM_CLR		(CFMC->CR &=~ BV_SELFPGM)
#define	FMCR_IFEN_SET			(CFMC->CR |=  BV_IFEN)
#define	FMCR_IFEN_CLR			(CFMC->CR &=~ BV_IFEN)
#define	FMCR_BBLOCK_SET			(CFMC->CR |=  BV_BBLOCK)
#define	FMCR_BBLOCK_CLR			(CFMC->CR &=~ BV_BBLOCK)
#define	FMCR_MAS_SET			(CFMC->CR |=  BV_MAS)
#define	FMCR_MAS_CLR			(CFMC->CR &=~ BV_MAS)
#define	FMCR_4KEN_SET			(CFMC->CR |=  BV_4KEN)
#define	FMCR_4KEN_CLR			(CFMC->CR &=~ BV_4KEN)
#define	FMCR_1KEN_SET			(CFMC->CR |=  BV_1KEN)
#define	FMCR_1KEN_CLR			(CFMC->CR &=~ BV_1KEN)
#define	FMCR_PMOD_SET			(CFMC->CR |=  BV_PMOD)
#define	FMCR_PMOD_CLR			(CFMC->CR &=~ BV_PMOD)
#define	FMCR_WADCK_SET			(CFMC->CR |=  BV_WADCK)
#define	FMCR_WADCK_CLR			(CFMC->CR &=~ BV_WADCK)
#define	FMCR_PGM_SET			(CFMC->CR |=  BV_PGM)
#define	FMCR_PGM_CLR			(CFMC->CR &=~ BV_PGM)
#define	FMCR_ERS_SET			(CFMC->CR |=  BV_ERS)
#define	FMCR_ERS_CLR			(CFMC->CR &=~ BV_ERS)
#define	FMCR_HVEN_SET			(CFMC->CR |=  BV_HVEN)
#define	FMCR_HVEN_CLR			(CFMC->CR &=~ BV_HVEN)


/**
********************************************************************************************************
* @ Name : Init
*
* @ Parameter
*		addr		device base address
*		clk			clock frequency (Hz)
*		func		function code (1=erase, 2=program, 3=verify)
*
* @ return value
*		0 = success
*		1 = error 
*
********************************************************************************************************
*/
void	__SET_OTP(unsigned long otp)
{
	CFMC->MR = 0x5A;
	CFMC->MR = 0xA5;
	if(otp==0) {
		CFMC->MR = 0;
	}
	else {
		CFMC->MR = (otp<<28);// | BV_OTPE;
		FMCR_IFEN_SET;
		FMCR_4KEN_CLR;
		//FMCR_1KEN_CLR;
		//FMCR_BBLOCK_SET;
	}
}

int Init (unsigned long addr, unsigned long clk, unsigned long func)
{
	__disable_irq();
	WDT->CR = (0x5A69 << 16)
            |(0x25<<10)
            |(0x1A<<4)
            |0;
	
	//---------------------------------------------------------------------------------
	// 1. ALL clock enable 
	//---------------------------------------------------------------------------------
	SCU->CSCR = (0xA507UL<<16)
            |(0x8<<12)
            |(0x8<<8)
            |(0x8<<4)   // 48MHz   
            |(0x8<<0); 
    
	SCU->SCCR = (0x570AUL<<16)
            |(0x02);   // HSIOSC enable
            
	return (0);
}


/**
********************************************************************************************************
* @ Name : UnInit
*
* @ Parameter
*		func		function code (1=erase, 2=program, 3=verify)
*
* @ return value
*		0 = success
*		1 = error 
*
********************************************************************************************************
*/
int UnInit (unsigned long func)
{
	return (0);
}

void	__WAIT(void)
{
	while(1) {
		// wait until MR_BUSYE is "0"
		if (FMMR_BUSY==0) 
			break; 
	}
}

void	__FLASH_ALL_ENABLE(void)
{
	CFMC->MR = 0x66;
	CFMC->MR = 0x99;
	CFMC->WPROT = 0x00000000;	// Remove FLASH write protection
	CFMC->MR = 0x00;
	CFMC->MR = 0x00;
}

/**
********************************************************************************************************
* @ Name : Lock EraseChip 
*
*
* @ return value
*		0 = success
*		1 = error (time-out error)
*
********************************************************************************************************
*/
int LockEraseChip (void)
{
	__FLASH_ALL_ENABLE();

	CFMC->MR = 0x81;CFMC->MR = 0x28;
	CFMC->CFG= FMCFG_ENTRY;
	CFMC->MR = 0x00;CFMC->MR = 0x00;

	CFMC->MR = 0x5A;CFMC->MR = 0xA5;

	// 1st try, erase PROT0 without bulk erase 
	CFMC->AR = 0;

	FMCR_PMOD_SET;
	FMCR_LOCKSEL_SET;
	FMCR_ERS_SET;
	FMCR_WADCK_SET; 
	
	FMCR_HVEN_SET; 
	__WAIT();					// Erase
	FMCR_HVEN_CLR;
	CFMC->BUSY=0x80; // WRBUSY flag clear
	
	FMCR_LOCKSEL_CLR;
	
//	=========================================
	CFMC->AR = 0;

	FMCR_MAS_SET;
	FMCR_WADCK_SET; 
	
	FMCR_HVEN_SET; 
	__WAIT();					// Chip Erase
	FMCR_HVEN_CLR;
	CFMC->BUSY=0x80; // WRBUSY flag clear
	
	FMCR_MAS_CLR; 
	
//	=========================================	
	CFMC->AR = 0;
	FMCR_LOCKSEL_SET;
	FMCR_ERS_SET;
	FMCR_WADCK_SET; 
	
	FMCR_HVEN_SET; 
	__WAIT();					// Erase
	FMCR_HVEN_CLR;
	CFMC->BUSY=0x80; // WRBUSY flag clear	
	
	FMCR_ERS_CLR;
	FMCR_LOCKSEL_CLR;
	CFMC->CR = 0x00000000;
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	CFMC->MR = 0x81;CFMC->MR = 0x28;	
	CFMC->CFG = FMCFG_EXIT; 
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	return (0); 									   // Success
}

/**
********************************************************************************************************
* @ Name : EraseChip 
*
*
* @ return value
*		0 = success
*		1 = error (time-out error)
*
********************************************************************************************************
*/
int EraseChip (void)
{
#if 0					//Erase Lock
	__SET_OTP(0x08);
	EraseSector(0);
	__SET_OTP(0x00);
#endif	
	
	__FLASH_ALL_ENABLE();

	CFMC->MR = 0x81;CFMC->MR = 0x28;
	CFMC->CFG= FMCFG_ENTRY;
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	CFMC->MR = 0x5A;CFMC->MR = 0xA5;

	FMCR_PMOD_SET;
	FMCR_MAS_SET;
	FMCR_ERS_SET;
	FMCR_WADCK_SET; 
	FMCR_HVEN_SET; 
	
	__WAIT();					// Chip Erase
	
	FMCR_HVEN_CLR;
	CFMC->BUSY=0x80; // WRBUSY flag clear
	FMCR_ERS_CLR;
	FMCR_MAS_CLR; 
	CFMC->CR = 0x00000000;
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	CFMC->MR = 0x81;CFMC->MR = 0x28;	
	CFMC->CFG = FMCFG_EXIT; 
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	return (0); 									   // Success
}

/**
********************************************************************************************************
* @ Name : EraseChip 
*
*
* @ return value
*		0 = success
*		1 = error (time-out error)
*
********************************************************************************************************
*/
int EraseChipBBlock (void)
{
	__FLASH_ALL_ENABLE();

	CFMC->MR = 0x5A;CFMC->MR = 0xA5;

	FMCR_PMOD_SET;
	FMCR_MAS_SET;
	FMCR_BBLOCK_SET;
	FMCR_ERS_SET;
	FMCR_WADCK_SET; 
	FMCR_HVEN_SET; 
	
	__WAIT();					// Chip Erase
	
	FMCR_HVEN_CLR;
	CFMC->BUSY=0x80; // WRBUSY flag clear
	FMCR_ERS_CLR;
	FMCR_MAS_CLR; 
	CFMC->CR = 0x00000000;
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	return (0); 									   // Success
}


/**
********************************************************************************************************
* @ Name : EraseSector
*
*
* @ Description

*
********************************************************************************************************
*/
int EraseSector (int sel, unsigned long addr)
{
	__FLASH_ALL_ENABLE();
	
	CFMC->AR = addr>>2;

	CFMC->MR = 0x81;CFMC->MR = 0x28;
	CFMC->CFG= FMCFG_ENTRY;
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
// flash mode entry 
	CFMC->MR = 0x5A;CFMC->MR = 0xA5;
	
	FMCR_PMOD_SET;
	FMCR_ERS_SET;	
	if (sel!=0){
		FMCR_IFEN_SET;   // FMCR_IFEN_CLR
	}
	FMCR_WADCK_SET;
	FMCR_HVEN_SET;
	
	__WAIT();
	
	FMCR_HVEN_CLR;
	CFMC->BUSY=0x80; // WRBUSY flag clear
	FMCR_ERS_CLR;	
	if (sel!=0){
		FMCR_IFEN_CLR;   // FMCR_IFEN_CLR
	}
	CFMC->CR = 0x00000000;
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	CFMC->MR = 0x81;CFMC->MR = 0x28;	
	CFMC->CFG = FMCFG_EXIT; 
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	return (0); 
}


/**
********************************************************************************************************
* @ Name : ProgramPage
*
*
* @ Parameters
*		addr				start address of flash memory to be written (in terms of byte)
*		size				write size (in terms of byte)
*		buf				start address of buffer 
*
*
* @ return value
*		0 = success
*		1 = error (time-out error, alignment error)
*
********************************************************************************************************
*/
int ProgramPage (int sel,unsigned long addr, unsigned long size, unsigned char *buf)
{
	int	i;
	unsigned long *ptr;
	
	//---------------------------------------------------------------------------------
	// check address and size alignment
	//---------------------------------------------------------------------------------
	if((addr & 0x03) || (size & 0x03)) {
		return (1); 
	}

	ptr = (unsigned long*)buf;

	__FLASH_ALL_ENABLE();
	
	CFMC->MR = 0x81;CFMC->MR = 0x28;
	CFMC->CFG= FMCFG_ENTRY;
	CFMC->MR = 0x00;CFMC->MR = 0x00;	

	// flash mode entry 
	CFMC->MR = 0x5A;CFMC->MR = 0xA5;

	FMCR_PMOD_SET;
	if (sel!=0){
		FMCR_IFEN_SET;   // FMCR_IFEN_CLR
	}
	CFMC->AR = addr>>2;
	FMCR_PGM_SET;

	for(i=0;i<size;i+=4) {
		CFMC->DR = *ptr++;
		FMCR_WADCK_SET;
		FMCR_HVEN_SET;
		
		__WAIT();
		
		FMCR_HVEN_CLR;
		CFMC->BUSY=0x80; // WRBUSY flag clear
	}
	
	FMCR_PGM_CLR;
	if (sel!=0){
		FMCR_IFEN_CLR;   // FMCR_IFEN_CLR
	}	
	CFMC->CR = 0x00000000;
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	CFMC->MR = 0x81;CFMC->MR = 0x28;	
	CFMC->CFG = FMCFG_EXIT; 
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	return 0;
}



/**
********************************************************************************************************
* @ Name : Verify
*
*
* @ Parameters
*		addr				start address of flash memory to be written (in terms of byte)
*		size				write size (in terms of byte)
*		buf				start address of buffer 
*
*
* @ return value
*		(addr+size)		success
*		other value		error 
*
********************************************************************************************************
*/
unsigned long Verify (unsigned long addr, unsigned long size, unsigned char *buf)
{
	unsigned long	i;
	unsigned long	*src;
	unsigned long	*dst;
	unsigned long	result; 

	//---------------------------------------------------------------------------------
	// check address and size alignment
	//---------------------------------------------------------------------------------
	if((addr & 0x03) || (size & 0x03)) {
		return (1); 
	}

	//---------------------------------------------------------------------------------
	// Verify
	//---------------------------------------------------------------------------------
	result = addr + size; 

	CFMC->MR = 0x81;CFMC->MR = 0x28;
	CFMC->CFG= FMCFG_ENTRY;
	CFMC->MR = 0x00;CFMC->MR = 0x00;
	
	CFMC->MR = 0x5A;
	CFMC->MR = 0xA5;

//	FMCR_PVER_SET;					//PGM-level Verify

	src = (unsigned long *)buf;
	dst = (unsigned long *)addr;
	for( i=0; i<size; i+=4) {
		if( *src++ != *dst++ ) {
			result = addr + i;
			break;
		}
	}
//	FMCR_PVER_CLR;
//	FMCR_EVER_SET;					//ERASE-level Verify

	src = (unsigned long *)buf;
	dst = (unsigned long *)addr;
	for( i=0; i<size; i+=4) {
		if( *src++ != *dst++ ) {
			result = addr + i;
			break;
		}
	}
//	FMCR_EVER_CLR;

	CFMC->CR = 0x00000000;
	
	CFMC->MR = 0x81;CFMC->MR = 0x28;		
	CFMC->CFG = FMCFG_EXIT; 
	CFMC->MR = 0x00;CFMC->MR = 0x00;

	return (result);
}

/* --------------------------------- End Of File ------------------------------ */
