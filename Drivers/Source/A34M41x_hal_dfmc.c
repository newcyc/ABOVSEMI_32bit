/**********************************************************************
* @file		A34M41x_dfmc.c
* @brief	Contains all functions support for fmc firmware library
* 			on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_dfmc.h"
#include "A34M41x_hal_libcfg.h"



/* FMC ------------------------------------------------------------------------------ */

/**********************************************************************
 * @brief		Data Flash Memory Wait Config
 * @param[in]	Memory Wait Value
 *					- 0~15
 * @return		None
 *
 **********************************************************************/
void HAL_DFMC_WaitCmd(uint8_t WaitValue)
{
	uint32_t		reg_val;
	
	reg_val = DFMC->CONF;
	
	reg_val &= ~(0x0F<<0);
	
	reg_val |= (WaitValue&0x0F);
	
	DFMC->CONF = reg_val;
}

/**********************************************************************
 * @brief		Data Flash Memory Access Key Register
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_DFMC_FlashAccessCmd(FunctionalState NewState)
{
	if (NewState == ENABLE)
	{
		DFMC->FLSKEY = DFMC_FLSKEY_ACCESS_KEY1;
		
		DFMC->FLSKEY = DFMC_FLSKEY_ACCESS_KEY2;
		
		DFMC->FLSKEY = DFMC_FLSKEY_ACCESS_KEY3;
	}

	else // if (NewState == CSP_ENABLE)
	{
		//DFMC->FLSKEY = DFMC_FLSKEY_ACCESS_RST;	//todo : Data Sheet와 설명이 다름
		DFMC->CTRL |= DFMC_CTRL_FLOCK;	//Flash Lock
}
}



/**********************************************************************
 * @brief		Data Flash Memory Protection Register
 * @param[in]	Protection Area Bit
 *					UnProtection Area
*					- DFMC_FLSPROT_UNPROT_0x0E007E00_0x0E007FFF	: (0x01UL<<31)
*					- DFMC_FLSPROT_UNPROT_0x0E007C00_0x0E007DFF	: (0x01UL<<30)
*					- DFMC_FLSPROT_UNPROT_0x0E007A00_0x0E007BFF	: (0x01UL<<29)
*					- DFMC_FLSPROT_UNPROT_0x0E007800_0x0E0079FF	: (0x01UL<<28)
*					- DFMC_FLSPROT_UNPROT_0x0E007600_0x0E0077FF	: (0x01UL<<27)
*					- DFMC_FLSPROT_UNPROT_0x0E007400_0x0E0075FF	: (0x01UL<<26)
*					- DFMC_FLSPROT_UNPROT_0x0E007200_0x0E0073FF	: (0x01UL<<25)
*					- DFMC_FLSPROT_UNPROT_0x0E007000_0x0E0071FF	: (0x01UL<<24)
 *					Protection Area
*					- DFMC_FLSPROT_PROT_0x0E007800_0x0E007FFF		: (0x01UL<<15)
*					- DFMC_FLSPROT_PROT_0x0E007000_0x0E0077FF		: (0x01UL<<14)
*					- DFMC_FLSPROT_PROT_0x0E006800_0x0E006FFF		: (0x01UL<<13)
*					- DFMC_FLSPROT_PROT_0x0E006000_0x0E0067FF		: (0x01UL<<12)
*					- DFMC_FLSPROT_PROT_0x0E005800_0x0E005FFF		: (0x01UL<<11)
*					- DFMC_FLSPROT_PROT_0x0E005000_0x0E0057FF		: (0x01UL<<10)
*					- DFMC_FLSPROT_PROT_0x0E004800_0x0E004FFF		: (0x01UL<<9)
*					- DFMC_FLSPROT_PROT_0x0E004000_0x0E0047FF		: (0x01UL<<8)
*					- DFMC_FLSPROT_PROT_0x0E003800_0x0E003FFF		: (0x01UL<<7)
*					- CFMC_FLSPROT_PROT_0x00030000_0x00037FFF		: (0x01UL<<6)
*					- DFMC_FLSPROT_PROT_0x0E002800_0x0E002FFF		: (0x01UL<<5)
*					- DFMC_FLSPROT_PROT_0x0E002000_0x0E0027FF		: (0x01UL<<4)
*					- DFMC_FLSPROT_PROT_0x0E001800_0x0E001FFF		: (0x01UL<<3)
*					- DFMC_FLSPROT_PROT_0x0E001000_0x0E0017FF		: (0x01UL<<2)
*					- DFMC_FLSPROT_PROT_0x0E000800_0x0E000FFF		: (0x01UL<<1)
 *					- DFMC_FLSPROT_PROT_0x0E000000_0x0E0007FF		: (0x01UL<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_DFMC_FlashPROTCmd(uint32_t ProtectionBit, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	reg_val = DFMC->FLSPROT;
	
	reg_val &= ~(ProtectionBit);
	
	if (NewState == ENABLE)
	{
		reg_val |= (ProtectionBit);
	}
	
	DFMC->FLSPROT = reg_val;
}



/**********************************************************************
 * @brief		Data Flash Lock Control Register
 * @param[in]	Lock Selection Bit
 *					- DFMC_CTRL_FLOCK	: (1<<31)
 * @return		None
 *
 **********************************************************************/
void HAL_DFMC_LockCmd(uint32_t LockBit)
{
	uint32_t		reg_val;
	
	reg_val = DFMC->CTRL;
	
	reg_val |= (LockBit&0x80000000UL);
	
	DFMC->CTRL = reg_val;
}



/**********************************************************************
 * @brief		Data Flash Interrupt Config
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_DFMC_InterruptCmd(FunctionalState NewStatus)
{
	uint32_t		reg_val;
	
	reg_val = DFMC->CTRL;
	
	reg_val &= ~DFMC_CTRL_WDIEN;
	
	if (NewStatus == ENABLE)
	{
		reg_val |= (DFMC_CTRL_WDIEN);
		
		NVIC_SetPriority(DFMC_IRQn, 7);
		NVIC_EnableIRQ(DFMC_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(DFMC_IRQn);
	}
	
	DFMC->CTRL = reg_val;
}


/**********************************************************************
 * @brief		Data Flash Access Status Register
 * @param[in]	None
 *
 * @return		Access Status Value
 *
 **********************************************************************/
uint32_t HAL_DFMC_GetAccessStatus(void)
{
	return (DFMC->STAT);
}


/**********************************************************************
 * @brief		Data Flash Access Status Clear
 * @param[in]	Clear Flag
 *					- DFMC_STAT_RPERR	: (1<<21)
 *					- DFMC_STAT_WSERR	: (1<<20)
 *					- DFMC_STAT_FPERR	: (1<<18)
 *					- DFMC_STAT_FLERR	: (1<<16)
 *					- DFMC_STAT_CDONE	: (1<<9)
 *					- DFMC_STAT_WDONE	: (1<<8)
 * @return		None
 *
 **********************************************************************/
void HAL_DFMC_ClearAccessStatus(uint32_t ClearSRC)
{
	DFMC->STAT = ClearSRC;
}


/**********************************************************************
 * @brief		Data Flash Checksum Control Register
 * @param[in]	Control bit
 *					- DFMC_CHKCTRL_DATA_RST	: (1<<16)
 *					- DFMC_CHKCTRL_INTR_ON	: (1<<8)
 *					- DFMC_CHKCTRL_BURST_ON	: (1<<1)
 *					- DFMC_CHKCTRL_BACKGROUND_ON	: (1<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_DFMC_FlashChecksumCmd(uint32_t ControlBit, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	reg_val = DFMC->CHKCTRL;
	
	reg_val &= ~(ControlBit);
	
	if (NewState == ENABLE)
	{
		reg_val |= (ControlBit);
	}
	
	DFMC->CHKCTRL = reg_val;
}


/**********************************************************************
 * @brief		Data Flash Checksum Data output Register
 * @param[in]	
 *
 * @return		Checksum Data
 *
 **********************************************************************/
uint32_t HAL_DFMC_GetChecksumData(void)
{
	return (DFMC->CHKDOUT);
}

/**********************************************************************
 * @brief		Get Data Flash Checksum Config Register
 * @param[in]	
 *
 * @return		Config Data
 *
 **********************************************************************/
uint32_t HAL_DFMC_GetChecksumCmd(void)
{
	return (DFMC->CHKCTRL);
}

/**********************************************************************
 * @brief		Data Flash Checksum Address Register
 * @param[in]	StartAddress
 * 				StartAddress = [31:6] bit
 * 				Fixed Address = [5:0] bit - '0'
 *
 *				EndAddress
 * 				EndAddress = [31:6] bit
 *				Fixed Address = [5:0] bit - '1'
 *
 * 				If Start address = 0, End Address = 0
 * 				   0x0000_0000 ~ 0x0000_003F
 *
 * 				Checksum Area Calculation
 *				   Start Address ~ (((End Address+1)*64)-1), (Start Address <= End Address)
 *
 * @return		None
 *
 **********************************************************************/
void HAL_DFMC_ChecksumAddrCmd(uint32_t StartAddress, uint32_t EndAddress)
{
	DFMC->CHKSADDR = (StartAddress<<6);
	
	DFMC->CHKEADDR = (EndAddress<<6);
}

// Flash Status Check
void HAL_DFMC_acc_status_check (void){
	if((HAL_DFMC_GetAccessStatus()&DFMC_STAT_WDONE) == DFMC_STAT_WDONE) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		DFMC->STAT |= (1<<8);
	}

	if((HAL_DFMC_GetAccessStatus()&DFMC_STAT_CDONE) == DFMC_STAT_CDONE) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		DFMC->STAT |= (1<<9);
	}

	if((HAL_DFMC_GetAccessStatus()&DFMC_STAT_FLERR) == DFMC_STAT_FLERR) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		DFMC->STAT |= (1<<16);
		// TODO: action for error
	}
	if((HAL_DFMC_GetAccessStatus()&DFMC_STAT_FPERR) == DFMC_STAT_FPERR) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		DFMC->STAT |= (1<<18);
		// TODO: action for error
	}
	if((HAL_DFMC_GetAccessStatus()&DFMC_STAT_WSERR) == DFMC_STAT_WSERR) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		DFMC->STAT |= (1<<20);
		// TODO: action for error
	}
	__NOP();
}

/**
********************************************************************************************************
* @ Name : HAL_DFMC_Erase
*
* @ Parameters
*		Ersmod			Erase mode select
*						- DFMC_CTRL_CERS		(1UL<<4)
*						- DFMC_CTRL_S4KERS		(1UL<<3)
*						- DFMC_CTRL_S1KERS		(1UL<<2)
*						- DFMC_CTRL_PERS		(1UL<<1)
*		addr				start address of flash memory to be erase (in terms of byte)
*
********************************************************************************************************
*/
void HAL_DFMC_Erase(uint32_t Ersmod, unsigned long addr)
{
	uint32_t	reg_val;
	
	__NOP();	__NOP();	__NOP();	__NOP();
	
	DFMC_WRITE_BUSY_POLLING;
	
	reg_val = DFMC->CTRL;
	
	reg_val &= ~(Ersmod);

	reg_val |= (Ersmod);
	
	DFMC->CTRL = reg_val;
	
	
	MIO32(addr) = addr;

	DFMC_WRITE_BUSY_POLLING;
	
	DFMC->CTRL &= ~(Ersmod);

	HAL_DFMC_acc_status_check();
	
}



/**
********************************************************************************************************
* @ Name : HAL_DFMC_ProgramPage
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
void HAL_DFMC_ProgramPage (unsigned long addr, unsigned long size, unsigned char *buf)
{
	unsigned long *ptr;
	int 					i;


	ptr = (unsigned long*)buf;

	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	
	__NOP();
	__NOP();
	__NOP();

	DFMC_WRITE_BUSY_POLLING;
	
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	
	DFMC->CTRL |= (1<<0);
	

	for (i=0; i<size; i++) {
		MIO32(addr+(i*4)) = *ptr++;
		
#ifdef FLASH_ACC_INT_MODE
	while (!flash_acc_write_done);
	flash_acc_write_done = 0;
#else
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	DFMC_WRITE_BUSY_POLLING;
#endif	
		
	}

	__NOP();
	__NOP();
	__NOP();
	__NOP();

	DFMC->CTRL &= ~(1<<0);
	
	HAL_DFMC_acc_status_check();

}


/**
********************************************************************************************************
* @ Name : HAL_DFMC_Verify
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
unsigned long HAL_DFMC_Verify (unsigned long addr, unsigned long size, unsigned char *buf)
{
	unsigned long			i; 
	unsigned char			*p_org, *p_flash; 
	unsigned long			result; 
	volatile unsigned long	reg_val; 



	//---------------------------------------------------------------------------------
	// init variable 
	//---------------------------------------------------------------------------------
	result = addr + size; 


	
	//---------------------------------------------------------------------------------
	// verify 
	//---------------------------------------------------------------------------------
	p_org = (unsigned char *) buf; 
	p_flash = (unsigned char *) addr; 


	for (i=0; i<size; i ++)
	{
		if (*p_flash++ != *p_org++)
		{
			result = addr + i; 
			break; 
		}
	}
	
	return (result); 

}




/* --------------------------------- End Of File ------------------------------ */

