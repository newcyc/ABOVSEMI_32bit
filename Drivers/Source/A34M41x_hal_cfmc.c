/**********************************************************************
* @file		A34M41x_cfmc.c
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
#include "A34M41x_hal_cfmc.h"
#include "A34M41x_hal_libcfg.h"



/* FMC ------------------------------------------------------------------------------ */

/**********************************************************************
 * @brief		Code Flash Memory Control Register
 * @param[in]	CacheSRC
 *					- CFMC_CONF_DATA_CACHE_ON : (1<<9)
 *					- CFMC_CONF_INST_CACHE_ON : (1<<8)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_CacheCmd(uint32_t CacheSRC, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->CONF;
	
	reg_val &= ~(CacheSRC&0x30300);
	
	if (NewState == ENABLE)
	{
		reg_val |= (CacheSRC&0x30300);
	}
	
	CFMC->CONF = reg_val;
}

/**********************************************************************
 * @brief		Code Flash Memory Boot Block Lock
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_BootBlockCmd(FunctionalState NewState)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->CONF;
	
	reg_val &= ~(0x01<<24);
	
	if (NewState == ENABLE)
	{
		reg_val |= (0x01<<24);
	}
	
	CFMC->CONF = reg_val;
}

/**********************************************************************
 * @brief		Code Flash Memory Wait Config
 * @param[in]	Memory Wait Value
 *					- 0~15
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_WaitCmd(uint8_t WaitValue)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->CONF;
	
	reg_val &= ~(0x0F<<0);
	
	reg_val |= (WaitValue&0x0F);
	
	CFMC->CONF = reg_val;
}

/**********************************************************************
 * @brief		Code Flash Memory Access Key Register
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_FlashAccessCmd(FunctionalState NewState)
{
	if (NewState == ENABLE)
	{
		CFMC->FLSKEY = CFMC_FLSKEY_ACCESS_KEY1;
		
		CFMC->FLSKEY = CFMC_FLSKEY_ACCESS_KEY2;
		
		CFMC->FLSKEY = CFMC_FLSKEY_ACCESS_KEY3;
	}
	
	else if (NewState == DISABLE)
	{
		//CFMC->FLSKEY = CFMC_FLSKEY_ACCESS_RST;	//todo : Data Sheet와 설명이 다름
		CFMC->CTRL |= CFMC_CTRL_FLOCK;	//Flash Lock
	}
}

/**********************************************************************
 * @brief		Code Flash OTP Access Key Register
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_OTPAccessCmd(FunctionalState NewState)
{
	if (NewState == ENABLE)
	{
		CFMC->OTPKEY = CFMC_OTPKEY_ACCESS_KEY1;
		
		CFMC->OTPKEY = CFMC_OTPKEY_ACCESS_KEY2;
		
		CFMC->OTPKEY = CFMC_OTPKEY_ACCESS_KEY3;
	}
	
	else if (NewState == DISABLE)
	{

		//CFMC->OTPKEY = CFMC_OTPKEY_ACCESS_RST;
		CFMC->CTRL |= CFMC_CTRL_FLOCK;	//Flash Lock		
	}
}


/**********************************************************************
 * @brief		Code Flash Memory Protection Register
 * @param[in]	Protection Area Bit
 *					UnProtection Area
*					- CFMC_FLSPROT_UNPROT_0x0007FE00_0x0007FFFF	: (0x01UL<<31)
*					- CFMC_FLSPROT_UNPROT_0x0007FC00_0x0007FDFF	: (0x01UL<<30)
*					- CFMC_FLSPROT_UNPROT_0x0007FA00_0x0007FBFF	: (0x01UL<<29)
*					- CFMC_FLSPROT_UNPROT_0x0007F800_0x0007F9FF	: (0x01UL<<28)
*					- CFMC_FLSPROT_UNPROT_0x0007F600_0x0007F7FF	: (0x01UL<<27)
*					- CFMC_FLSPROT_UNPROT_0x0007F400_0x0007F5FF	: (0x01UL<<26)
*					- CFMC_FLSPROT_UNPROT_0x0007F200_0x0007F3FF	: (0x01UL<<25)
*					- CFMC_FLSPROT_UNPROT_0x0007F000_0x0007F1FF	: (0x01UL<<24)
 *					Protection Area
*					- CFMC_FLSPROT_PROT_0x00078000_0x0007FFFF		: (0x01UL<<15)
*					- CFMC_FLSPROT_PROT_0x00070000_0x00077FFF		: (0x01UL<<14)
*					- CFMC_FLSPROT_PROT_0x00068000_0x0006FFFF		: (0x01UL<<13)
*					- CFMC_FLSPROT_PROT_0x00060000_0x00067FFF		: (0x01UL<<12)
*					- CFMC_FLSPROT_PROT_0x00058000_0x0005FFFF		: (0x01UL<<11)
*					- CFMC_FLSPROT_PROT_0x00050000_0x00057FFF		: (0x01UL<<10)
*					- CFMC_FLSPROT_PROT_0x00048000_0x0004FFFF		: (0x01UL<<9)
*					- CFMC_FLSPROT_PROT_0x00040000_0x00047FFF		: (0x01UL<<8)
*					- CFMC_FLSPROT_PROT_0x00038000_0x0003FFFF		: (0x01UL<<7)
*					- CFMC_FLSPROT_PROT_0x00030000_0x00037FFF		: (0x01UL<<6)
*					- CFMC_FLSPROT_PROT_0x00028000_0x0002FFFF		: (0x01UL<<5)
*					- CFMC_FLSPROT_PROT_0x00020000_0x00027FFF		: (0x01UL<<4)
*					- CFMC_FLSPROT_PROT_0x00018000_0x0001FFFF		: (0x01UL<<3)
*					- CFMC_FLSPROT_PROT_0x00010000_0x00017FFF		: (0x01UL<<2)
*					- CFMC_FLSPROT_PROT_0x00008000_0x0000FFFF		: (0x01UL<<1)
 *					- CFMC_FLSPROT_PROT_0x00000000_0x00007FFF		: (0x01UL<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_FlashPROTCmd(uint32_t ProtectionBit, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->FLSPROT;
	
	reg_val &= ~(ProtectionBit);
	
	if (NewState == ENABLE)
	{
		reg_val |= (ProtectionBit);
	}
	
	CFMC->FLSPROT = reg_val;
}


/**********************************************************************
 * @brief		Code Flash Memory Read Protection Register
 * @param[in]	Read Protection Area Bit
*					- CFMC_READPROT_RPROT_UNPROTECT	: (0xFFUL<<0)
*					- CFMC_READPROT_RPROT_LEVEL1		: (0x39UL<<0)
*					- CFMC_READPROT_RPROT_LEVEL2		: (0x00UL<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_FlashRPROTCmd(uint32_t ReadProtectionBit, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->READPROT;
	
	reg_val &= ~(CFMC_READPROT_RPROT_UNPROTECT);
	
	if (NewState == ENABLE)
	{
		reg_val |= (ReadProtectionBit);
	}
	
	CFMC->READPROT = reg_val;
}



/**********************************************************************
 * @brief		Code Flash OTP Protection Register
 * @param[in]	OTP Protection Area Bit
*					- CFMC_OTPPROT_PROT_0x0F000400_0x0F0005FF	: (0x01UL<<2)
*					- CFMC_OTPPROT_PROT_0x0F000200_0x0F0003FF	: (0x01UL<<1)
*					- CFMC_OTPPROT_PROT_0x0F000000_0x0F0001FF	: (0x01UL<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_OTPPROTCmd(uint32_t ProtectionBit, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->OTPPROT;
	
	reg_val &= ~(ProtectionBit);
	
	if (NewState == ENABLE)
	{
		reg_val |= (ProtectionBit);
	}
	
	CFMC->OTPPROT = reg_val;
}


/**********************************************************************
 * @brief		Code Flash Lock Control Register
 * @param[in]	Lock Selection Bit
 *					- CFMC_CTRL_FLOCK	: (1<<31)
 *					- CFMC_CTRL_OLOCK	: (1<<30)
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_LockCmd(uint32_t LockBit)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->CTRL;
	
	reg_val |= (LockBit&0xC0000000UL);
	
	CFMC->CTRL = reg_val;
}


/**********************************************************************
 * @brief		Code Flash Write Done Interrupt Config
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_InterruptCmd(FunctionalState NewStatus)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->CTRL;
	
	reg_val &= ~CFMC_CTRL_WDIEN;
	
	if (NewStatus == ENABLE)
	{
		reg_val |= (CFMC_CTRL_WDIEN);
		
		NVIC_SetPriority(CFMC_IRQn, 7);
		NVIC_EnableIRQ(CFMC_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(CFMC_IRQn);
	}
	
	CFMC->CTRL = reg_val;
}



/**********************************************************************
 * @brief		Code Flash Access Status Register
 * @param[in]	None
 * @return		Access Status Value
 *
 **********************************************************************/
uint32_t HAL_CFMC_GetAccessStatus(void)
{
	return (CFMC->STAT);
}


/**********************************************************************
 * @brief		Code Flash Access Status Clear
 * @param[in]	Clear Flag
 *					- CFMC_STAT_RPERR	: (1<<21)
 *					- CFMC_STAT_WSERR	: (1<<20)
 *					- CFMC_STAT_OPERR	: (1<<19)
 *					- CFMC_STAT_FPERR	: (1<<18)
 *					- CFMC_STAT_OLERR	: (1<<17)
 *					- CFMC_STAT_FLERR	: (1<<16)
 *					- CFMC_STAT_CDONE	: (1<<9)
 *					- CFMC_STAT_WDONE	: (1<<8)
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_ClearAccessStatus(uint32_t ClearSRC)
{
	CFMC->STAT = ClearSRC;
}


/**********************************************************************
 * @brief		Code Flash Checksum Control Register
 * @param[in]	Control bit
 *					- CFMC_CHKCTRL_DATA_RST	: (1<<16)
 *					- CFMC_CHKCTRL_INTR_ON	: (1<<8)
 *					- CFMC_CHKCTRL_BURST_ON	: (1<<1)
 *					- CFMC_CHKCTRL_BACKGROUND_ON	: (1<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 *
 **********************************************************************/
void HAL_CFMC_FlashChecksumCmd(uint32_t ControlBit, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	reg_val = CFMC->CHKCTRL;
	
	reg_val &= ~(ControlBit);
	
	if (NewState == ENABLE)
	{
		reg_val |= (ControlBit);
	}
	
	CFMC->CHKCTRL = reg_val;
}


/**********************************************************************
 * @brief		Code Flash Checksum Data output Register
 * @param[in]	
 *
 * @return		Checksum Data
 *
 **********************************************************************/
uint32_t HAL_CFMC_GetChecksumData(void)
{
	return (CFMC->CHKDOUT);
}


/**********************************************************************
 * @brief		Get Code Flash Checksum Config Register
 * @param[in]	
 *
 * @return		Config Data
 *
 **********************************************************************/
uint32_t HAL_CFMC_GetChecksumCmd(void)
{
	return (CFMC->CHKCTRL);
}


/**********************************************************************
 * @brief		Code Flash Checksum Address Register
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
void HAL_CFMC_ChecksumAddrCmd(uint32_t StartAddress, uint32_t EndAddress)
{
	CFMC->CHKSADDR = (StartAddress<<8);
	
	CFMC->CHKEADDR = (EndAddress<<8);
}

void HAL_CFMC_acc_status_check (void){
	if((HAL_CFMC_GetAccessStatus()&CFMC_STAT_WDONE) == CFMC_STAT_WDONE) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		CFMC->STAT |= (1<<8);
	}

	if((HAL_CFMC_GetAccessStatus()&CFMC_STAT_CDONE) == CFMC_STAT_CDONE) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		CFMC->STAT |= (1<<9);
	}

	if((HAL_CFMC_GetAccessStatus()&CFMC_STAT_FLERR) == CFMC_STAT_FLERR) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		CFMC->STAT |= (1<<16);
		// TODO: action for error
	}
	if((HAL_CFMC_GetAccessStatus()&CFMC_STAT_OLERR) == CFMC_STAT_OLERR) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		CFMC->STAT |= (1<<17);
	}
	if((HAL_CFMC_GetAccessStatus()&CFMC_STAT_FPERR) == CFMC_STAT_FPERR) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		CFMC->STAT |= (1<<18);
		// TODO: action for error
	}
	if((HAL_CFMC_GetAccessStatus()&CFMC_STAT_OPERR) == CFMC_STAT_OPERR) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		CFMC->STAT |= (1<<19);
		// TODO: action for error
	}
	if((HAL_CFMC_GetAccessStatus()&CFMC_STAT_WSERR) == CFMC_STAT_WSERR) {
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP();

		CFMC->STAT |= (1<<20);
		// TODO: action for error
	}
	__NOP();
}


/**
********************************************************************************************************
* @ Name : HAL_CFMC_Erase
*
* @ Parameters
*		Ersmod			Erase mode select
*						- CFMC_CTRL_CERS		(1UL<<4)
*						- CFMC_CTRL_S4KERS		(1UL<<3)
*						- CFMC_CTRL_S1KERS		(1UL<<2)
*						- CFMC_CTRL_PERS		(1UL<<1)
*		addr				start address of flash memory to be erase (in terms of byte)
*
********************************************************************************************************
*/
void HAL_CFMC_Erase(uint32_t Ersmod, unsigned long addr)
{
	uint32_t		reg_val;
	
	__NOP();	__NOP();	__NOP();	__NOP();
	
	CFMC_WRITE_BUSY_POLLING;
	
	reg_val = CFMC->CTRL;
	
	reg_val &= ~(Ersmod);

	reg_val |= (Ersmod);
	
	CFMC->CTRL = reg_val;
	
	MIO32(addr) = 0x0;
#ifdef FLASH_ACC_INT_MODE
	while(!flash_acc_write_done);
	flash_acc_write_done = 0;
#else
	CFMC_WRITE_BUSY_POLLING;
#endif
	
	CFMC->CTRL &= ~(Ersmod);
	
	HAL_CFMC_acc_status_check();
	
}



/**
********************************************************************************************************
* @ Name : HAL_CFMC_ProgramPage
*
*
* @ Parameters
*		addr				start address of flash memory to be written (in terms of byte)
*		size				write size (in terms of byte)
*		buf				start address of buffer 
*
*
*
********************************************************************************************************
*/
void HAL_CFMC_ProgramPage (unsigned long addr, unsigned long size, unsigned char *buf)
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

	CFMC_WRITE_BUSY_POLLING;
	
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	
	CFMC->CTRL |= (1<<0);
	

	for (i=0; i<size; i++) {
		MIO32(addr+(i*4)) = *ptr++;
	
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	CFMC_WRITE_BUSY_POLLING;
		
	}

	__NOP();
	__NOP();
	__NOP();
	__NOP();

	CFMC->CTRL &= ~(1<<0);
	
	HAL_CFMC_acc_status_check();

}


/**
********************************************************************************************************
* @ Name : HAL_CFMC_Verify
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
unsigned long HAL_CFMC_Verify (unsigned long addr, unsigned long size, unsigned char *buf)
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

