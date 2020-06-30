/**********************************************************************
* @file		A34M41x_cfmc.h
* @brief	Contains all macro definitions and function prototypes
* 			support for fmc firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_CFMC_H_
#define _A34M41x_CFMC_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CFMC_WRITE_BUSY_POLLING  while(((CFMC->STAT)&CFMC_STAT_WBUSY)==CFMC_STAT_WBUSY)

#define MIO32(addr)  (*(volatile unsigned int *)(addr))
	
// Flash Access Key
#define CFMC_FLSKEY_ACCESS_KEY1						0x01234567UL
#define CFMC_FLSKEY_ACCESS_KEY2						0x12345678UL
#define CFMC_FLSKEY_ACCESS_KEY3						0x23456789UL

#define CFMC_FLSKEY_ACCESS_RST							0x00000000UL

// OTP Access Key
#define CFMC_OTPKEY_ACCESS_KEY1						0x3456789AUL
#define CFMC_OTPKEY_ACCESS_KEY2						0x456789ABUL
#define CFMC_OTPKEY_ACCESS_KEY3						0x56789ABCUL

#define CFMC_OTPKEY_ACCESS_RST							0x00000000UL



// FMC CONF Macro
#define CFMC_CONF_DATA_CACHE_RST						(1UL<<17)
#define CFMC_CONF_INST_CACHE_RST						(1UL<<16)

#define CFMC_CONF_DATA_CACHE_ON						(1UL<<9)
#define CFMC_CONF_INST_CACHE_ON						(1UL<<8)


// FMC Flash Protection Macro
#define CFMC_FLSPROT_UNPROT_0x0007FE00_0x0007FFFF	(1UL<<31)
#define CFMC_FLSPROT_UNPROT_0x0007FC00_0x0007FDFF	(1UL<<30)
#define CFMC_FLSPROT_UNPROT_0x0007FA00_0x0007FBFF	(1UL<<29)
#define CFMC_FLSPROT_UNPROT_0x0007F800_0x0007F9FF	(1UL<<28)
#define CFMC_FLSPROT_UNPROT_0x0007F600_0x0007F7FF	(1UL<<27)
#define CFMC_FLSPROT_UNPROT_0x0007F400_0x0007F5FF	(1UL<<26)
#define CFMC_FLSPROT_UNPROT_0x0007F200_0x0007F3FF	(1UL<<25)
#define CFMC_FLSPROT_UNPROT_0x0007F000_0x0007F1FF	(1UL<<24)

#define CFMC_FLSPROT_PROT_0x00078000_0x0007FFFF		(1UL<<15)
#define CFMC_FLSPROT_PROT_0x00070000_0x00077FFF		(1UL<<14)
#define CFMC_FLSPROT_PROT_0x00068000_0x0006FFFF		(1UL<<13)
#define CFMC_FLSPROT_PROT_0x00060000_0x00067FFF		(1UL<<12)
#define CFMC_FLSPROT_PROT_0x00058000_0x0005FFFF		(1UL<<11)
#define CFMC_FLSPROT_PROT_0x00050000_0x00057FFF		(1UL<<10)
#define CFMC_FLSPROT_PROT_0x00048000_0x0004FFFF		(1UL<<9)
#define CFMC_FLSPROT_PROT_0x00040000_0x00047FFF		(1UL<<8)
#define CFMC_FLSPROT_PROT_0x00038000_0x0003FFFF		(1UL<<7)
#define CFMC_FLSPROT_PROT_0x00030000_0x00037FFF		(1UL<<6)
#define CFMC_FLSPROT_PROT_0x00028000_0x0002FFFF		(1UL<<5)
#define CFMC_FLSPROT_PROT_0x00020000_0x00027FFF		(1UL<<4)
#define CFMC_FLSPROT_PROT_0x00018000_0x0001FFFF		(1UL<<3)
#define CFMC_FLSPROT_PROT_0x00010000_0x00017FFF		(1UL<<2)
#define CFMC_FLSPROT_PROT_0x00008000_0x0000FFFF		(1UL<<1)
#define CFMC_FLSPROT_PROT_0x00000000_0x00007FFF		(1UL<<0)


// FMC Read Protection Register
#define CFMC_READPROT_RPROT_UNPROTECT				(0xFFUL<<0)
#define CFMC_READPROT_RPROT_LEVEL1					(0x39UL<<0)
#define CFMC_READPROT_RPROT_LEVEL2					(0x00UL<<0)

#define CFMC_READPROT_RPROT_MASK						(0xFFUL<<0)

// FMC OTP Protection Macro
#define CFMC_OTPPROT_PROT_0x0F000400_0x0F0005FF		(1UL<<2)
#define CFMC_OTPPROT_PROT_0x0F000200_0x0F0003FF		(1UL<<1)
#define CFMC_OTPPROT_PROT_0x0F000000_0x0F0001FF		(1UL<<0)

// FMC Checksum Control Macro
#define CFMC_CHKCTRL_DATA_RST				(1UL<<16)
#define CFMC_CHKCTRL_INTR_ON				(1UL<<8)
#define CFMC_CHKCTRL_BURST_ON				(1UL<<1)
#define CFMC_CHKCTRL_BACKGROUND_ON			(1UL<<0)

// FM_CONF
#define CFMC_CONF_BBLOCK			(1UL<<24)
#define CFMC_CONF_DCRST			(1UL<<17)
#define CFMC_CONF_ICRST			(1UL<<16)
#define CFMC_CONF_DCEN			(1UL<<9)
#define CFMC_CONF_ICEN			(1UL<<8)
#define CFMC_CONF_LATENCY		(1UL<<0)

// FM_CTRL
#define CFMC_CTRL_FLOCK			(1UL<<31)
#define CFMC_CTRL_OLOCK			(1UL<<30)
#define CFMC_CTRL_WDIEN			(1UL<<8)
#define CFMC_CTRL_CERS			(1UL<<4)
#define CFMC_CTRL_S4KERS			(1UL<<3)
#define CFMC_CTRL_S1KERS			(1UL<<2)
#define CFMC_CTRL_PERS			(1UL<<1)
#define CFMC_CTRL_PGM			(1UL<<0)

// FM_STAT
#define CFMC_STAT_RPERR			(1UL<<21)
#define CFMC_STAT_WSERR			(1UL<<20)
#define CFMC_STAT_OPERR			(1UL<<19)
#define CFMC_STAT_FPERR			(1UL<<18)
#define CFMC_STAT_OLERR			(1UL<<17)
#define CFMC_STAT_FLERR			(1UL<<16)
#define CFMC_STAT_CDONE			(1UL<<9)
#define CFMC_STAT_WDONE			(1UL<<8)
#define CFMC_STAT_CBUSY			(1UL<<1)
#define CFMC_STAT_WBUSY			(1UL<<0)

// FM_CHKCTRL
#define CFMC_CHKCTRL_CDRST		(1UL<<16)
#define CFMC_CHKCTRL_CDIEN		(1UL<<8)
#define CFMC_CHKCTRL_BSTEN		(1UL<<1)
#define CFMC_CHKCTRL_BGEN		(1UL<<0)

// FM_FLSPROT
#define CFMC_FLSPROT_FPBY16_0				(1UL<<0)
#define CFMC_FLSPROT_FPBY16_1				(1UL<<1)
#define CFMC_FLSPROT_FPBY16_2				(1UL<<2)
#define CFMC_FLSPROT_FPBY16_3				(1UL<<3)
#define CFMC_FLSPROT_FPBY16_4				(1UL<<4)
#define CFMC_FLSPROT_FPBY16_5				(1UL<<5)
#define CFMC_FLSPROT_FPBY16_6				(1UL<<6)
#define CFMC_FLSPROT_FPBY16_7				(1UL<<7)
#define CFMC_FLSPROT_FPBY16_8				(1UL<<8)
#define CFMC_FLSPROT_FPBY16_9				(1UL<<9)
#define CFMC_FLSPROT_FPBY16_10			(1UL<<10)
#define CFMC_FLSPROT_FPBY16_11			(1UL<<11)
#define CFMC_FLSPROT_FPBY16_12			(1UL<<12)
#define CFMC_FLSPROT_FPBY16_13			(1UL<<13)
#define CFMC_FLSPROT_FPBY16_14			(1UL<<14)
#define CFMC_FLSPROT_FPBY16_15			(1UL<<15)

#define CFMC_FLSPROT_FUP512B_0			(1UL<<24)
#define CFMC_FLSPROT_FUP512B_1			(1UL<<25)
#define CFMC_FLSPROT_FUP512B_2			(1UL<<26)
#define CFMC_FLSPROT_FUP512B_3			(1UL<<27)
#define CFMC_FLSPROT_FUP512B_4			(1UL<<28)
#define CFMC_FLSPROT_FUP512B_5			(1UL<<29)
#define CFMC_FLSPROT_FUP512B_6			(1UL<<30)
#define CFMC_FLSPROT_FUP512B_7			(1UL<<31)

	
/* Public Functions ----------------------------------------------------------- */
void HAL_CFMC_CacheCmd(uint32_t CacheSRC, FunctionalState NewState);
void HAL_CFMC_BootBlockCmd(FunctionalState NewState);
void HAL_CFMC_WaitCmd(uint8_t WaitValue);
void HAL_CFMC_FlashAccessCmd(FunctionalState NewState);
void HAL_CFMC_OTPAccessCmd(FunctionalState NewState);
void HAL_CFMC_FlashPROTCmd(uint32_t ProtectionBit, FunctionalState NewState);
void HAL_CFMC_FlashRPROTCmd(uint32_t ReadProtectionBit, FunctionalState NewState);
void HAL_CFMC_OTPPROTCmd(uint32_t ProtectionBit, FunctionalState NewState);
void HAL_CFMC_LockCmd(uint32_t LockBit);
void HAL_CFMC_InterruptCmd(FunctionalState NewStatus);
uint32_t HAL_CFMC_GetAccessStatus(void);
void HAL_CFMC_ClearAccessStatus(uint32_t ClearSRC);
void HAL_CFMC_FlashChecksumCmd(uint32_t ControlBit, FunctionalState NewState);
uint32_t HAL_CFMC_GetChecksumData(void);
void HAL_CFMC_ChecksumAddrCmd(uint32_t StartAddress, uint32_t EndAddress);
uint32_t HAL_CFMC_GetChecksumCmd(void);


void HAL_CFMC_Erase(uint32_t Ersmod,unsigned long addr);
void HAL_CFMC_ProgramPage (unsigned long addr, unsigned long size, unsigned char *buf);
unsigned long HAL_CFMC_Verify (unsigned long addr, unsigned long size, unsigned char *buf);

#ifdef __cplusplus
}
#endif


#endif /* end _A34M41x_CFMC_H_ */

/* --------------------------------- End Of File ------------------------------ */
