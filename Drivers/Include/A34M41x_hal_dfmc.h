/**********************************************************************
* @file		A34M41x_dfmc.h
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

#ifndef _A34M41x_DFMC_H_
#define _A34M41x_DFMC_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DFMC_WRITE_BUSY_POLLING  while(((DFMC->STAT)&DFMC_STAT_WBUSY)==DFMC_STAT_WBUSY)
	
// Flash Access Key
#define DFMC_FLSKEY_ACCESS_KEY1						0x01234567UL
#define DFMC_FLSKEY_ACCESS_KEY2						0x12345678UL
#define DFMC_FLSKEY_ACCESS_KEY3						0x23456789UL

#define DFMC_FLSKEY_ACCESS_RST							0x00000000UL



// OTP Access Key
#define DFMC_OTPKEY_ACCESS_KEY1						0x3456789AUL
#define DFMC_OTPKEY_ACCESS_KEY2						0x456789ABUL
#define DFMC_OTPKEY_ACCESS_KEY3						0x56789ABCUL

#define DFMC_OTPKEY_ACCESS_RST							0x00000000UL



// FMC CONF Macro
#define DFMC_CONF_DATA_CACHE_RST						(0x01UL<<17)
#define DFMC_CONF_INST_CACHE_RST						(0x01UL<<16)

#define DFMC_CONF_DATA_CACHE_ON						(0x01UL<<9)
#define DFMC_CONF_INST_CACHE_ON						(0x01UL<<8)


// FMC Flash Protection Macro
#define DFMC_FLSPROT_UNPROT_0x0E007E00_0x0E007FFF	(0x01UL<<31)
#define DFMC_FLSPROT_UNPROT_0x0E007C00_0x0E007DFF	(0x01UL<<30)
#define DFMC_FLSPROT_UNPROT_0x0E007A00_0x0E007BFF	(0x01UL<<29)
#define DFMC_FLSPROT_UNPROT_0x0E007800_0x0E0079FF	(0x01UL<<28)
#define DFMC_FLSPROT_UNPROT_0x0E007600_0x0E0077FF	(0x01UL<<27)
#define DFMC_FLSPROT_UNPROT_0x0E007400_0x0E0075FF	(0x01UL<<26)
#define DFMC_FLSPROT_UNPROT_0x0E007200_0x0E0073FF	(0x01UL<<25)
#define DFMC_FLSPROT_UNPROT_0x0E007000_0x0E0071FF	(0x01UL<<24)

#define DFMC_FLSPROT_PROT_0x0E007800_0x0E007FFF		(0x01UL<<15)
#define DFMC_FLSPROT_PROT_0x0E007000_0x0E0077FF		(0x01UL<<14)
#define DFMC_FLSPROT_PROT_0x0E006800_0x0E006FFF		(0x01UL<<13)
#define DFMC_FLSPROT_PROT_0x0E006000_0x0E0067FF		(0x01UL<<12)
#define DFMC_FLSPROT_PROT_0x0E005800_0x0E005FFF		(0x01UL<<11)
#define DFMC_FLSPROT_PROT_0x0E005000_0x0E0057FF		(0x01UL<<10)
#define DFMC_FLSPROT_PROT_0x0E004800_0x0E004FFF		(0x01UL<<9)
#define DFMC_FLSPROT_PROT_0x0E004000_0x0E0047FF		(0x01UL<<8)
#define DFMC_FLSPROT_PROT_0x0E003800_0x0E003FFF		(0x01UL<<7)
#define DFMC_FLSPROT_PROT_0x0E003000_0x0E0037FF		(0x01UL<<6)
#define DFMC_FLSPROT_PROT_0x0E002800_0x0E002FFF		(0x01UL<<5)
#define DFMC_FLSPROT_PROT_0x0E002000_0x0E0027FF		(0x01UL<<4)
#define DFMC_FLSPROT_PROT_0x0E001800_0x0E001FFF		(0x01UL<<3)
#define DFMC_FLSPROT_PROT_0x0E001000_0x0E0017FF		(0x01UL<<2)
#define DFMC_FLSPROT_PROT_0x0E000800_0x0E000FFF		(0x01UL<<1)
#define DFMC_FLSPROT_PROT_0x0E000000_0x0E0007FF		(0x01UL<<0)

// FMC Checksum Control Macro
#define DFMC_CHKCTRL_DATA_RST				(0x01UL<<16)
#define DFMC_CHKCTRL_INTR_ON				(0x01UL<<8)
#define DFMC_CHKCTRL_BURST_ON				(0x01UL<<1)
#define DFMC_CHKCTRL_BACKGROUND_ON			(0x01UL<<0)

#define MIO32(addr)  (*(volatile unsigned int *)(addr))

// ------------------- M418 --------------------
// FM_CONF
#define DFMC_CONF_LATENCY		(0x01UL<<0)

// FM_CTRL
#define DFMC_CTRL_FLOCK			(1UL<<31)
#define DFMC_CTRL_WDIEN			(1UL<<8)
#define DFMC_CTRL_CERS			(1UL<<4)
#define DFMC_CTRL_S4KERS			(1UL<<3)
#define DFMC_CTRL_S1KERS			(1UL<<2)
#define DFMC_CTRL_PERS			(1UL<<1)
#define DFMC_CTRL_PGM			(1UL<<0)

// FM_STAT
#define DFMC_STAT_RPERR			(1UL<<21)
#define DFMC_STAT_WSERR			(1UL<<20)
#define DFMC_STAT_FPERR			(1UL<<18)
#define DFMC_STAT_FLERR			(1UL<<16)
#define DFMC_STAT_CDONE			(1UL<<9)
#define DFMC_STAT_WDONE			(1UL<<8)
#define DFMC_STAT_CBUSY			(1UL<<1)
#define DFMC_STAT_WBUSY			(1UL<<0)


// FM_CHKCTRL
#define DFMC_CHKCTRL_CDRST		(1UL<<16)
#define DFMC_CHKCTRL_CDIEN		(1UL<<8)
#define DFMC_CHKCTRL_BSTEN		(1UL<<1)
#define DFMC_CHKCTRL_BGEN		(1UL<<0)

// FM_FLSPROT
#define DFMC_FLSPROT_FPBY16_0				(1UL<<0)
#define DFMC_FLSPROT_FPBY16_1				(1UL<<1)
#define DFMC_FLSPROT_FPBY16_2				(1UL<<2)
#define DFMC_FLSPROT_FPBY16_3				(1UL<<3)
#define DFMC_FLSPROT_FPBY16_4				(1UL<<4)
#define DFMC_FLSPROT_FPBY16_5				(1UL<<5)
#define DFMC_FLSPROT_FPBY16_6				(1UL<<6)
#define DFMC_FLSPROT_FPBY16_7				(1UL<<7)
#define DFMC_FLSPROT_FPBY16_8				(1UL<<8)
#define DFMC_FLSPROT_FPBY16_9				(1UL<<9)
#define DFMC_FLSPROT_FPBY16_10			(1UL<<10)
#define DFMC_FLSPROT_FPBY16_11			(1UL<<11)
#define DFMC_FLSPROT_FPBY16_12			(1UL<<12)
#define DFMC_FLSPROT_FPBY16_13			(1UL<<13)
#define DFMC_FLSPROT_FPBY16_14			(1UL<<14)
#define DFMC_FLSPROT_FPBY16_15			(1UL<<15)

#define DFMC_FLSPROT_FUP512B_0			(1UL<<24)
#define DFMC_FLSPROT_FUP512B_1			(1UL<<25)
#define DFMC_FLSPROT_FUP512B_2			(1UL<<26)
#define DFMC_FLSPROT_FUP512B_3			(1UL<<27)
#define DFMC_FLSPROT_FUP512B_4			(1UL<<28)
#define DFMC_FLSPROT_FUP512B_5			(1UL<<29)
#define DFMC_FLSPROT_FUP512B_6			(1UL<<30)
#define DFMC_FLSPROT_FUP512B_7			(1UL<<31)

	
/* Public Functions ------------------------------------------------------------- */
void HAL_DFMC_FlashAccessCmd(FunctionalState NewState);
void HAL_DFMC_FlashPROTCmd(uint32_t ProtectionBit, FunctionalState NewState);
void HAL_DFMC_LockCmd(uint32_t LockBit);
void HAL_DFMC_InterruptCmd(FunctionalState NewStatus);

uint32_t HAL_DFMC_GetAccessStatus(void);
void HAL_DFMC_ClearAccessStatus(uint32_t ClearSRC);
void HAL_DFMC_FlashChecksumCmd(uint32_t ControlBit, FunctionalState NewState);
uint32_t HAL_DFMC_GetChecksumData(void);
uint32_t HAL_DFMC_GetChecksumCmd(void);
void HAL_DFMC_ChecksumAddrCmd(uint32_t StartAddress, uint32_t EndAddress);
void HAL_DFMC_WaitCmd(uint8_t WaitValue);

void HAL_DFMC_Erase(uint32_t Ersmod, unsigned long addr);
void HAL_DFMC_ProgramPage (unsigned long addr, unsigned long size, unsigned char *buf);
unsigned long HAL_DFMC_Verify(unsigned long addr, unsigned long size, unsigned char *buf);

#ifdef __cplusplus
}
#endif


#endif /* end _A34M41x_DFMC_H_ */

/* --------------------------------- End Of File ------------------------------ */
