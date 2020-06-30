/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_flash.h
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

#include "a33g52x.h"


//typedef struct {
//	REGISTER_T	FMCFG;				// 0x4000_0100
//	REGISTER_T	FMCON;				// 0x4000_0104
//	REGISTER_T	FMODR;				// 0x4000_0108
//	REGISTER_T	MFIDR; 				// 0x4000_010C
//	
//	REGISTER_T	FMAR;				// 0x4000_0110
//	REGISTER_T	FMTEST;				// 0x4000_0114
//	REGISTER_T	FMCRC; 				// 0x4000_0118
//	REGISTER_T  FMPROTECT; 		// 0x4000_011C

//	REGISTER_T 	FMRPROT; 			// 0x4000_0120
//	REGISTER_T 	FMHWID; 			// 0x4000_0124
//	REGISTER_T 	FMSIZE; 			// 0x4000_0128
//	REGISTER_T 	FMBOOT; 			// 0x4000_012C

//	REGISTER_T 	FMDCT0; 			// 0x4000_0130
//	REGISTER_T 	FMDCT1; 			// 0x4000_0134
//	REGISTER_T 	FMDCT2; 			// 0x4000_0138
//	REGISTER_T 	FMDCT3; 			// 0x4000_013C
//	
//} FMC_Type; 



//typedef struct {                                    /*!< FMC Structure                                                         */
//  __IO uint32_t  CFG;                            /*!< Flash Memory Configuration Register                                   */
//  __IO uint32_t  CON;                             /*!< Flash Memory Control Register                                         */
//  __IO uint32_t  ODR;                             /*!< Flash Memory Output Data Register                                     */
//  __I  uint32_t   IDR;                             /*!< Flash Memory Input Data Register                                      */
//	
//  __IO uint32_t  AR;                              /*!< Flash Memory Address Register                                         */
//  __IO uint32_t  TEST;                            /*!< Flash Memory Extended Mode Control Register                           */
//  __IO uint32_t  CRC;                             /*!< Flash Memory CRC Register                                             */
//  __IO uint32_t  PROTECT;                        /*!< Flash Memory Protection Register                                      */
//	
//  __IO uint32_t  RPROT;                           /*!< Flash Memory Read Protection Register                                 */
//  __I  uint32_t  HWID;                            /*!< Flash Hardware ID Register                                            */
//  __I  uint32_t  SIZE;                            /*!< Flash Size Register                                                   */
//  __IO uint32_t  BOOT;                           /*!< Flash Boot Register                                */	
//	
//  __IO uint32_t  DCT0;                           /*!< Flash Memory DCT0                                 */
//  __IO uint32_t  DCT1;                           /*!< Flash Memory DCT1                                 */
//  __IO uint32_t  DCT2;                           /*!< Flash Memory DCT2                                 */
//  __IO uint32_t  DCT3;                           /*!< Flash Memory DCT3                                */		
//} FMC_Type;

//==========================================================================
// 	
//		D E F I N A T I O N S 	
//
//==========================================================================
#define DFLASH_ADDR_OFFSET_MASK 		(0x7FFF)



//============================================================================	
// 
//		FLASH ADDRESS 
//
//============================================================================	
#define CFLASH_BASE_ADDRESS					(0x00000000)

#define CFLASH_SIZE									(0x60000)			// A33G527
//#define CFLASH_SIZE									(0x40000)		// A33G526


#define DFLASH_BASE_ADDRESS					(0x0F000000)
#define DFLASH_SIZE									(0x8000)



//==========================================================================
// 	FMCFG
//		
//				@ address = 0x4000_0100
//
//==========================================================================
#define FMCFG_DWAIT_VAL(n)				(((n)&0x001FUL)<<8)
#define FMCFG_DWAIT_MASK 				(0x001FUL<<8)

#define FMCFG_CZWAIT					(0x0001UL<<7)

#define FMCFG_CWAIT_VAL(n) 			(((n)&0x001FUL)<<0)
#define FMCFG_CWAIT_MASK 				(0x001FUL<<0)


#define FMCFG_DEFAULT_VAL					(0x0000)
//#define FMCFG_DEFAULT_VAL					(0x0303)




//==========================================================================
// 	FMCON 
//		
//				@ address = 0x4000_0104
//
//==========================================================================
#define FMCON_RST						(0x0001UL<<31)
#define FMCON_STOP 						(0x0001UL<<30)
#define FMCON_SELF						(0x0001UL<<28)
#define FMCON_BBLK 						(0x0001UL<<21)
#define FMCON_FSRD 						(0x0001UL<<20)
#define FMCON_PAGE 						(0x0001UL<<18)
#define FMCON_TRSL 						(0x0001UL<<17)
#define FMCON_TSMD 						(0x0001UL<<16)
//-----------------------------------------------------------

#define FMCON_DSEL						(0x0001UL<<15)
#define FMCON_CSEL						(0x0001UL<<14)
#define FMCON_AE						(0x0001UL<<13)
#define FMCON_OE						(0x0001UL<<12)
#define FMCON_CS						(0x0001UL<<11)
//-----------------------------------------------------------
#define FMCON_DTBIT						(0x0001UL<<9)
#define FMCON_CTBIT						(0x0001UL<<8)
//-----------------------------------------------------------
#define FMCON_NVSTR					(0x0001UL<<7)
//-----------------------------------------------------------
#define FMCON_MASE						(0x0001UL<<2)
#define FMCON_SERA						(0x0001UL<<1)
#define FMCON_PROG						(0x0001UL<<0)



//==========================================================================
// 	FMODR 
//		
//				@ address = 0x4000_0108
//
//==========================================================================

//==========================================================================
// 	FMIDR 
//		
//				@ address = 0x4000_010C
//
//==========================================================================



//==========================================================================
// 	FMAR 
//		
//				@ address = 0x4000_0110
//
//==========================================================================
#define FMAR_DCTE 							(0x0001UL<<23)
#define FMAR_LDTE 							(0x0001UL<<22)
#define FMAR_IFREN 							(0x0001UL<<21)
#define FMAR_PRTE 							(0x0001UL<<20)
#define FMAR_REDE 							(0x0001UL<<19)

#define FMAR_ADDR(addr) 					(((addr)&0x0000FFFFUL)<<0)



//==========================================================================
// 	FMTEST
//		
//				@ address = 0x4000_0114
//
//==========================================================================
#define FMTEST_WRITE_KEY 					(0x004AUL<<8)
#define FMTEST_EX 							(0x0001UL<<0)



//==========================================================================
// 	FMCRC
//		
//				@ address = 0x4000_0124
//
//==========================================================================
#define FMCRC_CE 						(0x0001UL<<20)
#define FMCRC_CE_RESET					(0x0000<<20)
#define FMCRC_CE_ENABLE				(0x0001<<20)


#define FMCRC_SEL 						(0x0001<<16)
#define FMCRC_SEL_CODE 					(0x0000<<16)
#define FMCRC_SEL_DATA 				(0x0001<<16)


#define FMCRC_CRC16_MASK 				(0xFFFF<<0)


//==========================================================================
// 	FMPROTECT 
//		
//				@ address = 0x4000_011C
//
//==========================================================================
#define FMPROTECT_DIP					(0x0001UL<<15)
#define FMPROTECT_CIP 					(0x0001UL<<14)

#define FMPROTECT_BP5 					(0x0001UL<<13)
#define FMPROTECT_BP4 					(0x0001UL<<12) 


#define FMPROTECT_DP3					(0x0001UL<<11)
#define FMPROTECT_DP2					(0x0001UL<<10)
#define FMPROTECT_DP1					(0x0001UL<<9)
#define FMPROTECT_DP0					(0x0001UL<<8)

#define FMPROTECT_DP_MASK				(0x000FUL<<8) 
#define FMPROTECT_DPx_ALL				(0x000FUL<<8) 

#define FMPROTECT_BP3 					(0x0001UL<<7)
#define FMPROTECT_BP2 					(0x0001UL<<6) 
#define FMPROTECT_BP1 					(0x0001UL<<5)
#define FMPROTECT_BP0 					(0x0001UL<<4) 

#define FMPROTECT_SP3 					(0x0001UL<<3)
#define FMPROTECT_SP2 					(0x0001UL<<2) 
#define FMPROTECT_SP1 					(0x0001UL<<1)
#define FMPROTECT_SP0 					(0x0001UL<<0) 



//==========================================================================
// 	FMRPROT 
//		
//				@ address = 0x4000_0120
//
//==========================================================================
#define FMRPROT_LVL1 					(0x0001UL<<30)

#define FMRPROT_ENTER_LVL1_CODE		(0x0039UL<<0)




//==========================================================================
// 	FMSIZE 
//		
//				@ address = 0x4000_0128
//
//==========================================================================
#define FMSIZE_256K						(0x000000A5UL)
#define FMSIZE_384K 					(0x0000005AUL)



//==========================================================================
// 	FMBOOT
//		
//				@ address = 0x4000_012C
//
//==========================================================================
#define FMBOOT_MAGIC_KEY 				(0x13752451UL)
#define FMBOOT_MAGIC_NULL_KEY 		(0x13752450UL)
#define FMBOOT_BOOT 					(0x0001UL<<0)




//=========================================================================
//
//	D E F I N A T I O N S
//
//=========================================================================
#define DFC_CMD_CHIP_ERASE			FMCON_MASE
#define DFC_CMD_SECTOR_ERASE		FMCON_SERA
#define DFC_CMD_PROG_BYTE			FMCON_PROG



////===============================================================
//// MACROs
////===============================================================





//===============================================================
//
//		F U N C T I O N    D E C L A R A T I O N S
//
//===============================================================
void FLASH_Init (FMC_Type * const flash); 
int FLASH_EraseChip (FMC_Type * const flash, uint32_t addr); 
int FLASH_EraseSector (FMC_Type * const flash, uint32_t addr); 
//int FLASH_Erase4KSector (FMC_Type * const flash, uint32_t addr); 
int FLASH_Erase512Page (FMC_Type * const flash, uint32_t addr); 
int FLASH_ProgramByte (FMC_Type * const flash, uint32_t addr, uint8_t data); 
uint32_t FLASH_ExecuteFlashOpteration (FMC_Type * const flash, uint32_t cmd, uint32_t addr, uint32_t data, uint32_t limit); 


void FLASH_EnableProtection (FMC_Type * const flash, uint32_t mask); 
void FLASH_DisableProtection (FMC_Type * const flash, uint32_t mask); 

int FLASH_Self_EraseSector (FMC_Type * const flash, uint32_t addr); 
int FLASH_Self_ProgramWORD (FMC_Type * const flash, uint32_t addr, uint32_t data); 


