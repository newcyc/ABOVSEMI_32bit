/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_pcu.h
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


#ifndef __PCU_H__
#define __PCU_H__

#include "A33G52x.h"


//==========================================================================
// Pin No 
//==========================================================================
#define PIN_0								(0)
#define PIN_1								(1)
#define PIN_2								(2)
#define PIN_3								(3)
#define PIN_4								(4)
#define PIN_5								(5)
#define PIN_6								(6)
#define PIN_7								(7)
#define PIN_8								(8)
#define PIN_9								(9)
#define PIN_10							(10)
#define PIN_11							(11)
#define PIN_12							(12)
#define PIN_13							(13)
#define PIN_14							(14)
#define PIN_15							(15)



//==========================================================================
// Pin Direction 
//==========================================================================
#define PORT_DIR_IN						(1)
#define PORT_DIR_OUT					(0)





//==========================================================================
// 	PnMR
//		
//
//==========================================================================
#define PnMR_FUNC_MASK				(0x03UL)



//==========================================================================
// 	PAMR
//		
//				@ addr = 0x4000_0200
//
//==========================================================================
#define PA0_MUX_PA0						(0x0000UL<<0)
#define PA0_MUX_AN0						(0x0003UL<<0)

#define PA1_MUX_PA1						(0x0000UL<<2)
#define PA1_MUX_AN1						(0x0003UL<<2)

#define PA2_MUX_PA2						(0x0000UL<<4)
#define PA2_MUX_AN2						(0x0003UL<<4)

#define PA3_MUX_PA3						(0x0000UL<<6)
#define PA3_MUX_AN3						(0x0003UL<<6)

#define PA4_MUX_PA4						(0x0000UL<<8)
#define PA4_MUX_AN4						(0x0003UL<<8)

#define PA5_MUX_PA5						(0x0000UL<<10)
#define PA5_MUX_AN5						(0x0003UL<<10)

#define PA6_MUX_PA6						(0x0000UL<<12)
#define PA6_MUX_T0C						(0x0001UL<<12)
#define PA6_MUX_AN6						(0x0003UL<<12)

#define PA7_MUX_PA7						(0x0000UL<<14)
#define PA7_MUX_T1C						(0x0001UL<<14)
#define PA7_MUX_AN7						(0x0003UL<<14)

#define PA8_MUX_PA8						(0x0000UL<<16)
#define PA8_MUX_T2C						(0x0001UL<<16)

#define PA9_MUX_PA9						(0x0000UL<<18)
#define PA9_MUX_T3C						(0x0001UL<<18)

#define PA10_MUX_PA10						(0x0000UL<<20)
#define PA10_MUX_T4C						(0x0001UL<<20)

#define PA11_MUX_PA11						(0x0000UL<<22)
#define PA11_MUX_T5C						(0x0001UL<<22)

#define PA12_MUX_PA12						(0x0000UL<<24)
#define PA12_MUX_T6C						(0x0001UL<<24)

#define PA13_MUX_PA13						(0x0000UL<<26)
#define PA13_MUX_T7C						(0x0001UL<<26)

#define PA14_MUX_PA14						(0x0000UL<<28)
#define PA14_MUX_T8C						(0x0001UL<<28)
#define PA14_MUX_AN14						(0x0003UL<<28)

#define PA15_MUX_PA15						(0x0000UL<<30)
#define PA15_MUX_T9C						(0x0001UL<<30)
#define PA15_MUX_AN15						(0x0003UL<<30)





//==========================================================================
// 	PBMR
//		
//				@ addr = 0x4000_0220
//
//==========================================================================
#define PB0_MUX_PB0							(0x0000UL<<0)
#define PB0_MUX_T0O						(0x0001UL<<0)

#define PB1_MUX_PB1							(0x0000UL<<2)
#define PB1_MUX_T1O						(0x0001UL<<2)

#define PB2_MUX_PB2							(0x0000UL<<4)
#define PB2_MUX_T2O						(0x0001UL<<4)

#define PB3_MUX_PB3							(0x0000UL<<6)
#define PB3_MUX_T3O						(0x0001UL<<6)

#define PB4_MUX_PB4							(0x0000UL<<8)
#define PB4_MUX_T4O						(0x0001UL<<8)

#define PB5_MUX_PB5							(0x0000UL<<10)
#define PB5_MUX_T5O						(0x0001UL<<10)

#define PB6_MUX_PB6							(0x0000UL<<12)
#define PB6_MUX_T6O						(0x0001UL<<12)

#define PB7_MUX_PB7							(0x0000UL<<14)
#define PB7_MUX_T7O						(0x0001UL<<14)

#define PB8_MUX_PB8							(0x0000UL<<16)
#define PB8_MUX_T8O						(0x0001UL<<16)

#define PB9_MUX_PB9							(0x0000UL<<18)
#define PB9_MUX_T9O						(0x0001UL<<18)

#define PB10_MUX_PB10						(0x0000UL<<20)
#define PB10_MUX_SS0						(0x0001UL<<20)

#define PB11_MUX_PB11						(0x0000UL<<22)
#define PB11_MUX_SCK0						(0x0001UL<<22)

#define PB12_MUX_PB12						(0x0000UL<<24)
#define PB12_MUX_MOSI0						(0x0001UL<<24)

#define PB13_MUX_PB13						(0x0000UL<<26)
#define PB13_MUX_MISO0						(0x0001UL<<26)

#define PB14_MUX_PB14						(0x0000UL<<28)
#define PB14_MUX_SCL0						(0x0001UL<<28)

#define PB15_MUX_PB15						(0x0000UL<<30)
#define PB15_MUX_SDA0						(0x0001UL<<30)





//==========================================================================
// 	PCMR
//		
//				@ addr = 0x4000_0240
//
//==========================================================================
#define PC0_MUX_PC0							(0x0000UL<<0)
#define PC0_MUX_nTRST						(0x0001UL<<0)

#define PC1_MUX_PC1							(0x0000UL<<2)
#define PC1_MUX_TDI							(0x0001UL<<2)

#define PC2_MUX_PC2							(0x0000UL<<4)
#define PC2_MUX_TMS						(0x0001UL<<4)

#define PC3_MUX_PC3							(0x0000UL<<6)
#define PC3_MUX_TCK							(0x0001UL<<6)

#define PC4_MUX_PC4							(0x0000UL<<8)
#define PC4_MUX_TDO						(0x0001UL<<8)

#define PC5_MUX_PC5							(0x0000UL<<10)

#define PC6_MUX_PC6							(0x0000UL<<12)
#define PC6_MUX_nRESET						(0x0001UL<<12)

#define PC7_MUX_PC7							(0x0000UL<<14)

#define PC8_MUX_PC8							(0x0000UL<<16)
#define PC8_MUX_RXD0						(0x0001UL<<16)

#define PC9_MUX_PC9							(0x0000UL<<18)
#define PC9_MUX_TXD0						(0x0001UL<<18)

#define PC10_MUX_PC10						(0x0000UL<<20)
#define PC10_MUX_RXD2						(0x0001UL<<20)

#define PC11_MUX_PC11						(0x0000UL<<22)
#define PC11_MUX_TXD2						(0x0001UL<<22)

#define PC12_MUX_PC12						(0x0000UL<<24)
#define PC12_MUX_STBYO						(0x0001UL<<24)

#define PC13_MUX_PC13						(0x0000UL<<26)
#define PC13_MUX_CLKO						(0x0001UL<<26)

#define PC14_MUX_PC14						(0x0000UL<<28)
#define PC14_MUX_XTALO						(0x0001UL<<28)

#define PC15_MUX_PC15						(0x0000UL<<30)
#define PC15_MUX_XTALI						(0x0001UL<<30)
#define PC15_MUX_CLKIN						(0x0002UL<<30)




//==========================================================================
// 	PDMR
//		
//				@ addr = 0x4000_0260
//
//==========================================================================
#define PD0_MUX_PD0						(0x0000UL<<0)
#define PD0_MUX_PWMA0						(0x0001UL<<0)

#define PD1_MUX_PD1						(0x0000UL<<2)
#define PD1_MUX_PWMA1						(0x0001UL<<2)

#define PD2_MUX_PD2						(0x0000UL<<4)
#define PD2_MUX_PWMA2						(0x0001UL<<4)

#define PD3_MUX_PD3						(0x0000UL<<6)
#define PD3_MUX_PWMA3						(0x0001UL<<6)

#define PD4_MUX_PD4						(0x0000UL<<8)
#define PD4_MUX_PWMA4						(0x0001UL<<8)

#define PD5_MUX_PD5						(0x0000UL<<10)
#define PD5_MUX_PWMA5						(0x0001UL<<10)

#define PD6_MUX_PD6						(0x0000UL<<12)
#define PD6_MUX_PWMA6						(0x0001UL<<12)

#define PD7_MUX_PD7						(0x0000UL<<14)
#define PD7_MUX_PWMA7						(0x0001UL<<14)


#define PD8_MUX_PD8						(0x0000UL<<16)
#define PD8_MUX_SS1						(0x0001UL<<16)

#define PD9_MUX_PD9						(0x0000UL<<18)
#define PD9_MUX_SCK1						(0x0001UL<<18)

#define PD10_MUX_PD10						(0x0000UL<<20)
#define PD10_MUX_MOSI1						(0x0001UL<<20)

#define PD11_MUX_PD11						(0x0000UL<<22)
#define PD11_MUX_MISO1						(0x0001UL<<22)

#define PD12_MUX_PD12						(0x0000UL<<24)
#define PD12_MUX_RXD1						(0x0001UL<<24)

#define PD13_MUX_PD13						(0x0000UL<<26)
#define PD13_MUX_TXD1						(0x0001UL<<26)

#define PD14_MUX_PD14						(0x0000UL<<28)
#define PD14_MUX_SCL1						(0x0001UL<<28)

#define PD15_MUX_PD15						(0x0000UL<<30)
#define PD15_MUX_SDA1						(0x0001UL<<30)




//==========================================================================
// 	PEMR
//		
//				@ addr = 0x4000_0280
//
//==========================================================================
#define PE0_MUX_PE0							(0x0000UL<<0)
#define PE0_MUX_PWMB0						(0x0001UL<<0)
//#define PE0_MUX_WDTO						(0x0002UL<<0)		// THIS FUNCTION DOESN'T BE SUPPORTED

#define PE1_MUX_PE1							(0x0000UL<<2)
#define PE1_MUX_PWMB1						(0x0001UL<<2)

#define PE2_MUX_PE2							(0x0000UL<<4)
#define PE2_MUX_PWMB2						(0x0001UL<<4)
//#define PE2_MUX_FRTO						(0x0002UL<<4)

#define PE3_MUX_PE3							(0x0000UL<<6)
#define PE3_MUX_PWMB3						(0x0001UL<<6)

#define PE4_MUX_PE4							(0x0000UL<<8)
#define PE4_MUX_PWMB4						(0x0001UL<<8)

#define PE5_MUX_PE5							(0x0000UL<<10)
#define PE5_MUX_PWMB5						(0x0001UL<<10)

#define PE6_MUX_PE6							(0x0000UL<<12)
#define PE6_MUX_PWMB6						(0x0001UL<<12)
#define PE6_MUX_RXD3						(0x0002UL<<12)

#define PE7_MUX_PE7							(0x0000UL<<14)
#define PE7_MUX_PWMB7						(0x0001UL<<14)
#define PE7_MUX_TXD3						(0x0002UL<<14)


#define PE8_MUX_PE8							(0x0000UL<<16)
#define PE8_MUX_SXIN						(0x0001UL<<16)

#define PE9_MUX_PE9							(0x0000UL<<18)
#define PE9_MUX_SXOUT						(0x0001UL<<18)

#define PE10_MUX_PE10						(0x0000UL<<20)

#define PE11_MUX_PE11						(0x0000UL<<22)
#define PE11_MUX_TraceD3					(0x0001UL<<22)

#define PE12_MUX_PE12						(0x0000UL<<24)
#define PE12_MUX_TraceD2					(0x0001UL<<24)

#define PE13_MUX_PE13						(0x0000UL<<26)
#define PE13_MUX_TraceD1					(0x0001UL<<26)

#define PE14_MUX_PE14						(0x0000UL<<28)
#define PE14_MUX_TraceD0					(0x0001UL<<28)

#define PE15_MUX_PE15						(0x0000UL<<30)
#define PE15_MUX_TraceCLK					(0x0001UL<<30)




//==========================================================================
// 	PFMR
//		
//				@ addr = 0x4000_02A0
//
//==========================================================================
#define PF0_MUX_PF0							(0x0000UL<<0)
#define PF0_MUX_AN8						(0x0003UL<<0)

#define PF1_MUX_PF1							(0x0000UL<<2)
#define PF1_MUX_AN9						(0x0003UL<<2)

#define PF2_MUX_PF2							(0x0000UL<<4)
#define PF2_MUX_AN10						(0x0003UL<<4)

#define PF3_MUX_PF3							(0x0000UL<<6)
#define PF3_MUX_AN11						(0x0003UL<<6)

#define PF4_MUX_PF4							(0x0000UL<<8)
#define PF4_MUX_AN12						(0x0003UL<<8)

#define PF5_MUX_PF5							(0x0000UL<<10)
#define PF5_MUX_AN13						(0x0003UL<<10)



#define PF7_MUX_PF7							(0x0000UL<<14)

#define PF8_MUX_PF8							(0x0000UL<<16)

#define PF9_MUX_PF9							(0x0000UL<<18)

#define PF10_MUX_PF10						(0x0000UL<<20)

#define PF11_MUX_PF11						(0x0000UL<<22)
#define PF11_MUX_FRTM						(0x0002UL<<22)



//==========================================================================
// 	PnCR
//		
//
//==========================================================================
#define PnCR_OUTPUT_PUSH_PULL				(0x00UL)
#define PnCR_OUTPUT_OPEN_DRAIN			(0x01UL)
#define PnCR_INPUT_LOGIC							(0x02UL)
#define PnCR_INPUT_ANALOG						(0x03UL)

#define PnCR_MASK									(0x03UL)



//==========================================================================
// 	PnPCR
//		
//
//==========================================================================
// legacy mode 
#define PnPCR_PULLUP_DISABLE				(0x00UL)
#define PnPCR_PULLUP_ENABLE					(0x01UL)

// normal mode 
#define PnPCR_PULLUPDOWN_DISABLE		(0x00UL)
#define PnPCR_PULLUPDOWN_ENABLE 		(0x01UL)

#define PnPCR_PULLUPDOWN_UP				(0x00UL)
#define PnPCR_PULLUPDOWN_DOWN 			(0x01UL)



//==========================================================================
// 	PnDER
//		
//
//==========================================================================
#define PnDER_DEBOUNCE_DISABLE				(0x00)
#define PnDER_DEBOUNCE_ENABLE				(0x01)





//==========================================================================
// 	PnIER/PnICR
//		
//
//
//==========================================================================
#define PCU_NO_INTR									(0x00)
#define PCU_LOW_LEVEL_INTR					(0x11)
#define PCU_HIGH_LEVEL_INTR					(0x21)

#define PCU_FALLING_EDGE_INTR						(0x13)
#define PCU_RISING_EDGE_INTR							(0x23)
#define PCU_BOTH_FALLING_RISING_EDGE_INTR	(0x33)




//==========================================================================
// 	PnISR
//		
//
//==========================================================================
#define PnISR_MASK							(0x03UL)


//==========================================================================
// 	PnICR
//		
//
//==========================================================================


//==========================================================================
// 	PDPR
//		
//
//==========================================================================


//=============================================================
// function declarations
//=============================================================
void PCU_ConfigureFunction (PCU_Type * PCUx, uint32_t pin_no, uint32_t func);
void PCU_Set_Direction_Type (PCU_Type * PCUx, uint32_t pin_no, uint32_t dir_type); 
void PCU_ConfigurePullup (PCU_Type * PCUx, uint32_t pin_no, uint32_t pullup); 
void PCU_ConfigurePullup_Pulldown (PCU_Type * PCUx, uint32_t pin_no, uint32_t updown, uint32_t enable); 
void PCU_ConfigureInterrupt (PCU_Type * PCUx, uint32_t pin_no, uint32_t intr_mask, uint32_t enable); 
void PCU_Debounce(PCU_Type * PCUx, uint32_t pin_no, uint8_t dpr, uint32_t enable);

#endif


