/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_wdt.h
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






//==========================================================================
// 	WDTCNT
//		
//				WDTCNT			@ address = 0x4000_0404
//
//
//==========================================================================





//==========================================================================
// 	WDTCON
//		
//				WDTCON			@ address = 0x4000_0408
//
//
//==========================================================================
#define WDTCON_WDH								(0x0001<<15)

#define WDTCON_WUF								(0x0001<<8)

#define WDTCON_WIE								(0x0001<<7)
#define WDTCON_WRE								(0x0001<<6)
#define WDTCON_WEN								(0x0001<<5)

#define WDTCON_WEC								(0x0001<<3)
#define WDTCON_WEC_PCLK							(0x0000<<3)
#define WDTCON_WEC_PMUPCSR						(0x0001<<3)

#define WDTCON_WPRS_WDTCLKIN_DIV_BY_1			(0x0000<<0)
#define WDTCON_WPRS_WDTCLKIN_DIV_BY_4			(0x0001<<0)
#define WDTCON_WPRS_WDTCLKIN_DIV_BY_8			(0x0002<<0)
#define WDTCON_WPRS_WDTCLKIN_DIV_BY_16			(0x0003<<0)
#define WDTCON_WPRS_WDTCLKIN_DIV_BY_32			(0x0004<<0)
#define WDTCON_WPRS_WDTCLKIN_DIV_BY_64			(0x0005<<0)
#define WDTCON_WPRS_WDTCLKIN_DIV_BY_128			(0x0006<<0)
#define WDTCON_WPRS_WDTCLKIN_DIV_BY_256			(0x0007<<0)
#define WDTCON_WPRS_MASK						(0x0007<<0)



#define WDT_DBG_ENABLE		(1)
#define WDT_DBG_DISABLE		(0)

#define WDT_INTR_ENABLE		(1)
#define WDT_INTR_DISABLE		(0)

#define WDT_RST_ENABLE		(1)
#define WDT_RST_DISABLE		(0)

#define WDT_START				(1)
#define WDT_STOP				(0)

#define WDT_CLK_PCLK			(0)
#define WDT_CLK_PMUPCSR	(1)

enum {
		WDT_DIV_1=0,
		WDT_DIV_4,
		WDT_DIV_8,
		WDT_DIV_16,
		WDT_DIV_32,
		WDT_DIV_64,
		WDT_DIV_128,
		WDT_DIV_256	
};

typedef struct Wdt_Config
{
	uint8_t wdtDebugEn;			/**< if ENABLE -> the WDT enable in debug mode	*/
	uint8_t wdtInterruptEn;			/**< if ENABLE -> the Interrupt bit is enabled				*/	
	uint8_t wdtResetEn;			/**< if ENABLE -> the Reset bit is enabled				*/
	uint8_t wdtClkSel;				/* SET -> external, RESET -> PCLK */
	uint8_t wdtPrescaler;			/* prescaler */
	
} WDT_CFG_Type;


/* Public Functions ----------------------------------------------------------- */
void WDT_Init(void);
void WDT_Configure(WDT_CFG_Type wdtCfg);
void WDT_Start(int ctrl);
void WDT_UpdateTimeOut(int timeOut);
uint32_t WDT_GetCurrentCount(void);

