/**********************************************************************
* @file		A34M41x_wdt.h
* @brief	Contains all macro definitions and function prototypes
* 			support for WDT firmware library on A34M41x
* @version	1.0
* @date		
* @authorABOV Application-3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
#ifndef _A34M41x_WDT_H_
#define _A34M41x_WDT_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C"
{
#endif


//----------------------- Define Macro ----------------------------------
// WDT Enable
#define WDT_ACCESS_EN()  						do { WDT->AEN=0xA55A; } while(0)
#define WDT_ACCESS_DIS()  						do { WDT->AEN=0x0000; } while(0) 
#define WDT_IMMEDIATE_RELOAD()					do { WDT->AEN=0x555A; } while(0)



/* --------------------- BIT DEFINITIONS -------------------------------------- */
/** WDT interrupt enable bit */
#define WDT_CON_CKSEL			    ((uint16_t)(1<<3))  // wdt clock sel	
#define WDT_CON_WDTEN			    ((uint16_t)(1<<4))  // wdt enable
#define WDT_CON_WDTRE			    ((uint16_t)(1<<6))  // wdt reset enable 
#define WDT_CON_WDTIE			    ((uint16_t)(1<<7))  // wdt timer count interrupt enable 
#define WDT_CON_WUF				    ((uint16_t)(1<<8))  // wdt underflow flag 	
#define WDT_CON_WDBG			    ((uint16_t)(1<<15))  // wdt dbg 		
/* Public Types --------------------------------------------------------------- */


/**********************************************************************
 * @brief WDT structure definitions
 **********************************************************************/
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
	uint8_t wdtResetEn;			/**< if ENABLE -> the Reset bit is enabled				*/
	uint8_t wdtCountEn;			/**< if ENABLE -> the Protect bit is enabled			*/
	uint8_t wdtClkSel;			/* SET -> external, RESET -> PCLK */
	uint16_t wdtPrescaler;		/* prescaler */
	uint8_t wdtDbgEn;			/**< if ENABLE -> the debug bit is enabled				*/
	uint32_t wdtTmrConst;		/**< Set the constant value to timeout the WDT (us)		*/
} WDT_CFG_Type;

/* Public Functions ----------------------------------------------------------- */
void HAL_WDT_Init(void);
void HAL_WDT_UpdateTimeOut(uint32_t TimeOut);
void HAL_WDT_Configure(WDT_CFG_Type wdtCfg);
void HAL_WDT_Start(FunctionalState ctrl);
uint32_t HAL_WDT_GetCurrentCount(void);

#ifdef __cplusplus
}
#endif

#endif /* _A34M41x_WDT_H_ */

/* --------------------------------- End Of File ------------------------------ */
