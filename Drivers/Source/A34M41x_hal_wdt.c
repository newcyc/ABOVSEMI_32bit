/**********************************************************************
* @file		A34M41x_wdt.c
* @brief	Contains all functions support for WDT firmware library
* 			on A34M41x
* @version	1.0
* @date		
* @authorABOV Application-3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_wdt.h"
#include "A34M41x_hal_libcfg.h"



uint32_t _WDTCLK;

/* Public Functions ----------------------------------------------------------- */
/**********************************************************************
* @brief 		Initial for Watchdog function
* @param[in]	None
* @return 	HAL Status
 **********************************************************************/
void HAL_WDT_Init(void)
{
	WDT_ACCESS_EN();
	
	WDT->CON = 0; 
	WDT->LR = 0;
	
	WDT_ACCESS_DIS();
}

/*********************************************************************
 * @brief 		Update WDT timeout value and feed
 * @param[in]	TimeOut	TimeOut value to be updated, should be in range:
 * 				2048 .. 134217728
 * @return		HAL Status
 *********************************************************************/
void HAL_WDT_UpdateTimeOut(uint32_t TimeOut)
{
	WDT_ACCESS_EN();
	
	WDT->LR = TimeOut;
	
	WDT_ACCESS_DIS();
}

/**********************************************************************
* @brief 		Enable/Disable WDT activity
* @param[in]	Struct Type WDT
* @return 	HAL Status
 **********************************************************************/
void HAL_WDT_Configure(WDT_CFG_Type wdtCfg)
{
	uint32_t reg_val;
	
	WDT_ACCESS_EN();
	
	reg_val = 0;
	
	WDT->LR = wdtCfg.wdtTmrConst;
	
	reg_val = wdtCfg.wdtPrescaler;
	
	if(wdtCfg.wdtResetEn == ENABLE) {
		reg_val |= WDT_CON_WDTRE;
	}
	else {
		reg_val &= ~WDT_CON_WDTRE;
	}

	if(wdtCfg.wdtCountEn) {
		reg_val |= WDT_CON_WDTIE;
	}
	else {
		reg_val &= ~WDT_CON_WDTIE;
	}

	if(wdtCfg.wdtClkSel) {
		reg_val |= WDT_CON_CKSEL; //external clock from MCCR3
	}
	else {
		reg_val &= ~WDT_CON_CKSEL; // PCLK
	}
	
	if(wdtCfg.wdtDbgEn) {
		reg_val |= WDT_CON_WDBG; //Watchdog counter stops in debug mode
	}
	else {
		reg_val &= ~WDT_CON_WDBG; // Watchdog counter operation in debug mode
	}
	
	WDT->CON = reg_val;
	
	WDT_ACCESS_DIS();
}

/**********************************************************************
* @brief 		Enable WDT activity
* @param[in]	FunctionalState ctrl 
*						- DISABLE: wdt enable
*						- ENABLE: wdt disable 
* @return 		HAL Status
 **********************************************************************/
void HAL_WDT_Start(FunctionalState ctrl)
{
	WDT_ACCESS_EN();
	
	if (ctrl == ENABLE){
		WDT->CON |= WDT_CON_WDTEN;
	}
	else {
		WDT->CON &= ~WDT_CON_WDTEN;		
	}
	
	WDT_ACCESS_DIS();
}

/*********************************************************************
 * @brief 		Get the current value of WDT
 * @param[in]	None
 * @return		current value of WDT
 *********************************************************************/
uint32_t HAL_WDT_GetCurrentCount(void)
{
	return WDT->CNT;
}

/*********************************************************************
 * @brief 		Get the timer status register of WDT
 * @param[in]	None
 * @return		the status register of WDT
 *********************************************************************/
uint32_t HAL_WDT_GetStatus(void)
{
	return WDT->CON;
}


/* --------------------------------- End Of File ------------------------------ */
