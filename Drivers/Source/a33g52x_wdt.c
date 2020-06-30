/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_wdt.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : August, 2017
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
#include "a33g52x_pmu.h"
#include "a33g52x_pcu.h"
#include "a33g52x_wdt.h"



/* Public Functions ----------------------------------------------------------- */
/**********************************************************************
* @brief 		Initial for Watchdog function
* @param[in]	none
* @return 		None
 **********************************************************************/
void WDT_Init(void)
{
	WDT->CON = 0; 
	WDT->LR = 0;
}

///**********************************************************************
//* @brief 		Enable/Disable WWDT activity
//* @param[in]	None
//* @return 		None
// **********************************************************************/
void WDT_Configure(WDT_CFG_Type wdtCfg)
{
	uint16_t reg_val;
	
	reg_val = 0;
	
	
	// Set Watchdog Prescaler
	reg_val = wdtCfg.wdtPrescaler;
	
	// Set Watchdog Operation in Debug mode
	if(wdtCfg.wdtDebugEn == WDT_DBG_ENABLE) {
		reg_val |= WDTCON_WDH;
	}
	else if(wdtCfg.wdtDebugEn == WDT_DBG_DISABLE) {
		reg_val &= ~WDTCON_WDH;
	}	


	if(wdtCfg.wdtInterruptEn == WDT_INTR_ENABLE) {
		reg_val |= WDTCON_WIE;
	}
	else if(wdtCfg.wdtInterruptEn == WDT_INTR_DISABLE) {
		reg_val &= ~WDTCON_WIE;
	}		
	
	if(wdtCfg.wdtResetEn == WDT_RST_ENABLE) {
		reg_val |= WDTCON_WRE;
	}
	else if(wdtCfg.wdtResetEn == WDT_RST_DISABLE){
		reg_val &= ~WDTCON_WRE;
	}

	if(wdtCfg.wdtClkSel == WDT_CLK_PMUPCSR) {
		reg_val |= WDTCON_WEC_PMUPCSR; 		// External clock from PMUPCSR
	}
	else if (wdtCfg.wdtClkSel == WDT_CLK_PCLK) {
		reg_val &= ~WDTCON_WEC_PMUPCSR; 		// PCLK
	}
	
		
	WDT->CON = reg_val;
}

///**********************************************************************
//* @brief 		Enable WWDT activity
//* @param[in]	FunctionalState ctrl 
//*						- DISABLE: wdt enable
//*						- ENABLE: wdt disable 
//* @return 		None
// **********************************************************************/
void WDT_Start(int ctrl)
{
	uint32_t		reg_val;
	if (ctrl == WDT_START){
		WDT->CON |= WDTCON_WEN;
	}
	else if(ctrl == WDT_STOP) {
		reg_val = WDT->CON;
		reg_val &= ~(WDTCON_WEN);
		WDT->CON = reg_val;
	}
}

///*********************************************************************
// * @brief 		Update WDT timeout value and feed
// * @param[in]	TimeOut	TimeOut value to be updated, should be in range:
// * 				2048 .. 134217728
// * @return		None
// *********************************************************************/
void WDT_UpdateTimeOut(int timeOut)
{
	WDT->LR = timeOut;
}


///*********************************************************************
// * @brief 		Get the current value of WDT
// * @param[in]	None
// * @return		current value of WDT
// *********************************************************************/
uint32_t WDT_GetCurrentCount(void)
{
	return WDT->CVR;
}

