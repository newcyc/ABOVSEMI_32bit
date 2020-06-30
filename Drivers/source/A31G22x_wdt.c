/***************************************************************************//**
* @file     A31G22x_wdt.c
* @brief    Contains all functions support for watchdog timer driver on A31G22x
* @author   AE Team, ABOV Semiconductor Co., Ltd.
* @version  V0.0.1
* @date     30. Jul. 2018
*
* Copyright(C) 2018, ABOV Semiconductor
* All rights reserved.
*
*
********************************************************************************
* DISCLAIMER 
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

/*******************************************************************************
* Included File
*******************************************************************************/
#include "A31G22x_wdt.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define WDT_CR_WTIDKY					(0x5A69UL << WDT_CR_WTIDKY_Pos)
#define WDT_CR_RSTEN_ENABLE_RESET		(0x00UL << WDT_CR_RSTEN_Pos)
#define WDT_CR_RSTEN_DISABLE_RESET		(0x25UL << WDT_CR_RSTEN_Pos)
#define WDT_CR_CNTEN_ENABLE_WDT			(0x00UL << WDT_CR_CNTEN_Pos)
#define WDT_CR_CNTEN_DISABLE_WDT		(0x1AUL << WDT_CR_CNTEN_Pos)

#define WDT_SR_STATUS_GET_MASK			(WDT_STATUS_WINDOW_MATCH_INT | WDT_STATUS_UNDERFLOW_INT)
#define WDT_SR_STATUS_CLEAR_MASK		(WDT_STATUS_WINDOW_MATCH_INT | WDT_STATUS_UNDERFLOW_INT)

#define WDT_CNTR_CNTR_RELOAD			(0x6AUL << WDT_CNTR_CNTR_Pos)

#define WDT_SR_STATUS_DEBUG_ENABLE		(0x01UL << WDT_SR_DBGCNTEN_Pos)

/*******************************************************************************
* Private Typedef
*******************************************************************************/


/*******************************************************************************
* Private Variable
*******************************************************************************/


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/


/*******************************************************************************
* Public Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Initialize watchdog timer peripheral
* @param      pConfig : Pointer contains configuration of watchdog timer
* @return     None
*******************************************************************************/
void WDT_Init(WDT_CFG_Type *pConfig)
{
	volatile uint32_t Reg32;

	WDT->DR = (pConfig->Data & WDT_DR_DATA_Msk);
	WDT->WINDR = (pConfig->WindowData & WDT_WINDR_WDATA_Msk);

	Reg32 = ((pConfig->ClockDivider << WDT_CR_CLKDIV_Pos) & WDT_CR_CLKDIV_Msk);
	if (pConfig->EnableReset == TRUE) {
		Reg32 |= WDT_CR_RSTEN_ENABLE_RESET;
	} else {
		Reg32 |= WDT_CR_RSTEN_DISABLE_RESET;
	}

	WDT->CR = WDT_CR_WTIDKY | WDT_CR_CNTEN_DISABLE_WDT | Reg32;
}

/***************************************************************************//**
* @brief      De-Initialize watchdog timer peripheral
* @param      None
* @return     None
*******************************************************************************/
void WDT_DeInit(void)
{
	WDT_Stop();
}

/***************************************************************************//**
* @brief      Configure interrupt of watchdog timer
* @param      EnableWindowMatch :
*              - TRUE : Enable window match interrupt
*              - FALSE : Disable window match interrupt
* @param      EnableUnderflow :
*              - TRUE : Enable underflow interrupt
*              - FALSE : Disable underflow interrupt
* @return     None
*******************************************************************************/
void WDT_ConfigureInterrupt(WDT_INT_WMI_Type EnableWindowMatch, WDT_INT_UNFI_Type EnableUnderflow)
{
	volatile uint32_t Reg32;
	
	Reg32 = WDT->CR;
	Reg32 &= ~(WDT_CR_WINMIEN_Msk | WDT_CR_UNFIEN_Msk);
	Reg32 |= ((EnableWindowMatch << WDT_CR_WINMIEN_Pos) | (EnableUnderflow << WDT_CR_UNFIEN_Pos));

	WDT->CR = WDT_CR_WTIDKY | Reg32;
}

/***************************************************************************//**
* @brief      Reload WDT counter
* @param      None
* @return     None
*******************************************************************************/
void WDT_ReloadTimeCounter(void)
{
	WDT->CNTR = WDT_CNTR_CNTR_RELOAD;
}

/***************************************************************************//**
* @brief      Get the current count of watchdog timer
* @param      None
* @return     Current count of watch timer
*******************************************************************************/
uint32_t WDT_GetCurrentCount(void)
{
	return WDT->CNT;
}

/***************************************************************************//**
* @brief      Get watchdog timer status 
* @param      None
* @return     WDT_STATUS_Type
*              - WDT_STATUS_UNDERFLOW_INT : Underflow occurred
*              - WDT_STATUS_WINDOW_MATCH_INT : Window match occurred
*******************************************************************************/
WDT_STATUS_Type WDT_GetStatus(void)
{
	return (WDT_STATUS_Type)(WDT->SR & WDT_SR_STATUS_GET_MASK);
}

/***************************************************************************//**
* @brief      Clear status of watchdog timer
* @param      Status : Status selected, should be
*              - WDT_STATUS_UNDERFLOW_INT : Underflow occurred
*              - WDT_STATUS_WINDOW_MATCH_INT : Window match occurred
* @return     None
*******************************************************************************/
void WDT_ClearStatus(WDT_STATUS_Type Status)
{
	volatile uint32_t Reg32 = 0;
	
	if ((Status & WDT_STATUS_UNDERFLOW_INT) == WDT_STATUS_UNDERFLOW_INT) {
		Reg32 |=  WDT_STATUS_UNDERFLOW_INT;
	}

	if ((Status & WDT_STATUS_WINDOW_MATCH_INT) == WDT_STATUS_WINDOW_MATCH_INT) {
		Reg32 |=  WDT_STATUS_WINDOW_MATCH_INT;
	}
	
	if ((WDT->SR & WDT_SR_STATUS_DEBUG_ENABLE) == WDT_SR_STATUS_DEBUG_ENABLE) {
		Reg32 |= WDT_SR_STATUS_DEBUG_ENABLE;
	}
	
	WDT->SR = Reg32;
}

/***************************************************************************//**
* @brief      Start watchdog timer
* @param      None
* @return     None
*******************************************************************************/
void WDT_Start(void)
{
	volatile uint32_t Reg32;
	
	Reg32 = WDT->CR;
	Reg32 &= ~WDT_CR_CNTEN_Msk;
	Reg32 |= WDT_CR_CNTEN_ENABLE_WDT;

	WDT->CR = WDT_CR_WTIDKY | Reg32;
}

/***************************************************************************//**
* @brief      Stop watchdog timer
* @param      None
* @return     None
*******************************************************************************/
void WDT_Stop(void)
{
	volatile uint32_t Reg32;
	
	Reg32 = WDT->CR;
	Reg32 &= ~WDT_CR_CNTEN_Msk;
	Reg32 |= WDT_CR_CNTEN_DISABLE_WDT;

	WDT->CR = WDT_CR_WTIDKY | Reg32;
}

/***************************************************************************//**
* @brief      Watch-dog Timer Counter Enable when core is halted in the debug mode.
* @param      Mode : Enable Watch-dog Timer Counter in the debug mode, should be
*              - ENABLE : Enable WDT Counter in Debug Mode.
*              - DISABLE : Disable WDT Counter in Debug Mode.
* @return     None
*******************************************************************************/
void WDT_CountEnableInDebug(FunctionalState Mode) 
{
	if (Mode == ENABLE) {
		WDT->SR |= WDT_SR_STATUS_DEBUG_ENABLE;
	} else {
		WDT->SR &= ~WDT_SR_STATUS_DEBUG_ENABLE;
	}
}

/* --------------------------------- End Of File ------------------------------ */
