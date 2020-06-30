/**********************************************************************
* @file		A34M41x_SystemClock.c
* @brief	Contains all macro definitions and function prototypes
* 			support for PCU firmware library
* @version	1.0
* @date		
* @author	ABOV M team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/
#include "main_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USED_CLKO
/*#define USED_LSI	*/	
/*#define USED_HSI */		
#define USED_HSE		
/*#define USED_LSE	*/	
/*#define USED_PLL120	*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config (void);
/* Private variables ---------------------------------------------------------*/


/**************************************************************************
 * @brief			Initialize default clock for A34M41x Board
 * @param[in]		None
 * @return			None
 **************************************************************************/
void SystemClock_Config (void)
{
	// Flash Wait Config
	HAL_CFMC_WaitCmd(7);
	
	// CLKO Function Setting, Check PORT Setting (PC9)
#ifdef USED_CLKO
	HAL_SCU_CORCmd(SCU_COR_CLKOINSEL_MCLK, 0x5, ENABLE);			// CLKO=PCLK, Divider=5
#else
	HAL_SCU_CORCmd(SCU_COR_CLKOINSEL_MCLK, 0x5, DISABLE);			
#endif
	
	//!! For Clock Change, Enable All Clock Source !!
	HAL_SCU_ClockSRCCmd((SCU_CSCR_LSE_ON|SCU_CSCR_HSE_ON|SCU_CSCR_HSI_ON|SCU_CSCR_LSI_ON), ENABLE);
	
	// Main Clock Selection
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_LSI);
	
	// MCLK Monitoring Disable
	HAL_SCU_CLKMNTCmd(SCU_CMR_MCLKMNT, DISABLE);
	
#ifdef USED_LSI		//500khz
	
	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_LSI);

	SystemCoreClock = 500000;
	SystemPeriClock = 500000;

	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_LSI, 3);	
#endif

#ifdef USED_HSI		//32Mhz

	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_HSI);
	
	SystemCoreClock = 32000000;
	SystemPeriClock = 32000000;	

	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_HSI, 2);
#endif

#ifdef USED_HSE		//8MHz

	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_HSE);
	
	SystemCoreClock = 8000000;
	SystemPeriClock = 8000000;
	
	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_HSE, 2);
#endif

#ifdef USED_LSE		//32.768khz

	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_LSE);
	
	SystemCoreClock = 32768;
	SystemPeriClock = 32768;
	
	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_LSE, 2);
#endif

#ifdef USED_PLL120		// 120MHz, Source HSE

	CFMC_WaitCmd(7);
	
	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_PLLINCLKCmd(SCU_SCCR_PLLCLKSEL_HSE);	// PLL Input Clock = HSE
	
	HAL_SCU_PLLPREDIVCmd(0);		// PLLINCLK = 8MHz
	
	HAL_SCU_PLLCmd(SCU_PLLCON_PLLMODE_FOUT, 3, 59, 0, 0, ENABLE);

	// Lock Status Check
	while(1)
	{
		if ((SCU_GetPLLStatus()&(SCU_PLLCON_LOCKSTS))==SCU_PLLCON_LOCKSTS)
		{
			break;
		}
	}
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_PLL);
	
	SystemCoreClock = 120000000;
	SystemPeriClock = 120000000;
	
	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_PLL, 2);
#endif

	
}


