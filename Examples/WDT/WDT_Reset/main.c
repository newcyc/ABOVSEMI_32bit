/**********************************************************************
* @file		main.c
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
/* Private function prototypes -----------------------------------------------*/
void DEBUG_Print_menu(void);
void DEBUG_Init(void);
void PCU_Configure(void);
void WDT_Reset_RUN(void);
void mainloop(void);
int main (void);
static void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
"Watch dog timer Reset demo\n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"Systick Interrupt(0.5ms) Run 20 times, after WDTLR(1s) reset \n\r"
"************************************************\n\r";

uint32_t msec_w_r;


/**********************************************************************
 * @brief		SysTick_Handler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SysTick_Handler_IT(void)
{
	if(msec_w_r)msec_w_r--;
}


/**********************************************************************
 * @brief		Print menu
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void DEBUG_Print_menu(void)
{
	#ifdef _DEBUG_MSG
	_DBG(menu);
	#endif
}


/**********************************************************************
 * @brief		DEBUG_Init
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void DEBUG_Init(void)
{
	#ifdef _DEBUG_MSG
	debug_frmwrk_init();
	#endif
}


/**********************************************************************
 * @brief		PCU_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void PCU_Configure(void)
{
	// Test Pin setting PB0
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_SetPin(PB, _BIT(0));
}


/**********************************************************************
 * @brief		WDT_Reset_RUN
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void WDT_Reset_RUN(void)
{
	WDT_CFG_Type wdtCfg;
	__IO uint32_t wdtcnt;
	uint8_t i,j;
		
	HAL_WDT_Init();

	HAL_SCU_SetMCCRx(1, WDT_TYPE, SCU_MCCR_CSEL_HSE, 1);		// HSE = 8MHz
	
	wdtCfg.wdtResetEn = ENABLE;
	wdtCfg.wdtCountEn = DISABLE;
	wdtCfg.wdtClkSel = SET; 				// SET:external from MCCR1, RESET:PCLK
	wdtCfg.wdtPrescaler = WDT_DIV_1;  	// WDTCLK = WDTCLKIN / PRS =  8MHz / 1 = 125ns
	wdtCfg.wdtTmrConst = 8000000; 		// 1s
	
	HAL_WDT_Configure(wdtCfg);
	
	SysTick_Config(SystemCoreClock/1000);  //1msec interrupt 
	
	__enable_irq();
	
	WDT_ACCESS_EN();
		
	HAL_WDT_Start(ENABLE);
	
	/* Systick Interrupt(0.5ms) Run 20 times, after WDTLR(1s) reset */
	i=20;j=0;
	while(i){
		msec_w_r = 500;
		while(msec_w_r);
		
		HAL_WDT_UpdateTimeOut(8000000);
		i--;
		
		if (j==0){
			j=1;
			HAL_GPIO_ClearPin(PB, _BIT(0));
		}
		else {
			j=0;
			HAL_GPIO_SetPin(PB, _BIT(0));
		}
	}
	while(1);	
}


/**********************************************************************
 * @brief		Main loop
 * @param[in]	None
 * @return	None
 **********************************************************************/
void mainloop(void)
{
	/*Configure menu prinf*/
	DEBUG_Print_menu();

	/*Configure port peripheral*/
	PCU_Configure();
  
	/* Enable IRQ Interrupts */
	__enable_irq();

	WDT_Reset_RUN();
	
	while(1)
	{

	}
}

/**********************************************************************
 * @brief		Main program
 * @param[in]	None
 * @return	None
 **********************************************************************/
int main (void)
{

	 /* Initialize all port */
	Port_Init(); 

	/* Configure the system clock to HSE 8 MHz */
	SystemClock_Config();
	
	/* Initialize Debug frame work through initializing USART port  */
	DEBUG_Init();		
	
	/* Infinite loop */
	mainloop();  


	return (0);
}

/**********************************************************************
  * @brief  Reports the name of the source file and the source line number
  *   where the check_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: check_param error line source number
  * @retval : None
 **********************************************************************/
void Error_Handler(void)
{
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**********************************************************************
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
 **********************************************************************/
void check_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

   /* Infinite loop */
   while (1)
   {
   }
}
#endif

