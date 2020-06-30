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
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void Timer_PWM_RUN(void);
void mainloop(void);
int main (void);
void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
" Using Timer in PWM mode\n\r"
" PA4 - DUTY : 0.5ms, PERIOD : 2ms \n\r"
"************************************************\n\r";
const uint8_t cmdm_t_p[] =
"A34M41x> ";


/**********************************************************************
 * @brief		Print menu
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void DEBUG_MenuPrint(void)
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
void GPIO_Configure(void)
{
	// T0O pin config set if need
	HAL_GPIO_ConfigureFunction(PA, 4, PA4_MUX_T0IO);
	HAL_GPIO_ConfigOutput(PA, 4, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PA, 4, PULL_UP_DOWN_DISABLE);
	
	HAL_GPIO_ConfigureFunction(PA, 3, PA3_MUX_PA3);
	HAL_GPIO_ConfigOutput(PA, 3, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PA, 3, PULL_UP_DOWN_DISABLE);
	HAL_GPIO_ClearPin(PA,_BIT(3));	
}


/**********************************************************************
 * @brief		TIMER_PWMRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void TIMER_PWMRun(void)
{
	int i;
	
	TIMER_PWMCFG_Type Tx_Config;

	Tx_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
	Tx_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

	Tx_Config.GRA = (500); // 0.5msec	DUTY = GRA
	Tx_Config.GRB = (2000); // 2msec	PERIOD = GRB
	Tx_Config.Cnt=0;		
	Tx_Config.StartLevel=START_LOW;
//	Tx_Config.StartLevel=START_HIGH;	
	Tx_Config.AdcTrgEn=DISABLE;
	
	if(HAL_TIMER_Init(TIMER0, PWM_MODE, &Tx_Config) != HAL_OK) // timer0 setting 
	{
		Error_Handler();
	}

	for(i=0; i<10000; i++);
	HAL_GPIO_SetPin(PA,_BIT(3));		
	HAL_TIMER_Cmd(TIMER0, ENABLE); // timer start

	while(1){}	
}


/**********************************************************************
 * @brief		Main loop
 * @param[in]	None
 * @return	None
 **********************************************************************/
void mainloop(void)
{
	/*Configure menu prinf*/
	DEBUG_MenuPrint();

	/*Configure port peripheral*/
	GPIO_Configure();
  
	/* Enable IRQ Interrupts */
	__enable_irq();

	/*timer PWM */
	TIMER_PWMRun();
	
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

