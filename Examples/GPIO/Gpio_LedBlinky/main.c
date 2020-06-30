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
void SysTick_Handler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void SysTick_Configure(void);
void GPIO_LedBlinkyRun(void);
void mainloop(void);
int main (void);
//void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/
uint32_t msec;
const uint8_t cmdm_g_l[] =
"A34M41x> ";
const uint8_t menu[] =
"************************************************\n\r"
" GPIO demo \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
" GPIO LEDBlinky \n\r"
"************************************************\n\r";


/**********************************************************************
 * @brief		SysTick handler sub-routine (1ms)
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SysTick_Handler_IT(void)
{
	if(msec)msec--;
}


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
	/*Configure IOs in output push-pull mode to drive external LEDs */
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);

	HAL_GPIO_ConfigureFunction(PB, 1, PB1_MUX_PB1);
	HAL_GPIO_ConfigOutput(PB, 1, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 1, PULL_UP_DOWN_DISABLE);

	HAL_GPIO_ConfigureFunction(PB, 2, PB2_MUX_PB2);
	HAL_GPIO_ConfigOutput(PB, 2, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 2, PULL_UP_DOWN_DISABLE);

	HAL_GPIO_ConfigureFunction(PB, 3, PB3_MUX_PB3);
	HAL_GPIO_ConfigOutput(PB, 3, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 3, PULL_UP_DOWN_DISABLE);

	HAL_GPIO_ConfigureFunction(PB, 4, PB4_MUX_PB4);
	HAL_GPIO_ConfigOutput(PB, 4, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 4, PULL_UP_DOWN_DISABLE);

	HAL_GPIO_ConfigureFunction(PB, 5, PB5_MUX_PB5);
	HAL_GPIO_ConfigOutput(PB, 5, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 5, PULL_UP_DOWN_DISABLE);
}


/**********************************************************************
 * @brief		SysTick_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void SysTick_Configure( void)
{
	SysTick_Config(SystemCoreClock/1000);  //1msec interrupt 
}


/**********************************************************************
 * @brief		GPIO_LedBlinkyRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_LedBlinkyRun(void)
{	
	unsigned char i;
	
	msec = 200;
	while(msec);
	
	for(i=0;i<6;i++){
		msec = 50;
		while(msec);
		HAL_GPIO_ClearPin(PB,_BIT(i));
	}
	
	for(i=0;i<6;i++){
		msec = 50;
		while(msec);
		HAL_GPIO_SetPin(PB,_BIT(i));
	}
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
	
	/*SysTick Configure*/
	SysTick_Configure();

	 /* Enable IRQ Interrupts */
	__enable_irq();
	
   /* Infinite loop */
	while(1)
	{
		GPIO_LedBlinkyRun();
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

#if 0
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
#endif
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
