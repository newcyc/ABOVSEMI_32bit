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
void UART_PollingRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"************************************************\n\r";


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
	// Initialize UART0 pin connect
	HAL_GPIO_ConfigureFunction(PC, 14, PC14_MUX_RXD0);
	HAL_GPIO_ConfigOutput(PC, 14, INPUT);
	HAL_GPIO_ConfigPullup(PC, 14, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PC, 15, PC15_MUX_TXD0);
	HAL_GPIO_ConfigOutput(PC, 15, PUSH_PULL_OUTPUT);	
	HAL_GPIO_ConfigPullup(PC, 15, PULL_UP_ENABLE);	
}


/**********************************************************************
 * @brief		UART_PollingRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void UART_PollingRun(void)
{
	UART_CFG_Type UARTConfigStruct;
	uint8_t ch;

	// default : 38400-8-N-1
	HAL_UART_ConfigStructInit(&UARTConfigStruct);
	
	// Initialize peripheral with given to corresponding parameter
	if(HAL_UART_Init(UART0, &UARTConfigStruct) != HAL_OK)
	{
		Error_Handler();
	}
	
	HAL_UART_Transmit(UART0,(uint8_t *)"Polling test \n\r",15,BLOCKING);	
	HAL_UART_Transmit(UART0,(uint8_t *)"- Press any button !! \n\r",26,BLOCKING);	
	
	while(1){
		HAL_UART_Receive(UART0,&ch,1,BLOCKING);
		HAL_UART_Transmit(UART0, &ch,1,BLOCKING);
		
		if (ch==0x0d) 
		{
			HAL_UART_Transmit(UART0,(uint8_t *)"\n\r",2,BLOCKING);
		}
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

	/* Enable IRQ Interrupts */
	__enable_irq();

	/* uart start */
	UART_PollingRun();
	
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

