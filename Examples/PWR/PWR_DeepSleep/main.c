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
void GPIOB_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void PWR_DeepSleepRun(void);
void mainloop(void);
int main (void);
//void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
" Deep Sleep demo \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
" PB0 toggle every 1s after wake up \n\r"
"************************************************\n\r";
uint32_t msec_p_d;


/**********************************************************************
 * @brief		SysTick_Handler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SysTick_Handler_IT(void)
{
	if(msec_p_d)msec_p_d--;	
}


/**********************************************************************
 * @brief		GPIOB_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void GPIOB_IRQHandler_IT(void)
{
	uint32_t status;
	
	status = HAL_GPIO_EXTI_GetStatus(PB);
	if (status & (3<<(1<<1)))
	{
		HAL_GPIO_EXTI_ClearPin(PB, status&(3<<(1<<1)));
	}	
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
	// Test Pin setting PB0
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_ClearPin(PB, _BIT(0));
	
	// External Interrupt Test pin PB1
	HAL_GPIO_ConfigOutput(PB, 1, INPUT);
	HAL_GPIO_ConfigPullup(PB, 1, PULL_UP_ENABLE);

	// PB1 External Interrupt : Falling Edge
	HAL_GPIO_EXTI_Config(PB, 1, IER_EDGE, ICR_FALLING_EDGE_INT);
	
	NVIC_SetPriority(GPIOE_IRQn, 7);
	NVIC_EnableIRQ(GPIOE_IRQn);
	
	// External Test pin PB2
	HAL_GPIO_ConfigOutput(PB, 2, INPUT);
	HAL_GPIO_ConfigPullup(PB, 2, PULL_DOWN_ENABLE);
}


/**********************************************************************
 * @brief		PWR_DeepSleepRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void PWR_DeepSleepRun(void)
{
	// PB Wakeup soure Enable
	HAL_SCU_WakeUpSRCCmd(GPIOBWUE, ENABLE);
	
	SysTick_Config(SystemCoreClock/1000);  //1msec interrupt 
	
	__enable_irq();
	
	_DBG("Press PB2 to VDD enter deep sleep mode !");
	while((HAL_GPIO_ReadPin(PB) & (1UL<<2)) != (1UL<<2));
		
	_DBG("\n\rEnter Deep Sleep... Connect PB1 pin to VDD to exit sleep mode !  ");
	
	//System Clock : LSI Setting
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_LSI);

	// Except LSI, Clock Disable
	HAL_SCU_ClockSRCCmd((SCU_CSCR_LSE_ON|SCU_CSCR_HSE_ON|SCU_CSCR_HSI_ON), DISABLE);
	
	// Run PowerDown Mode 
	HAL_PWR_EnterPowerDownMode();
	
	HAL_SCU_ClockInit();
	
	_DBG("\n\rWaked Up from Deep sleep mode!");
	
	while(1)
	{
		msec_p_d = 1000;
		while(msec_p_d);
		
		HAL_GPIO_SetPin(PB, _BIT(0));
		
		msec_p_d = 1000;
		while(msec_p_d);
		
		HAL_GPIO_ClearPin(PB, _BIT(0));
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

	PWR_DeepSleepRun();
	
//	while(1)
//	{

//	}
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

