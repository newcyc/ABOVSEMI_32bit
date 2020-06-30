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
void TIMER0_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void TIMER_MatchInterruptRun(void);
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
" Using Timer in Match interrupt mode\n\r"
" PB0 toggle every 1ms \n\r"
"************************************************\n\r";
const uint8_t cmdm_t_mi[] =
"A34M41x> ";

__IO	uint32_t mfa_count_t_mi;
__IO	uint32_t mfa_flag_t_mi;


/**********************************************************************
 * @brief		TIMER0_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void TIMER0_IRQHandler_IT(void)
{
	uint32_t status;
	
	status = HAL_TIMER_GetStatus(TIMER0);
	if ((status & TIMER_SR_MFA) == TIMER_SR_MFA){
		if (mfa_flag_t_mi==0){
			mfa_flag_t_mi=1;
			HAL_GPIO_ClearPin(PB, _BIT(0));		
		}
		else{
			mfa_flag_t_mi=0;
			HAL_GPIO_SetPin(PB, _BIT(0));
		}				
	}
	HAL_TIMER_ClearCounter(TIMER0);
	HAL_TIMER_ClearStatus(TIMER0, status);			
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
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_SetPin(PB, _BIT(0));
	
	// T0O pin config set if need
	HAL_GPIO_ConfigureFunction(PA, 4, PA4_MUX_T0IO);
	HAL_GPIO_ConfigOutput(PA, 4, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PA, 4, PULL_UP_DOWN_DISABLE);
}


/**********************************************************************
 * @brief		TIMER_MatchInterruptRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void TIMER_MatchInterruptRun(void)
{
	TIMER_PERIODICCFG_Type T0_Config;

	T0_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
	T0_Config.Prescaler = 999;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 1000 = 4Khz ->250us

	T0_Config.GRA = (4000); 	//250us x 4000 = 1s	
	T0_Config.Cnt=0;		
	T0_Config.StartLevel=START_LOW;
	T0_Config.AdcTrgEn=DISABLE;
	
	if(HAL_TIMER_Init(TIMER0, PERIODIC_MODE, &T0_Config) != HAL_OK)			// timer0 setting 
	{
		Error_Handler();
	}
	HAL_TIMER_ConfigInterrupt(TIMER0, TIMER_INTCFG_MAIE, ENABLE);	// Match Interrupt Enable

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, ((0x01<<1)|0x01));
	/* Enable Interrupt for TIMER0 channel */
	NVIC_EnableIRQ(TIMER0_IRQn);

	HAL_TIMER_Cmd(TIMER0, ENABLE); // timer start
	
	__enable_irq();
		
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
	DEBUG_MenuPrint();

	/*Configure port peripheral*/
	GPIO_Configure();

	/* Enable IRQ Interrupts */
	__enable_irq();

	TIMER_MatchInterruptRun();
	
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

