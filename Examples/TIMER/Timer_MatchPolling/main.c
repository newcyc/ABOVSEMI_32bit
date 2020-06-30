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
void TIMER_MatchPollingRun(void);
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
" Using Timer in Match Polling mode\n\r"
" PB0 toggle every 1ms \n\r"
"************************************************\n\r";
const uint8_t cmdm_t_mp[] =
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
 * @brief		TIMER_MatchPolling_RUN
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void TIMER_MatchPollingRun(void)
{
	TIMER_PERIODICCFG_Type Tx_Config;
	TIMER_Type *Tx;
	uint32_t a_count, a_flag;
	
	Tx = TIMER0;
	
	Tx_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
	Tx_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

	Tx_Config.GRA = (1000); // 1us * 1000 = 1ms		
	Tx_Config.Cnt=0;		
	Tx_Config.StartLevel=START_LOW;
	Tx_Config.AdcTrgEn=DISABLE;
	
	if(HAL_TIMER_Init(Tx, PERIODIC_MODE, &Tx_Config) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIMER_Cmd(Tx, ENABLE); // timer start & clear
		
	while(1){
		
		if((HAL_TIMER_GetStatus(Tx) & TIMER_SR_MFA)== TIMER_SR_MFA){	
			HAL_TIMER_ClearStatus(Tx, TIMER_SR_MFA);
			HAL_TIMER_ClearCounter(Tx);
			a_count++;
			if (a_count >= 1000){	// 1ms * 1000 = 1s
				a_count = 0;
				if (a_flag==0){
					a_flag=1;
					HAL_GPIO_ClearPin(PB, _BIT(0));					
				}
				else {
					a_flag=0;
					HAL_GPIO_SetPin(PB, _BIT(0));
				}				
			}
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
	
	TIMER_MatchPollingRun();
	
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

