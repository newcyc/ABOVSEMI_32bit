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
void RNG_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void RNG_TestRun(void);
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
"\t RNG Demo...  \n\r"
"\t 0. RNG Polling Test \n\r"
"\t 1. RNG Interrupt Test \n\r"
"************************************************\n\r";
const uint8_t cmdm_r[] =
"A34M41x> ";

uint32_t flag_rdy;
uint32_t random_number;


/**********************************************************************
 * @brief		RNG_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void RNG_IRQHandler_IT(void)
{
	uint32_t stat;
	
	stat = HAL_RNG_GetStatus();
	
	// Data Ready Interrupt Check
	if((stat & (RNG_STAT_RDY|RNG_STAT_RDYI)) == (RNG_STAT_RDY|RNG_STAT_RDYI)){
		random_number = HAL_RNG_GetData();
		HAL_RNG_ClearPending((RNG_STAT_RDYI|RNG_STAT_RDY));
		flag_rdy = 1;
	}
	
	// Error Interrupt Check
	if(stat & (RNG_STAT_ERR|RNG_STAT_ERRI)){
		HAL_RNG_ClearPending((RNG_STAT_ERRI|RNG_STAT_ERR));
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

}


/**********************************************************************
 * @brief		RNG_TestRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void RNG_TestRun(void)
{
	uint8_t		cmdn_cnt=0, ch_rtn, mod_check[3] = {0x30, 0x31, 0x32};
		
	while(1)
	{
		if (cmdn_cnt == 0)
		{
			// RNG Interrupt Disable
			HAL_RNG_ConfigInterrupt(RNG_INTCFG_RDYIE, DISABLE);
			HAL_RNG_ConfigInterrupt(RNG_INTCFG_ERRIE, DISABLE);
			
			HAL_RNG_Init();
			
			// Generation Counter Parameter Setting
			HAL_RNG_SetGCP(0x1234);
			
			//  Cellular Automata Shift Register Setting
			HAL_RNG_SetCASRClock(RNG_CLOCK_OSC);
			
			// Linear Feedboack Shift Register Setting
			HAL_RNG_SetLFSRClock(RNG_CLOCK_OSC);
			
			HAL_RNG_SetSeed(0xABCD1234);
	
			_DBG(cmdm_r);
			cmdn_cnt=1;
		}
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		if (ch_rtn == mod_check[0])
		{
			HAL_RNG_Cmd(ENABLE);
			
			// RNG Data Ready Status Check
			while(!(HAL_RNG_GetStatus() & RNG_STAT_RDY)){}
				
			_DBG("Random Data: 0x");
			random_number = HAL_RNG_GetData();
			_DBH32(random_number);
			_DBG("\n\r\n\r");
				
			cmdn_cnt=0;
		}
		if (ch_rtn == mod_check[1])
		{
			// RNG Interrupt Enable
			HAL_RNG_ConfigInterrupt(RNG_INTCFG_RDYIE, ENABLE);
			HAL_RNG_ConfigInterrupt(RNG_INTCFG_ERRIE, ENABLE);
			
			NVIC_EnableIRQ(RNG_IRQn);
			NVIC_SetPriority(RNG_IRQn, 0);
			
			// Generation Counter Parameter Setting
			HAL_RNG_SetGCP(0xFFFF);
			
			HAL_RNG_Cmd(ENABLE);
			
			// Error Interrupt Test
			HAL_RNG_GetData();
						
			flag_rdy =0;
			while(!flag_rdy);
			_DBG("Random Data: 0x");
			_DBH32(random_number);
			_DBG("\n\r\n\r");
			
			cmdn_cnt=0;
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

	RNG_TestRun();
	
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

