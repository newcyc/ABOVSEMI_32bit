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
void FRT0_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void FRT_Configure(void);
void FRT_TimerRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);	

uint32_t matchintcnt;
uint32_t overflowintcnt;
uint32_t g_FRT_Flag;
uint32_t g_FRT_Frun_init;

uint32_t 	counter;
uint8_t		cmdn_cnt=0, ch_rtn, ch_mtn, mod_check[3] = {0x30, 0x31, 0x32};
	
/* Private variables ---------------------------------------------------------*/	
const uint8_t cmdm[] =
"A34M41x> ";
const uint8_t menu[] =
"************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t FRT Test Example \n\r"
"\t 1. FRT Init \n\r"
"\t 2. FRT Test \n\r"
"************************************************\n\r";
const uint8_t modmsg[] =
"-----------------------------------------------\n\r"
"\t FRT Mode Selection\r\n"
"\t\t 0. Freerun Mode\r\n"
"\t\t 1. Match Mode\r\n"
"-----------------------------------------------\n\r";

/**********************************************************************
 * @brief		FRT_IRQHandler 
 * @param[in]	none
 * @return 		none
 **********************************************************************/
void FRT0_IRQHandler_IT(void)
{
	uint16_t status;
	
	status = HAL_FRT_GetStatus(FRT0);				// Get FRT Status

	HAL_FRT_ClearCounter(FRT0);
	
	if (status & FRT_STAT_MATCHI){ // match interrupt  & PB0 Toggle
		HAL_FRT_ClearStatus(FRT0,FRT_STAT_MATCHI);
		PB->ODR ^=0x01;
		matchintcnt++;
	}
	
	if (status &FRT_STAT_OVFI){ // overflow interrupt & PB1 Toggle
		HAL_FRT_ClearStatus(FRT0,FRT_STAT_OVFI);
		PB->ODR ^=0x02;
		overflowintcnt++;
	}
	
	g_FRT_Flag=1;
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
		// Pin setting PB0
		HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
		HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
		HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);	
		HAL_GPIO_SetPin(PB, _BIT(0));

		// Pin setting PB1
		HAL_GPIO_ConfigureFunction(PB, 1, PB1_MUX_PB1);
		HAL_GPIO_ConfigOutput(PB, 1, PUSH_PULL_OUTPUT);
		HAL_GPIO_ConfigPullup(PB, 1, PULL_UP_DOWN_DISABLE);	
		HAL_GPIO_SetPin(PB, _BIT(1));
}

/**********************************************************************
 * @brief		FRT_TimerRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void FRT_TimerRun(void)
{	
	if (cmdn_cnt == 0)
		{
			_DBG(cmdm);
			cmdn_cnt=1;
		}
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		if (ch_rtn == 0x0D)
		{
			_DBG("\r\n");
		}
		
		if (ch_rtn == mod_check[1])
		{				
			
			_DBG(modmsg);	// FRT mode(Freerun/Match) select
			ch_mtn = UARTGetChar(UART0);
			_DBC(ch_mtn);
			_DBG("\r\n");
			
			/* FRT Initialization */
			if (ch_mtn == 0x30){
				if((HAL_FRT_Init(FRT0, FRT_CTRL_MODE_FREE_RUN))!=HAL_OK)
				{
					/* Initialization Error */
					Error_Handler();
				}
			}
			else if (ch_mtn == 0x31){
				if((HAL_FRT_Init(FRT0, FRT_CTRL_MODE_MATCH))!=HAL_OK)
				{
					/* Initialization Error */
					Error_Handler();
				}
				HAL_FRT_SetMatchCounter(FRT0, 8000000); //1s
			}
			else{
				_DBG("...Please input character Check!  \n\r");
			}
			/* FRT Clock Source */
			HAL_FRT_ClockSource(FRT0, SCU_MCCR_CSEL_HSE, 1);
		
			/* FRT Interrupt(Freerun/Match) Setting */
			if (ch_mtn == 0x30)
			{
				HAL_FRT_OverflowInterrupt(FRT0, ENABLE);
			}
			else if (ch_mtn == 0x31)
			{
				HAL_FRT_MatchInterrupt(FRT0, ENABLE);
			}
		
			NVIC_SetPriority(FRT0_IRQn, ((0x01<<1)|0x01));	// SET FRT Interrupt Priority
			NVIC_EnableIRQ(FRT0_IRQn);						// ENABLE FRT Interrupt 	
					
			__enable_irq();
			
			cmdn_cnt=0;
		}
		else if (ch_rtn == mod_check[2])
		{
			if (ch_mtn == 0x30)		// Freerun Mode
			{
				counter = 0;
				overflowintcnt=0;
				
				HAL_FRT_Run(FRT0, ENABLE);			// FRT Enable
				HAL_FRT_ClearCounter(FRT0);		// Clear FRT Counter
						
				while(1)
				{		
					
					if(overflowintcnt == 0 && g_FRT_Frun_init == 0)
					{
						counter = overflowintcnt;
						_DBG(" - Count :"); _DBH32(counter); _DBG("\r\n");
						g_FRT_Frun_init = 1;
					}
					
					if(g_FRT_Flag == 1)
					{
						counter = overflowintcnt;		// Get FRT Counter Value
						_DBG("\n - Count :"); _DBH32(counter); _DBG("\r\n");
						g_FRT_Flag = 0;
					}
					
					_DBG(" - FRT->CNT : "); _DBH32(HAL_FRT_GetCounterVal(FRT0)); _DBG("\r");
					
				}
			}
			else if (ch_mtn == 0x31)	// Match Mode
			{
				counter = 0;
				matchintcnt=0;
				
				HAL_FRT_Run(FRT0, ENABLE);			// FRT Enable
				HAL_FRT_ClearCounter(FRT0);		// Clear FRT Counter
				
				while(1) 
				{		
					if(g_FRT_Flag == 1)
					{
						counter = matchintcnt;		// Get FRT Counter 
						_DBG(" - Count :"); _DBD16(counter); _DBG("\r\n");
						g_FRT_Flag = 0;
					}
									
				}
			}
			
			cmdn_cnt=0;
		}
		else 
		{
			_DBG("...Please input character 1:Init, 2:Test!  \n\r");
			cmdn_cnt=0;
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
	
	/* Enable IRQ Interrupts */
	__enable_irq();
	
	/*PCU Configure*/
  GPIO_Configure(); 
  	
   /* Infinite loop */
  while(1)
	{
		/* FRT Message */
		FRT_TimerRun();
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

	/* Configure the system clock to 8 MHz */
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


