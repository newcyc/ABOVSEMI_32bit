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
void GPIOA_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void delay(void);
void PA0_EXINT_TEST(void);
void mainloop(void);
int main (void);

	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
" GPIO demo \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
" This example used to configure the\n\r "
" GPIO PA0 as source of external Interrupt \n\r"
"************************************************\n\r";
uint8_t mode_level_g_e[]=
	"\t - 0: Prohibit external Interrupt \n\r"
	"\t - 1: low level Interrupt \n\r"
	"\t - 2: high level Interrupt \n\r";
uint8_t mode_edge_g_e[]=
	"\t - 0: Prohibit external Interrupt \n\r"
	"\t - 1: falling edge Interrupt \n\r"
	"\t - 2: rising edge Interrupt \n\r"
	"\t - 3: both edge Interrupt \n\r";

uint8_t flag;
uint8_t val_level_or_edge, mode;

/**********************************************************************
 * @brief		SysTick handler sub-routine (1ms)
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void GPIOA_IRQHandler_IT(void)
{
	uint32_t status;

	// PA0 External Interrupt
	status = 0;
	status = HAL_GPIO_EXTI_GetStatus(PA);

	if(status)
	{
		HAL_GPIO_EXTI_ClearPin(PA, status);
		_DBG("\r\nPA0 Interrupt ok!!\r\n");
		HAL_GPIO_SetPin(PB, _BIT(0));
		delay();
		HAL_GPIO_ClearPin(PB, _BIT(0));
		delay();
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
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);	
	
	// external interrupt test pin PA0
	HAL_GPIO_ConfigureFunction(PA, 0, PA0_MUX_PA0);
	HAL_GPIO_ConfigOutput(PA, 0, INPUT);
	HAL_GPIO_ConfigPullup(PA, 0, PULL_UP_ENABLE);	
}


/**********************************************************************
 * @brief		delay
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void delay(void)
{
	uint32_t i;
	
	for (i = 0; i < 0x100000; i++) ;	
}


/**********************************************************************
 * @brief		GPIO_ExitIntRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_ExitIntRun(void)
{	

	HAL_SCU_SetMCCRx(4,PGA_TYPE,SCU_MCCR_CSEL_LSI,100);		//LSI=500KHz, N=100, 500KHz/100=5KHz, 0.2ms
	
	/* Get some inputs */
	_DBG("\n\rPlease input character 1:LEVEL(Non-pending), 2:LEVEL(Pending), 3: EDGE! \n\r");	

	val_level_or_edge = 0;
	flag = 1;	

	// Setting val_level_or_edge
	while(flag)
	{
		val_level_or_edge = _DG;
		switch (val_level_or_edge)
		{
			case '1':
				_DBC(val_level_or_edge);
				_DBG(" : Level(non-pending) is selected  \n\r");
				_DBG(mode_level_g_e);
				val_level_or_edge = IER_LEVEL_NON_PENDING;
				flag = 0; 
				break;			
			case '2': 
				_DBC(val_level_or_edge);
				_DBG(" : Level(pending) is selected  \n\r");
				_DBG(mode_level_g_e);
				val_level_or_edge = IER_LEVEL_PENDING;
				flag = 0; 
				break;
			case '3':
				_DBC(val_level_or_edge);
				_DBG(" : Edge is selected  \n\r");
				_DBG(mode_edge_g_e);		
				val_level_or_edge = IER_EDGE;
				flag = 0; 
				break;
			default:
				_DBG("\n\r...Please input character 1:LEVEL(Non-pending), 2:LEVEL(Pending), 3: EDGE!  \n\r");
				break;
		}
	}

	// Setting mode
	_DBG("\n\r# Select INT Mode : ");
	mode = 0;
	flag = 1;
	while (flag)
	{
		mode = _DG;
		if (val_level_or_edge==IER_LEVEL_NON_PENDING){
			switch (mode)
			{
				case '0':
					_DBC(mode);
					_DBG(" : Prohibit external interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 
					break;
				case '1': 
					_DBC(mode);
					_DBG(" : low level interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 
					break;
				case '2': 					
					_DBC(mode);
					_DBG(" : high level interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 
					break;
				default:
					_DBG("\n\r...Please input digit from 0 to 2 only! \n\r");
					break;
			}
		}

		else if (val_level_or_edge==IER_LEVEL_PENDING)
		{
			switch (mode)
			{
				case '0':
					_DBC(mode);
					_DBG(" : Prohibit external interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 
					break;
				case '1': 
					_DBC(mode);
					_DBG(" : low level interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 
					break;
				case '2': 					
					_DBC(mode);
					_DBG(" : high level interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 
					break;
				default:
					_DBG("\n\r...Please input digit from 0 to 2 only! \n\r");
					break;
			}			
		}
		
		else if (val_level_or_edge==IER_EDGE)
		{
			switch (mode)
			{
				case '0':
					_DBC(mode);
					_DBG(" : Prohibit external interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 					
					break;
				case '1': 
					_DBC(mode);
					_DBG(" : falling edge interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 
					break;
				case '2': 		
					_DBC(mode);
					_DBG(" : rising edge interrupt  \n\r");				
					mode = mode -0x30;
					flag = 0; 
					break;
				case '3': 						
					_DBC(mode);
					_DBG(" : both edge interrupt  \n\r");
					mode = mode -0x30;
					flag = 0; 
					break;
				default:
					_DBG("\n\r...Please input digit from 0 to 3 only! \n\r");
					break;
			}			
		}
	}

	HAL_GPIO_EXTI_Config(PA, 0, val_level_or_edge, mode);

	NVIC_SetPriority(GPIOA_IRQn, 3);		
	NVIC_EnableIRQ(GPIOA_IRQn);
	__enable_irq();
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
	
	GPIO_ExitIntRun();

	/* Infinite loop */
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

	/* Configure the system clock to 48 MHz */
	SystemClock_Config();
	
	/* Initialize Debug frame work through initializing USART port  */
	DEBUG_Init();		
	
	/* Infinite loop */
	mainloop();  
	

	return (0);
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

