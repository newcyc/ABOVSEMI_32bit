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
void delay(void);
void GPIO_inputRun(void);
void mainloop(void);
int main (void);
//void Error_Handler(void);
	
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
" GPIO demo \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
" GPIO Input test \n\r"
" High : PB0 Toggle, Low : PB1 Toggle \n\r"
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
	// Test Pin setting PB0
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_ENABLE);	
	HAL_GPIO_SetPin(PB, _BIT(0));
	
	// Test Pin setting PB1
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_ENABLE);	
	HAL_GPIO_SetPin(PB, _BIT(1));
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
 * @brief		GPIO_Input_Test
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_inputRun(void)
{	
	uint8_t flag,status;
	uint8_t port, pin, tmp;
	PCU_Type *PCx;
		
	/* Get some inputs */
	_DBG("\n\rPlease enter PORT number (from A to G) \n\r");	

	port = 0;
	flag = 1;
	while (flag)
	{
		port = _DG;
		switch (port)
		{
			case 'A': _DBC(port); PCx = PA; flag = 0; break;
			case 'B': _DBC(port); PCx = PB; flag = 0; break;
			case 'C': _DBC(port); PCx = PC; flag = 0; break;
			case 'D': _DBC(port); PCx = PD; flag = 0; break;
			case 'E': _DBC(port); PCx = PE; flag = 0; break;
			case 'F': _DBC(port); PCx = PF; flag = 0; break;
			case 'G': _DBC(port); PCx = PG; flag = 0; break;
			default:
				_DBG("\n\r...Please input character from A to G only! \n\r");
				break;
		}
	}

	flag = 0;
	tmp = 0;
	pin = 0;
	
	while(flag < 1)
	{
		if(flag==0)
			_DBG("\n\rPlease enter PIN number (from 0 to 9):\n\r");
		tmp = _DG;
		switch(tmp)
		{
			case '0':case'1':case '2':case '3':case '4':case '5':case '6':case '7':case '8':case '9':
				pin = tmp - 0x30;
				_DBD(pin);
				tmp = 0;
				flag = 1;
				break;
			default:
				_DBG("\n\r...Please input digits from 0 to 9 only! \n\r");
				flag = 0; pin = 0; tmp = 0;
				break;
		}
	}
	
	HAL_GPIO_ConfigOutput(PCx, pin, INPUT);
	HAL_GPIO_ConfigPullup(PCx, pin, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_SetDebouncePin(PCx, pin,ENABLE);
	
	if(PCx==PA || PCx==PB){
		HAL_SCU_SetMCCRx(4,PGA_TYPE,SCU_MCCR_CSEL_LSI,100);		//LSI=500KHz, N=100, 500KHz/100=5KHz, 0.2ms
	}
	else if(PCx==PC||PCx==PD){
		HAL_SCU_SetMCCRx(5,PGB_TYPE,SCU_MCCR_CSEL_LSI,100);		//LSI=500KHz, N=100, 500KHz/100=5KHz, 0.2ms
	}
	else{
		HAL_SCU_SetMCCRx(5,PGC_TYPE,SCU_MCCR_CSEL_LSI,100);		//LSI=500KHz, N=100, 500KHz/100=5KHz, 0.2ms
	}
	
	while(1){
		status = (HAL_GPIO_ReadPin(PCx) & (1<<pin));
		if ((status&(1<<pin))==(1<<pin))						// Port Input High
		{
			HAL_GPIO_ClearPin(PB, _BIT(0));
			delay();
			HAL_GPIO_SetPin(PB, _BIT(0));
			delay();
		}
		else 
		{
			HAL_GPIO_ClearPin(PB, _BIT(1));		// Port Input Low
			delay();
			HAL_GPIO_SetPin(PB, _BIT(1));
			delay();									
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
	
	GPIO_inputRun();
				
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
	debug_frmwrk_init();

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
