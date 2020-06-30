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
void delay (void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void COMP_Configure(void);
void COMP_PollingRun(void);

void mainloop(void);
int main (void);
void Error_Handler(void);	

/* Private variables ---------------------------------------------------------*/	
const uint8_t menu[] =
"************************************************\n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
" COMP Polling Test \n\r"
" Input(-) : BGR1V, Input(+) : CP0A[PA4]\n\r"
" Reference voltage < Input voltage [PB1 Toggle] \n\r"
" Reference voltage > Input voltage [PB0 Toggle] \n\r"
"************************************************\n\r";
uint32_t status;


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


/*********************************************************************//**
 * @brief		Delay function
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void delay (void) 
{
	uint32_t i;
	
	for (i = 0; i < 0x100000; i++) ;
}

/**********************************************************************
 * @brief		PCU_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_Configure(void)
{
	// PB0  test pin setting
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);
	
	// PB1 test pin setting
	HAL_GPIO_ConfigureFunction(PB, 1, PB1_MUX_PB1);
	HAL_GPIO_ConfigOutput(PB, 1, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 1, PULL_UP_DOWN_DISABLE);
	
	// PA4,5,6,7  COMP0 pin setting
	// CP0A(PA4)
	HAL_GPIO_ConfigureFunction(PA, 4, PA4_MUX_CP0A);
	HAL_GPIO_ConfigOutput(PA, 4, INPUT);
	HAL_GPIO_ConfigPullup(PA, 4, PULL_UP_DOWN_DISABLE);
	// CP0B(PA5)
	HAL_GPIO_ConfigureFunction(PA, 5, PA5_MUX_CP0B);
	HAL_GPIO_ConfigOutput(PA, 5, INPUT);
	HAL_GPIO_ConfigPullup(PA, 5, PULL_UP_DOWN_DISABLE);	
	// CP0C(PA6)	
	HAL_GPIO_ConfigureFunction(PA, 6, PA6_MUX_CP0C);
	HAL_GPIO_ConfigOutput(PA, 6, INPUT);
	HAL_GPIO_ConfigPullup(PA, 6, PULL_UP_DOWN_DISABLE);
	// CREF0(PA7)	
	HAL_GPIO_ConfigureFunction(PA, 7, PA7_MUX_CREF0);
	HAL_GPIO_ConfigOutput(PA, 7, INPUT);
	HAL_GPIO_ConfigPullup(PA, 7, PULL_UP_DOWN_DISABLE);
}


/**********************************************************************
 * @brief		COMP_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void COMP_Configure( void)
{
	
	if((HAL_COMP_Init(COMP0, ENABLE))!=HAL_OK)	// Comparator0 Peri, Peri Clock Enable
	{
		/* Initialization Error */
     Error_Handler();		
	}
	
	HAL_COMP_ConfigHysterisis(COMP0, COMP_CONF_HYSSEL_20MV, ENABLE); //Hysterisis Enable : 20mV
														  
	HAL_COMP_ConfigInputLevel(COMP0, COMP_CONF_INNSEL_BRG1V, COMP_CONF_INPSEL_CP0A);	// (-) sel : BGR1V, (+) sel : CP0A 
	
	HAL_COMP_Cmd(COMP0, ENABLE); //Comparator0 enable
	
	HAL_COMP_ConfigDebounce(COMP0, 10); // 1/8MHz = 125ns, 125ns * 10 = 1.25us
	
	HAL_COMP_ClearStatusAll_IT(COMP0);
	
	PORT_ACCESS_EN();
	
	
}

/**********************************************************************
 * @brief		COMP_PollingRun
 * @param[in]	None
 * @return	None
 **********************************************************************/
void COMP_PollingRun(void)
{

	status =HAL_COMP_GetStatus_IT(COMP0);
	
	if ((status & (1<<0))){ 	// Reference voltage < Input voltage [PB1 Toggle]
		PB->ODR ^= (1<<1);
		delay();
	}
	
	if (!(status & (1<<0))){		// Reference voltage > Input voltage [PB0 Toggle]
		PB->ODR ^= (1<<0);
		delay();
	}
	
	HAL_COMP_ClearStatusAll_IT(COMP0);

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
	
	/*COMP Configure*/
    COMP_Configure(); 

	/* Enable IRQ Interrupts */
	__enable_irq();
		
	/* Infinite Loop */
    while(1)
	{
      COMP_PollingRun();
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
	debug_frmwrk_init();
	
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

