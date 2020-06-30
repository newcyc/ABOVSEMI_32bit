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
void WDT_InterruptRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
"Watch dog timer interrupt(1s) demo\n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"PB0 toggle every 1s \n\r"
"************************************************\n\r";
uint8_t wdtf;
WDT_CFG_Type wdtCfg;

/**********************************************************************
 * @brief		WDT_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void WDT_IRQHandler_IT(void)
{
	HAL_WDT_UpdateTimeOut(8000000);	// 1s
	
	if (wdtf == 0){
		wdtf=1;
		HAL_GPIO_ClearPin(PB, _BIT(0));
	}
	else {
		wdtf=0;
		HAL_GPIO_SetPin(PB, _BIT(0));
	}
	
	_DBG(">> WDT Interrupt OK\n\r");
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
}


/**********************************************************************
 * @brief		WDT_InterruptRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void WDT_InterruptRun(void)
{

	HAL_WDT_Init();

	HAL_SCU_SetMCCRx(1, WDT_TYPE, SCU_MCCR_CSEL_MCLK, 1);		//MCLK = 8MHz, N=1, 8MHz/1 =  125ns
	
	wdtCfg.wdtResetEn = DISABLE;
	wdtCfg.wdtCountEn = ENABLE;
	wdtCfg.wdtDbgEn = ENABLE;
	wdtCfg.wdtClkSel = RESET;
	wdtCfg.wdtPrescaler = WDT_DIV_1;  //WDTCLK = WDTCLKIN / PRS =  8MHz/1 = 125ns
	wdtCfg.wdtTmrConst = 8000000; //1s
	
	HAL_WDT_Configure(wdtCfg);
	
	/* Enable Interrupt for WDT channel */
	NVIC_SetPriority(WDT_IRQn, ((0x01<<1)|0x01));
	NVIC_EnableIRQ(WDT_IRQn);
	
	HAL_WDT_Start(ENABLE);
	
	

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
	
    /*WDT Interrupt */
	WDT_InterruptRun();
	
    /* Enable IRQ Interrupts */
	__enable_irq();
	
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

