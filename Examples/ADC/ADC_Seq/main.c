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
void ADC_IRQHandler_IT(void);
void TIMER_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void UART_DebugMessage(void);
void GPIO_Configure(void);
void TIMER_Configure( void);
void ADC_Configure(void);
void ADC_SepRun(void);
void ADC_MessageRun(void);

void mainloop(void);
int main (void);
void Error_Handler(void);	
/* Private variables ---------------------------------------------------------*/
ADC_CFG_Type AD_config;
TIMER_PERIODICCFG_Type TIMER1_Config;
	
const uint8_t menu[] =
"************************************************\n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t Using ADC in sequence mode  \n\r"
"************************************************\n\r";

uint32_t fflag_a_b;
TIMER_PERIODICCFG_Type TIMER1_Config;
uint32_t count;


/**********************************************************************
 * @brief		ADC0_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void ADC_IRQHandler_IT(void)
{
	uint16_t status;
	
	HAL_ADC_GetData(ADC0);	// dummy read
	
	status = HAL_ADC_GetStatus(ADC0);

	if ((status & ADC_STAT_SEQ)==ADC_STAT_SEQ){
		fflag_a_b=1;	
		HAL_ADC_ClearStatus(ADC0, ADC_STAT_SEQ);
	}
	if ((status & ADC_STAT_TRIGGER)==ADC_STAT_TRIGGER){
		HAL_ADC_ClearStatus(ADC0, ADC_STAT_TRIGGER);	
		PB->ODR ^= 0x01;
	}
	
}

/**********************************************************************
 * @brief		TIMER1_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void TIMER1_IRQHandler_IT(void)
{
	if ((HAL_TIMER_GetStatus(TIMER1) & TIMER_SR_MFB) == TIMER_SR_MFB){
			HAL_TIMER_ClearStatus(TIMER1, TIMER_SR_MFB);
			count++;
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
 * @brief		UART_DebugMessage
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void UART_DebugMessage(void)
{
	#ifdef _DEBUG_MSG
 	_DBG("CH0 = "); _DBH16(ADC0->DR0>>4); _DBG(" ");
	_DBG("CH2 = "); _DBH16(ADC0->DR1>>4); _DBG(" ");
	_DBG("CH4 = "); _DBH16(ADC0->DR2>>4); _DBG(" ");
	_DBG("CH6 = "); _DBH16(ADC0->DR3>>4); _DBG_(" ");
	#endif

} 
/**********************************************************************
 * @brief		PCU_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_Configure(void)
{
	// Toggle PORT Setting
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);
	
       // Timer1 Setting
	HAL_GPIO_ConfigureFunction(PA, 5, PA5_MUX_T1IO);
	HAL_GPIO_ConfigOutput(PA, 5, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PA, 5, PULL_UP_DOWN_DISABLE);	
	
	//PORT(AN0~AN3) Setting
	HAL_GPIO_ConfigureFunction(PA, 0, PA0_MUX_AN0);
	HAL_GPIO_ConfigOutput(PA, 0, INPUT);
	HAL_GPIO_ConfigPullup(PA, 0, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 1, PA1_MUX_AN1);
	HAL_GPIO_ConfigOutput(PA, 1, INPUT);
	HAL_GPIO_ConfigPullup(PA, 1, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 2, PA2_MUX_AN2);
	HAL_GPIO_ConfigOutput(PA, 2, INPUT);
	HAL_GPIO_ConfigPullup(PA, 2, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 3, PA3_MUX_AN3);
	HAL_GPIO_ConfigOutput(PA, 3, INPUT);
	HAL_GPIO_ConfigPullup(PA, 3, PULL_UP_DOWN_DISABLE);	
}
/**********************************************************************
 * @brief		TIMER_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void TIMER_Configure(void)
{
	TIMER1_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
	TIMER1_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

	TIMER1_Config.GRA = (500); 	// adc trigger signal 
	TIMER1_Config.GRB = (10000); // 10msec			
	TIMER1_Config.Cnt=0;		
	TIMER1_Config.StartLevel=START_LOW;
	TIMER1_Config.AdcTrgEn=ENABLE;
	
	if(HAL_TIMER_Init(TIMER1, PERIODIC_MODE, &TIMER1_Config) != HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();
	}
	HAL_TIMER_ConfigInterrupt(TIMER1, TIMER_INTCFG_MBIE, ENABLE);
	
	
}

/**********************************************************************
 * @brief		ADC_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void ADC_Configure(void)
{	
	// ADC Configuration
		AD_config.Mode = ADC_SINGLE_MODE;
		AD_config.SamplingTime=0x1f;
		AD_config.SeqCnt = 4;
		AD_config.RestartEn = 1;
		AD_config.TrgSel = ADC_TRIGGER_TIMER;
		
		AD_config.UseClk = ADC_EXTERNAL_CLK;
		AD_config.InClkDiv = 1; 
		
		HAL_SCU_SetMCCRx(4, ADC_TYPE, SCU_MCCR_CSEL_HSI, 8);
		
		if(HAL_ADC_Init(ADC0, &AD_config) != HAL_OK)
		{
		   /* Initialization Error */
       Error_Handler();
	  }
		count=0;

}

/**********************************************************************
 * @brief		ADC_SepRun
 * @param[in]	None
 * @return	None
 **********************************************************************/
void ADC_SepRun(void)
{
	  /* timer1 start */
		/* preemption = 1, sub-priority = 1 */
		NVIC_SetPriority(TIMER1_IRQn, ((0x01<<1)|0x01));
		/* Enable Interrupt for TIMER0 channel */
		NVIC_EnableIRQ(TIMER1_IRQn);	
		HAL_TIMER_Cmd(TIMER1, ENABLE); 
	
    /* adc start */
		HAL_ADC_TriggerlSel(ADC0, 0x00001111); //trigger timer1
		HAL_ADC_ChannelSel(ADC0, 0x06040200);	//ch0, ch1, ch2, ch3

		HAL_ADC_ClearStatus(ADC0, 0xff); // clear status
		HAL_ADC_ConfigInterrupt(ADC0, ( ADC_INTEN_SEQ | ADC_INTEN_TRIGGER), ENABLE);

		NVIC_SetPriority(ADC0_IRQn, 3);
		NVIC_EnableIRQ(ADC0_IRQn);
	
}

/**********************************************************************
 * @brief		ADC_MessageRun
 * @param[in]	None
 * @return	None
 **********************************************************************/
void ADC_MessageRun(void)
{
	while(fflag_a_b==0){}
	fflag_a_b=0;		
	UART_DebugMessage();

		
	
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
	
	/*TIMER1 Configure*/
	TIMER_Configure();
	
	/*ADC Configure*/
  ADC_Configure(); 	
	
	/* Enable IRQ Interrupts */
	__enable_irq();  
	
	/*ADC start*/
  ADC_SepRun();
	
  while(1){	
		ADC_MessageRun();
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

