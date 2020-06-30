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
void TIMER0_IRQHandler_IT(void);
void TIMER1_IRQHandler_IT(void);
void TIMER2_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void UART_DebugMessage(void);
void GPIO_Configure(void);
void TIMER_Configure( void);
void ADC_Configure(void);
void ADC_multipeRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);	
/* Private variables ---------------------------------------------------------*/	
static uint32_t fflag;
static const uint8_t menu[] =
"************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t Using ADC in Multiple mode \n\r"
"************************************************\n\r";

static TIMER_PERIODICCFG_Type TIMERa_Config;
static uint32_t count_a;
static TIMER_PERIODICCFG_Type TIMERb_Config;
static uint32_t count_b;
static TIMER_PERIODICCFG_Type TIMERc_Config;
static uint32_t count_c;
static ADC_CFG_Type AD_config;
	



/**********************************************************************
 * @brief		ADC0_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void ADC_IRQHandler_IT(void)
{
	uint16_t status;
	status = HAL_ADC_GetStatus(ADC0);

	if (status & ADC_STAT_SEQ){
		HAL_ADC_ClearStatus(ADC0, ADC_STAT_SEQ);	
	}
	if (status & ADC_STAT_TRIGGER){
		HAL_ADC_ClearStatus(ADC0, ADC_STAT_TRIGGER);	
		fflag=1;
		PB->ODR ^= 0x01;
	}	
}


/**********************************************************************
 * @brief		TIMER0_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void TIMER0_IRQHandler_IT(void)
{
	if ((HAL_TIMER_GetStatus(TIMER0) & TIMER_SR_MFB) == TIMER_SR_MFB){
		HAL_TIMER_ClearStatus(TIMER0, TIMER_SR_MFB);
		count_a++;
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
		count_b++;
	}
}


/**********************************************************************
 * @brief		TIMER2_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void TIMER2_IRQHandler_IT(void)
{
	if ((HAL_TIMER_GetStatus(TIMER2) & TIMER_SR_MFB) == TIMER_SR_MFB){
		HAL_TIMER_ClearStatus(TIMER2, TIMER_SR_MFB);
		count_c++;
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
		_DBG("\r\n");
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
	
	HAL_GPIO_ConfigureFunction(PB, 1, PB1_MUX_PB1);
	HAL_GPIO_ConfigOutput(PB, 1, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 1, PULL_UP_DOWN_DISABLE);	
	
	
	//PORT(AN0~3) Setting
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
	//------------------------------------------------------
	// TIMER 0 Setting
	//------------------------------------------------------
	TIMERa_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
	TIMERa_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

	TIMERa_Config.GRA = (500); 	// adc trigger signal 
	TIMERa_Config.GRB = (10000); // 10msec			
	TIMERa_Config.Cnt=0;		
	TIMERa_Config.StartLevel=START_LOW;
	TIMERa_Config.AdcTrgEn=ENABLE;
	
	if( HAL_TIMER_Init(TIMER0, PERIODIC_MODE, &TIMERa_Config)!= HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();
	}
	HAL_TIMER_ConfigInterrupt(TIMER0, TIMER_INTCFG_MBIE, ENABLE);

	/* preemption = 1, sub-priority = 1 */
  NVIC_SetPriority(TIMER0_IRQn, ((0x01<<1)|0x01));
	/* Enable Interrupt for TIMER0 channel */
  NVIC_EnableIRQ(TIMER0_IRQn);

  
	//------------------------------------------------------
	// TIMER 1 Setting
	//------------------------------------------------------
	TIMERb_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
	TIMERb_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

	TIMERb_Config.GRA = (500); 	// adc trigger signal 
	TIMERb_Config.GRB = (10000); // 10msec			
	TIMERb_Config.Cnt=0;		
	TIMERb_Config.StartLevel=START_LOW;
	TIMERb_Config.AdcTrgEn=ENABLE;
	
	if( HAL_TIMER_Init(TIMER1, PERIODIC_MODE, &TIMERb_Config)!= HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();
	}
	HAL_TIMER_ConfigInterrupt(TIMER1, TIMER_INTCFG_MBIE, ENABLE);

  /* preemption = 1, sub-priority = 1 */
  NVIC_SetPriority(TIMER1_IRQn, ((0x01<<1)|0x01));
	/* Enable Interrupt for TIMER0 channel */
  NVIC_EnableIRQ(TIMER1_IRQn);

  

	//------------------------------------------------------
	// TIMER 2 Setting
	//------------------------------------------------------
	TIMERc_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
	TIMERc_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

	TIMERc_Config.GRA = (500); 	// adc trigger signal 
	TIMERc_Config.GRB = (10000); // 10msec			
	TIMERc_Config.Cnt=0;		
	TIMERc_Config.StartLevel=START_LOW;
	TIMERc_Config.AdcTrgEn=ENABLE;
	
	if( HAL_TIMER_Init(TIMER2, PERIODIC_MODE, &TIMERc_Config)!= HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();
	}
	HAL_TIMER_ConfigInterrupt(TIMER2, TIMER_INTCFG_MBIE, ENABLE);
	
	/* preemption = 1, sub-priority = 1 */
  NVIC_SetPriority(TIMER2_IRQn, ((0x01<<1)|0x01));
	/* Enable Interrupt for TIMER0 channel */
  NVIC_EnableIRQ(TIMER2_IRQn);

}


/**********************************************************************
 * @brief		ADC_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void ADC_Configure(void)
{	
	// ADC Configuration
	AD_config.Mode = ADC_MULTI_MODE;
	AD_config.SamplingTime=0x1f;
	AD_config.SeqCnt = 4;
	AD_config.RestartEn = 0;
	AD_config.TrgSel = ADC_TRIGGER_TIMER;
	
	AD_config.UseClk = ADC_EXTERNAL_CLK;
	AD_config.InClkDiv = 1;

	// ADC Clock : 32/8 = 4MHz
	HAL_SCU_SetMCCRx(4, ADC_TYPE, SCU_MCCR_CSEL_HSI, 8);

	if (HAL_ADC_Init(ADC0, &AD_config)!= HAL_OK)
  {
		/* Initialization Error */
     Error_Handler();
	}

	HAL_ADC_TriggerlSel(ADC0, 0xFFFFF120); //trigger Selection
	HAL_ADC_ChannelSel(ADC0, 0x00040200);	//ch0, ch1, ch2, ch3

	HAL_ADC_ClearStatus(ADC0, 0xff); // clear status
	HAL_ADC_ConfigInterrupt(ADC0, ( ADC_INTEN_SEQ | ADC_INTEN_TRIGGER), ENABLE);
	
	NVIC_SetPriority(ADC0_IRQn, 3);
	NVIC_EnableIRQ(ADC0_IRQn);


}

/**********************************************************************
 * @brief		ADC_multipeRun
 * @param[in]	None
 * @return	None
 **********************************************************************/
void ADC_multipeRun(void)
{
	count_a=0;	
	count_b=0;	
	count_c=0;
	
	HAL_TIMER_Cmd(TIMER0, ENABLE);	

	__NOP();
	__NOP();
	__NOP();

	HAL_TIMER_Cmd(TIMER1, ENABLE);
	
	__NOP();
	__NOP();
	__NOP();
		

	HAL_TIMER_Cmd(TIMER2, ENABLE);
	__NOP();
	__NOP();
	__NOP();
	

}
/**********************************************************************
 * @brief		ADC_MessageRun
 * @param[in]	None
 * @return	None
 **********************************************************************/
void ADC_MessageRun(void)
{
		fflag=0;
		while(fflag==0){}

		_DBG("CH0 = "); _DBH16(ADC0->DR0>>4); _DBG(" ");
		_DBG("CH2 = "); _DBH16(ADC0->DR1>>4); _DBG(" ");
		_DBG("CH4 = "); _DBH16(ADC0->DR2>>4); _DBG(" ");			
		_DBG("\r\n");
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
	
	/*TIMER Configure*/
	TIMER_Configure();
	
	/*ADC Configure*/
  ADC_Configure(); 
  
	
	/*ADC Channel, timer start*/
	ADC_multipeRun();

 /* Enable IRQ Interrupts */
	__enable_irq();

   /* Infinite loop */
  while(1)
	{
		/* ADC Message */
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

