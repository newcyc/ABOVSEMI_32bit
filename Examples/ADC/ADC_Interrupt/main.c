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
uint16_t adcaverage(uint16_t *val);
void GPIO_Configure(void);
void ADC_Configure(void);
void ADC_InterruptRun(void);
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
"\t Using ADC in interrupt mode \n\r"
"************************************************\n\r";
static ADC_CFG_Type AD_config;
static uint16_t adcval[8];	

/**********************************************************************
 * @brief		ADC0_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void ADC_IRQHandler_IT(void)
{
		uint16_t status;
	status = HAL_ADC_GetStatus(ADC0);

	if ((status & ADC_STAT_SINGLE) == ADC_STAT_SINGLE){				// Sequence Conversion End Interrupt Flag
		HAL_ADC_ClearStatus(ADC0, ADC_STAT_SINGLE);
		fflag=1;
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
 * @brief		adcaverage
 * @param[in]	None
 * @return 		None
 **********************************************************************/
uint16_t adcaverage(uint16_t *val)
{
	uint32_t i;
	uint32_t adcavg;
	
	adcavg=0;			
	for (i=0;i<8;i++){	
		adcavg+=val[i];
	}
	adcavg>>=3;
	
	return (adcavg);
}
/**********************************************************************
 * @brief		PCU_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_Configure(void)
{
	//PORT Setting
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
	
	HAL_GPIO_ConfigureFunction(PA, 4, PA4_MUX_AN4);
	HAL_GPIO_ConfigOutput(PA, 4, INPUT);
	HAL_GPIO_ConfigPullup(PA, 4, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 5, PA5_MUX_AN5);
	HAL_GPIO_ConfigOutput(PA, 5, INPUT);
	HAL_GPIO_ConfigPullup(PA, 5, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 6, PA6_MUX_AN6);
	HAL_GPIO_ConfigOutput(PA, 6, INPUT);
	HAL_GPIO_ConfigPullup(PA, 6, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 7, PA7_MUX_AN7);
	HAL_GPIO_ConfigOutput(PA, 7, INPUT);
	HAL_GPIO_ConfigPullup(PA, 7, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 8, PA8_MUX_AN8);
	HAL_GPIO_ConfigOutput(PA, 8, INPUT);
	HAL_GPIO_ConfigPullup(PA, 8, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 9, PA9_MUX_AN9);
	HAL_GPIO_ConfigOutput(PA, 9, INPUT);
	HAL_GPIO_ConfigPullup(PA, 9, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 10, PA10_MUX_AN10);
	HAL_GPIO_ConfigOutput(PA, 10, INPUT);
	HAL_GPIO_ConfigPullup(PA, 10, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 11, PA11_MUX_AN11);
	HAL_GPIO_ConfigOutput(PA, 11, INPUT);
	HAL_GPIO_ConfigPullup(PA, 11, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 12, PA12_MUX_AN12);
	HAL_GPIO_ConfigOutput(PA, 12, INPUT);
	HAL_GPIO_ConfigPullup(PA, 12, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 13, PA13_MUX_AN13);
	HAL_GPIO_ConfigOutput(PA, 13, INPUT);
	HAL_GPIO_ConfigPullup(PA, 13, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 14, PA14_MUX_AN14);
	HAL_GPIO_ConfigOutput(PA, 14, INPUT);
	HAL_GPIO_ConfigPullup(PA, 14, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PA, 15, PA15_MUX_AN15);
	HAL_GPIO_ConfigOutput(PA, 15, INPUT);
	HAL_GPIO_ConfigPullup(PA, 15, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PD, 4, PD4_MUX_AN16);
	HAL_GPIO_ConfigOutput(PD, 4, INPUT);
	HAL_GPIO_ConfigPullup(PD, 4, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PD, 5, PD5_MUX_AN17);
	HAL_GPIO_ConfigOutput(PD, 5, INPUT);
	HAL_GPIO_ConfigPullup(PD, 5, PULL_UP_DOWN_DISABLE);	

	HAL_GPIO_ConfigureFunction(PD, 6, PD6_MUX_AN18);
	HAL_GPIO_ConfigOutput(PD, 6, INPUT);
	HAL_GPIO_ConfigPullup(PD, 6, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PD, 7, PD7_MUX_AN19);
	HAL_GPIO_ConfigOutput(PD, 7, INPUT);
	HAL_GPIO_ConfigPullup(PD, 7, PULL_UP_DOWN_DISABLE);	

	HAL_GPIO_ConfigureFunction(PE, 1, PE1_MUX_AN20);
	HAL_GPIO_ConfigOutput(PE, 1, INPUT);
	HAL_GPIO_ConfigPullup(PE, 1, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PE, 2, PE2_MUX_AN21);
	HAL_GPIO_ConfigOutput(PE, 2, INPUT);
	HAL_GPIO_ConfigPullup(PE, 2, PULL_UP_DOWN_DISABLE);	

	HAL_GPIO_ConfigureFunction(PF, 2, PF2_MUX_AN22);
	HAL_GPIO_ConfigOutput(PF, 2, INPUT);
	HAL_GPIO_ConfigPullup(PF, 2, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PF, 3, PF3_MUX_AN23);
	HAL_GPIO_ConfigOutput(PF, 3, INPUT);
	HAL_GPIO_ConfigPullup(PF, 3, PULL_UP_DOWN_DISABLE);	
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
	AD_config.SamplingTime = 1;
	AD_config.SeqCnt = 1;
	AD_config.RestartEn = 0;
	AD_config.TrgSel = ADC_TRIGGER_DISABLE;
	
	AD_config.UseClk = ADC_EXTERNAL_CLK;
	AD_config.InClkDiv = 1;

	HAL_SCU_SetMCCRx(4, ADC_TYPE, SCU_MCCR_CSEL_HSI, 8);
	
	if( HAL_ADC_Init(ADC0, &AD_config) != HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();
	}
	
	HAL_ADC_ConfigInterrupt(ADC0, ADC_INTEN_SINGLE, ENABLE);
	NVIC_SetPriority(ADC0_IRQn, 3);
	NVIC_EnableIRQ(ADC0_IRQn);
	
	SYST_ACCESS_EN();
	
	SCU->PER1 |= (1<<4);
	SCU->PCER1 |= (1<<4);
	
	SYST_ACCESS_DIS();
	
}


/**********************************************************************
 * @brief		ADC_InterruptRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void ADC_InterruptRun(void)
{
	uint32_t i,j;
	uint32_t adcavg;
	uint32_t tmp;
	
	for(i=0;i<23;i++){
			HAL_ADC_ChannelSel_1(ADC0, i); // select ch
			if((i & 0x7) ==0){
				#ifdef _DEBUG_MSG
				_DBG("\n\r");
				#endif
		  }
			
			for (j=0;j<(8);j++)
			{
				fflag=0;
				HAL_ADC_Start(ADC0); // start 	
				while(fflag==0){}
				adcval[j]=HAL_ADC_GetData(ADC0);
			}
			adcavg=adcaverage(adcval);
			#ifdef _DEBUG_MSG
			_DBG(" CH"); _DBD(i); _DBG(" = "); _DBH16(adcavg); _DBG(" /");
			#endif
		}
	  #ifdef _DEBUG_MSG
		_DBG("\n\r");
    #endif
		for(tmp = 0; tmp < 0x1FFFF; tmp++);
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
	
	/*ADC Configure*/
    ADC_Configure(); 
	
	/* Enable IRQ Interrupts */
	__enable_irq();

   /* Infinite loop */
   while(1)
   {
		ADC_InterruptRun();
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

