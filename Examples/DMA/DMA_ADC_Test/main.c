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
#define ADC_DMA_Num		100
#define ADC_DMA_loop_Count		16

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ADC0_IRQHandler_IT(void);
void ADC1_IRQHandler_IT(void);
void TIMER0_IRQHandler_IT(void);

void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void TIMER_StartRun(void);
void DMA_StartRun(void);


void DMA_ADCTestRun(void);

void mainloop(void);
int main (void);
void Error_Handler(void);	

/* Private variables ---------------------------------------------------------*/	
const uint8_t cmdm[] =
"A34M41x> ";
const uint8_t menu[] =
"\r\n\r\n************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t DMA ADC FUNCTION TEST ...  \n\r"
"\t Start \n\r"
"************************************************\n\r";

uint32_t fflag_ad0;

uint32_t adcirq_seq_ad0;
uint32_t adcirq_trigger_ad0;
uint32_t adcirq_count_ad0;

uint32_t fflag_ad1;

uint32_t adcirq_seq_ad1;
uint32_t adcirq_trigger_ad1;
uint32_t adcirq_count_ad1;

uint32_t fflag;
uint32_t count;

uint32_t dma_rx,dma_add,tmp;
DMA_CFG_Type	DMA_Config;

uint32_t data0[300]={0,};
uint32_t data1[300]={0,};
uint32_t ii;

/**********************************************************************
 * @brief		ADC0_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void ADC0_IRQHandler_IT(void)
{
	uint16_t status;
//	uint16_t ia=0;
	status = HAL_ADC_GetStatus(ADC0);

	if ((status & ADC_STAT_SEQ)==ADC_STAT_SEQ)
	{
		fflag_ad0=1;
		adcirq_seq_ad0++;
		HAL_ADC_ClearStatus(ADC0, ADC_STAT_SEQ);	
	}
	if ((status & ADC_STAT_TRIGGER)==ADC_STAT_TRIGGER){
		HAL_ADC_ClearStatus(ADC0, ADC_STAT_TRIGGER);	
		fflag_ad0=2;
	}	

	if ((status & ADC_STAT_DMA)==ADC_STAT_DMA)
	{
		fflag_ad0=1;
		HAL_ADC_ClearStatus(ADC0, ADC_STAT_DMA);
		HAL_GPIO_ClearPin(PB,_BIT(0));
		HAL_GPIO_SetPin(PB,_BIT(0));

	#if 0
		for(ia=0;ia<ADC_DMA_Num;ia++)
		{
			data0_b[ia+(adcirq_count_ad0*ADC_DMA_Num)] = (data0[ia] >> 4) & 0x0fff;
		}
	#endif
	
		adcirq_count_ad0++;
		adcirq_seq_ad0= count = 0;

		dma_rx = ADC0_RX;
		dma_add = 0x4000B02C;

		DMA_Config.transcnt = ADC_DMA_Num;
		DMA_Config.perisel = dma_rx;
		DMA_Config.bussize = DMA_CR_WORD_TRANS;
		DMA_Config.dirsel = DMA_CR_DIR_PERI_TO_MEM;
		
		HAL_DMA_Cmd(DMA0, &DMA_Config);
		
		HAL_DMA_SetPAR(DMA0, dma_add);
		HAL_DMA_SetMAR(DMA0, (uint32_t)data0);

//		DMA_Start(DMA0);		
		HAL_GPIO_ClearPin(PB,_BIT(0));		
	}
}

/**********************************************************************
 * @brief		ADC1_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void ADC1_IRQHandler_IT(void)
{
	uint16_t status;
//	uint16_t ia=0;
	status = HAL_ADC_GetStatus(ADC1);


	if (status & ADC_STAT_SEQ)
	{
		fflag_ad1=1;
		HAL_ADC_ClearStatus(ADC1, ADC_STAT_SEQ);	
		adcirq_seq_ad1++;
	}
	if (status & ADC_STAT_TRIGGER){
		HAL_ADC_ClearStatus(ADC1, ADC_STAT_TRIGGER);	
		fflag_ad1=2;
	}	

	if (status & ADC_STAT_DMA)
	{
		fflag_ad1=3;

//		HAL_GPIO_ClearPin(PB,_BIT(1));
//		HAL_GPIO_SetPin(PB,_BIT(1));
	#if 0
		for(ia=0;ia<ADC_DMA_Num;ia++)
		{
			data1_b[ia+(adcirq_count_ad1*ADC_DMA_Num)] = (data1[ia] >> 4) & 0x0fff;
		}
	#endif
	
		adcirq_count_ad1++;

#if 1	
		adcirq_seq_ad1= count = 0;
		HAL_ADC_ClearStatus(ADC1, ADC_STAT_DMA);	
		dma_rx = ADC1_RX;
		dma_add = 0x4000B12C;
		
		DMA_Config.transcnt = ADC_DMA_Num;
		DMA_Config.perisel = dma_rx;
		DMA_Config.bussize = DMA_CR_WORD_TRANS;
		DMA_Config.dirsel = DMA_CR_DIR_PERI_TO_MEM;
		
		HAL_DMA_Cmd(DMA1, &DMA_Config);
		HAL_DMA_SetPAR(DMA1, dma_add);
		HAL_DMA_SetMAR(DMA1, (uint32_t)data1);
//		DMA_Start(DMA1);		
		
#endif	
//		HAL_GPIO_ClearPin(PB,_BIT(1));

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
		count++;

		if(count%2)
			HAL_GPIO_ClearPin(PB,_BIT(1));
		else
			HAL_GPIO_SetPin(PB,_BIT(1));	
	}
}

/**********************************************************************
 * @brief		adc average (8-data)
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
 * @brief		TIMER_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void TIMER_Configure(void)
{
	TIMER_PERIODICCFG_Type TIMER0_Config;

	TIMER0_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
	TIMER0_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

	TIMER0_Config.GRA = (500); 	// adc trigger signal 
	TIMER0_Config.GRB = (10000); // 1msec			
	TIMER0_Config.Cnt=0;		
	TIMER0_Config.StartLevel=START_LOW;
	TIMER0_Config.AdcTrgEn=ENABLE;
	
	if((HAL_TIMER_Init(TIMER0, PERIODIC_MODE, &TIMER0_Config))!=HAL_OK)	// timer0 setting 
	{
			/* Initialization Error */
     Error_Handler();
	} 
	HAL_TIMER_ConfigInterrupt(TIMER0, TIMER_INTCFG_MBIE, ENABLE);

	NVIC_SetPriority(TIMER0_IRQn, 3);
	/* Enable Interrupt for TIMER0 channel */
	NVIC_EnableIRQ(TIMER0_IRQn);
}

/**********************************************************************
 * @brief		DMA_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_Configure(void)
{
	HAL_DMA_Init();

	dma_rx = ADC0_RX;
	dma_add = 0x4000B02C;

	DMA_Config.transcnt = ADC_DMA_Num;
	DMA_Config.perisel = dma_rx;
	DMA_Config.bussize = DMA_CR_WORD_TRANS;
	DMA_Config.dirsel = DMA_CR_DIR_PERI_TO_MEM;
	
	HAL_DMA_Cmd(DMA0, &DMA_Config);
	
	HAL_DMA_SetPAR(DMA0, dma_add);
	HAL_DMA_SetMAR(DMA0, (uint32_t)data0);
	//DMA_SetMAR(DMA0, 0x20001000);
	
	dma_rx = ADC1_RX;
	dma_add = 0x4000B12C;
	
	DMA_Config.transcnt = ADC_DMA_Num;
	DMA_Config.perisel = dma_rx;
	DMA_Config.bussize = DMA_CR_WORD_TRANS;
	DMA_Config.dirsel = DMA_CR_DIR_PERI_TO_MEM;
	
	HAL_DMA_Cmd(DMA1, &DMA_Config);
	HAL_DMA_SetPAR(DMA1, dma_add);
	HAL_DMA_SetMAR(DMA1, (uint32_t)data1);
}

/**********************************************************************
 * @brief		PCU_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_Configure(void)
{
	// Test Pin Setting (PB0, PB1)
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_ConfigureFunction(PB, 1, PB1_MUX_PB1);
	HAL_GPIO_ConfigOutput(PB, 1, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 1, PULL_UP_DOWN_DISABLE);	

	// Analog Pin Setting (AN0-1, AN6-7, AN16-17, AN18-19)
	HAL_GPIO_ConfigureFunction(PA, 0, PA0_MUX_AN0);
	HAL_GPIO_ConfigOutput(PA, 0, INPUT);
	HAL_GPIO_ConfigPullup(PA, 0, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_ConfigureFunction(PA, 1, PA1_MUX_AN1);
	HAL_GPIO_ConfigOutput(PA, 1, INPUT);
	HAL_GPIO_ConfigPullup(PA, 1, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_ConfigureFunction(PA, 6, PA6_MUX_AN6);
	HAL_GPIO_ConfigOutput(PA, 6, INPUT);
	HAL_GPIO_ConfigPullup(PA, 6, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_ConfigureFunction(PA, 7, PA7_MUX_AN7);
	HAL_GPIO_ConfigOutput(PA, 7, INPUT);
	HAL_GPIO_ConfigPullup(PA, 7, PULL_UP_DOWN_DISABLE);	
	
	HAL_GPIO_ConfigureFunction(PD, 4,  PD4_MUX_AN16);
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
}

/**********************************************************************
 * @brief		ADC_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void ADC_Configure(void)
{
	ADC_CFG_Type AD_config;
	
	AD_config.Mode = ADC_BURST_MODE;
	AD_config.SamplingTime=0x1e;
	AD_config.SeqCnt = 4;
	AD_config.RestartEn = 1;
	AD_config.TrgSel = ADC_TRIGGER_TIMER;
	
	AD_config.UseClk = ADC_EXTERNAL_CLK;
	AD_config.InClkDiv = 1;

	HAL_SCU_SetMCCRx(4, ADC_TYPE, SCU_MCCR_CSEL_HSI, 8);

	if((HAL_ADC_Init(ADC0, &AD_config))!=HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();
	}
	if((HAL_ADC_Init(ADC1, &AD_config))!=HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();
	}
	;
	HAL_ADC_Init(ADC1, &AD_config);
	
	HAL_ADC_TriggerlSel(ADC0, 0x00000000); //trigger timer0
	HAL_ADC_TriggerlSel(ADC1, 0x00000000); //trigger timer0
		
	HAL_ADC_ChannelSel_1(ADC0, 0x0a090200);	//ch10(AN7), ch9(AN6), ch2(AN1), ch0(AN0)
	HAL_ADC_ChannelSel_1(ADC1, 0x1211100f);	//ch18(AN19), ch17(AN18), ch16(AN17), ch15(AN16)  
	
	HAL_ADC_ClearStatus(ADC0, 0xff); // clear status
	HAL_ADC_ConfigInterrupt(ADC0, ( ADC_INTEN_SEQ | ADC_INTEN_TRIGGER), ENABLE);	// Sequence, Trigger Interrupt Enable

	HAL_ADC_ClearStatus(ADC1, 0xff); // clear status
	HAL_ADC_ConfigInterrupt(ADC1, ( ADC_INTEN_SEQ | ADC_INTEN_TRIGGER), ENABLE);	 // Sequence, Trigger Interrupt Enable
	
	/* Channel information option */
	ADC0->MR |= (1<<20);
	ADC0->MR |= (1<<21);
	ADC1->MR |= (1<<20);
	ADC1->MR |= (1<<21);
	
	NVIC_SetPriority(ADC0_IRQn, 3);
	NVIC_EnableIRQ(ADC0_IRQn);

	NVIC_SetPriority(ADC1_IRQn, 4);
	NVIC_EnableIRQ(ADC1_IRQn);
	
	adcirq_count_ad0 = adcirq_count_ad1= 0;

	count=0;
}

/**********************************************************************
 * @brief		DMA_StartRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_StartRun(void)
{
	HAL_DMA_Start(DMA0);
	HAL_DMA_Start(DMA1);
}

/**********************************************************************
 * @brief		TIMER_StartRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void TIMER_StartRun(void)
{
	HAL_TIMER_Cmd(TIMER0, ENABLE); // timer start	
}

/**********************************************************************
 * @brief		DMA_ADCTestRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_ADCTestRun(void)
{
		if(adcirq_count_ad0> 0)
		{
			__disable_irq();
				
			adcirq_count_ad0 = adcirq_count_ad1 = 0;
			_DBG("\r\n\r\n ADC0 [100] : \r\n");
			for( ii=1;ii<=ADC_DMA_Num;ii++)
			{
				_DBD(data0[ii-1]>>16);
				_DBG("-");
				_DBH16((data0[ii-1]>>4)&0x0fff);
				_DBG(", ");
				if(!(ii%8))					
					_DBG("\r\n");
			}

			_DBG("\r\n\r\n ADC1 [100] : \r\n");
			for( ii=1;ii<=ADC_DMA_Num;ii++)
			{
				_DBD(data1[ii-1]>>16);
				_DBG("-");
				_DBH16((data1[ii-1]>>4)&0x0fff);
				_DBG(", ");				
				if(!(ii%8))
					_DBG("\r\n");
			}

			_DBG("\r\nPlease AnyKey..");
			UARTGetChar(UART0);
			
			/* Enable IRQ Interrupts */
			__enable_irq();
	  	
			/* DMA Start */
			DMA_StartRun(); 
		}
	
		for(tmp = 0; tmp < 0x3FFFF; tmp++);
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
	
	/*PCU Configure*/
  GPIO_Configure(); 
	
	/*TIMER0 Configure*/
  TIMER_Configure(); 
	
	/*DMA0, DMA1 Configure*/
  DMA_Configure(); 

	/*ADC0,ADC1 Configure*/
  ADC_Configure(); 
	
	/*DMA0, DMA1 Start*/
  DMA_StartRun(); 

	/*ADC Trigger : TIMER0, TIMER0 Start*/
  TIMER_StartRun(); 
	
	/* Enable IRQ Interrupts */
	__enable_irq();
	  	
  /* Infinite loop */
  while(1)
	{
		/* ADC0, ADC1 DMA Check Message */
		DMA_ADCTestRun();
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
