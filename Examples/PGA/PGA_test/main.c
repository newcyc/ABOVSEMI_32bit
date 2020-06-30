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
void PGA_TestRun(void);

void mainloop(void);
int main (void);
void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
"\t PGA demo \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t PGA Test Example \n\r"
"\t Gain Selection \n\r"
"\t 0. 1.200 \n\r"
"\t 1. 1.304 \n\r"
"\t 2. 1.404 \n\r"
"\t 3. 1.500 \n\r"
"\t 4. 1.600 \n\r"
"\t 5. 1.702 \n\r"
"\t 6. 1.805 \n\r"
"\t 7. 1.905 \n\r"
"\t 8. 2.000 \n\r"
"\t 9. 2.182 \n\r"
"************************************************\n\r";
const uint8_t cmdm_p_t[] =
"A34M41x> ";
uint32_t status_p_t;
uint16_t adcval_p_t[8];




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
	// AN0,AN1 pin setting
	// AN0(PA0)
	HAL_GPIO_ConfigureFunction(PA, 0, PA0_MUX_AN0);
	HAL_GPIO_ConfigOutput(PA, 0, INPUT);
	HAL_GPIO_ConfigPullup(PA, 0, PULL_UP_DOWN_DISABLE);
	// AN1(PA1)
	HAL_GPIO_ConfigureFunction(PA, 1, PA1_MUX_AN1);
	HAL_GPIO_ConfigOutput(PA, 1, INPUT);
	HAL_GPIO_ConfigPullup(PA, 0, PULL_UP_DOWN_DISABLE);
}


/**********************************************************************
 * @brief		adcaverage_p_t
 * @param[in]	None
 * @return 	None
 **********************************************************************/
uint16_t adcaverage_p_t(uint16_t *val)
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
 * @brief		PGA_TestRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void PGA_TestRun(void)
{
	PGA_CFG_Type PGA_config;
	ADC_CFG_Type AD_config;
	uint32_t cmdn_cnt, ch_rtn;
	uint32_t j,k,i;
	uint32_t adcavg;
			
	while(1)
	{
		Pga_test :
		if (cmdn_cnt ==0)
		{
			_DBG(cmdm_p_t);
			cmdn_cnt=1;
		}
			
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\n\r");
		
		if (ch_rtn == 0x0D)
		{
			_DBG("\n\r");
		}
		
		if (ch_rtn == 0x30)		
		{
			_DBG("Gain 1.200 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_1_200;
		}
		else if (ch_rtn == 0x31)		
		{
			_DBG("Gain 1.304 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_1_304;
		}
		else if (ch_rtn == 0x32)		
		{
			_DBG("Gain 1.404 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_1_404;
		}
		else if (ch_rtn == 0x33)		
		{
			_DBG("Gain 1.500 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_1_500;
		}
		else if (ch_rtn == 0x34)		
		{
			_DBG("Gain 1.600 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_1_600;
		}
		else if (ch_rtn == 0x35)		
		{
			_DBG("Gain 1.702 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_1_702;
		}		
		else if (ch_rtn == 0x36)		
		{
			_DBG("Gain 1.805 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_1_805;
		}	
		else if (ch_rtn == 0x37)		
		{
			_DBG("Gain 1.905 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_1_905;
		}
		else if (ch_rtn == 0x38)		
		{
			_DBG("Gain 2.000 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_2_000;
		}
		else if (ch_rtn == 0x39)		
		{
			_DBG("Gain 2.182 Setting Success !! \r\n");
			PGA_config.GainSel = PGA_GAINSEL_2_182;
		}	
		else
		{
			_DBG("...Please input character 0~9 !!! \n\r");
			cmdn_cnt=0;
			goto Pga_test;
		}
		
		/* PGA Configure Setting */
		PGA_config.AMPISel = PGA_CR_AMPISEL_0;
		PGA_config.UGainEnSel = DISABLE;
		PGA_config.AMPEnSel = ENABLE;
		
		HAL_PGA_ClockInit(ENABLE);

		if(HAL_PGA_Init(PGA0, &PGA_config) != HAL_OK)
		{
			Error_Handler();			
		}
		
		// ADC Configuration
		AD_config.Mode = ADC_SINGLE_MODE;
		AD_config.SamplingTime = 1;
		AD_config.SeqCnt = 1;
		AD_config.RestartEn = 0;
		AD_config.TrgSel = ADC_TRIGGER_DISABLE;
	
		AD_config.UseClk = ADC_EXTERNAL_CLK;
		AD_config.InClkDiv = (((SystemPeriClock/1000000UL) * 2)/ (3*(15+AD_config.SamplingTime))) + 1; 
		
		HAL_SCU_SetMCCRx(4, ADC_TYPE, SCU_MCCR_CSEL_HSI, 8);
		
		if(HAL_ADC_Init(ADC0, &AD_config) != HAL_OK)
		{
			Error_Handler();			
		}
	
		while(1)
		{
			for(i=0;i<=1;i++)
			{
				HAL_ADC_ChannelSel(ADC0, i); // select ch
				for (j=0;j<(8);j++)
				{
					HAL_ADC_Start(ADC0); // start 	
					while((HAL_ADC_GetStatus(ADC0) & (ADC_STAT_SINGLE)) !=(ADC_STAT_SINGLE)){}		
					HAL_ADC_ClearStatus(ADC0, ADC_STAT_SINGLE);
					adcval_p_t[j] = ADC0->DR0>>4;
				}
				adcavg=adcaverage_p_t(adcval_p_t);
				_DBG("CH"); _DBD(i); _DBG("="); _DBH16(adcavg); _DBG(" "); 
			}
			for(k=0; k<0x1FFFF; k++);
			_DBG("\n\r");
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

	/*Configure port peripheral*/
	PGA_TestRun();
	
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

