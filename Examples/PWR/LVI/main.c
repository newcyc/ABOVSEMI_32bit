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
void PWR_LVIRun(void);
void mainloop(void);
int main (void);
//void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
" LVI demo \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART1 - 38400 bps \n\r"
"\t LVI Test Example \n\r"
"\t LVI Interrupt Mode Level Selection \n\r"
"\t 0. 1.60V \n\r"
"\t 1. 1.69V \n\r"
"\t 2. 1.78V \n\r"
"\t 3. 1.90V \n\r"
"\t 4. 1.99V \n\r"
"\t 5. 2.12V \n\r"
"\t 6. 2.30V \n\r"
"\t 7. 2.47V \n\r"
"\t 8. 2.67V \n\r"
"\t 9. 3.04V \n\r"
"************************************************\n\r";
const uint8_t cmdm_p_l[] =
"A34M41x> ";
uint32_t msec_p_l;

/**********************************************************************
 * @brief		LVI_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void LVI_IRQHandler_IT(void)
{
	_DBG("LVI Interrupt Done !!\r\n");
	
	while(SCU->LVISR&(1<<0));
	
	SCU->LVISR |= ((0x7A<<24)|(1<<5));		//LVI Flag Clear	
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

}


/**********************************************************************
 * @brief		PWR_LVIRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void PWR_LVIRun(void)
{
	LVI_CFG_Type	LVI_config;
	uint8_t cmdn_cnt=0, ch_rtn;
	uint32_t i;
		
	while(1)
	{
		Lvi_test :
		if (cmdn_cnt ==0)
		{
			_DBG(cmdm_p_l);
			cmdn_cnt=1;
		}
		
		SYST_ACCESS_EN();
		// LVR Disable
		SCU->LVRCR = 0xAA<<8;
		
		LVI_config.EnSel = ENABLE;
		LVI_config.IntSel = ENABLE;
		LVI_config.DeepSel = DISABLE;
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\n\r");
		
		if (ch_rtn == 0x0D)
		{
			_DBG("\n\r");
		}
		
		if (ch_rtn == 0x30)		
		{
			_DBG("LVI Flag (1.60V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_1_60V;
		}
		else if (ch_rtn == 0x31)		
		{
			_DBG("LVI Flag (1.69V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_1_69V;
		}
		else if (ch_rtn == 0x32)		
		{
			_DBG("LVI Flag (1.78V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_1_78V;
		}
		else if (ch_rtn == 0x33)		
		{
			_DBG("LVI Flag (1.90V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_1_90V;
		}
		else if (ch_rtn == 0x34)		
		{
			_DBG("LVI Flag (1.99V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_1_99V;
		}
		else if (ch_rtn == 0x35)		
		{
			_DBG("LVI Flag (2.12V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_2_12V;
		}		
		else if (ch_rtn == 0x36)		
		{
			_DBG("LVI Flag (2.30V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_2_30V;
		}	
		else if (ch_rtn == 0x37)		
		{
			_DBG("LVI Flag (2.47V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_2_47V;
		}
		else if (ch_rtn == 0x38)		
		{
			_DBG("LVI Flag (2.67V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_2_67V;
		}
		else if (ch_rtn == 0x39)		
		{
			_DBG("LVI Flag (3.04V) Waitting \r\n");
			LVI_config.LvlSel = LVI_LVL_3_04V;
		}	
		else
		{
			_DBG("...Please input character 0~9 !!! \n\r");
			cmdn_cnt=0;
			goto Lvi_test;
		}

		HAL_LVI_Init(&LVI_config);
			
		NVIC_SetPriority(LVI_IRQn, 3);
		NVIC_EnableIRQ(LVI_IRQn);

		__enable_irq();
		
		while(1)
		{
			_DBG(" .");
					
			for (i=0; i<0x0FFFF; i++);
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

	PWR_LVIRun();
	
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

