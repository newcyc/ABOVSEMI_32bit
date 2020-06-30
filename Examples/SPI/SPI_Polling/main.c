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
void SPI_PollingRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t SPI POLLING MODE TEST...   \n\r"
"\t MASTER : SPI1, SLAVE : SPI0  \n\r"
"\t 1. SPI Init \n\r"
"\t 2. SPI Test \n\r"
"************************************************\n\r";
const uint8_t cmdm_s_p[] =
"A34M41x> ";
uint8_t gSPI_data(uint8_t ch);


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
			// SLAVE : SPI0(SS0, SCK0, MOSI0, MISO0) PIN FUNCTION SETTING
			HAL_GPIO_ConfigureFunction(PA, 12, PA12_MUX_SS0);
			HAL_GPIO_ConfigOutput(PA, 12, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PA, 12, PULL_UP_DOWN_DISABLE);

			HAL_GPIO_ConfigureFunction(PA, 13, PA13_MUX_SCK0);
			HAL_GPIO_ConfigOutput(PA, 13, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PA, 13, PULL_UP_DOWN_DISABLE);

			HAL_GPIO_ConfigureFunction(PA, 14, PA14_MUX_MOSI0);
			HAL_GPIO_ConfigOutput(PA, 14, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PA, 14, PULL_UP_DOWN_DISABLE);

			HAL_GPIO_ConfigureFunction(PA, 15, PA15_MUX_MISO0);
			HAL_GPIO_ConfigOutput(PA, 15, INPUT);
			HAL_GPIO_ConfigPullup(PA, 15, PULL_UP_ENABLE);

			// MASTER : SPI1(SS1, SCK1, MOSI1, MISO1) PIN FUNCTION SETTING
			HAL_GPIO_ConfigureFunction(PD, 0, PD0_MUX_SS1);
			HAL_GPIO_ConfigOutput(PD, 0, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PD, 0, PULL_UP_DOWN_DISABLE);

			HAL_GPIO_ConfigureFunction(PD, 1, PD1_MUX_SCK1);
			HAL_GPIO_ConfigOutput(PD, 1, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PD, 1, PULL_UP_DOWN_DISABLE);

			HAL_GPIO_ConfigureFunction(PD, 2, PD2_MUX_MOSI1);
			HAL_GPIO_ConfigOutput(PD, 2, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PD, 2, PULL_UP_DOWN_DISABLE);

			HAL_GPIO_ConfigureFunction(PD, 3, PD3_MUX_MISO1);
			HAL_GPIO_ConfigOutput(PD, 3, INPUT);
			HAL_GPIO_ConfigPullup(PD, 3, PULL_UP_ENABLE);
}


/**********************************************************************
 * @brief		SPI_PollingRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void SPI_PollingRun(void)
{
	uint32_t rxdata;
	uint32_t tmp;
	uint32_t i, j;
	SPI_CFG_Type SPS_config;
	SPI_CFG_Type SPM_config;
	
	uint8_t		cmdn_cnt=0, ch_rtn, mod_check[3] = {0x30, 0x31, 0x32};
		
	while(1)
	{
		if (cmdn_cnt == 0)
		{
			_DBG(cmdm_s_p);
			cmdn_cnt=1;
		}
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		

		if (ch_rtn == mod_check[1])
		{
			_DBG("SPI Init OK\r\n");
			SPS_config.Databit = SPI_DS_8BITS;
			SPS_config.CPHA = SPI_CPHA_LO;
			SPS_config.CPOL = SPI_CPOL_LO;
			SPS_config.DataDir = SPI_MSB_FIRST;
			SPS_config.Mode = SPI_SLAVE_MODE;
			SPS_config.BaudRate = 79; // PCLK / (79+1) 
		
			// SPI0 EANBLE
			if(HAL_SPI_Init(SPI0, &SPS_config) != HAL_OK)
			{
				Error_Handler();
			}
			HAL_SPI_Enable(SPI0, ENABLE);  	

			SPM_config.Databit = SPI_DS_8BITS;
			SPM_config.CPHA = SPI_CPHA_LO;
			SPM_config.CPOL = SPI_CPOL_LO;
			SPM_config.DataDir = SPI_MSB_FIRST;
			SPM_config.Mode = SPI_MASTER_MODE;
			SPM_config.BaudRate = 79; // PCLK / (79+1) 
			
			// SPI1 ENABLE
			if(HAL_SPI_Init(SPI1, &SPM_config) != HAL_OK)
			{
				Error_Handler();
			}
			HAL_SPI_Enable(SPI1, ENABLE);  

			cmdn_cnt=0;
		}
		
		else if (ch_rtn == mod_check[2])
		{
			_DBG("SPI Test Start!!\r\n");
			_DBG_(" ");
			
			j=0;
			i=0x30;
			_DBG(" init1 tx= ");_DBH(i);_DBG_(" ");	
			while(1){
				
				// CHECK TX BUFFER EMPTY
				while((HAL_SPI_GetStatus(SPI1) & SPI_STAT_TXBUF_EMPTY) != SPI_STAT_TXBUF_EMPTY);  
				HAL_SPI_Enable(SPI1, DISABLE);  // SPI disable 
				HAL_SPI_TransmitData(SPI1, i);
				HAL_SPI_Enable(SPI1, ENABLE);  // SPI enable 	
			
				// CHECK RX BUFFER FULL
				while((HAL_SPI_GetStatus(SPI0) & SPI_STAT_RXBUF_READY) != SPI_STAT_RXBUF_READY); 
				rxdata = HAL_SPI_ReceiveData(SPI0);
				_DBG(" tx= ");_DBH(i);_DBG(" rx= ");_DBH(rxdata); _DBG_(" ");
			
				i++;
				if (i==0x3a)
				{
					i=0x30;
					j++;
				}
				
				if (j==0x01)
				{
					cmdn_cnt=0;
					break;
				}
				
				for (tmp=0;tmp <1000000; tmp++);	
				
			}
		}
		else
		{
			_DBG("...Please input character 1:Init, 2:Test!  \n\r");
			cmdn_cnt=0;
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

	SPI_PollingRun();
	
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

