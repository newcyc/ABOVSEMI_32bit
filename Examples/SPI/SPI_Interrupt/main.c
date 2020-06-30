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
void SPI0_IRQHandler_IT(void);
void SPI1_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void SPI_InterruptRun(void);
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
"\t SPI INTERRUPT MODE TEST \n\r"
"\t MASTER : SPI1, SLAVE : SPI0 \n\r"
"\t 1. SPI Init \n\r"
"\t 2. SPI Test \n\r"
"************************************************\n\r";
const uint8_t cmdm_s_i[] =
"A34M41x> ";

uint8_t gSPI_data(uint8_t ch);
uint32_t fflag_s_i;
volatile uint32_t rxdata_s_i[10], txdata_s_i[10];
uint32_t cnt_s_i, cntrx_s_i;


/**********************************************************************
 * @brief		SPI0_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SPI0_IRQHandler_IT(void)
{
	uint32_t status;
	
	status = HAL_SPI_GetStatus(SPI0);
	
	if ( (status & SPI_STAT_RXBUF_READY) && (HAL_SPI_GetControl(SPI0) & SPI_INTCFG_RXIE)){ // rx buffer full			
		rxdata_s_i[cntrx_s_i] = SPI0->RDR;
		cntrx_s_i++;
		if (cntrx_s_i==10){
			fflag_s_i=1;
		}
	}	
}


/**********************************************************************
 * @brief		SPI1_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SPI1_IRQHandler_IT(void)
{
	uint32_t status;
	
	status = HAL_SPI_GetStatus(SPI1);
	if ( (status & SPI_STAT_TXBUF_EMPTY) && (HAL_SPI_GetControl(SPI1) & SPI_INTCFG_TXIE)){ // tx buffer empty		
		if ((status & SPI_STAT_IDLE))
		{
			HAL_SPI_Enable(SPI1, DISABLE);  // SPI disable
			HAL_SPI_TransmitData(SPI1, txdata_s_i[cnt_s_i]); cnt_s_i++;
			HAL_SPI_Enable(SPI1, ENABLE);  // SPI enable
		}
		else{
			HAL_SPI_TransmitData(SPI1, txdata_s_i[cnt_s_i]); cnt_s_i++;				
		}	
		
		if (cnt_s_i==10){
			HAL_SPI_ConfigInterrupt(SPI1, SPI_INTCFG_TXIE, DISABLE); //TXIE disable
		}
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
 * @brief		PCU_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_Configure(void)
{
			//SLAVE :SPI0(SS0, SCK0, MOSI0, MISO0) Pin Setting
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
	
			//MASTER : SPI1(SS1, SCK1, MOSI1, MISO1) Pin Setting
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
 * @brief		SPI_InterruptRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void SPI_InterruptRun(void)
{
	uint32_t tmp;
	uint32_t i,j;
	uint8_t		cmdn_cnt=0, ch_rtn, mod_check[3] = {0x30, 0x31, 0x32};
	SPI_CFG_Type SP_config;
		
	while(1)
	{
		if (cmdn_cnt == 0)
		{
			_DBG(cmdm_s_i);
			cmdn_cnt=1;
		}
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		

		if (ch_rtn == mod_check[1])
		{			
			_DBG("SPI Init OK  \n\r");
			SP_config.Databit = SPI_DS_8BITS;
			SP_config.CPHA = SPI_CPHA_LO;
			SP_config.CPOL = SPI_CPOL_LO;
			SP_config.DataDir = SPI_MSB_FIRST;
			SP_config.Mode = SPI_SLAVE_MODE;
			SP_config.BaudRate = 2; // PCLK / (2+1) 
			
			if(HAL_SPI_Init(SPI0, &SP_config) != HAL_OK)
			{
				Error_Handler();
			}

			// SPI0 rx interrupt enable 
			HAL_SPI_ConfigInterrupt(SPI0, SPI_INTCFG_RXIE, ENABLE); 
			 // SPI0 enable 
			HAL_SPI_Enable(SPI0, ENABLE); 

			/* preemption = 1, sub-priority = 1 */
			NVIC_SetPriority(SPI0_IRQn, ((0x01<<1)|0x01));		
			NVIC_EnableIRQ(SPI0_IRQn);
						
			SP_config.Databit = SPI_DS_8BITS;
			SP_config.CPHA = SPI_CPHA_LO;
			SP_config.CPOL = SPI_CPOL_LO;
			SP_config.DataDir = SPI_MSB_FIRST;
			SP_config.Mode = SPI_MASTER_MODE;
			SP_config.BaudRate = 79; // PCLK / (79+1) 
			
			if(HAL_SPI_Init(SPI1, &SP_config) != HAL_OK)
			{
				Error_Handler();
			}
			
			// SPI1 enable
			HAL_SPI_Enable(SPI1, ENABLE);  
			
			/* preemption = 1, sub-priority = 1 */
			NVIC_SetPriority(SPI1_IRQn, ((0x01<<1)|0x01));
			NVIC_EnableIRQ(SPI1_IRQn);

			__enable_irq();
			
			cmdn_cnt=0;
		}
		else if(ch_rtn == mod_check[2])
		{
			_DBG("SPI Test start!!  \n\r");	
			while(1){
				for(i=0;i<10;i++){
					txdata_s_i[i]=i+0x30;
					rxdata_s_i[i]=0;
				}
				cnt_s_i=0; cntrx_s_i=0;
				fflag_s_i=0;
				
				// Tx Interrupt Enable 
				HAL_SPI_ConfigInterrupt(SPI1, SPI_INTCFG_TXIE, ENABLE);		
				
				// check rx buffer full 
				while(fflag_s_i == 0); 
				
				for (i=0;i<10;i++){
					_DBG(" tx = "); _DBH(txdata_s_i[i]); _DBG(" rx = "); _DBH(rxdata_s_i[i]); _DBG_(" ");		
				}
				
				j++;
				
				for (tmp=0;tmp <1000000; tmp++);	
				
				if (j==0x03)
				{
					j=0;
					break;
				}
			}
			
			cmdn_cnt=0;
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

	SPI_InterruptRun();
	
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

