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
#define Rx_DMA_C  672
#define SPI_BORATE_set 7

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SPI0_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void DMA_SPIPollingRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);	

/* Private variables ---------------------------------------------------------*/	
const uint8_t cmdm[] =
"A34M41x> ";
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

uint32_t rx_spi_dma_don = 0;

uint8_t CC=0;
uint8_t Loop_Data_C =0;
uint8_t data_p[6]={0xac,0xc3,0x3f,0x10,0xf8,0x72};

uint32_t spi_tx,spi_rx, spi_add_rx,spi_add_tx,status_tx;
uint32_t ii,i,j,SPI_TX_OK,SPI_RX_OK;
SPI_CFG_Type SPS_config;
SPI_CFG_Type SPM_config;
DMA_CFG_Type	DMA_Config_rx;
DMA_CFG_Type	DMA_Config_tx;

uint8_t SPI_rx_b[Rx_DMA_C]={0,};
uint8_t SPI_tx_b[Rx_DMA_C]={0,};

/**********************************************************************
 * @brief		SPI0_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SPI0_IRQHandler_IT(void)
{
	uint32_t status;
	
	status = HAL_SPI_GetStatus(SPI0);
	
	if ( (status & SPI_STAT_RXDMA_DONE) == SPI_STAT_RXDMA_DONE)
		{ // rx buffer full		
		i = 0;
			_DBG(" dMA\r\n ");
			rx_spi_dma_don = 0x77;
				HAL_SPI_ClearPending(SPI0,SPI_STAT_RXDMA_DONE);
		}
	else 
		i = 7;
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
 * @brief		SPI_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void SPI_Configure(void)
{
			SPI_TX_OK = 0;
	
			// SPI0 Slave mode setting
			SPS_config.Databit = SPI_DS_8BITS;
			SPS_config.CPHA = SPI_CPHA_LO;
			SPS_config.CPOL = SPI_CPOL_LO;
			SPS_config.DataDir = SPI_MSB_FIRST;
			SPS_config.Mode = SPI_SLAVE_MODE;
			SPS_config.BaudRate = SPI_BORATE_set; // PCLK / (7+1) == 8Mhz / (7+1) = 1Mhz
	
			// SPI0 EANBLE
			if((HAL_SPI_Init(SPI0, &SPS_config))!=HAL_OK)
			{
				/* Initialization Error */
				Error_Handler();				
			}
			HAL_SPI_Enable(SPI0, ENABLE);  
	
			// SPI0 rx interrupt enable 
			HAL_SPI_ConfigInterrupt(SPI0, SPI_INTCFG_RXDIE , ENABLE); 

			/* preemption = 1, sub-priority = 1 */
			NVIC_SetPriority(SPI0_IRQn, ((0x01<<1)|0x01));		
			NVIC_EnableIRQ(SPI0_IRQn);
	
			// SPI1 Master mode setting
			SPM_config.Databit = SPI_DS_8BITS;
			SPM_config.CPHA = SPI_CPHA_LO;
			SPM_config.CPOL = SPI_CPOL_LO;
			SPM_config.DataDir = SPI_MSB_FIRST;
			SPM_config.Mode = SPI_MASTER_MODE;
			SPM_config.BaudRate = SPI_BORATE_set;// PCLK / (7+1) == 8Mhz / (7+1) = 1Mhz
			
			// SPI1 ENABLE
			if((HAL_SPI_Init(SPI1, &SPM_config))!=HAL_OK)
			{
				/* Initialization Error */
				Error_Handler();				
			}
			HAL_SPI_Enable(SPI1, ENABLE);  
}

/**********************************************************************
 * @brief		DMA_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_Configure(void)
{
			HAL_DMA_Init();
			
			// DMA0 : SPI0_RX
			spi_rx = SPI0_RX;
			spi_add_rx = 0x40009000;		
	
			DMA_Config_rx.transcnt = Rx_DMA_C;
			DMA_Config_rx.perisel = spi_rx;
			DMA_Config_rx.bussize = DMA_CR_BYTE_TRANS;
			DMA_Config_rx.dirsel = DMA_CR_DIR_PERI_TO_MEM;			
	
			HAL_DMA_Cmd(DMA0, &DMA_Config_rx);			
			HAL_DMA_SetPAR(DMA0, spi_add_rx);
			HAL_DMA_SetMAR(DMA0, (uint32_t)SPI_rx_b);
					
			// DMA1 : SPI1_TX			
			spi_tx = SPI1_TX;
			spi_add_tx = 0x40009100;					
			
			DMA_Config_tx.transcnt = Rx_DMA_C;
			DMA_Config_tx.perisel = spi_tx;
			DMA_Config_tx.bussize = DMA_CR_BYTE_TRANS;
			DMA_Config_tx.dirsel = DMA_CR_DIR_MEM_TO_PERI;			
			HAL_DMA_Cmd(DMA1, &DMA_Config_tx);			
			HAL_DMA_SetPAR(DMA1, spi_add_tx);
			HAL_DMA_SetMAR(DMA1, (uint32_t)SPI_tx_b);
}

/**********************************************************************
 * @brief		DMA_SPIPollingRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_SPIPollingRun(void)
{
	SPI_RX_OK=0x77;

	if(CC==0)
	{
		CC++;
	}
	else if(CC>=1)
	{
			if(SPI_RX_OK == 0x77)
			{
				_DBG("\r\n\r\n Tx Num ");_DBD16(Rx_DMA_C); _DBG(" : "); _DBH(data_p[Loop_Data_C]); _DBG_(" ");
				HAL_DMA_Start(DMA0);
				SPI_RX_OK = 0;
			}
			while(1)
			{
				status_tx = HAL_SPI_GetStatus(SPI1);				
				if((status_tx & SPI_STAT_TXDMA_DONE) != SPI_STAT_TXDMA_DONE )
				{
						SPI_tx_b[i] = data_p[Loop_Data_C];
						i++;

						if(i >= Rx_DMA_C)
						{
							spi_tx = SPI1_TX;
							spi_add_tx = 0x40009100;					
							DMA_Config_tx.transcnt = Rx_DMA_C;
							DMA_Config_tx.perisel = spi_tx;
							DMA_Config_tx.bussize = DMA_CR_BYTE_TRANS;
							DMA_Config_tx.dirsel = DMA_CR_DIR_MEM_TO_PERI; 		
							HAL_DMA_Cmd(DMA1, &DMA_Config_tx); 		
							HAL_DMA_SetPAR(DMA1, spi_add_tx);
							HAL_DMA_SetMAR(DMA1, (uint32_t)SPI_tx_b);
							i=0;
							SPI_TX_OK = 0x77;
							break;
						}
				}
			else
			{
				if((status_tx & SPI_STAT_TXDMA_DONE) == SPI_STAT_TXDMA_DONE)
						HAL_SPI_ClearPending(SPI1,SPI_STAT_TXDMA_DONE);
			}
		}			
		if(SPI_TX_OK == 0x77)
		{
			SPI_TX_OK = 0;
			HAL_DMA_Start(DMA1);
		}	
		while(1)
		{
			if((status_tx & SPI_STAT_TXDMA_DONE) == SPI_STAT_TXDMA_DONE)
				HAL_SPI_ClearPending(SPI1,SPI_STAT_TXDMA_DONE);

			if(rx_spi_dma_don == 0x77)					
			{
				rx_spi_dma_don = 0;

				_DBG(" rx= ");
				for(ii=0;ii<Rx_DMA_C;ii++)
				{
					if(SPI_rx_b[ii] != data_p[Loop_Data_C])
						_DBG("\r\nFail!!\r\n");
					_DBH(SPI_rx_b[ii]); 
				}
				j = 0;
				if(ii ==Rx_DMA_C )
					_DBG(" -SUCCESS!!");

				
				Loop_Data_C++;
				if(Loop_Data_C >= 6)
					Loop_Data_C = 0;

				
				spi_rx = SPI0_RX;
				spi_add_rx = 0x40009000;
				DMA_Config_rx.transcnt = Rx_DMA_C;
				DMA_Config_rx.perisel = spi_rx;
				DMA_Config_rx.bussize = DMA_CR_BYTE_TRANS;
				DMA_Config_rx.dirsel = DMA_CR_DIR_PERI_TO_MEM;
				HAL_DMA_Cmd(DMA0, &DMA_Config_rx);
				HAL_DMA_SetPAR(DMA0, spi_add_rx);
				HAL_DMA_SetMAR(DMA0, (uint32_t)SPI_rx_b);
				
				HAL_SPI_ClearPending(SPI0,SPI_STAT_RXDMA_DONE);
				SPI_RX_OK = 0x77;
				break;
			}
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
	
	/* Enable IRQ Interrupts */
	__enable_irq();
	
	/* PCU Configure*/
  GPIO_Configure(); 
 
	/* SPI0 Configure - SPI0 : Slave, SPI1: Master */
  SPI_Configure(); 

	/* DMA Configure - DMA0 : SPI0_RX, DMA1 : SPI1_TX */
  DMA_Configure(); 
 	
   /* Infinite loop */
  while(1)
	{
		/* SPI DMA Message */
		DMA_SPIPollingRun();
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

