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
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void SPI_Configure(void);
void mainloop(void);
int main (void);
void Error_Handler(void);	

static uint32_t spi_rx,spi_tx, spi_add;
static SPI_CFG_Type SP_config;
static DMA_CFG_Type	DMA_Config;

/* Private variables ---------------------------------------------------------*/	
const uint8_t cmdm[] =
"A34M41x> ";
const uint8_t menu[] =
"************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t DMA FUNCTION TEST ...  \n\r"
"\t 1. Peripheral(SPI0) -> Memory \n\r"
"\t 2. Memory -> Peripheral(SPI0) \n\r"
"************************************************\n\r";

/**********************************************************************
 * @brief		SPI0_IRQHandle
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SPI0_IRQHandler_IT(void)
{
	uint32_t status;
	
	status = HAL_SPI_GetStatus(SPI0);
	if ( (status & SPI_STAT_TXBUF_EMPTY) && (HAL_SPI_GetControl(SPI0) & SPI_INTCFG_TXIE)){ // tx buffer empty
		HAL_SPI_ConfigInterrupt(SPI0, SPI_INTCFG_TXIE, DISABLE);	
		HAL_SPI_TransmitData(SPI0, 0x00);
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
 * @brief		SPI_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void SPI_Configure(void)
{
	/* SPI enable */
	SP_config.Databit = SPI_DS_8BITS;
	SP_config.CPHA = SPI_CPHA_LO;
	SP_config.CPOL = SPI_CPOL_LO;
	SP_config.DataDir = SPI_MSB_FIRST;
	SP_config.Mode = SPI_MASTER_MODE;
	SP_config.BaudRate = 79; // PCLK / (79+1) 
	
	if((HAL_SPI_Init(SPI0, &SP_config))!=HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();		
	}
	
	HAL_SPI_ConfigInterrupt(SPI0, SPI_INTCFG_TXIE, ENABLE); // tx interrupt enable 
	HAL_SPI_Enable(SPI0, ENABLE);  // SPI enable 

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(SPI0_IRQn, ((0x01<<1)|0x01));
	NVIC_EnableIRQ(SPI0_IRQn);
	
}

/**********************************************************************
 * @brief		DMA_SPIRxConfigure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_SPIRxConfigure(void)
{
		spi_rx = SPI0_RX;
		spi_add = 0x40009004;
				
		HAL_DMA_Init();
		
		DMA_Config.transcnt = 1;
		DMA_Config.perisel = spi_rx;
		DMA_Config.bussize = DMA_CR_WORD_TRANS;
		DMA_Config.dirsel = DMA_CR_DIR_PERI_TO_MEM;
		
		HAL_DMA_Cmd(DMA0, &DMA_Config);
		
		HAL_DMA_SetPAR(DMA0, spi_add);
		HAL_DMA_SetMAR(DMA0, 0x20001000);
		
		HAL_DMA_Start(DMA0);
		
		#ifdef _DEBUG_MSG
		_DBG("1. Peripheral(SPI0) -> Memory \n\r"); 
		_DBG(" SPI0[0x4000_9004] : 0x"); _DBH32(SPI0->CR); 
		_DBG("\n\r Memory[0x2000_1000] : 0x"); _DBH32((*(volatile unsigned int *)(0x20001000)));
		_DBG("\n\r");
		#endif
}


/**********************************************************************
 * @brief		DMA_SPITxConfigure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_SPITxConfigure(void)
{
	spi_tx = SPI0_TX;
	spi_add = 0x40009004;
	
	HAL_DMA_Init();
	
	DMA_Config.transcnt = 1;
	DMA_Config.perisel = spi_tx;
	DMA_Config.bussize = DMA_CR_WORD_TRANS;
	DMA_Config.dirsel = DMA_CR_DIR_MEM_TO_PERI;
	
	HAL_DMA_Cmd(DMA0, &DMA_Config);
	
	MIO32(0x20001000) = 0x3F;
	
	#ifdef _DEBUG_MSG
	_DBG("2. Memory -> Peripheral(SPI0) \n\r"); 
	_DBG(" Memory[0x2000_1000] : 0x"); _DBH32((*(volatile unsigned int *)(0x20001000)));
	_DBG("\n\r");
	#endif
	
	HAL_DMA_SetPAR(DMA0, spi_add);
	HAL_DMA_SetMAR(DMA0, 0x20001000);

	HAL_DMA_Start(DMA0);

	#ifdef _DEBUG_MSG
	_DBG(" SPI0[0x4000_9004] : 0x"); _DBH32(SPI0->CR); _DBG("\n\r");
	#endif
}


/**********************************************************************
 * @brief		Main loop
 * @param[in]	None
 * @return	None
 **********************************************************************/
void mainloop(void)
{
		uint8_t		cmdn_cnt=0, ch_rtn, mod_check[3] = {0x30, 0x31, 0x32};
	/*Configure menu prinf*/
	DEBUG_MenuPrint();
	
	/*SPI Configure*/
  SPI_Configure();
	
	/* Enable IRQ Interrupts */
	__enable_irq(); 
  	

   /* Infinite loop */
   while(1)
	{
		if (cmdn_cnt == 0)
		{
			_DBG(cmdm);
			cmdn_cnt=1;
		}
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
	
		 if (ch_rtn == mod_check[1])
		{
				/*DMA Configure - DMA0(SPI0_RX) */
    DMA_SPIRxConfigure();
			cmdn_cnt = 0;
		}
		else if (ch_rtn == mod_check[2])
		{		
				/*SPI Configure - DMA1(SPI0_TX) */
    DMA_SPITxConfigure();
			cmdn_cnt = 0;
		}
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

