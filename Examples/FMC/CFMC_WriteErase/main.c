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
#define size			512

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void CFMC_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void CFMC_WriteEraseRun(void);
void CFMC_Configure(void);
void mainloop(void);
int main (void);


/* Private variables ---------------------------------------------------------*/	
const uint8_t cmdm[] =
"A34M41x> ";
const uint8_t menu[] =
"************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t Code Flash Test  \n\r"
"\t 1. Flash Init  \n\r"
"\t 2. Flash Erase[512KB, 0xF000~0xF200]  \n\r"
"\t 3. Flash Write[512B, 0xF000~0xF200] \n\r"
"\t 4. Flash Verify[512B, 0xF000~0xF200]  \n\r"
"************************************************\n\r";

static uint8_t	ch_rtn, cmdn_cnt=0, WaitVal;
static uint8_t mod_check[9] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x38, 0x39};
static uint32_t	CacheSel;
static int	i, tmp;
static unsigned char	fbuffer[size];
static unsigned long	result; 

void CFMC_FlashInit(uint8_t WaitVal, uint32_t CacheSel, FunctionalState CacheEnDis);


/**********************************************************************
 * @brief		CFMC_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void CFMC_IRQHandler_IT(void)
{
	HAL_CFMC_ClearAccessStatus(CFMC_STAT_WDONE);
	
	#ifdef _DEBUG_MSG
	_DBG(" Interrupt Done!\r\n");
	#endif
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
 * @brief		CFMC_FlashInit loop
 * @param[in]	WaitVal, Flash wait 0~15
 * @param[in]	CacheSel, CFMC_CONF_INST_CACHE_ON, CFMC_CONF_DATA_CACHE_ON
 * @param[in]	CacheEnDis, ENABLE/DISABLE
 * @return	None
 **********************************************************************/
void CFMC_FlashInit(uint8_t WaitVal, uint32_t CacheSel, FunctionalState CacheEnDis)
{
	HAL_CFMC_WaitCmd(WaitVal);
	
	HAL_CFMC_CacheCmd(CacheSel, CacheEnDis);
}

/**********************************************************************
 * @brief		CFMC_WriteEraseRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void CFMC_WriteEraseRun(void)
{
	if (cmdn_cnt==0)
	{
		_DBG(cmdm);
		cmdn_cnt=1;
	}
		
	ch_rtn = UARTGetChar(UART0);
		
	_DBC(ch_rtn);
	_DBG("\r\n");
		
	if (ch_rtn == mod_check[1])		// Flash Init
	{
		//-------------------------------------------------------
		// Get Test Value Line
		//-------------------------------------------------------
		// Get Wait Value
		_DBG("Wait Value(0~7) >");
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		WaitVal = ch_rtn;
		
		// Get Cache Selection
		_DBG("Cache Selection (0:Instruction, 1:Data, 2:Instruction+Data) >");
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		if (ch_rtn == 0x30)
		{
			CacheSel = CFMC_CONF_INST_CACHE_ON;
		}
		else if (ch_rtn == 0x31)
		{
			CacheSel = CFMC_CONF_DATA_CACHE_ON;
		}
		else if (ch_rtn == 0x32)
		{
			CacheSel = (CFMC_CONF_DATA_CACHE_ON | CFMC_CONF_INST_CACHE_ON);
		}
		
		// Get Cache Enable/Disable
		_DBG("Cache Config (0:Disable, 1:Enable) >");
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		if (ch_rtn == 0x31)
		{
			CFMC_FlashInit(WaitVal, CacheSel, ENABLE);
		}
		else
		{
			CFMC_FlashInit(WaitVal, CacheSel, DISABLE);
		}
		
		
		// Interrupt Enable/Disable
		_DBG("Interrpt Config (0:Disable, 1:Enable) >");
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		if (ch_rtn == 0x31)
		{
			HAL_CFMC_InterruptCmd(ENABLE);
		}
		else
		{
			HAL_CFMC_InterruptCmd(DISABLE);
		}
					
		// Flash Unlock
		HAL_CFMC_FlashAccessCmd(ENABLE);
		
		// Boot Block Lock
		CFMC->CONF |= (1<<24);
		
		cmdn_cnt=0;
	}
	else if (ch_rtn == mod_check[2])		// Flash Erase
	{
		
		__disable_irq();

		HAL_CFMC_Erase(CFMC_CTRL_PERS,0xF000);
		
		__enable_irq();
		
		for(i=0; i<10; i++)
		{
			_DBG(" .");
			
			for (tmp=0; tmp<0x2FF; tmp++);
		}
		
		_DBG("\r\n Erase Done !!! \r\n");
		
		cmdn_cnt=0;
	}
	else if (ch_rtn == mod_check[3])		// Flash Write
	{
		
		for(i=0;i<size;i++) {
			if (i%2 == 0)
			{
				fbuffer[i] = 0xAA;
			}
			else
			{
				fbuffer[i] = 0x55;
			}
		}
		
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		
		__disable_irq();
		HAL_CFMC_ProgramPage(0xF000, size, fbuffer);
		__enable_irq();
		
		for(i=0; i<10; i++)
		{
			_DBG(" .");
			
			for (tmp=0; tmp<0x2FF; tmp++);
		}
			
		_DBG("\r\n Write Done !!! \r\n");
		

		cmdn_cnt=0;
	}
	else if (ch_rtn == mod_check[4])		// Flash Verify
	{
		
		result = HAL_CFMC_Verify (0xF000, size, fbuffer); 
		
		if (result != (0xF000+size))
		{
			_DBG(" Verify Error !! \r\n");
		}
		else
		{
			for(i=0; i<10; i++)
			{
				_DBG(" .");
				
				for (tmp=0; tmp<0x2FF; tmp++);
			}
			
			_DBG("\r\n Verify Done !!! \r\n");
		}
		
		cmdn_cnt=0;
	
	}
	else 
	{
		_DBG("...Please input character 1~4 !  \n\r");
		cmdn_cnt=0;
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
	
   /* Infinite loop */
  while(1)
	{
		/* CFMC Message */
		CFMC_WriteEraseRun();
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


