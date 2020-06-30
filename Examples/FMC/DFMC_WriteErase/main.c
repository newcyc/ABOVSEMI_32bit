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
#define size				1024

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void DFMC_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void DFMC_WriteEraseRun(void);
void mainloop(void);
int main (void);

/* Private variables ---------------------------------------------------------*/	
const uint8_t cmdm[] =
"A34M41x> ";
const uint8_t menu[] =
"*****************************************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t Data Flash Test  \n\r"
"\t 1. DATA Flash Init  \n\r"
"\t 2. DATA Flash Erase[Page Erase - 1KB, 0x0E00_1000 ~ 0x0E00_1400]  \n\r"
"\t 3. DATA Flash Write[1KB, 0x0E00_1000 ~ 0x0E00_1400] \n\r"
"\t 4. DATA Flash Verify [1KB, 0x0E00_1000 ~ 0xE000_1400]  \n\r"
"*****************************************************************************\n\r";

	static uint8_t ch_rtn, cmdn_cnt=0, WaitVal;
	static uint8_t mod_check[5] = {0x30, 0x31, 0x32, 0x33, 0x34};
	
	static int	i, tmp;
	static unsigned char	fbuffer[size];
	static unsigned long	result; 
	
/**********************************************************************
 * @brief		DFMC_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void DFMC_IRQHandler_IT(void)
{
	HAL_DFMC_ClearAccessStatus(DFMC_STAT_WDONE);
	
	#ifdef _DEBUG_MSG
	_DBG("Interrupt Done !\r\n");
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
 * @brief		DFMC_WriteEraseRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DFMC_WriteEraseRun(void)
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
			
			_DBG("Interrupt Config (0:Disable, 1:Enable) > ");
			
			ch_rtn = UARTGetChar(UART0);
			
			_DBC(ch_rtn);
			_DBG("\r\n");
			
			if (ch_rtn == 0x31)
			{
				HAL_DFMC_InterruptCmd(ENABLE);
				__enable_irq();
			}
			else 
			{
				HAL_DFMC_InterruptCmd(DISABLE);
				__disable_irq();
			}
								
			HAL_DFMC_WaitCmd(WaitVal);
			
			// Flash Unlock
			HAL_DFMC_FlashAccessCmd(ENABLE);
				
			cmdn_cnt=0;
		}
		else if (ch_rtn == mod_check[2])		// Flash Erase
		{
			
			HAL_DFMC_Erase(DFMC_CTRL_S1KERS,0x0E001000);
			
			
			for(i=0; i<20; i++)
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
			HAL_DFMC_ProgramPage(0xE001000, size, fbuffer);
			__enable_irq();
			
			for(i=0; i<20; i++)
			{
				_DBG(" .");
				
				for (tmp=0; tmp<0x2FF; tmp++);
			}
			
			_DBG("\r\n Write Done !!! \r\n");	
			
			cmdn_cnt=0;
		}
		else if (ch_rtn == mod_check[4])		// Flash Verify
		{
			
			result = HAL_DFMC_Verify(0xE001000, size, fbuffer); 
			
			if (result != (0xE001000+size))
			{
				_DBG("Verify Error !! \r\n");
			}
			else
			{
				for(i=0; i<20; i++)
				{
					_DBG(" .");
					
					for (tmp=0; tmp<0x2FF; tmp++);
				}
				
				_DBG("\r\n Verify Done !!! \r\n");
			}
			
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
		/* DFMC Message */
		DFMC_WriteEraseRun();
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

