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
void SRAM_WriteEraseRun(void);
void mainloop(void);
int main (void);
//void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"**************************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t SRAM TEST  \n\r"
"\t 1. SRAM Erase[32KB, 0x2000_2000 ~ 0x2001_0000]  \n\r"
"\t 2. SRAM Write [32KB, 0x2000_2000 ~ 0x2001_0000] \n\r"
"\t 3. SRAM Verify  [32KB, 0x2000_2000 ~ 0x2001_0000] \n\r"
"**************************************************************\n\r";
const uint8_t cmdm_s_w[] =
"A34M41x> ";


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
 * @brief		SRAM_WriteEraseRun
 * @param[in]	None
 * @return	None
 **********************************************************************/
void SRAM_WriteEraseRun(void)
{
	uint8_t			ch_rtn, cmdn_cnt=0;
	uint8_t			v_flag=0;
	uint8_t			mod_check[5] = {0x30, 0x31, 0x32, 0x33, 0x34};
	
	int				i, tmp;
	unsigned long	Buf_tmp; 
		
	__enable_irq();
	
	
	while(1)
	{
		if (cmdn_cnt==0)
		{
			_DBG(cmdm_s_w);
			cmdn_cnt=1;
		}
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
			
		if (ch_rtn == mod_check[1])		// Flash Erase 32KB
		{
			for (i=0x2000; i<0x10000; i=i+4) {
				MIO32(0x20000000+i) = 0xFFFFFFFF;
			}
			
			for(i=0; i<10; i++)
			{
				_DBG(" .");
				
				for (tmp=0; tmp<0x2FFFF; tmp++);
			}
			
			_DBG("\n\r Erase Done !!\r\n");
			
			cmdn_cnt=0;
		}
		else if (ch_rtn == mod_check[2])		// Flash Write 
		{
			for (i=0x2000; i<0x10000; i=i+4) {
				MIO32(0x20000000+i) = 0x55AA55AA;
			}
			
			for(i=0; i<10; i++)
			{
				_DBG(" .");
				
				for (tmp=0; tmp<0x2FFFF; tmp++);
			}
			
			_DBG("\n\r Write Done !!\r\n");
			
			cmdn_cnt=0;
		}
		else if (ch_rtn == mod_check[3])		// Flash Verify
		{
			for (i=0x2000; i<0x10000; i=i+4) {
				Buf_tmp = MIO32(0x20000000+i);
				
				if (Buf_tmp != (0x55AA55AA))
				{
					_DBG("Verify Fail !!\r\n");
					v_flag=1;
					break;
				}
			}
			
			if(v_flag==0)
			{
				for(i=0; i<10; i++)
				{
					_DBG(" .");
					
					for (tmp=0; tmp<0x2FFFF; tmp++);
				}
				
				_DBG("\n\r Verify Done !!\r\n");
			}
			
			v_flag=0;
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

	/* Write*/
	SRAM_WriteEraseRun();
	
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

