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
void encode_AES128(uint32_t *key, uint32_t *in, uint32_t *out);
void decode_AES128(uint32_t *key, uint32_t *in, uint32_t *out);
void AES128_Configure(void);;
void AES128_PollingRun(void);
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
"\t AES128 Demo...  \n\r"
"\t 0. HW Test \n\r"
"\t 1. SW Test \n\r"
"************************************************\n\r";


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
 * @brief		AEC128_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void AES128_Configure(void)
{
	//AES128
	HAL_AES128_Init();
}

/**********************************************************************
 * @brief		AES128_PollingRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void AES128_PollingRun(void)
{
	uint8_t		cmdn_cnt=0, ch_rtn, mod_check[3] = {0x30, 0x31, 0x32};
	uint32_t 	key[4], in[4], out[4];
		
	if (cmdn_cnt == 0)
		{
			_DBG(cmdm);
			cmdn_cnt=1;
		}
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		if (ch_rtn == mod_check[0])
		{
			key[0] = 0x09cf4f3c;
			key[1] = 0xabf71588;
			key[2] = 0x28aed2a6;
			key[3] = 0x2b7e1516;
			
			/* Hard Ware */
			// set Input Data
			in[0] = 0xe0370734;
			in[1] = 0x313198a2;
			in[2] = 0x885a308d;
			in[3] = 0x3243f6a8;
			
			_DBG_("\n\r[HW]");
			_DBG_("\r\n[Input Data]");
			_DBH32(in[3]);	_DBG(" ");
			_DBH32(in[2]);	_DBG(" ");
			_DBH32(in[1]);	_DBG(" ");
			_DBH32(in[0]);	_DBG("\r\n");

			/* CIPHER MODE */
			HAL_AES128_ModSel(eAES128_CipherMode);
			HAL_AES128_OrderSel(eAES128_OrderedByLSB_WORD, eAES128_OrderedByLSB_WORD);
			
			HAL_AES128_SetAESKeyIn(key);
					
			//AES128 HW Process
			HAL_AES128_SetAESTextIn(in);
			
			while(!(HAL_AES128_GetStatus() & AES128_STAT_CIPDONE)){}
			HAL_AES128_ClearStatus(AES128_STAT_CIPDONE);

			HAL_AES128_GetAESTextOut(out);
			
			_DBG_("\r\n[Encoded Data]");
			_DBH32(out[3]);	_DBG(" ");
			_DBH32(out[2]);	_DBG(" ");
			_DBH32(out[1]);	_DBG(" ");
			_DBH32(out[0]);	_DBG("\r\n");
			
			/* INV CIPHER MODE */
			HAL_AES128_ModSel(eAES128_InvCipherMode);
			HAL_AES128_OrderSel(eAES128_OrderedByLSB_WORD, eAES128_OrderedByLSB_WORD);
			
			//AES128 HW Process
			HAL_AES128_SetAESTextIn(out);
				
			while(!(HAL_AES128_GetStatus() & AES128_STAT_INVCIPDONE)){}
			HAL_AES128_ClearStatus(AES128_STAT_INVCIPDONE);
				
			HAL_AES128_GetAESTextOut(in);
			
			_DBG_("\r\n[Decoded Data]");
			_DBH32(in[3]);	_DBG(" ");
			_DBH32(in[2]);	_DBG(" ");
			_DBH32(in[1]);	_DBG(" ");
			_DBH32(in[0]);	_DBG("\r\n");
			
			cmdn_cnt =0;
		}
		
		if (ch_rtn == mod_check[1])
		{
			key[0] = 0x2b7e1516;
			key[1] = 0x28aed2a6;
			key[2] = 0xabf71588;
			key[3] = 0x09cf4f3c;
			
			// SW
			// set Input Data
			in[0] = 0x3243f6a8;
			in[1] = 0x885a308d;
			in[2] = 0x313198a2;
			in[3] = 0xe0370734;

			_DBG_("\r\n[SW]");
			_DBG_("\r\n[Input Data]");
			_DBH32(in[0]);	_DBG(" ");
			_DBH32(in[1]);	_DBG(" ");
			_DBH32(in[2]);	_DBG(" ");
			_DBH32(in[3]);	_DBG("\r\n");	
			
			//Encode
			encode_AES128(key, in, out);
			
			_DBG_("\r\n[Encoded Data]");
			_DBH32(out[0]);	_DBG(" ");
			_DBH32(out[1]);	_DBG(" ");
			_DBH32(out[2]);	_DBG(" ");
			_DBH32(out[3]);	_DBG("\r\n");	

			cmdn_cnt =0;
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
	
	/*AES128 Configure*/
    AES128_Configure(); 
				
  while(1){
		/* AES128 Message */
		AES128_PollingRun();
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
	debug_frmwrk_init();
	
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

