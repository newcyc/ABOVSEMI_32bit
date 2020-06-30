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
void AES128_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void AES128_Configure(void);
void AES128_InterruptRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);	
void encode_AES128(uint32_t *key, uint32_t *in, uint32_t *out);
void decode_AES128(uint32_t *key, uint32_t *in, uint32_t *out);

/* Private variables ---------------------------------------------------------*/	
const uint8_t cmdm[] =
"A34M41x> ";
const uint8_t menu[] =
"************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\t AES128 Interrupt Demo...  \n\r"
"\t 0. AES128 Interrupt Test \n\r"
"\t 1. AES128 Reference Example by Software \n\r"
"************************************************\n\r";

uint8_t flag_cypherMode = 0 ;
uint8_t flag_invCypherMode = 0;

/**********************************************************************
 * @brief		AES128_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void AES128_IRQHandler_IT(){
	uint32_t stat;
	stat = HAL_AES128_GetStatus();
	if(stat & (1<<16)){
		//Cypher Mode Done
		HAL_AES128_ClearStatus(1<<0);
		flag_cypherMode = 1;
	}
	if(stat & (1<<17)){
		//Inverse Cypher Mode Done
		HAL_AES128_ClearStatus(1<<1);
		flag_invCypherMode = 1;
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
 * @brief		AES128_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void AES128_Configure(void)
{
	//AES128
	HAL_AES128_Init();
}

/**********************************************************************
 * @brief		AES128_InterruptRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void AES128_InterruptRun(void)
{
		uint8_t		cmdn_cnt=0, ch_rtn, mod_check[3] = {0x30, 0x31, 0x32};
	    uint32_t 	key[4], in[4], out[4], rd_in[4]; 
	
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
			
			// Input : Default Word Row(LSB), 					Output : Default Word Row(LSB)
			HAL_AES128_OrderSel(eAES128_OrderedByLSB_WORD, eAES128_OrderedByLSB_WORD);
			
			HAL_AES128_ConfigInterrupt(AES128_CTRL_CIPHERIE, ENABLE);
			HAL_AES128_ConfigInterrupt(AES128_CTRL_INVCIPHERIE, ENABLE);
			
			NVIC_EnableIRQ(AES128_IRQn);
			NVIC_SetPriority(AES128_IRQn, 3);
			
			HAL_AES128_SetAESKeyIn(key);
			
			//AES128 HW Process
			HAL_AES128_SetAESTextIn(in);

			while(!flag_cypherMode){}
			flag_cypherMode=0;
				
			HAL_AES128_ClearStatus(AES128_STAT_CIPDONE);
				
			HAL_AES128_GetAESTextOut(out);
			HAL_AES128_GetAESTextIn(rd_in);
							
			_DBG_("\r\n[AES128 - Input]");
			_DBH32(rd_in[3]);	_DBG(" ");
			_DBH32(rd_in[2]);	_DBG(" ");
			_DBH32(rd_in[1]);	_DBG(" ");
			_DBH32(rd_in[0]);	_DBG("\r\n");
				
			_DBG_("\r\n[LSB Encoded Data for compare]");
			_DBH32(0x3925841D);	_DBG(" ");
			_DBH32(0x02DC09FB);	_DBG(" ");
			_DBH32(0xDC118597);	_DBG(" ");
			_DBH32(0x196A0B32);	_DBG("\r\n");
			
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
			
			while(!(flag_invCypherMode)){}
			flag_invCypherMode=0;
				
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
  	
   /* Infinite loop */
  while(1)
	{
		/* AES128 Message */
		AES128_InterruptRun();
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
