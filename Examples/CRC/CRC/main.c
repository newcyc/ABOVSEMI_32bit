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
#define MAX_TEST_STRING				(4)

#define DMA_CRC

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void CRC_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void CRC_Configure(void);
void CRC_CalculationRun(void);
uint32_t reverse_data(unsigned long crc, int bitnum);
uint32_t get_newcrcinit(uint32_t init);
void generate_crc_table(void);
uint32_t crc_usetable(const uint8_t *pData, uint32_t len);
uint32_t crc_bitbybit(const uint8_t *pData, uint32_t len);
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
"\t - CRC DEMO \n\r"
"\t 0. CRC32 \n\r"
"\t 1. CRC16 \n\r"
"\t 2. CRC8 \n\r"
"\t 3. CRC7 \n\r"
"************************************************\n\r";

const uint8_t test_string[MAX_TEST_STRING] = {
	'0', '1', '2', '3'
};

static CRC_CFG_Type CRC_ConfigStruct;
static	uint8_t ch_rtn;
static	uint32_t i;
static	uint32_t cmdn_cnt;
static	uint32_t crc_result;

static Bool reverse_input;
static Bool reverse_out;
static uint32_t crc_mask;
static uint32_t crc_init;
static uint32_t crc_order;
static uint32_t crc_xor;
static uint32_t crc_highbit;
static uint32_t crc_polynominal;
static uint32_t crc_table[256];
DMA_CFG_Type	DMA_ConfigStruct;


/**********************************************************************
 * @brief		CRC_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void CRC_IRQHandler_IT(void)
{
	uint32_t tempreg;
	
	tempreg = HAL_CRC_GetStatus();
	
	if (tempreg)
	{
		HAL_CRC_ClearPending(tempreg);
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
 * @brief		reverse_data
 * @param[in]	None
 * @return 		None
 **********************************************************************/
uint32_t reverse_data(unsigned long crc, int bitnum)
{
	uint32_t i;
	uint32_t j;
	uint32_t rev_data;

	j = 1;
	rev_data = 0;
	for (i = 0x1UL << (bitnum - 1); i != 0; i >>= 1) {
		if (crc & i) {
			rev_data |= j;
		}
		j <<= 1;
	}

	return (rev_data);
}

/**********************************************************************
 * @brief		get_newcrcinit
 * @param[in]	None
 * @return 		None
 **********************************************************************/
uint32_t get_newcrcinit(uint32_t init)
{
	uint32_t i;
	uint32_t bit;
	uint32_t value;

	value = init;
	for (i = 0; i < crc_order; i++) {
		bit = value & 1;
		if (bit) {
			value ^= crc_polynominal;
		}

		value >>= 1;

		if (bit) {
			value |= crc_highbit;
		}
	}

	return value;
}

/**********************************************************************
 * @brief		generate_crc_table
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void generate_crc_table(void)
{
	uint32_t i;
	uint32_t j;
	uint32_t bit;
	uint32_t value;

	for (i = 0; i < 256; i++) {
		value = i;
		if (reverse_input == TRUE) {
			value = reverse_data(value, 8);
		}
		value <<= (crc_order - 8);

		for (j = 0; j < 8; j++) {
			bit = value & crc_highbit;
			value <<= 1;
			if (bit) {
				value ^= crc_polynominal;
			}
		}			

		if (reverse_input == TRUE) {
			value = reverse_data(value, crc_order);
		}
		value &= crc_mask;
		crc_table[i] = value;
	}
}

/**********************************************************************
 * @brief		generate_crc_table
 * @param[in]	None
 * @return 		None
 **********************************************************************/
uint32_t crc_usetable(const uint8_t *pData, uint32_t len)
{
	// suited for crc_polynominal orders between 8, 16, 32
	uint32_t value;

	// direct algorithm
	value = get_newcrcinit(crc_init);
	if (reverse_input == TRUE) {
		value = reverse_data(value, crc_order);

		while (len--) {
			value = ((value >> 8) | (*pData++ << (crc_order - 8))) ^ crc_table[value & 0xFF];
		}

		while (++len < (crc_order / 8)) {
			value = (value >> 8) ^ crc_table[value & 0xFF];
		}
	} else {
		while (len--) { 
			value = ((value << 8) | *pData++) ^ crc_table[(value >> (crc_order - 8)) & 0xFF];
		}

		while (++len < (crc_order / 8)) {
			value = (value << 8) ^ crc_table[ (value >> (crc_order - 8))  & 0xFF];
		}
	}

	if (reverse_out ^ reverse_input) {
		value = reverse_data(value, crc_order);
	}

	value ^= crc_xor;
	value &= crc_mask;

	return(value);
}


/**********************************************************************
 * @brief		crc_bitbybit
 * @param[in]	None
 * @return 		None
 **********************************************************************/
uint32_t crc_bitbybit(const uint8_t *pData, uint32_t len)
{
	// suited for 7 crc order
	uint32_t i;
	uint32_t j;
	uint32_t bit;
	uint32_t data;
	uint32_t value;

	// direct algorithm
	value = get_newcrcinit(crc_init);
	for (i = 0; i < len; i++) {
		data = (uint32_t)(*pData++);
		if (reverse_input == TRUE) {
			data = reverse_data(data, 8);
		}

		for (j = 0x80; j != 0; j >>= 1) {
			bit = value & crc_highbit;
			value <<= 1;
			if (data & j) {
				value |= 1;
			}
			if (bit) {
				value ^= crc_polynominal;
			}
		}
	}

	for (i = 0; i < crc_order; i++) {
		bit = value & crc_highbit;
		value <<= 1;
		if (bit) {
			value ^= crc_polynominal;
		}
	}

	if (reverse_out) {
		value = reverse_data(value, crc_order);
	}

	value ^= crc_xor;
	value &= crc_mask;

	return value;
}

/**********************************************************************
 * @brief		CRC_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void CRC_Configure(void)
{	

	CRC_ConfigStruct.Out_inv  = ENABLE;
	CRC_ConfigStruct.Out_rev  = ENABLE;
	CRC_ConfigStruct.In_rev = ENABLE;
	CRC_ConfigStruct.DMAD_Int = ENABLE;
	
	if (CRC_ConfigStruct.Out_inv == ENABLE) {
		crc_xor = 0xFFFFFFFFUL;
	} else {
		crc_xor = 0;
	}

	if (CRC_ConfigStruct.Out_rev == ENABLE) {
		reverse_out = TRUE;
	} else {
		reverse_out = FALSE;
	}

	if (CRC_ConfigStruct.In_rev == ENABLE) {
		reverse_input = TRUE;
	} else {
		reverse_input = FALSE;
	}
}

/**********************************************************************
 * @brief		DMA_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_Configure(void)
{	
		HAL_DMA_Init();
		
		DMA_ConfigStruct.transcnt = 1;
		DMA_ConfigStruct.perisel = CRC_TX;
		DMA_ConfigStruct.bussize = DMA_CR_WORD_TRANS;
		DMA_ConfigStruct.dirsel = DMA_CR_DIR_MEM_TO_PERI;
		
		HAL_DMA_Cmd(DMA0, &DMA_ConfigStruct);

		HAL_DMA_SetPAR(DMA0, 0x41002008);
		
		HAL_DMA_SetMAR(DMA0, 0x20001000);
	
		CRC_ConfigStruct.DMAD_Int = ENABLE;
		
		NVIC_SetPriority(CRC_IRQn, 7);
		NVIC_EnableIRQ(CRC_IRQn);
}

/**********************************************************************
 * @brief		DMA_Start
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void DMA_Start(void)
{	
		HAL_DMA_Start(DMA0);
}


/**********************************************************************
 * @brief		CRC_CalculationRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void CRC_CalculationRun(void)
{

		if (cmdn_cnt == 0) {
			_DBG(cmdm);
			cmdn_cnt = 1;
		}

		ch_rtn = UARTGetChar(UART0);

		_DBC(ch_rtn);
		_DBG("\r\n");

		switch (ch_rtn) {
			case '0' :
				CRC_ConfigStruct.Poly = CRC_POLY_CRC32;
				crc_mask = 0xFFFFFFFFUL;
				crc_init = 0xFFFFFFFFUL;
				crc_order = 32;
				crc_highbit = 0x80000000UL;
				crc_polynominal = 0x04C11DB7;
				break;

			case '1' :
				CRC_ConfigStruct.Poly = CRC_POLY_CRC16;
				crc_mask = 0x0000FFFFUL;
				crc_init = 0;
				crc_order = 16;
				crc_highbit = 0x00008000UL;
				crc_polynominal = 0x00008005UL;
				break;

			case '2' :
				CRC_ConfigStruct.Poly = CRC_POLY_CRC8;
				crc_mask = 0x000000FFUL;
				crc_init = 0;
				crc_order = 8;
				crc_highbit = 0x00000080UL;
				crc_polynominal = 0x00000007UL;
				break;

			case '3' :
				CRC_ConfigStruct.Poly = CRC_POLY_CRC7;
				crc_mask = 0x0000007FUL;
				crc_init = 0;
				crc_order = 7;
				crc_highbit = 0x00000040UL;
				crc_polynominal = 0x00000009UL;
				break;

			default :
			break;
		}

		// S/W
		if (CRC_ConfigStruct.Poly == CRC_POLY_CRC7) {
			crc_result = crc_bitbybit(test_string, MAX_TEST_STRING);
		} else {
			generate_crc_table();
			crc_result = crc_usetable(test_string, MAX_TEST_STRING);
		}
		_DBG("[SW] CRCDATA = ");
		_DBH32(crc_result);
		_DBG("\r\n");

		// HW
		HAL_CRC_Init(&CRC_ConfigStruct);

		HAL_CRC_SetInitVal(crc_init);
		HAL_CRC_ApplyInitVal(ENABLE);

		NVIC_EnableIRQ(CRC_IRQn);
		NVIC_SetPriority(CRC_IRQn, 7);
		
		for(i = 0; i < MAX_TEST_STRING; i++) {
			HAL_CRC_SetInputData8(test_string[i]);
		}

		crc_result = HAL_CRC_GetOutputData();
		_DBG("[HW] CRCDATA-8bit = ");
		_DBH32(crc_result);
		_DBG("\r\n");
		
		HAL_CRC_ApplyInitVal(ENABLE);
		
		HAL_CRC_SetInputData32(0x33323130);
		
		crc_result = HAL_CRC_GetOutputData();
	
		_DBG("[HW] CRCDATA-32bit = ");
		_DBH32(crc_result);
		_DBG("\r\n");

		cmdn_cnt=0;

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
	
	/*CRC Configure*/
	CRC_Configure();
	
#ifdef DMA_CRC
	/*DMA Configure*/
	DMA_Configure();
	
	/* DMA Test Start */
	DMA_Start();
#endif
	
	/* Enable IRQ Interrupts */
	__enable_irq();
	
   /* Infinite loop */
  while(1)
	{
		/* CRC Message */
		CRC_CalculationRun();
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

	/* Configure the system clock to 48 MHz */
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


