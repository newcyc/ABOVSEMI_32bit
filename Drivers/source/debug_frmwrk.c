/***************************************************************************//**
* @file     debug_frmwrk.c
* @brief    Contains all functions support for debugging through UART
* @author   AE Team, ABOV Semiconductor Co., Ltd.
* @version  V0.0.1
* @date     30. Jul. 2018
*
* Copyright(C) 2018, ABOV Semiconductor
* All rights reserved.
*
*
********************************************************************************
* DISCLAIMER 
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

/*******************************************************************************
* Included File
*******************************************************************************/
#include <stdarg.h>
#include <stdio.h>

#include "A31G22x_pcu.h"
#include "A31G22x_uartn.h"
#include "A31G22x_usart1n.h"
#include "debug_frmwrk.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/


/*******************************************************************************
* Private Typedef
*******************************************************************************/
/**
 * @brief  Debug Port configuration structure definition
 */
typedef struct {
	void *pPeriAddress;

	PORT_Type *pTxPin_Group;
	uint32_t TxPin_Number;
	PCU_ALT_FUNCTION_Type TxPin_AltFunc;

	PORT_Type *pRxPin_Group;
	uint32_t RxPin_Number;
	PCU_ALT_FUNCTION_Type RxPin_AltFunc;
} DEBUG_INTERFACE_Type;


/*******************************************************************************
* Private Variable
*******************************************************************************/
static const DEBUG_INTERFACE_Type DEBUG_INTERFACE_CFG[DEBUG_INTERFACE_PERI_MAX] = {
	{(void *)  UART0, PB,  4, PCU_ALT_FUNCTION_1, PB,  5, PCU_ALT_FUNCTION_1},
	{(void *)  UART1, PB,  6, PCU_ALT_FUNCTION_1, PB,  7, PCU_ALT_FUNCTION_1},
	{(void *)USART10, PB,  0, PCU_ALT_FUNCTION_1, PB,  1, PCU_ALT_FUNCTION_1},
	{(void *)USART11, PD,  2, PCU_ALT_FUNCTION_1, PD,  3, PCU_ALT_FUNCTION_1},
	{(void *)USART12, PE, 12, PCU_ALT_FUNCTION_1, PE, 13, PCU_ALT_FUNCTION_1},
	{(void *)USART13, PE,  8, PCU_ALT_FUNCTION_1, PE,  9, PCU_ALT_FUNCTION_1}
};

static void *pDebugAddress;
static DEBUG_INTERFACE_PERI_Type InterfacePeri;


/*******************************************************************************
* Private Variable
*******************************************************************************/
const static uint32_t Digit_Dec[DEBUG_DATA_MAX] = { 3, 5, 10};
const static uint32_t Digit_Hex[DEBUG_DATA_MAX] = { 2, 4, 8};


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/
static void DEBUG_InitPeripheral(void *pAddress);


/*******************************************************************************
* Public Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Print function that supports format as same as printf()
              function of <stdio.h> library
* @param      pFormat : pointer of formated string to be print
* @return     None
*******************************************************************************/
void cprintf(const    char *pFormat, ...)
{
	uint8_t buffer[512 + 1];
	va_list vArgs;
	va_start(vArgs, pFormat);
	vsprintf((char *)buffer, (char const *)pFormat, vArgs);
	va_end(vArgs);

	DEBUG_PutString(buffer);
}

/***************************************************************************//**
* @brief      Initialize debug framework
* @param      Peri : Debug interface peripheral source selected, should be
*              - DEBUG_INTERFACE_PERI_UART0 : UART0
*              - DEBUG_INTERFACE_PERI_UART1 : UART1
*              - DEBUG_INTERFACE_PERI_USART10 : USART10
*              - DEBUG_INTERFACE_PERI_USART11 : USART11
*              - DEBUG_INTERFACE_PERI_USART12 : USART12
*              - DEBUG_INTERFACE_PERI_USART13 : USART13
* @return     None
*******************************************************************************/
void DEBUG_Init(DEBUG_INTERFACE_PERI_Type Peri)
{
	const DEBUG_INTERFACE_Type * pConfig;

	InterfacePeri = Peri;

	pConfig = &(DEBUG_INTERFACE_CFG[Peri]);

	PCU_ConfigureDirection(pConfig->pTxPin_Group, pConfig->TxPin_Number, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(pConfig->pTxPin_Group, pConfig->TxPin_Number, pConfig->TxPin_AltFunc);
	PCU_ConfigurePullupdown(pConfig->pTxPin_Group, pConfig->TxPin_Number, PCU_PUPD_PULL_UP);

	PCU_ConfigureDirection(pConfig->pRxPin_Group, pConfig->RxPin_Number, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(pConfig->pRxPin_Group, pConfig->RxPin_Number, pConfig->RxPin_AltFunc);
	PCU_ConfigurePullupdown(pConfig->pRxPin_Group, pConfig->RxPin_Number, PCU_PUPD_PULL_UP);

	pDebugAddress = pConfig->pPeriAddress;
	DEBUG_InitPeripheral(pConfig->pPeriAddress);
	
}

/***************************************************************************//**
* @brief      De-Initialize debug framework
* @param      None
* @return     None
*******************************************************************************/
void DEBUG_DeInit(void)
{
	if ((InterfacePeri == DEBUG_INTERFACE_PERI_UART0) || (InterfacePeri == DEBUG_INTERFACE_PERI_UART1)) {
		UART_DeInit((UART_Type *)pDebugAddress);
	} else {
		USART_DeInit((USART_Type *)pDebugAddress);
	}

	pDebugAddress = (void *)0;
	InterfacePeri = DEBUG_INTERFACE_PERI_MAX;
}

/***************************************************************************//**
* @brief      Puts a character to debug peripheral
* @param      Ch : Character to put
* @return     None
*******************************************************************************/
void DEBUG_PutChar(uint8_t Ch)
{
	if ((InterfacePeri == DEBUG_INTERFACE_PERI_UART0) || (InterfacePeri == DEBUG_INTERFACE_PERI_UART1)) {
		UART_Send((UART_Type *)pDebugAddress, &Ch, 1, BLOCKING);
	} else {
		USART_Send((USART_Type *)pDebugAddress, &Ch, 1);
	}
}

/***************************************************************************//**
* @brief      Put string to debug peripheral
* @param      pString : pointer of string
* @return     None
*******************************************************************************/
void DEBUG_PutString(const void *pString)
{
	uint8_t *pS = (uint8_t *)pString;

	while (*pS) {
		DEBUG_PutChar(*pS++);
	}
}

/***************************************************************************//**
* @brief      Put string and CRLF to debug peripheral
* @param      pString : pointer of string
* @return     None
*******************************************************************************/
void DEBUG_PutStringCRLF(const void *pString)
{
	DEBUG_PutString(pString);
	DEBUG_PutString("\r\n");
}

/***************************************************************************//**
* @brief      Put decimal number to debug peripheral
* @param      Number : Number
* @param      DataType : Data type
               - DEBUG_DATA_CHAR : char (8-bit)
               - DEBUG_DATA_SHORT : short (16-bit)
               - DEBUG_DATA_INT : int (32-bit)
* @return     None
*******************************************************************************/
void DEBUG_PutDec(uint32_t Number, DEBUG_DATA_Type DataType)
{
	uint32_t i;
	uint32_t temp;
	uint8_t String[11];
	uint32_t Loop;

	Loop = Digit_Dec[DataType];

	for(i = 1; i < Loop; i++) {
		temp = Number;
		Number /= 10;
		temp -= (Number * 10);
		String[Loop - i] = (uint8_t)(0x30UL + temp);
	}
	String[0] = (uint8_t)(0x30UL + Number);
	String[Loop] = 0x00;

	DEBUG_PutString(String);
}

/***************************************************************************//**
* @brief      Put hex number to debug peripheral
* @param      Number : Number
* @param      DataType : Data type
               - DEBUG_DATA_CHAR : char (8-bit)
               - DEBUG_DATA_SHORT : short (16-bit)
               - DEBUG_DATA_INT : int (32-bit)
* @return     None
*******************************************************************************/
void DEBUG_PutHex(uint32_t Number, DEBUG_DATA_Type DataType)
{
	uint32_t i;
	uint32_t Nibble;
	uint32_t Loop;

	Loop = Digit_Hex[DataType];
	for(i = Loop; i > 0; i--) {
		Nibble = (Number >> (4 * (i - 1))) & 0x0FUL;
		DEBUG_PutChar((uint8_t)((Nibble > 9) ? (0x41UL + Nibble - 10) : (0x30UL + Nibble))); // 'A' = 0x41, '0' = 0x30
	}
}

/***************************************************************************//**
* @brief      Get a character from debug peripheral
* @param      None
* @return     Character value
*******************************************************************************/
uint8_t DEBUG_GetChar(void)
{
	uint8_t Ch = 0;

	if ((InterfacePeri == DEBUG_INTERFACE_PERI_UART0) || (InterfacePeri == DEBUG_INTERFACE_PERI_UART1)) {
		UART_Receive((UART_Type *)pDebugAddress, &Ch, 1, BLOCKING);
	} else {
		USART_Receive((USART_Type *)pDebugAddress, &Ch, 1);
	}

	return Ch;
}


/*******************************************************************************
* Private Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Initialize Peripheral for debug interface
* @param      None
* @return     None
*******************************************************************************/
static void DEBUG_InitPeripheral(void *pAddress)
{
	if ((InterfacePeri == DEBUG_INTERFACE_PERI_UART0) || (InterfacePeri == DEBUG_INTERFACE_PERI_UART1)) {
		UART_CFG_Type UART_Config;

		/* Initialize UART Configuration parameter structure to default state:
		 * Baudrate = 38400bps
		 * 8 data bit
		 * None parity
		 * 1 Stop bit
		 */
		UART_GetDefaultConfig(&UART_Config);
		
		// Initialize DEBUG_UART_PORT peripheral with given to corresponding parameter
		UART_Init((UART_Type *)pAddress, &UART_Config);
	} else {
		USART_CFG_Type USART_Config;

		/* Initialize USART Configuration parameter structure to default state:
		 * Mode = UART
		 * Baudrate = 38400bps
		 * 8 data bit
		 * None parity
		 * 1 Stop bit
		 */
		USART_UART_Mode_ConfigStructInit(&USART_Config);
		
		// Initialize DEBUG_UART_PORT peripheral with given to corresponding parameter
		USART_Init((USART_Type *)pAddress, &USART_Config);
	}
}

/* --------------------------------- End Of File ------------------------------ */
