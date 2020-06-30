/**********************************************************************
* @file		A34M41x_debug_frmwrk.c
* @brief	Contains some utilities that used for debugging through UART
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _DEBUG_FRMWRK_
#define _DEBUG_FRMWRK_

/* Includes ------------------------------------------------------------------- */
#include <stdarg.h>
#include <stdio.h>
#include "A34M41x_debug_frmwrk.h"
#include "A34M41x_hal_pcu.h"

/* Debug framework */

#ifdef _DEBUG_MSG
void (*_db_msg)(UART_Type *UARTx, const void *s);
void (*_db_msg_)(UART_Type *UARTx, const void *s);
void (*_db_char)(UART_Type *UARTx, uint8_t ch);
void (*_db_dec)(UART_Type *UARTx, uint8_t decn);
void (*_db_dec_16)(UART_Type *UARTx, uint16_t decn);
void (*_db_dec_32)(UART_Type *UARTx, uint32_t decn);
void (*_db_hex)(UART_Type *UARTx, uint8_t hexn);
void (*_db_hex_16)(UART_Type *UARTx, uint16_t hexn);
void (*_db_hex_32)(UART_Type *UARTx, uint32_t hexn);
uint8_t (*_db_get_char)(UART_Type *UARTx);
uint8_t (*_db_get_ch)(UART_Type *UARTx, uint8_t *ch);

//uint8_t UARTGetChar (UART_Type *UARTx) __attribute__((section(".ARM.__at_0x00000100")));
//void UARTPuts(UART_Type *UARTx, const void *str) __attribute__((section(".ARM.__at_0x00000120")));
//void UARTPutHex32 (UART_Type *UARTx, uint32_t hexnum) __attribute__((section(".ARM.__at_0x00000140")));
//void UARTPutChar (UART_Type *UARTx, uint8_t ch) __attribute__((section(".ARM.__at_0x00000180")));



/*********************************************************************//**
 * @brief		Puts a character to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	ch		Character to put
 * @return		None
 **********************************************************************/
void UARTPutChar (UART_Type *UARTx, uint8_t ch)
{
	HAL_UART_Transmit(UARTx, &ch, 1, BLOCKING);
}


/*********************************************************************//**
 * @brief		Get a character to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @return		character value that returned
 **********************************************************************/
uint8_t UARTGetChar (UART_Type *UARTx)
{
	uint8_t tmp = 0;
	HAL_UART_Receive(UARTx, &tmp, 1, BLOCKING);
	return(tmp);
}

/*********************************************************************//**
 * @brief		Get a character to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	*ch		Character to gett
 * @return		if getting value, return '1'. if not, return '0'
 **********************************************************************/
uint8_t UARTGetCh(UART_Type *UARTx, uint8_t *ch)
{
	if (!(UARTx->LSR & UART_LSR_RDR)){
		*ch=0;
		return(0);		
	}
	else {
		*ch = HAL_UART_ReceiveByte(UARTx);
		return(1);	
	}
}


/*********************************************************************//**
 * @brief		Puts a string to UART port
 * @param[in]	UARTx 	Pointer to UART peripheral
 * @param[in]	str 	string to put
 * @return		None
 **********************************************************************/
void UARTPuts(UART_Type *UARTx, const void *str)
{
	uint8_t *s = (uint8_t *) str;

	while (*s)
	{
		UARTPutChar(UARTx, *s++);
	}
}


/*********************************************************************//**
 * @brief		Puts a string to UART port and print new line
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	str		String to put
 * @return		None
 **********************************************************************/
void UARTPuts_(UART_Type *UARTx, const void *str)
{
	UARTPuts (UARTx, str);
	UARTPuts (UARTx, "\n\r");
}


/*********************************************************************//**
 * @brief		Puts a decimal number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	decnum	Decimal number (8-bit long)
 * @return		None
 **********************************************************************/
void UARTPutDec(UART_Type *UARTx, uint8_t decnum)
{
	uint8_t c1=decnum%10;
	uint8_t c2=(decnum/10)%10;
	uint8_t c3=(decnum/100)%10;
	UARTPutChar(UARTx, '0'+c3);
	UARTPutChar(UARTx, '0'+c2);
	UARTPutChar(UARTx, '0'+c1);
}

/*********************************************************************//**
 * @brief		Puts a decimal number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	decnum	Decimal number (8-bit long)
 * @return		None
 **********************************************************************/
void UARTPutDec16(UART_Type *UARTx, uint16_t decnum)
{
	uint8_t c1=decnum%10;
	uint8_t c2=(decnum/10)%10;
	uint8_t c3=(decnum/100)%10;
	uint8_t c4=(decnum/1000)%10;
	uint8_t c5=(decnum/10000)%10;
	UARTPutChar(UARTx, '0'+c5);
	UARTPutChar(UARTx, '0'+c4);
	UARTPutChar(UARTx, '0'+c3);
	UARTPutChar(UARTx, '0'+c2);
	UARTPutChar(UARTx, '0'+c1);
}

/*********************************************************************//**
 * @brief		Puts a decimal number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	decnum	Decimal number (8-bit long)
 * @return		None
 **********************************************************************/
void UARTPutDec32(UART_Type *UARTx, uint32_t decnum)
{
	uint8_t c1=decnum%10;
	uint8_t c2=(decnum/10)%10;
	uint8_t c3=(decnum/100)%10;
	uint8_t c4=(decnum/1000)%10;
	uint8_t c5=(decnum/10000)%10;
	uint8_t c6=(decnum/100000)%10;
	uint8_t c7=(decnum/1000000)%10;
	uint8_t c8=(decnum/10000000)%10;
	uint8_t c9=(decnum/100000000)%10;
	uint8_t c10=(decnum/1000000000)%10;
	UARTPutChar(UARTx, '0'+c10);
	UARTPutChar(UARTx, '0'+c9);
	UARTPutChar(UARTx, '0'+c8);
	UARTPutChar(UARTx, '0'+c7);
	UARTPutChar(UARTx, '0'+c6);
	UARTPutChar(UARTx, '0'+c5);
	UARTPutChar(UARTx, '0'+c4);
	UARTPutChar(UARTx, '0'+c3);
	UARTPutChar(UARTx, '0'+c2);
	UARTPutChar(UARTx, '0'+c1);
}

/*********************************************************************//**
 * @brief		Puts a hex number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	hexnum	Hex number (8-bit long)
 * @return		None
 **********************************************************************/
void UARTPutHex (UART_Type *UARTx, uint8_t hexnum)
{
	uint8_t nibble, i;

//	UARTPuts(UARTx, "0x");
	i = 1;
	do {
		nibble = (hexnum >> (4*i)) & 0x0F;
		UARTPutChar(UARTx, (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
	} while (i--);
}


/*********************************************************************//**
 * @brief		Puts a hex number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	hexnum	Hex number (16-bit long)
 * @return		None
 **********************************************************************/
void UARTPutHex16 (UART_Type *UARTx, uint16_t hexnum)
{
	uint8_t nibble, i;

//	UARTPuts(UARTx, "0x");
	i = 3;
	do {
		nibble = (hexnum >> (4*i)) & 0x0F;
		UARTPutChar(UARTx, (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
	} while (i--);
}

/*********************************************************************//**
 * @brief		Puts a hex number to UART port
 * @param[in]	UARTx	Pointer to UART peripheral
 * @param[in]	hexnum	Hex number (32-bit long)
 * @return		None
 **********************************************************************/
void UARTPutHex32 (UART_Type *UARTx, uint32_t hexnum)
{
	uint8_t nibble, i;

//	UARTPuts(UARTx, "0x");
	i = 7;
	do {
		nibble = (hexnum >> (4*i)) & 0x0F;
		UARTPutChar(UARTx, (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble));
	} while (i--);
}

/*********************************************************************//**
 * @brief		print function that supports format as same as printf()
 * 				function of <stdio.h> library
 * @param[in]	format formated string to be print
 * @return		None
 **********************************************************************/
void  cprintf (const  char *format, ...)
{
    char  buffer[512 + 1];
            va_list     vArgs;
    va_start(vArgs, format);
    vsprintf((char *)buffer, (char const *)format, vArgs);
    va_end(vArgs);

    _DBG(buffer);
}

/*********************************************************************//**
 * @brief		Initialize Debug frame work through initializing UART port
 * @param[in]	None
 * @return		None
 **********************************************************************/
void debug_frmwrk_init(void)
{
	UART_CFG_Type UARTConfigStruct;

#if (USED_UART_DEBUG_PORT==0)
	/*
	 * Initialize UART0 pin connect
	 */
	HAL_GPIO_ConfigureFunction(PC, 14, PC14_MUX_RXD0);
	HAL_GPIO_ConfigOutput(PC, 14, INPUT);
	HAL_GPIO_ConfigPullup(PC, 14, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PC, 15, PC15_MUX_TXD0);
	HAL_GPIO_ConfigOutput(PC, 15, PUSH_PULL_OUTPUT);	
	HAL_GPIO_ConfigPullup(PC, 15, PULL_UP_ENABLE);	
#elif (USED_UART_DEBUG_PORT==1)
	/*
	 * Initialize UART1 pin connect
	 */
//	// FPGA TEST Only
//	HAL_GPIO_ConfigureFunction(PC, 5, PC5_MUX_RXD1);
//	HAL_GPIO_ConfigOutput(PC, 5, INPUT);
//	HAL_GPIO_ConfigPullup(PC, 5, PULL_UP_ENABLE);
//	
//	HAL_GPIO_ConfigureFunction(PC, 6, PC6_MUX_TXD1);
//	HAL_GPIO_ConfigOutput(PC, 6, PUSH_PULL_OUTPUT);	
//	HAL_GPIO_ConfigPullup(PC, 6, PULL_UP_ENABLE);	
//	//------------------------------------------

	HAL_GPIO_ConfigureFunction(PA, 10, PA10_MUX_RXD1);
	HAL_GPIO_ConfigOutput(PA, 10, INPUT);
	HAL_GPIO_ConfigPullup(PA, 10, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PA, 11, PA11_MUX_TXD1);
	HAL_GPIO_ConfigOutput(PA, 11, PUSH_PULL_OUTPUT);	
	HAL_GPIO_ConfigPullup(PA, 11, PULL_UP_ENABLE);	
#elif (USED_UART_DEBUG_PORT==2)
	/*
	 * Initialize UART2 pin connect
	 */
	HAL_GPIO_ConfigureFunction(PD, 7, PD7_MUX_RXD2);
	HAL_GPIO_ConfigOutput(PD, 7, INPUT);
	HAL_GPIO_ConfigPullup(PD, 7, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PD, 6, PD6_MUX_TXD2);
	HAL_GPIO_ConfigOutput(PD, 6, PUSH_PULL_OUTPUT);	
	HAL_GPIO_ConfigPullup(PD, 6, PULL_UP_ENABLE);	
#elif (USED_UART_DEBUG_PORT==3)
	/*
	 * Initialize UART3 pin connect
	 */
//	// FPGA TEST Only
//	HAL_GPIO_ConfigureFunction(PB, 8, PB8_MUX_RXD3);
//	HAL_GPIO_ConfigOutput(PB, 8, INPUT);
//	HAL_GPIO_ConfigPullup(PB, 8, PULL_UP_ENABLE);
//	
//	HAL_GPIO_ConfigureFunction(PB, 9, PB9_MUX_TXD3);
//	HAL_GPIO_ConfigOutput(PB, 9, PUSH_PULL_OUTPUT);	
//	HAL_GPIO_ConfigPullup(PB, 9, PULL_UP_ENABLE);	
//	//------------------------------------------
	
	HAL_GPIO_ConfigureFunction(PG, 8, PG8_MUX_RXD3);
	HAL_GPIO_ConfigOutput(PG, 8, INPUT);
	HAL_GPIO_ConfigPullup(PG, 8, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PG, 9, PG9_MUX_TXD3);
	HAL_GPIO_ConfigOutput(PG, 9, PUSH_PULL_OUTPUT);	
	HAL_GPIO_ConfigPullup(PG, 9, PULL_UP_ENABLE);	
#elif (USED_UART_DEBUG_PORT==4)
	/*
	 * Initialize UART4 pin connect
	 */
	HAL_GPIO_ConfigureFunction(PE, 14, PE14_MUX_RXD4);
	HAL_GPIO_ConfigOutput(PE, 14, INPUT);
	HAL_GPIO_ConfigPullup(PE, 14, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PE, 13, PE13_MUX_TXD4);
	HAL_GPIO_ConfigOutput(PE, 13, PUSH_PULL_OUTPUT);	
	HAL_GPIO_ConfigPullup(PE, 13, PULL_UP_ENABLE);
#elif (USED_UART_DEBUG_PORT==5)
	/*
	 * Initialize UART5 pin connect
	 */
	HAL_GPIO_ConfigureFunction(PF, 5, PF5_MUX_RXD5);
	HAL_GPIO_ConfigOutput(PF, 5, INPUT);
	HAL_GPIO_ConfigPullup(PF, 5, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PF, 4, PF4_MUX_TXD5);
	HAL_GPIO_ConfigOutput(PF, 4, PUSH_PULL_OUTPUT);	
	HAL_GPIO_ConfigPullup(PF, 4, PULL_UP_ENABLE);	
#endif


	/* Initialize UART Configuration parameter structure to default state:
	 * Baudrate = 38400bps
	 * 8 data bit
	 * 1 Stop bit
	 * None parity
	 */
	HAL_UART_ConfigStructInit(&UARTConfigStruct);
	UARTConfigStruct.Baud_rate = 38400;
	
	// Initialize DEBUG_UART_PORT peripheral with given to corresponding parameter
	HAL_UART_Init((UART_Type*)DEBUG_UART_PORT, &UARTConfigStruct);

	_db_msg	= UARTPuts;
	_db_msg_ = UARTPuts_;
	_db_char = UARTPutChar;
	_db_hex = UARTPutHex;
	_db_hex_16 = UARTPutHex16;
	_db_hex_32 = UARTPutHex32;
	_db_dec = UARTPutDec;
	_db_dec_16 = UARTPutDec16;
	_db_dec_32 = UARTPutDec32;
	_db_get_char = UARTGetChar;
	_db_get_ch = UARTGetCh;	
}
#endif   /* _DEBUG_MSG */
#endif /* _DEBUG_FRMWRK_ */
 
/* --------------------------------- End Of File ------------------------------ */
