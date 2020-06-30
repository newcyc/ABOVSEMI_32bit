/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : console.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : August, 2017
*
* @ Description 
*   ABOV Semiconductor is supplying this software for use with A33G52x
*   processor. This software contains the confidential and proprietary information
*   of ABOV Semiconductor Co., Ltd ("Confidential Information").
*
*
**************************************************************************************
* DISCLAIMER 
*
* 	THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS  
* 	WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE  
* 	TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* 	DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING  
* 	FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE  
* 	CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.  
*
**************************************************************************************
*/


#include <stdio.h>
#include <string.h>
#include "console.h"
#include "a33g52x_uart.h"


int				console_port;
char				g_cstr[80]; 


/**
**************************************************************************************************
* @ Name : csetport
*
* @ Parameter
*		- port_no : 0~2
*
* @ return 
*		0 = success
*		1 = error 
*
**************************************************************************************************
*/
int csetport (int port_no)
{
	if (port_no >= N_UART) return (1); 
	
	console_port = port_no; 

	return (0); 
}



/**
**************************************************************************************************
* @ Name : cinit
*
* @ Description
*		This function initializes UART port for monitoring.
*
*
**************************************************************************************************
*/
void cinit (void)
{
	_UART_Init (console_port);
}



/**
**************************************************************************************************
* @ Name : cputc
*
* @ Parameter
*		- ch : character to write to the UART port 
*
*
**************************************************************************************************
*/
void cputc(char ch)
{
	uint8_t	str[2];

	str[0] = (uint8_t) ch;
	str[1] = 0; 

	UART_WriteBuffer(console_port, str, 1); 

}



/**
**************************************************************************************************
* @ Name : cputs
*
* @ Parameter
*		- str : string to write to the UART port 
*
*
**************************************************************************************************
*/
void cputs(uint8_t *str)
{
	uint32_t		i; 

	for (i=0; ; i++)
	{
		if (*(str+i) == 0) break; 
	}

	UART_WriteBuffer(console_port, str, i); 
}


/**
**************************************************************************************************
* @ Name : cgetchar 
*
* @ return 
*		8-bit data except -1		= success
*		-1						= error
*
**************************************************************************************************
*/
int cgetchar (void)
{
	return UART_GetChar(console_port);
}



/*
****************************************************************************************
* cprintf function with variable arguments
*
*
* <Descriptions>
*    @ va_list : To start processing the variable argument list you declare a pointer of type "va_list"
*
*    @ va_start : The va_start macro is used to initialize arg_ptr so that it points to the first argument in the list:
*
*                       va_start(arg_ptr, fmt); 
*
*                       The second argument to the macro is the name of the fixed parameter that precedes the ellipsis 
*                       in the parameter, and this is used by the macro to determine where the first variable argument is.
*
*    @ va_end : When you are finished retrieving argument values, you reset arg_ptr with the statement:
*  
*                      va_end(arg_ptr);
*
*                      The va_end macro resets the pointer of type "va_list" that you pass as the argument to it to null.
*
*    @ va_arg : The "va_arg" macro returns the value of the argument at the location specified by "arg_ptr" and increments 
*                     "arg_ptr" to point to the next argument value.
*
*                      The second argument to the va_arg macro is the argument type
*=====================================================================
*
* C A U T I O N !!!
*
*     Because these functions are for test only, not for commercial, there is no algorithm to check buffer overflow, 
*     so, be careful not to write more than eighty characters in the buffer (g_cstr[80]). 
*
*******************************************************************************************
*/
void cprintf(const char *fmt, ...)
{
	va_list	arg_ptr;

	va_start(arg_ptr,fmt);
	vaprintf(fmt,arg_ptr);
	va_end(arg_ptr);

}


void vaprintf(const char *fmt, va_list arg_ptr)
{
	char	buf[12];
	int		fmt_len; 
	int		i, j, str_ptr;  
	char	ch; 
	int		percent_detected=0, zero_padding=0, width=0, left_align=0, capital=0; 
	int		item_int;
	char	item_ch; 
//	char	*item_str; 
	int		count; 

	//cout << endl << "vaprintf start...";

	str_ptr = 0; 
	fmt_len = strlen(fmt);

	for (i=0; i<fmt_len; i++)
	{
		ch = *fmt++; 

		if (!percent_detected)
		{
			if (ch == '%')
			{
				percent_detected = 1; 
			}
			else
			{
				g_cstr[str_ptr++] = ch; 
			}

		}
		else 
		{
			if (width == 0 && ch == '-') 
			{
				left_align = 1; 
				continue;
			}

			if (width == 0 && ch == '0')
			{
				zero_padding = 1;
				continue;
			}

			if (ch >= '0' && ch <= '9') 
			{
				width = width * 10 + (ch - '0');
				continue;
			}

			switch (ch)
			{
			case 'd':
				item_int = va_arg(arg_ptr, int); 
				count = conv_val(buf, 12, item_int, 10, width, zero_padding, left_align, capital); 

				for (j=0; j<count; j++)
				{
					g_cstr[str_ptr++] = buf[j]; 
				}

				break; 

			case 'X':
				capital = 1; 

			case 'x':
				item_int = va_arg(arg_ptr, int); 
				count = conv_val(buf, 12, item_int, 16, width, zero_padding, left_align, capital); 

				for (j=0; j<count; j++)
				{
					g_cstr[str_ptr++] = buf[j]; 
				}

				break;

			case 'c':
				item_ch = va_arg(arg_ptr, int); 
				g_cstr[str_ptr++] = item_ch; 

				break; 

			case 's':
//				item_str = va_arg(arg_ptr, char *); 
				break; 
			}

			percent_detected = 0; 
			zero_padding = 0; 
			width = 0; 
			left_align = 0; 
			capital = 0; 

		}
	}

	g_cstr[str_ptr] = 0; 

	cputs((uint8_t *)g_cstr); 
	//cout << endl << "str=" << str << endl;
	//cout << endl << "vaprintf end..."; 
}


int conv_val(char *dest, int dest_len, int val, int base, int width, int zero_padding, int left_align, int capital)
{
	int		remnant;
	unsigned long	remnant_hex, val_hex, base_hex; 
	
	int		src_index, dest_index; 
	char	ch; 
	int		start_index; 

	dest_index = dest_len - 1; 


	if (base == 16)
	{
		val_hex = (unsigned long) val; 
		base_hex = (unsigned long) base; 

		do {

			remnant_hex = val_hex % base_hex; 
			val_hex /= base_hex; 
			
			if (remnant_hex < 10) ch = (char) (remnant_hex + '0');
			else ch = (char) (capital? remnant_hex - 10 + 'A' : remnant_hex - 10 + 'a'); 

			*(dest + dest_index--) = ch; 
			
		} while (val_hex); 
			
	}
	else
	{
		do { 
			remnant = val % base; 
			val /= base; 
	
			ch = (char) (remnant + '0'); 

			*(dest + dest_index--) = ch; 
		} while (val); 
	}
	

	if (left_align)
	{
		src_index = ++dest_index;
		dest_index = 0; 

		//---------------------------------------------------------------------------
		// move the value to the leftmost position 
		//---------------------------------------------------------------------------
		for (; src_index<dest_len; src_index++)
		{
			*(dest + dest_index++) = *(dest + src_index);
		}

		//---------------------------------------------------------------------------
		// pad the blank characters
		//---------------------------------------------------------------------------
		for (; dest_index<width; dest_index++)
		{
			*(dest + dest_index) = ' ';
		}
	}
	else if (zero_padding)
	{
		start_index = dest_len - width; 

		//---------------------------------------------------------------------------
		// pad zeros
		//---------------------------------------------------------------------------
		for (; dest_index >= start_index; dest_index--)
		{
			*(dest + dest_index) = '0'; 
		}

		src_index = ++dest_index; 
		dest_index = 0; 

		//---------------------------------------------------------------------------
		// move the position 
		//---------------------------------------------------------------------------
		for (; src_index<dest_len; src_index++)
		{
			*(dest + dest_index++) = *(dest + src_index); 
		}
	}
	else 
	{
		src_index = ++dest_index; 
		dest_index = 0; 

		//---------------------------------------------------------------------------
		// move the position 
		//---------------------------------------------------------------------------
		for (; src_index<dest_len; src_index++)
		{
			*(dest + dest_index++) = *(dest + src_index); 
		}
	}


	return (dest_index); 

}


