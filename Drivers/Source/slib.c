/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : slib.c
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
//#include "driver_header.h"
#include "console.h"
#include "slib.h"



//=========================================================================
//	global variables
//=========================================================================
char	InData[80];
int		InFlag;
int		InCount; 





/**
************************************************************************************************************
* @ Name : init_slib
*
*
*
************************************************************************************************************
*/
void init_slib(void)
{
	InData[0] = 0; 
	InFlag = 0; 
	InCount = 0; 
}



/**
************************************************************************************************************
* @ Name : getstring
*
*
*
************************************************************************************************************
*/
int getstring(void)
{
	int		ch; 

	ch = cgetchar(); 

	if (ch > 0)
	{
		if (InCount < 80)
		{
			if (InCount == 0 && ch < 0x20)
			{
				InData[0] = 0; 
				return ch; 
			}

			cputc(ch); 
			if (ch == ASCII_BACKSPACE)
			{
				InCount--; 
				return ch; 
			}

			if (ch == ASCII_CARRIAGE_RETURN)
			{
				InData[InCount] = 0; 
				InFlag = 1; 
				return ch; 
			}

			InData[InCount++] = ch; 

		}
	}

	return 0; 
}


/**
************************************************************************************************************
* @ Name : get_number
*
* @ Parameter
*		- str : character string   ex) "124", "0x23f"
*		- p_nextPos : the address for the next start address
*
* @ Return
*		- number 
*
************************************************************************************************************
*/
uint32_t get_number (char *str, char* *p_nextPos)
{
	int				fHex = 0; 
	char			ch, temp; 
	unsigned long	val; 


	//----------------------------------------------------------------
	// clear initial blank characters
	//----------------------------------------------------------------
	while (1)
	{
		ch = *str; 

		if (ch == 0x20)
			str++;
		else 
			break; 

	}
	

	//----------------------------------------------------------------
	// end of string ? 
	//----------------------------------------------------------------
	if ((ch = *str) == 0)
	{
		*p_nextPos = str; 
		return (0); 
	}


	//----------------------------------------------------------------
	// Hex or Decimal ? 
	//----------------------------------------------------------------
	if (*str == '0' && *(str+1) == 'x')
	{
		fHex = 1; 
		str += 2; 
	}


	//----------------------------------------------------------------
	// get value
	//----------------------------------------------------------------
	val = 0; 
	
	while (1)
	{
		ch = *str; 

		if (ch == 0 || ch == 0x20) break; 
			
		if (ch >= '0' && ch <= '9') temp = ch & 0x0F; 
		else if (fHex && (ch >= 'A' && ch <= 'F')) temp = ch - 'A' + 10; 
		else if (fHex && (ch >= 'a' && ch <= 'f')) temp = ch - 'a' + 10; 
		else ch = 0; 

		if (fHex) val = (val << 4) + temp; 
		else val = (val * 10) + temp; 

		str++; 

	}


	//----------------------------------------------------------------
	// value & next position 
	//----------------------------------------------------------------
	*p_nextPos = str; 

	return (val); 

}




