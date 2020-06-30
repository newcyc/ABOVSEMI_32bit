/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : console.h
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
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
#include <stdarg.h>
#include <stdint.h>

#ifndef __CONSOLE_H__
#define __CONSOLE_H__


extern int					console_port;


//=================================================================================
// function declaration
//=================================================================================
int csetport (int port_no);
void cinit (void); 
void cputc(char ch); 
void cputs(uint8_t *str); 

int	cgetchar(void); 

void cprintf(const char *fmt, ...); 
void vaprintf(const char *fmt, va_list arg_ptr);
int conv_val(char *dest, int dest_len, int val, int base, int width, int zero_padding, int left_align, int capital);



#endif 
