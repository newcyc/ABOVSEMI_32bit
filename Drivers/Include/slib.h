/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : slib.h
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

#ifndef __SLIB_H__
#define __SLIB_H__

#include <stdint.h>


//=========================================================================
//	constants
//=========================================================================
#define ASCII_BACKSPACE					(0x08)
#define ASCII_LINEFEED					(0x0A)
#define ASCII_CARRIAGE_RETURN			(0x0D)





//=========================================================================
//	export variables
//=========================================================================
extern char		InData[80];
extern int		InFlag;
extern int		InCount; 



//=========================================================================
//	function declarations
//=========================================================================
void init_slib(void); 
int getstring(void); 
uint32_t get_number (char *str, char* *p_nextPos); 


#endif 

