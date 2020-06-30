/**********************************************************************
* @file		A34M418_libcfg.h
* @brief	Contains all functions support for PCU firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 Team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M418_LIBCFG_H_
#define _A34M418_LIBCFG_H_

#include "A34M41x_aa_types.h"

/******************* PERIPHERAL FW LIBRARY CONFIGURATION DEFINITIONS *****************/
/* Comment the line below to disable the specific peripheral inclusion */

/*******************************************************************************************
* Included File
*******************************************************************************************/

 /* CHECK PARAM --------------------------------------------- */
 /* Un-comment the line below to compile the library in DEBUG mode, this will expanse
 the "CHECK_PARAM" macro in the FW library code */
//#define	USE_FULL_ASSERT
	
 /* DEBUG_FRAMWORK ---------------------------------------- */
#define _DEBUG_MSG

/*******************************************************************************************
* Public Macro
*******************************************************************************************/
#ifdef  USE_FULL_ASSERT
/*******************************************************************************************
* @brief                The CHECK_PARAM macro is used for function's parameters check.
*                               It is used only if the library is compiled in DEBUG mode.
* @param	    		expr - If expr is false, it calls check_failed() function
*                       which reports the name of the source file and the source
*                       line number of the call that failed.
*                    - If expr is true, it returns no value.
* @return               None
*******************************************************************************************/
#define CHECK_PARAM(expr) ((expr) ? (void)0 : check_failed((uint8_t *)__FILE__, __LINE__))
#else
#define CHECK_PARAM(expr) ((void)0U)
#endif 

/****************************************************************************************
* Public Typedef
****************************************************************************************/
 
 
/*******************************************************************************
* Exported Public Function
*******************************************************************************/
#ifdef  USE_FULL_ASSERT
void check_failed(uint8_t *file, uint32_t line);
#endif


#endif /* _A34M418_LIBCFG_H_ */
