/***************************************************************************//**
* @file     flash.h
* @brief    Contains all macro definitions and function prototypes support
*           for flash on A31G22x
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

#ifndef _FLASH_H_
#define _FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------- */
#include "A31G22x.h"
#include "aa_types.h"

int flash_erase_macro(int dsel);
int flash_erase_sector(int dsel, unsigned long addr);
int flash_write_sector(int dsel, unsigned long addr,unsigned long *pdata);

int EraseSector (unsigned long addr);
int ProgramPage (unsigned long addr, unsigned long size, unsigned char *buf);
unsigned long Verify (unsigned long addr, unsigned long size, unsigned char *buf);

#ifdef __cplusplus
}
#endif

#endif /* _FLASH_H_ */
/* --------------------------------- End Of File ------------------------------ */
