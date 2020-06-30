/**********************************************************************
* @file		A34M41x_aes128.h
* @brief	Contains all macro definitions and function prototypes
* 			support for AES128 firmware library on A34M41x
* @version	1.0
* @date		
* @author ABOV Application2 team
*
* Copyright(C)  2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_AES128_H_
#define _A34M41x_AES128_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif
/* Public Macros -------------------------------------------------------------- */
#define FLUSH_IN_FIFO()		(AES128->CTRL |= (1<<8))
#define FLUSH_OUT_FIFO()	(AES128->CTRL |= (1<<9))
	
/*********************************************************************//**
 * @brief AES128 Status defines
 **********************************************************************/
#define AES128_STAT_CIPDONE			((uint32_t)(1<<16))	/* CIPDONE flag bit */
#define AES128_STAT_INVCIPDONE		((uint32_t)(1<<17))	/* INVCIPDONE flag bit */
#define AES128_STAT_DMAOUT			((uint32_t)(1<<18))
#define AES128_STAT_DMAIN			((uint32_t)(1<<19))

/*********************************************************************//**
 * @brief AES128 Control defines
 **********************************************************************/
#define AES128_CTRL_MODE			((uint32_t)(3<<0))
#define AES128_CTRL_WORDORDER		((uint32_t)(1<<2))
#define AES128_CTRL_INFIFOFLUSH		((uint32_t)(1<<8))
#define AES128_CTRL_OUTFIFOFLUSH	((uint32_t)(1<<9))

#define AES128_CTRL_CIPHERIE		((uint32_t)(1<<16))
#define AES128_CTRL_INVCIPHERIE		((uint32_t)(1<<17))
#define AES128_CTRL_DMAOUTIE		((uint32_t)(1<<18))
#define AES128_CTRL_DMAINIE			((uint32_t)(1<<19))

	
/* Public Types --------------------------------------------------------------- */
/***********************************************************************
 * @brief AES128 enumeration
**********************************************************************/
typedef enum{
	eAES128_CipherMode = 1,
	eAES128_InvCipherMode = 2,
}eAES128_mode;

typedef enum{
	eAES128_OrderedByLSB_WORD = 0,
	eAES128_OrderedByMSB_WORD = 1,
	eAES128_OrderedByMSB_BYTE_IN_WORD = 2,
	eAES128_OrderedByMSB_BYTE = 4
}eAES128_order;


void HAL_AES128_Init(void);
void HAL_AES128_SetAESTextIn(uint32_t *in);
void HAL_AES128_GetAESTextIn(uint32_t *in);
void HAL_AES128_GetAESTextOut(uint32_t *out);
void HAL_AES128_SetAESKeyIn(uint32_t *key);
void HAL_AES128_GetAESKeyIn(uint32_t *key);
void HAL_AES128_ModSel(eAES128_mode mode);
void HAL_AES128_OrderSel(eAES128_order In_order, eAES128_order Out_order);
void HAL_AES128_ConfigInterrupt(uint32_t IntType, FunctionalState NewState);
uint32_t 	HAL_AES128_GetStatus(void);
void HAL_AES128_ClearStatus(uint32_t status);

#ifdef __cplusplus
}
#endif


#endif /* _A34M41x_AES128_H_ */

/* --------------------------------- End Of File ------------------------------ */
