/**********************************************************************
* @file		A34M41x_crc.h
* @brief	Contains all macro definitions and function prototypes
* 			support for CRC firmware library on A34M41x
* @version	1.0
* @date		
* @author ABOV Application2 team
*
* Copyright(C)  2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_CRC_H_
#define _A34M41x_CRC_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

#define CRC_OUT_INV_DIS                  (0x0uL<<21)
#define CRC_OUT_INV_EN                   (0x1uL<<21)

#define CRC_OUT_REV_DIS                  (0x0uL<<20)
#define CRC_OUT_REV_EN                   (0x1uL<<20)

#define CRC_IN_REV_DIS                   (0x0uL<<16)
#define CRC_IN_REV_EN                    (0x1uL<<16)
	
#define CRC_DMAD_INT_DIS                 (0x0uL<<8)
#define CRC_DMAD_INT_EN                  (0x1uL<<8)
	
#define CRC_INIT_DIS                     (0x0uL<<0)
#define CRC_INIT_EN                      (0x1uL<<0)

#define CRC_DMAD_INT                     (0x1uL<<8)

	
#ifdef __cplusplus
}
#endif 

typedef enum {
	CRC_POLY_CRC32 =0,
	CRC_POLY_CRC16,
	CRC_POLY_CRC8,
	CRC_POLY_CRC7
}eCRC_POLY;

/********************************************************************
* @brief UART Configuration Structure definition
**********************************************************************/
typedef struct {
  uint32_t Out_inv;
  uint32_t Out_rev;
  uint32_t In_rev;
  uint32_t DMAD_Int;
  uint32_t Poly;
} CRC_CFG_Type;



HAL_Status_Type HAL_CRC_Init(CRC_CFG_Type *CRC_ConfigStruct);
void HAL_CRC_SetInitVal(uint32_t init);
void HAL_CRC_ApplyInitVal(FunctionalState en);
uint32_t	HAL_CRC_GetInitVal(void);
void HAL_CRC_SetInputData8(uint8_t data8);
void HAL_CRC_SetInputData32(uint32_t data32);
uint32_t	HAL_CRC_GetOutputData(void);
uint32_t	HAL_CRC_GetStatus(void);
void HAL_CRC_ClearPending(uint32_t Type);

#endif /* _A34M41x_CRC_H_ */

/* --------------------------------- End Of File ------------------------------ */
