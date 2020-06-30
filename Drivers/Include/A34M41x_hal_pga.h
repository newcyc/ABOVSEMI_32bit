/**********************************************************************
* @file		A34M41x_pga.h
* @brief	Contains all macro definitions and function prototypes
* 			support for UART firmware library on A34M41x
* @version	1.0
* @date		
* @author ABOV M team
*
* Copyright(C)  2015, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_PGA_H_
#define _A34M41x_PGA_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
//#include "def.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif


/* Private Macros ------------------------------------------------------------- */

/* --------------------- BIT DEFINITIONS -------------------------------------- */
/**********************************************************************
 * Macro defines for Macro defines for Comparator control register
 **********************************************************************/
#define PGA_CR_AMPISEL_0	((uint32_t)(0<<16))
#define PGA_CR_AMPISEL_17	((uint32_t)(1<<16))
#define PGA_CR_AMPISEL_21	((uint32_t)(2<<16))
#define PGA_CR_AMPISEL_12	((uint32_t)(3<<16))
#define PGA_CR_AMPISEL_MASK		((uint32_t)(3<<16))

#define PGA_GAINSEL_1_200				(0x0uL<<0)
#define PGA_GAINSEL_1_304				(0x1uL<<0)
#define PGA_GAINSEL_1_404				(0x2uL<<0)
#define PGA_GAINSEL_1_500				(0x3uL<<0)
#define PGA_GAINSEL_1_600				(0x4uL<<0)
#define PGA_GAINSEL_1_702				(0x5uL<<0)
#define PGA_GAINSEL_1_805				(0x6uL<<0)
#define PGA_GAINSEL_1_905				(0x7uL<<0)
#define PGA_GAINSEL_2_000				(0x8uL<<0)
#define PGA_GAINSEL_2_182				(0x9uL<<0)
#define PGA_GAINSEL_2_330				(0xAuL<<0)
#define PGA_GAINSEL_2_500				(0xBuL<<0)
#define PGA_GAINSEL_2_667				(0xCuL<<0)
#define PGA_GAINSEL_2_927				(0xDuL<<0)
#define PGA_GAINSEL_3_000				(0xEuL<<0)
#define PGA_GAINSEL_3_158				(0xFuL<<0)
#define PGA_GAINSEL_3_478				(0x10uL<<0)
#define PGA_GAINSEL_3_871				(0x11uL<<0)
#define PGA_GAINSEL_4_000				(0x12uL<<0)
#define PGA_GAINSEL_4_364				(0x13uL<<0)
#define PGA_GAINSEL_5_000				(0x14uL<<0)
#define PGA_GAINSEL_5_854				(0x15uL<<0)
#define PGA_GAINSEL_6_000				(0x16uL<<0)
#define PGA_GAINSEL_6_857				(0x17uL<<0)
#define PGA_GAINSEL_8_000				(0x18uL<<0)
#define PGA_GAINSEL_8_571				(0x19uL<<0)
#define PGA_GAINSEL_10_00				(0x1AuL<<0)
#define PGA_GAINSEL_12_00				(0x1BuL<<0)
#define PGA_GAINSEL_15_00				(0x1CuL<<0)
#define PGA_GAINSEL_16_00				(0x1DuL<<0)

#define UGAIN_DISABLE					(0x0uL<<1)
#define UGAIN_ENABLE					(0x1uL<<1)

#define AMP_DISABLE					(0x0uL<<0)
#define AMP_ENABLE					(0x1uL<<0)	

typedef struct {
	uint32_t AMPISel;
	uint32_t GainSel;
	uint32_t UGainEnSel;
	uint32_t AMPEnSel;
} PGA_CFG_Type;

/* Public Types --------------------------------------------------------------- */
/***********************************************************************
 * @brief Comparator enumeration
**********************************************************************/

/* Public Functions ----------------------------------------------------------- */
void HAL_PGA_ClockInit(FunctionalState NewState);
HAL_Status_Type HAL_PGA_Init(PGA_Type *PGAx, PGA_CFG_Type *PGA_ConfigStruct);
#ifdef __cplusplus
}
#endif


#endif /* _PGA_H_ */

/* --------------------------------- End Of File ------------------------------ */
