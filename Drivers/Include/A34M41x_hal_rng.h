/**********************************************************************
* @file		A34M41x_rng.h
* @brief	Contains all macro definitions and function prototypes
* 			support for RNG firmware library on A34M41x
* @version	1.0
* @date		
* @author ABOV Application2 team
*
* Copyright(C)  2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_RNG_H_
#define _A34M41x_RNG_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* Public Macros -------------------------------------------------------------- */
/*********************************************************************//**
 * RNG Status defines
 **********************************************************************/
#define RNG_STAT_ERRI		((uint32_t)(1<<9))	/** RNG status Error Interrupt bit */
#define RNG_STAT_RDYI		((uint32_t)(1<<8))	/** RNG status Ready Interrupt bit */
#define RNG_STAT_ERR		((uint32_t)(1<<1))	/** RNG status Error flag bit */
#define RNG_STAT_RDY		((uint32_t)(1<<0))	/** RNG status Ready flag bit */

	
/* Public Types --------------------------------------------------------------- */
/***********************************************************************
 * @brief RNG enumeration
**********************************************************************/
typedef enum {
	RNG_CLOCK_OSC = 0,
	RNG_CLOCK_LSIHSI
}RNG_CLOCK;

typedef enum {
	RNG_INTCFG_RDYIE = 8,	/*!< RDY Interrupt enable*/
	RNG_INTCFG_ERRIE,		/*!< ERR Interrupt enable*/
} RNG_INT_Type;



/* Public Functions ----------------------------------------------------------- */
void HAL_RNG_Init(void);
void HAL_RNG_SetGCP(uint16_t gcp);
void HAL_RNG_SetCASRClock(RNG_CLOCK clk);
void HAL_RNG_SetLFSRClock(RNG_CLOCK clk);
void HAL_RNG_ConfigInterrupt(RNG_INT_Type RNGIntCfg, FunctionalState NewState);
void HAL_RNG_Cmd(FunctionalState NewState);
void HAL_RNG_SetSeed(uint32_t seed);
uint32_t HAL_RNG_GetSeed(void);
uint32_t HAL_RNG_GetData(void);
uint32_t HAL_RNG_GetStatus(void);
void HAL_RNG_ClearPending(uint32_t Type);

#ifdef __cplusplus
}
#endif


#endif /* _A34M41x_RNG_H_ */

/* --------------------------------- End Of File ------------------------------ */
