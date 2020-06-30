/**********************************************************************
* @file		A34M41x_rng.c
* @brief	Contains all functions support for rng firmware library
* 			on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application2 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_rng.h"
#include "A34M41x_hal_libcfg.h"




/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief		Initializes the RNG peripheral 
 * @param[in]	None
 * @return 		HAL Status
 *********************************************************************/
void HAL_RNG_Init(void)
{
	SYST_ACCESS_EN();
	
	SCU->PER2 &= ~(1UL<<31);
	SCU->PCER2 &= ~(1UL<<31);
	
	SCU->PER2 |= (1UL<<31);
	SCU->PCER2 |= (1UL<<31);
	
	SYST_ACCESS_DIS();

}
/**********************************************************************
 * @brief		Set Generation Counter Parameter
 * @param[in]	GCP	Generation Counter Parameter
 *				Decide generation time of LFSR & CASR
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_RNG_SetGCP(uint16_t gcp){
	uint32_t tmp;
	tmp = RNG->CTRL;
	tmp &= ~(0xFFFFUL << 16);
	tmp |= (gcp << 16);
	RNG->CTRL = tmp;
}

/**********************************************************************
 * @brief		CASR Clock Selection
 * @param[in]	clk	CASR Clock source,
 * 				should be one of the following:
 * 					- RNG_CLOCK_LSI 	:LSI clock
 * 					- RNG_CLOCK_HSI 	:HSI clock
 * @return		HAL Status
 **********************************************************************/
void HAL_RNG_SetCASRClock(RNG_CLOCK clk){
	if(clk == RNG_CLOCK_OSC){
		RNG->CTRL &= ~(1<<15);
	}else if(clk == RNG_CLOCK_LSIHSI){
		RNG->CTRL |= (1<<15);
	}
}


/**********************************************************************
 * @brief		LFSR Clock Selection
 * @param[in]	clk	LFSR Clock source,
 * 				should be one of the following:
 * 					- RNG_CLOCK_LSI 	:LSI clock
 * 					- RNG_CLOCK_HSI 	:HSI clock
 * @return		HAL Status
 **********************************************************************/
void HAL_RNG_SetLFSRClock(RNG_CLOCK clk){
	if(clk == RNG_CLOCK_OSC){
		RNG->CTRL &= ~(1<<14);
	}else if(clk == RNG_CLOCK_LSIHSI){
		RNG->CTRL |= (1<<14);
	}
}

/********************************************************************
 * @brief 		Enable or disable specified RNG interrupt.
 * @param[in]	RNGIntCfg	Specifies the interrupt flag,
 * 				should be one of the following:
 * 					- UART_INTCFG_RDYIE 	:RDY Interrupt enable
 * 					- UART_INTCFG_ERRIE 	:ERR Interrupt enable
 * @param[in]	NewState New state of specified RNG interrupt type,
 * 				should be:
 * 					- ENALBE	:Enable this RNG interrupt type.
 * 					- DISALBE	:Disable this RNG interrupt type.
 * @return 		HAL Status
 *********************************************************************/
void HAL_RNG_ConfigInterrupt(RNG_INT_Type RNGIntCfg, FunctionalState NewState){
	if(RNGIntCfg == RNG_INTCFG_RDYIE){
		if(NewState){
			RNG->CTRL |= (1<<8);
		}else{
			RNG->CTRL &= ~(1<<8);
		}
	}else if(RNGIntCfg == RNG_INTCFG_ERRIE){
		if(NewState){
			RNG->CTRL |= (1<<9);
		}else{
			RNG->CTRL &= ~(1<<9);
		}
	}
}

/**********************************************************************
 * @brief		Enable or disable RNG peripheral's operation
 * @param[in]	NewState New State of RNG peripheral's operation, should be:
 * 					- ENABLE
 * 					- DISABLE
 * @return 		HAL Status
 **********************************************************************/
void HAL_RNG_Cmd(FunctionalState NewState)
{
	if (NewState == ENABLE){
		RNG->CTRL |= 0x1;
	}else{
		RNG->CTRL &= ~(0x1);
	}
}


/**********************************************************************
 * @brief		Set Random seed number
 * @param[in]	seed	RND Seed Setting value
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_RNG_SetSeed(uint32_t seed){
	RNG->SEED = seed;
}

/**********************************************************************
 * @brief		Get Random seed number
 * @param[in]	None
 * @return		RND Seed Setting value
 *
 **********************************************************************/
uint32_t HAL_RNG_GetSeed(void){
	return RNG->SEED;
}

/**********************************************************************
 * @brief		Get RNG Data
 * @param[in]	None
 * @return		Random generated data
 *
 **********************************************************************/
uint32_t HAL_RNG_GetData(void){
	return RNG->RNGD;
}

/**********************************************************************
 * @brief		Checks whether the specified RNG status flag is set or not
 * @return		RNG status flag
 **********************************************************************/
uint32_t HAL_RNG_GetStatus(void){
	return RNG->STAT;
}

/**********************************************************************
 * @brief		Clear specified pending in RNG peripheral
 * @param[in]	Type	Interrupt pending to clear
 * @return		HAL Status
 **********************************************************************/
void HAL_RNG_ClearPending(uint32_t Type)
{
	RNG->STAT = Type;
}

