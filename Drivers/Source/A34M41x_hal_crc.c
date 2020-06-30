/**********************************************************************
* @file		A34M41x_crc.c
* @brief	Contains all functions support for fmc firmware library
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
#include "A34M41x_hal_crc.h"
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_libcfg.h"



/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief		Initializes the CRC peripheral 
 * @param[in]	None
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_CRC_Init(CRC_CFG_Type *CRC_ConfigStruct)
{
	uint32_t ctrl =0;

	 /* Check CRC  handle */
         if(CRC_ConfigStruct == NULL)
        {
            return HAL_ERROR;
         }
		 
	SYST_ACCESS_EN();
	
        SCU->PER2 &=~(1UL<<29);
	SCU->PCER2&=~(1UL<<29);
        /* Set up peripheral clock for CRC module */
        SCU->PER2 |= (1UL<<29);
	SCU->PCER2|= (1UL<<29);
	
	SYST_ACCESS_DIS();
	
	if(CRC_ConfigStruct->Out_inv == ENABLE){
		ctrl |= CRC_OUT_INV_EN;
	}else{
		ctrl |= CRC_OUT_INV_DIS;
	}

	if(CRC_ConfigStruct->Out_rev == ENABLE){
		ctrl |= CRC_OUT_REV_EN;
	}else{
		ctrl |= CRC_OUT_REV_DIS;
	}

	if(CRC_ConfigStruct->In_rev == ENABLE){
		ctrl |= CRC_IN_REV_DIS;
	}else{
		ctrl |= CRC_IN_REV_EN;
	}
	
	if(CRC_ConfigStruct->DMAD_Int == ENABLE){
		ctrl |= CRC_DMAD_INT_EN;
	}else{
		ctrl |= CRC_DMAD_INT_DIS;
	}
	
	ctrl |= CRC_ConfigStruct->Poly << 1;
	
	CRC->CTRL = ctrl;
   return HAL_OK;

}

/**********************************************************************
 * @brief		Set CRC Init value
 * @param[in]	init	CRC Initial value
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_CRC_SetInitVal(uint32_t init)
{
		 
	CRC->INIT = init;
}

/**********************************************************************
 * @brief		Apply CRC Init value
 * @param[in]	en	CRC Apply initial value
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_CRC_ApplyInitVal(FunctionalState en)
{
	if(en == ENABLE){
		CRC->CTRL |= CRC_INIT_EN;
	}else{
		CRC->CTRL |= CRC_INIT_DIS;
	}
}

/**********************************************************************
 * @brief		Get CRC Init
 * @param[in]	None
 * @return		CRC Initial value
 *
 **********************************************************************/
uint32_t HAL_CRC_GetInitVal(void){
	return CRC->INIT;
}


/**********************************************************************
 * @brief		Set CRC Input Data8
 * @param[in]	in	CRC Input data
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_CRC_SetInputData8(uint8_t data8)
{
	CRC->IDR = data8;
}

/**********************************************************************
 * @brief		Set CRC Input Data32
 * @param[in]	in	CRC Input data
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_CRC_SetInputData32(uint32_t data32)
{
	CRC->IDR = ((data32>>0)&0xff);
	CRC->IDR = ((data32>>8)&0xff);
	CRC->IDR = ((data32>>16)&0xff);
	CRC->IDR = ((data32>>24)&0xff);
}


/**********************************************************************
 * @brief		Get CRC Output Data
 * @param[in]	None
 * @return		CRC Output Data
 *
 **********************************************************************/
uint32_t HAL_CRC_GetOutputData(void)
{
	return CRC->ODR;

}

/**********************************************************************
 * @brief		Checks whether the specified CRC status flag is set or not
 * @return		CRC status flag
 **********************************************************************/
uint32_t HAL_CRC_GetStatus(void)
{
	return CRC->STAT;

}

/**********************************************************************
 * @brief		Clear specified pending in CRC peripheral
 * @param[in]	Type	Interrupt pending to clear
 * @return		HAL Status
 **********************************************************************/
void HAL_CRC_ClearPending(uint32_t Type)
{
	CRC->STAT = Type;
}

