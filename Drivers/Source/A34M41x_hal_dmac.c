/**********************************************************************
* @file		A34M41x_dmac.c
* @brief	Contains all functions support for DMAC firmware library
* 			on A34M41x
* @version	1.0
* @date		
* @author	ABOV M team
*
* Copyright(C) 2015, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Peripheral group ----------------------------------------------------------- */

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_dmac.h"
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_libcfg.h"




/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief 		Initialize DMA controller
 * @param[in] 	None
 * @return 		HAL Status
 *********************************************************************/
void HAL_DMAC_Init(void)
{
	//System config enable
	SCU->SYSTEN=0x57; SCU->SYSTEN=0x75;
	
	/* to be defined Enable DMA and clock */
	// enabled default on reset
	SCU->PER1&=~(1<<4); // (1<<4) DMA peri enable 
	SCU->PCER1&=~(1<<4); // (1<<4) DMA peri clock enable 
	
	SCU->PER1|=(1<<4); // (1<<4) DMA peri enable 
	SCU->PCER1|=(1<<4); // (1<<4) DMA peri clock enable 
	
	//System config disable
	SCU->SYSTEN=0x00;
}

/*********************************************************************
 * @brief 		Setup DMAC channel peripheral according to the specified
 *              parameters in the GPDMAChannelConfig.
 * @param[in]	DMACx	Pointer to selected DMAC peripheral, should be:
 * 					- DMAC0~3	:DMAC0~3 peripheral
 * @param[in]	DMACChannelConfig Pointer to a DMAC_CH_CFG_Type structure
 * 				that contains the configuration information for the specified
 * 				channel peripheral.
 * @return		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_DMAC_Setup(DMA_Type *DMACx, DMAC_Ch_CFG_Type *DMACChConfig)
{
         /* Check DMACx  handle */
         if(DMACx == NULL)
        {
            return HAL_ERROR;
         }
		 
	DMACx->CR = ((DMACChConfig->TransferCnt & 0xfff)<<16) 
		| ((DMACChConfig->PeriSel & 0x1f)<<8)	
		| ((DMACChConfig->TransferWidth & 0x03)<<2)
		| ((DMACChConfig->Dir & 0x01)<<1)	
		;
	DMACx->PAR = DMACChConfig->PeriAddr;	
	DMACx->MAR = DMACChConfig->MemAddr;
	return HAL_OK;
}

/**********************************************************************
 * @brief		Enable/Disable DMA channel
 * @param[in]	DMACx	Pointer to selected DMAC peripheral, should be:
 * 					- DMAC0~3	:DMAC0~3 peripheral
 * @param[in]	NewState	New State of this command, should be:
 * 					- ENABLE.
 * 					- DISABLE.
 * @return		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_DMAC_ChCmd(DMA_Type *DMACx, FunctionalState NewState)
{
         /* Check DMACx  handle */
         if(DMACx == NULL)
        {
            return HAL_ERROR;
         }
		 
	if (NewState == ENABLE) {
		DMACx->SR |= DMAC_SR_DMAEN_Msk;
	} else {
		DMACx->SR &= ~DMAC_SR_DMAEN_Msk;
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		Check if corresponding channel does have an active interrupt
 * 				request or not
 * @param[in]	DMACx	Pointer to selected DMAC peripheral, should be:
 * 					- DMAC0~3	:DMAC0~3 peripheral
 * @return		FlagStatus	status of DMA channel interrupt after masking
 * 				Should be:
 * 					- SET	: all data is transferred 
 * 					- RESET	:data to be transferred is existing 
 **********************************************************************/
FlagStatus HAL_DMAC_GetStatus(DMA_Type *DMACx)
{
	if (DMACx->SR & DMAC_SR_EOT_Msk){	
		return SET;
	}
	else {
		return RESET;
	}
}



/* --------------------------------- End Of File ------------------------------ */

