/**********************************************************************
* @file		A34M41x_dma.c
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
#include "A34M41x_hal_dma.h"
#include "A34M41x_hal_libcfg.h"




/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief		Initializes the DMA peripheral 
 * @param[in]	None
 * @return 		HAL Status
 *********************************************************************/
void HAL_DMA_Init(void)
{
	SYST_ACCESS_EN();
	
	SCU->PER1 &= ~(1UL<<4);
	SCU->PCER1 &= ~(1UL<<4);
	
	SCU->PER1 |= (1UL<<4);
	SCU->PCER1 |= (1UL<<4);
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief		DMA Config Function
 * @param[in]	DMAx	Pointer to selected DMA peripheral, should be: 0~15
 * @param[in]	Struct Type DMA
 * @return		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_DMA_Cmd(DMA_Type* DMAx, DMA_CFG_Type *dma_cfg)
{
         /* Check DMAx  handle */
         if(DMAx == NULL)
        {
            return HAL_ERROR;
         }
		 
	DMAx->CR = 0
	| ((dma_cfg->transcnt&0xFFF)<<16)
	| ((dma_cfg->perisel&0x1F)<<8)
	| ((dma_cfg->bussize&(0x3<<2)))
	| ((dma_cfg->dirsel&(1<<1)))
	;
	return HAL_OK;
	
}


/**********************************************************************
 * @brief		DMA Start
 * @param[in]	DMAx	Pointer to selected DMA peripheral, should be: 0~15
 * @return		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_DMA_Start(DMA_Type* DMAx)
{
         /* Check DMAx  handle */
         if(DMAx == NULL)
        {
            return HAL_ERROR;
         }
	DMAx->SR |= (1<<0);
	return HAL_OK;
}


/**********************************************************************
 * @brief		DMA Stop
 * @param[in]	DMAx	Pointer to selected DMA peripheral, should be: 0~15
 * @return		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_DMA_Stop(DMA_Type* DMAx)
{
         /* Check DMAx  handle */
         if(DMAx == NULL)
        {
            return HAL_ERROR;
         }
	DMAx->SR &= ~(1<<0);
	return HAL_OK;
}


/**********************************************************************
 * @brief		DMA Get Status
 * @param[in]	DMAx	Pointer to selected DMA peripheral, should be: 0~15
 * @return		DMAx Status
 **********************************************************************/
uint32_t HAL_DMA_GetStatus(DMA_Type* DMAx)
{
	return (DMAx->SR);

}


/**********************************************************************
 * @brief		DMA Set Peripheral Address Register
 * @param[in]	DMAx	Pointer to selected DMA peripheral, should be: 0~15
 * @param[in]	DMA Type : DMAx, Peri Address
 * @return		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_DMA_SetPAR(DMA_Type* DMAx, uint32_t peri_addr)
{
         /* Check DMAx  handle */
         if(DMAx == NULL)
        {
            return HAL_ERROR;
         }
	DMAx->PAR = peri_addr;
	return HAL_OK;
}


/**********************************************************************
 * @brief		DMA Set Memory Address Register
 * @param[in]	DMAx	Pointer to selected DMA peripheral, should be: 0~15
 * @param[in]	DMA Type : DMAx, Mem Address
 * @return		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_DMA_SetMAR(DMA_Type* DMAx, uint32_t mem_addr)
{
         /* Check DMAx  handle */
         if(DMAx == NULL)
        {
            return HAL_ERROR;
         }
	DMAx->MAR = mem_addr;
	return HAL_OK;
}





