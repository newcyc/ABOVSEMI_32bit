/**********************************************************************
* @file		A34M41x_spi.c
* @brief	Contains all functions support for SPI firmware library on A34M41x
* @version	1.0
* @date
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_spi.h"
#include "A34M41x_hal_libcfg.h"



/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief		Initializes the SPIx peripheral according to the specified
 *              parameters in the SPI_ConfigStruct.
 * @param[in]	SPIx SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	SPI_ConfigStruct Pointer to a SPI_CFG_Type structure that
 * 				contains the configuration information for the specified
 * 				SPI peripheral.
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_SPI_Init(SPI_Type *SPIx, SPI_CFG_Type *SPI_ConfigStruct)
{
	uint32_t tmp;

	/* Check SPI  handle */
         if(SPIx == NULL)
        {
            return HAL_ERROR;
        }

	SYST_ACCESS_EN();
	
	if(SPIx == SPI0) {
		/* SPI0 enable */
		SCU->PER2 &= ~(1<<0);
		SCU->PCER2 &= ~(1<<0);
		
		SCU->PER2 |= (1<<0);
		SCU->PCER2 |= (1<<0);
	} 
	else if(SPIx == SPI1) {
		/* SPI1 enable */
		SCU->PER2 &= ~(1<<1);
		SCU->PCER2 &= ~(1<<1);
		
		SCU->PER2 |= (1<<1);
		SCU->PCER2 |= (1<<1);
	} 
	else if(SPIx == SPI2) {
		/* SPI2 enable */
		SCU->PER2 &= ~(1<<2);
		SCU->PCER2 &= ~(1<<2);
		
		SCU->PER2 |= (1<<2);
		SCU->PCER2 |= (1<<2);
	} 
	
	SYST_ACCESS_DIS();
	
	/* Configure SPI, interrupt is disable, LoopBack mode is disable,
	 * SPI is disable, SS is auto mode.
	 */
	SPIx->EN = 0;
	
	tmp = 0
	|(1<<20)  // TXBC tx buffer clear  
	|(1<<19)  // RXBC rx buffer clear
	|(0<<18)  // TXDIE
	|(0<<17)  // RXDIE
	|(0<<16)  // SSCIE
	|(0<<15)  // TXIE
	|(0<<14)  // RXIE
	|(0<<13)  // SSMOD       0:SS auto, 1:SS is controlled by SSOUT
	|(0<<12)  // SSOUT        0:SS is low, 1:SS is high
	|(0<<11)  // LBE             loop-back mode 
	|(0<<10)  // SSMASK     0:receive data when SS signal is active, 1:receive data at SCLK edge
	|(1<<9)   // SSMO          0:SS output signal is disabled, 1:SS output signal is enabled
	|(0<<8)   // SSPOL         0:SS is active-low, 1:SS is active-high 
	|((SPI_ConfigStruct->Mode & 0x01) << 5)   // MS               0:slave mode, 1:master mode 
	|((SPI_ConfigStruct->DataDir & 0x01) << 4)   // MSBF           0:LSB first, 1:MSB first
	|((SPI_ConfigStruct->CPHA & 0x01) <<3)   // CPHA
	|((SPI_ConfigStruct->CPOL & 0x01) <<2)   // CPOL
	|((SPI_ConfigStruct->Databit & SPI_DS_BITMASK)<<0)   // BITSZ           0:8bit, 1:9bit, 2:16bit, 3:17bit
		;
	SPIx->CR = tmp;
	
	if (SPI_ConfigStruct->Mode == SPI_MASTER_MODE){
		SPIx->BR = (SPI_ConfigStruct->BaudRate & 0xFFFF); // PCLK / (BR + 1) ex) if BR=71, 72Mhz / (71+1)
	}
	else { // SPI_SLAVE_MODE   if using slave, BR has fast clock set and LR has minimum set. 
		SPIx->BR = 2; // PCLK / (BR + 1) ex) if BR=2 and PCLK=72Mhz,  72Mhz / (2 +1)	
	}
	
	SPIx->LR = 0
	|(1<<16)  // SPL >= 1 
	|(1<<8)   // BTL >= 1
	|(1<<0)   // STL >= 1
		;	
	return HAL_OK;
}

/**********************************************************************
 * @brief		De-initializes the SPIx peripheral registers to their
 *              default reset values.
 * @param[in]	SPIx SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_SPI_DeInit(SPI_Type* SPIx)
{
	/* Check SPI  handle */
         if(SPIx == NULL)
        {
            return HAL_ERROR;
        }

	/* Disable SPI operation*/
	SPIx->SR = 0; // register clear 

	if(SPIx == SPI0) {
		/* SPI0 enable */
		SCU->PER2 &= ~(1<<0);
		SCU->PCER2 &= ~(1<<0);
	} 
	else if(SPIx == SPI1) {
		/* SPI1 enable */
		SCU->PER2 &= ~(1<<1);
		SCU->PCER2 &= ~(1<<1);
	} 
	else if(SPIx == SPI2) {
		/* SPI2 enable */
		SCU->PER2 &= ~(1<<2);
		SCU->PCER2 &= ~(1<<2);
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		configure SPI delay length 
 * @param[in]	SPIx	SPI peripheral, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	StartDelay : SPL >= 1 (1~255)
 * @param[in]	BurstDelay : BTL >= 1 (1~255)
 * @param[in]	StopDelay : STL >= 1 (1~255)
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_SPI_DelayConfig(SPI_Type* SPIx, uint8_t StartDelay, uint8_t BurstDelay,uint8_t StopDelay)
{
	/* Check SPI  handle */
         if(SPIx == NULL)
        {
            return HAL_ERROR;
        }

	if (SPIx->CR & (SPI_MASTER_MODE << 5)){
		SPIx->LR = 0
		|((StopDelay&0xFF)<<16)  // SPL >= 1 (1~255) 
		|((BurstDelay&0xFF)<<8)   // BTL >= 1 (1~255)
		|((StartDelay&0xFF)<<0)   // STL >= 1 (1~255)
			;		
	}
	else {
		SPIx->LR = 0
		|(1<<16)  // SPL >= 1 
		|(1<<8)   // BTL >= 1
		|(1<<0)   // STL >= 1
			;
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		Enable or disable SPI peripheral's operation
 * @param[in]	SPIx	SPI peripheral, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	NewState New State of SPIx peripheral's operation, should be:
 * 					- ENABLE
 * 					- DISABLE
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_SPI_Enable(SPI_Type* SPIx, FunctionalState NewState)
{
	/* Check SPI  handle */
	 if(SPIx == NULL)
	{
			return HAL_ERROR;
	}

	if (NewState == ENABLE)
	{
		SPIx->EN = 1;
	}
	else
	{
		SPIx->EN = 0;
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		Enable or disable Loop Back mode function in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	NewState	New State of Loop Back mode, should be:
 * 					- ENABLE
 * 					- DISABLE
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_SPI_LoopBackCmd(SPI_Type* SPIx, FunctionalState NewState)
{
	/* Check SPI  handle */
         if(SPIx == NULL)
        {
            return HAL_ERROR;
        }

	if (NewState == ENABLE)
	{
		SPIx->CR |= SPI_LBM_EN;
	}
	else
	{
		SPIx->CR &= ~SPI_LBM_EN;
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		control SS Output function in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	State	State of Slave Output function control, should be:
 * 					- SS_AUTO	: SS is controlled automatically
 * 					- SS_MANUAL	:SS is controlled by SSOUT 12bit in CR
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_SPI_SSOutputCmd(SPI_Type* SPIx, uint32_t State)
{
	/* Check SPI  handle */
         if(SPIx == NULL)
        {
            return HAL_ERROR;
        }

	if (State == SS_AUTO)
	{
		SPIx->CR &= (~SPI_SSMOD_MANUAL);
	}
	else
	{
		SPIx->CR |= SPI_SSMOD_MANUAL;
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		control SS Output in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	 State	 State of Slave Output , should be:
 * 					- SS_OUT_LO : SS out value is Low
 * 					- SS_OUT_HI  : SS out value is High
 *  use this function after setting as SPI_SSOutputCmd(SPIx,SS_MANUAL)
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_SPI_SSOutput(SPI_Type* SPIx, uint32_t State)
{
	/* Check SPI  handle */
         if(SPIx == NULL)
        {
            return HAL_ERROR;
        }

	if (State == SS_OUT_LO)
	{
		SPIx->CR &= (~SPI_SSOUT_HI);
	}
	else
	{
		SPIx->CR |= SPI_SSOUT_HI;
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		Transmit a single data through SPIx peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	Data	Data to transmit 
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_SPI_TransmitData(SPI_Type* SPIx, uint32_t Data)
{
	/* Check SPI  handle */
         if(SPIx == NULL)
        {
            return HAL_ERROR;
        }

	SPIx->TDR = (Data);
	return HAL_OK;
}

/**********************************************************************
 * @brief		Receive a single data from SPIx peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be
 * 				 	- SP	:SPI0~2 peripheral
 * @return 		Data received 
 **********************************************************************/
uint32_t HAL_SPI_ReceiveData(SPI_Type* SPIx)
{
	return ((uint32_t) (SPIx->RDR));
}

/**********************************************************************
 * @brief		Checks whether the specified SPI status flag is set or not
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @return		SPI status flag
 **********************************************************************/
uint32_t HAL_SPI_GetStatus(SPI_Type* SPIx)
{
	return (SPIx->SR);
}

/**********************************************************************
 * @brief		Clear specified pending in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	Type	Interrupt pending to clear
 * 					- SPI_STAT_TXDMA_DONE 		((uint32_t)(9<<0))
 * 					- SPI_STAT_RXDMA_DONE  		((uint32_t)(8<<0))
 * 					- SPI_STAT_SS_DET 			((uint32_t)(6<<0))
 * 					- SPI_STAT_SS_ACT  			((uint32_t)(5<<0))
 * 					- SPI_STAT_TXUNDERRUN_ERR 	((uint32_t)(4<<0))
 * 					- SPI_STAT_RXOVERRUN_ERR 	((uint32_t)(3<<0))
 * @return		None
 **********************************************************************/
HAL_Status_Type HAL_SPI_ClearPending(SPI_Type* SPIx, uint32_t Type)
{
	/* Check SPI  handle */
         if(SPIx == NULL)
        {
            return HAL_ERROR;
        }

	SPIx->SR = (~Type);
	return HAL_OK;
}

/***********************************************************************
 * @brief		Enable or disable specified interrupt type in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	IntType	Interrupt type in SPI peripheral, should be:
 * 					- SPI_INTCFG_TXDIE		: DMA TX done interrupt enable
 * 					- SPI_INTCFG_RXDIE		: DMA RX done interrupt enable
 * 					- SPI_INTCFG_SSCIE		: SS edge change interrupt enable 
 * 					- SPI_INTCFG_TXIE			: TX interrupt enable
 * 					- SPI_INTCFG_RXIE			: RX interrupt enable
 * @param[in]	NewState New State of specified interrupt type, should be:
 * 					- ENABLE	:Enable this interrupt type
 * 					- DISABLE	:Disable this interrupt type
 * @return		None
 **********************************************************************/
HAL_Status_Type HAL_SPI_ConfigInterrupt(SPI_Type* SPIx, uint32_t IntType, FunctionalState NewState)
{
	/* Check SPI  handle */
	 if(SPIx == NULL)
	{
			return HAL_ERROR;
	}

	if (NewState == ENABLE)
	{
		SPIx->CR |= IntType;
	}
	else
	{
		SPIx->CR &= (~IntType);
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		get control register from SPIx peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be
 * 				 	- SP	:SPI0~2 peripheral
 * 					- SP1	:SPI1 peripheral
 * @return 		Data received 
 **********************************************************************/
uint32_t HAL_SPI_GetControl(SPI_Type* SPIx)
{
	return ((uint32_t) (SPIx->CR));
}



/* --------------------------------- End Of File ------------------------------ */

