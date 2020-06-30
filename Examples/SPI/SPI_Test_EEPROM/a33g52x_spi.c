/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_spi.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
*
* @ Description 
*   ABOV Semiconductor is supplying this software for use with HART-m310
*   processor. This software contains the confidential and proprietary information
*   of ABOV Semiconductor Co., Ltd ("Confidential Information").
*
*
**************************************************************************************
* DISCLAIMER 
*
* 	THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS  
* 	WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE  
* 	TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* 	DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING  
* 	FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE  
* 	CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.  
*
**************************************************************************************
*/


#include <stdint.h>
#include "A33G52x.h"
#include "a33g52x_spi.h"
#include "a33g52x_pcu.h"
#include "a33g52x_pmu.h"
#include "aa_types.h"


/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief		Initializes the SPIx peripheral according to the specified
 *              parameters in the SPI_ConfigStruct.
 * @param[in]	SPIx SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	SPI_ConfigStruct Pointer to a SPI_CFG_Type structure that
 * 				contains the configuration information for the specified
 * 				SPI peripheral.
 * @return 		None
 *********************************************************************/
void SPI_Init(SPI_Type *SPIx, SPI_CFG_Type *SPI_ConfigStruct)
{
	uint32_t tmp;
//	uint32_t prescale, cr0_div, cmp_clk, ssp_clk;

	if (SPIx == SPI0) {
		/* SPI0 enable */
		PMU->PER &= ~(1<<16);
		PMU->PCCR &= ~(1<<16);
		
		PMU->PER |= (1<<16);
		PMU->PCCR |= (1<<16);
        
	} else {
		/* SPI1 enable */
		PMU->PER &= ~(1<<17);
		PMU->PCCR &= ~(1<<17);
		
		PMU->PER |= (1<<17);
		PMU->PCCR |= (1<<17);
	} 
	
	/* Configure SPI, interrupt is disable, LoopBack mode is disable,
	 * SPI is disable, SS is auto mode.
	 */
    
	SPIx->EN = 0;
	
	tmp = 0
	|(1<<20)  // TXBC tx buffer clear  
	|(1<<19)  // RXBC rx buffer clear
	|(0<<15)  // SSCIE
	|(0<<14)  // TXIE
	|(0<<13)  // RXIE
	|(0<<12)  // SSMOD       0:SS auto, 1:SS is controlled by SSOUT
	|(0<<11)  // SSOUT        0:SS is low, 1:SS is high
	|(0<<10)  // LBE             loop-back mode 
	|(0<<9)  // SSMASK     0:receive data when SS signal is active, 1:receive data at SCLK edge
	|(1<<8)   // SSMO          0:SS output signal is disabled, 1:SS output signal is enabled
	|(0<<7)   // SSPOL         0:SS is active-low, 1:SS is active-high 
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
	|(6<<10)  // SPL >= 1 
	|(3<<5)   // BTL >= 1
	|(6<<0)   // STL >= 1
		;		
}

/**********************************************************************
 * @brief		De-initializes the SPIx peripheral registers to their
 *              default reset values.
 * @param[in]	SPIx SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @return 		None
 **********************************************************************/
void SPI_DeInit(SPI_Type* SPIx)
{
	/* Disable SPI operation*/
	SPIx->SR = 0; // register clear 

	if(SPIx == SPI0) {
		/* SPI0 enable */
		PMU->PER &= ~(1<<16);
		PMU->PCCR &= ~(1<<16);
	} 
	else if(SPIx == SPI1) {
		/* SPI1 enable */
		PMU->PER &= ~(1<<17);
		PMU->PCCR &= ~(1<<17);
	} 
}

/**********************************************************************
 * @brief		configure SPI delay length 
 * @param[in]	SPIx	SPI peripheral, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	StartDelay : SPL >= 1 (1~255)
 * @param[in]	BurstDelay : BTL >= 1 (1~255)
 * @param[in]	StopDelay : STL >= 1 (1~255)
 * @return 		none
 **********************************************************************/
void SPI_DelayConfig(SPI_Type* SPIx, uint8_t StartDelay, uint8_t BurstDelay,uint8_t StopDelay)
{
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
}

/**********************************************************************
 * @brief		Enable or disable SPI peripheral's operation
 * @param[in]	SPIx	SPI peripheral, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	NewState New State of SPIx peripheral's operation, should be:
 * 					- ENABLE
 * 					- DISABLE
 * @return 		none
 **********************************************************************/
void SPI_Cmd(SPI_Type* SPIx, FunctionalState NewState)
{
	if (NewState == ENABLE)
	{
		SPIx->EN = 1;
	}
	else
	{
		SPIx->EN = 0;
	}
}

/**********************************************************************
 * @brief		Enable or disable Loop Back mode function in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	NewState	New State of Loop Back mode, should be:
 * 					- ENABLE
 * 					- DISABLE
 * @return 		None
 **********************************************************************/
void SPI_LoopBackCmd(SPI_Type* SPIx, FunctionalState NewState)
{
	if (NewState == ENABLE)
	{
		SPIx->CR |= SPI_LBM_EN;
	}
	else
	{
		SPIx->CR &= ~SPI_LBM_EN;
	}
}

/**********************************************************************
 * @brief		control SS Output function in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	State	State of Slave Output function control, should be:
 * 					- SS_AUTO	: SS is controlled automatically
 * 					- SS_MANUAL	:SS is controlled by SSOUT 12bit in CR
 * @return 		None
 **********************************************************************/
void SPI_SSOutputCmd(SPI_Type* SPIx, uint32_t State)
{
	if (State == SS_AUTO)
	{
		SPIx->CR &= (~SPI_SSMOD_MANUAL);
	}
	else
	{
		SPIx->CR |= SPI_SSMOD_MANUAL;
	}
}

/**********************************************************************
 * @brief		control SS Output in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	 State	 State of Slave Output , should be:
 * 					- SS_OUT_LO : SS out value is Low
 * 					- SS_OUT_HI  : SS out value is High
 *  use this function after setting as SPI_SSOutputCmd(SPIx,SS_MANUAL)
 * @return 		None
 **********************************************************************/
void SPI_SSOutput(SPI_Type* SPIx, uint32_t State)
{
	if (State == SS_OUT_LO)
	{
		SPIx->CR &= (~SPI_SSOUT_HI);
	}
	else
	{
		SPIx->CR |= SPI_SSOUT_HI;
	}
}

/**********************************************************************
 * @brief		Transmit a single data through SPIx peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @param[in]	Data	Data to transmit 
 * @return 		none
 **********************************************************************/
void SPI_SendData(SPI_Type* SPIx, uint32_t Data)
{
	SPIx->RDR_TDR = (Data);
}

/**********************************************************************
 * @brief		Receive a single data from SPIx peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be
 * 				 	- SP	:SPI0~2 peripheral
 * @return 		Data received 
 **********************************************************************/
uint32_t SPI_ReceiveData(SPI_Type* SPIx)
{
	return ((uint32_t) (SPIx->RDR_TDR));
}

/**********************************************************************
 * @brief		Checks whether the specified SPI status flag is set or not
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @return		SPI status flag
 **********************************************************************/
uint32_t SPI_GetStatus(SPI_Type* SPIx)
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
void SPI_ClearPending(SPI_Type* SPIx, uint32_t Type)
{
	SPIx->SR = (~Type);
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
void SPI_IntConfig(SPI_Type* SPIx, uint32_t IntType, FunctionalState NewState)
{
	if (NewState == ENABLE)
	{
		SPIx->CR |= IntType;
	}
	else
	{
		SPIx->CR &= (~IntType);
	}
}

/**********************************************************************
 * @brief		get control register from SPIx peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be
 * 				 	- SP	:SPI0~2 peripheral
 * 					- SP1	:SPI1 peripheral
 * @return 		Data received 
 **********************************************************************/
uint32_t SPI_GetControl(SPI_Type* SPIx)
{
	return ((uint32_t) (SPIx->CR));
}
