/***************************************************************************//**
* @file     A31G22x_spi.c
* @brief    Contains all functions support for SPI dirver on A31G22x
* @author   AE Team, ABOV Semiconductor Co., Ltd.
* @version  V0.0.1
* @date     19. Apr. 2019
*
* Copyright(C) 2019, ABOV Semiconductor
* All rights reserved.
*
*
********************************************************************************
* DISCLAIMER 
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A31G22x_scu.h"
#include "A31G22x_spi.h"
#include "A31G22x_pcu.h"


/* Public Functions ----------------------------------------------------------- */

/***************************************************************************//**
 * @brief		Initializes the SPIx peripheral according to the specified
 *              parameters in the SPI_ConfigStruct.
 * @param[in]	SPIx SPI peripheral selected, should be:
 * 				 	- SPI	:SPI2n peripheral
 * @param[in]	SPI_ConfigStruct Pointer to a SPI_CFG_Type structure that
 * 				contains the configuration information for the specified
 * 				SPI peripheral.
 * @return 		None
*******************************************************************************/
void SPI_Init(SPI_Type *SPIx, SPI_CFG_Type *SPI_ConfigStruct)
{
	uint32_t tmp;

	SPI_DeInit(SPIx);
	
	if(SPIx == SPI20) {
		/* SPI0 enable */
		SCU->PER2 &= ~(SCU_PER2_SPI20_Msk);
		SCU->PCER2 &= ~(SCU_PCER2_SPI20_Msk);
		
		SCU->PER2 |= (SCU_PER2_SPI20_Msk);
		SCU->PCER2 |= (SCU_PCER2_SPI20_Msk);
	} else if(SPIx == SPI21) {
		/* SPI1 enable */
		SCU->PER2 &= ~(SCU_PER2_SPI21_Msk);
		SCU->PCER2 &= ~(SCU_PCER2_SPI21_Msk);
		
		SCU->PER2 |= (SCU_PER2_SPI21_Msk);
		SCU->PCER2 |= (SCU_PCER2_SPI21_Msk);
	} 
	
	/* Configure SPI, interrupt is disable, LoopBack mode is disable,
	 * SPI is disable, SS is auto mode.
	 */
	SPIx->EN = 0;
	
	tmp = 0
	|(1 << SPI_CR_TXBC_Pos)  // TXBC tx buffer clear  
	|(1 << SPI_CR_RXBC_Pos)  // RXBC rx buffer clear
	|(0 << SPI_CR_TXDIE_Pos)  // TXDIE
	|(0 << SPI_CR_RXDIE_Pos)  // RXDIE
	|(0 << SPI_CR_SSCIE_Pos)  // SSCIE
	|(0 << SPI_CR_TXIE_Pos)  // TXIE
	|(0 << SPI_CR_RXIE_Pos)  // RXIE
	|(0 << SPI_CR_SSMOD_Pos)  // SSMOD       0:SS auto, 1:SS is controlled by SSOUT
	|(0 << SPI_CR_SSOUT_Pos)  // SSOUT        0:SS is low, 1:SS is high
	|(0 << SPI_CR_LBE_Pos)  // LBE             loop-back mode 
	|(0 << SPI_CR_SSMASK_Pos)  // SSMASK     0:receive data when SS signal is active, 1:receive data at SCLK edge
	|(1 << SPI_CR_SSMO_Pos)   // SSMO          0:SS output signal is disabled, 1:SS output signal is enabled
	|(0 << SPI_CR_SSPOL_Pos)   // SSPOL         0:SS is active-low, 1:SS is active-high 
	|((SPI_ConfigStruct->Mode & 0x01) << SPI_CR_MS_Pos)   // MS               0:slave mode, 1:master mode 
	|((SPI_ConfigStruct->DataDir & 0x01) << SPI_CR_MSBF_Pos)   // MSBF           0:LSB first, 1:MSB first
	|((SPI_ConfigStruct->CPHA & 0x01) << SPI_CR_CPHA_Pos)   // CPHA
	|((SPI_ConfigStruct->CPOL & 0x01) << SPI_CR_CPOL_Pos)   // CPOL
	|((SPI_ConfigStruct->Databit & SPI_DS_BITMASK)<< SPI_CR_BITSZ_Pos)   // BITSZ    0:8bit, 1:9bit, 2:16bit, 3:17bit
		;
	SPIx->CR = tmp;
	
	if (SPI_ConfigStruct->Mode == SPI_MASTER_MODE){
		SPIx->BR = (SPI_ConfigStruct->BaudRate & 0xFFFF); // PCLK / (BR + 1) ex) if BR=31, 32Mhz / (31+1)
	}
	else { // SPI_SLAVE_MODE   if using slave, BR has fast clock set and LR has minimum set. 
		SPIx->BR = 3; // PCLK / (BR + 1) ex) if BR=2 and PCLK=32Mhz,  32Mhz / (3 +1)	
	}
	
	SPIx->LR = 0
	|(1 << SPI_LR_SPL_Pos)  // SPL >= 1 
	|(1 << SPI_LR_BTL_Pos)   // BTL >= 1
	|(1 << SPI_LR_STL_Pos)   // STL >= 1
		;		
}

/***************************************************************************//**
 * @brief		De-initializes the SPIx peripheral registers to their
 *              default reset values.
 * @param[in]	SPIx SPI peripheral selected, should be:
 * 				 	- SP	:SPI0~2 peripheral
 * @return 		None
*******************************************************************************/
void SPI_DeInit(SPI_Type* SPIx)
{
	/* Disable SPI operation*/
	SPIx->SR = 0; // register clear 

	if(SPIx == SPI20) {
		/* SPI0 enable */
		SCU->PER2 &= ~(SCU_PER2_SPI20_Msk);
		SCU->PCER2 &= ~(SCU_PCER2_SPI20_Msk);
	} 
	else if(SPIx == SPI21) {
		/* SPI1 enable */
		SCU->PER2 &= ~(SCU_PER2_SPI21_Msk);
		SCU->PCER2 &= ~(SCU_PCER2_SPI21_Msk);
	}
}

/***************************************************************************//**
 * @brief		configure SPI delay length 
 * @param[in]	SPIx	SPI peripheral, should be:
 * 				 	- SP	:SPI20~21 peripheral
 * @param[in]	StartDelay : SPL >= 1 (1~255)
 * @param[in]	BurstDelay : BTL >= 1 (1~255)
 * @param[in]	StopDelay : STL >= 1 (1~255)
 * @return 		none
*******************************************************************************/
void SPI_DelayConfig(SPI_Type* SPIx, uint8_t StartDelay, uint8_t BurstDelay,uint8_t StopDelay)
{
	if (SPIx->CR & (SPI_MASTER_MODE << SPI_CR_MS_Pos)){
		SPIx->LR = 0
		|((StopDelay & 0xFF) << SPI_LR_SPL_Pos)  // SPL >= 1 (1~255) 
		|((BurstDelay & 0xFF) << SPI_LR_BTL_Pos)   // BTL >= 1 (1~255)
		|((StartDelay & 0xFF) << SPI_LR_STL_Pos)   // STL >= 1 (1~255)
			;		
	}
	else {
		SPIx->LR = 0
		|(1 << SPI_LR_SPL_Pos)  // SPL >= 1 
		|(1 << SPI_LR_BTL_Pos)   // BTL >= 1
		|(1 << SPI_LR_STL_Pos)   // STL >= 1
			;
	}
}

/***************************************************************************//**
 * @brief		Enable or disable SPI peripheral's operation
 * @param[in]	SPIx	SPI peripheral, should be:
 * 				 	- SP	:SPI20~21 peripheral
 * @param[in]	NewState New State of SPIx peripheral's operation, should be:
 * 					- ENABLE
 * 					- DISABLE
 * @return 		none
*******************************************************************************/
void SPI_Cmd(SPI_Type* SPIx, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		SPIx->EN = (SPI_EN_ENABLE_Msk);
	} else {
		SPIx->EN &= ~(SPI_EN_ENABLE_Msk);
	}
}

/***************************************************************************//**
 * @brief		Enable or disable Loop Back mode function in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI20~21 peripheral
 * @param[in]	NewState	New State of Loop Back mode, should be:
 * 					- ENABLE
 * 					- DISABLE
 * @return 		None
*******************************************************************************/
void SPI_LoopBackCmd(SPI_Type* SPIx, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		SPIx->CR |= (SPI_CR_LBE_Msk);
	} else {
		SPIx->CR &= ~(SPI_CR_LBE_Msk);
	}
}

/***************************************************************************//**
 * @brief		control SS Output function in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI20~21 peripheral
 * @param[in]	State	State of Slave Output function control, should be:
 * 					- SS_AUTO	: SS is controlled automatically
 * 					- SS_MANUAL	:SS is controlled by SSOUT 12bit in CR
 * @return 		None
*******************************************************************************/
void SPI_SSOutputCmd(SPI_Type* SPIx, uint32_t State)
{
	if (State == SS_AUTO) {
		SPIx->CR &= (~SPI_SSMOD_MANUAL);
	} else {
		SPIx->CR |= SPI_SSMOD_MANUAL;
	}
}

/***************************************************************************//**
 * @brief		control SS Output in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI20~21 peripheral
 * @param[in]	State	 State of Slave Output , should be:
 * 					- SS_OUT_LO : SS out value is Low
 * 					- SS_OUT_HI  : SS out value is High
 *  use this function after setting as SPI_SSOutputCmd(SPIx,SS_MANUAL)
 * @return 		None
*******************************************************************************/
void SPI_SSOutput(SPI_Type* SPIx, uint32_t State)
{
	if (State == SS_OUT_LO) {
		SPIx->CR &= ~(SPI_SSOUT_HI);
	} else {
		SPIx->CR |= SPI_SSOUT_HI;
	}
}

/***************************************************************************//**
 * @brief		Transmit a single data through SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI20~21 peripheral
 * @param[in]	Data	Data to transmit 
 * @return 		none
*******************************************************************************/
void SPI_SendData(SPI_Type* SPIx, uint32_t Data)
{
	SPIx->TDR = (Data);
}

/***************************************************************************//**
 * @brief		Receive a single data from SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be
 * 				 	- SP	:SPI20~21 peripheral
 * @return 		Data received 
*******************************************************************************/
uint32_t SPI_ReceiveData(SPI_Type* SPIx)
{
	return ((uint32_t) (SPIx->RDR));
}

/***************************************************************************//**
 * @brief		Checks whether the specified SPI status flag is set or not
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI20~21 peripheral
 * @return		SPI status flag
*******************************************************************************/
uint32_t SPI_GetStatus(SPI_Type* SPIx)
{
	return (SPIx->SR);
}

/***************************************************************************//**
 * @brief		Clear specified pending in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI20~21 peripheral
 * @param[in]	Type	Interrupt pending to clear
 * 					- SPI_STAT_TXDMA_DONE 		((uint32_t)(9<<0))
 * 					- SPI_STAT_RXDMA_DONE  		((uint32_t)(8<<0))
 * 					- SPI_STAT_SS_DET 			((uint32_t)(6<<0))
 * 					- SPI_STAT_SS_ACT  			((uint32_t)(5<<0))
 * 					- SPI_STAT_TXUNDERRUN_ERR 	((uint32_t)(4<<0))
 * 					- SPI_STAT_RXOVERRUN_ERR 	((uint32_t)(3<<0))
 * @return		None
*******************************************************************************/
void SPI_ClearPending(SPI_Type* SPIx, uint32_t Type)
{
	SPIx->SR = (~Type);
}

/***************************************************************************//**
 * @brief		Enable or disable specified interrupt type in SPI peripheral
 * @param[in]	SPIx	SPI peripheral selected, should be:
 * 				 	- SP	:SPI20~21 peripheral
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
*******************************************************************************/
void SPI_IntConfig(SPI_Type* SPIx, uint32_t IntType, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		SPIx->CR |= IntType;
	} else {
		SPIx->CR &= (~IntType);
	}
}

/***************************************************************************//**
* @brief      Get control register from SPI peripheral
* @param      SPIx	SPI peripheral selected, should be
*              - SPI20	:SPI20 peripheral
*              - SP121	:SPI21 peripheral
* @return     None
*******************************************************************************/
uint32_t SPI_GetControl(SPI_Type* SPIx)
{
	return ((uint32_t) (SPIx->CR));
}

/***************************************************************************//**
* @brief     Initializes SPI2n Channel
* @param      SPIx	SPI peripheral selected, should be
*              - SPI20	:SPI20 peripheral
*              - SP121	:SPI21 peripheral
* @return     None
*******************************************************************************/
void SPI_PortInit(SPI_Type* SPIx)
{
	if (SPIx == SPI20) {
		// SPI20 PIN FUNCTION SETTING
		// SS20 (PD0)
		PCU_ConfigureDirection(PD, 0, PCU_MODE_ALT_FUNC);
		PCU_ConfigureFunction(PD, 0, PCU_ALT_FUNCTION_2);
		PCU_ConfigurePullupdown(PD, 0, PCU_PUPD_DISABLE);

		// SCK20 (PC4)   
		PCU_ConfigureDirection(PC, 4, PCU_MODE_ALT_FUNC);
		PCU_ConfigureFunction(PC, 4, PCU_ALT_FUNCTION_2);
		PCU_ConfigurePullupdown(PC, 4, PCU_PUPD_DISABLE);
		
		// MOSI20 (PC2)
		PCU_ConfigureDirection(PC, 2, PCU_MODE_ALT_FUNC);
		PCU_ConfigureFunction(PC, 2, PCU_ALT_FUNCTION_2);
		PCU_ConfigurePullupdown(PC, 2, PCU_PUPD_DISABLE);

		// MISO20 (PC3)	
		PCU_ConfigureDirection(PC, 3, PCU_MODE_ALT_FUNC);
		PCU_ConfigureFunction(PC, 3, PCU_ALT_FUNCTION_2);
		PCU_ConfigurePullupdown(PC, 3, PCU_PUPD_PULL_UP);
		
	} else if (SPIx == SPI21) {
		// SPI21 PIN FUNCTION SETTING
		// SS21 (PE2)
		PCU_ConfigureDirection(PE, 2, PCU_MODE_ALT_FUNC);
		PCU_ConfigureFunction(PE, 2, PCU_ALT_FUNCTION_2);
		PCU_ConfigurePullupdown(PE, 2, PCU_PUPD_DISABLE);

		// SCK21 (PE3)   
		PCU_ConfigureDirection(PE, 3, PCU_MODE_ALT_FUNC);
		PCU_ConfigureFunction(PE, 3, PCU_ALT_FUNCTION_2);
		PCU_ConfigurePullupdown(PE, 3, PCU_PUPD_DISABLE);

		// MOSI21 (PE5)
		PCU_ConfigureDirection(PE, 5, PCU_MODE_ALT_FUNC);
		PCU_ConfigureFunction(PE, 5, PCU_ALT_FUNCTION_2);
		PCU_ConfigurePullupdown(PE, 5, PCU_PUPD_DISABLE);

		// MISO21 (PE4)	
		PCU_ConfigureDirection(PE, 4, PCU_MODE_ALT_FUNC);
		PCU_ConfigureFunction(PE, 4, PCU_ALT_FUNCTION_2);
		PCU_ConfigurePullupdown(PE, 4, PCU_PUPD_PULL_UP);
	}
}

/***************************************************************************//**
* @brief      This function is activates the Re-map function and initializes the SPI channel with the re-mapped ports.
* @param      Remap
*              - REMAP_USART10 : Remaps USART10 pin to SPI20 channel.
*              - REMAP_USART11 : Remaps USART11 pin to SPI21 channel.
* @param      ENABLE
*              - ENABLE : ENABLE pin re-map function
*              - DISABLE : DISABLE pin re-map function
* @return     None
*******************************************************************************/
void SPI_PortRemap(SPI_REMAP_Type Remap, FunctionalState ENABLE)
{
	// Enable SPI2n Pin Remap
	if (ENABLE) {
		if (Remap == REMAP_USART10)	{
			// USART10 PIN FUNCTION SETTING
			// SS10 (PB3)
			PCU_ConfigureDirection(PB, 3, PCU_MODE_ALT_FUNC);
			PCU_ConfigureFunction(PB, 3, PCU_ALT_FUNCTION_2);
			PCU_ConfigurePullupdown(PB, 3, PCU_PUPD_DISABLE);

			// SCK10 (PB2)   
			PCU_ConfigureDirection(PB, 2, PCU_MODE_ALT_FUNC);
			PCU_ConfigureFunction(PB, 2, PCU_ALT_FUNCTION_2);
			PCU_ConfigurePullupdown(PB, 2, PCU_PUPD_DISABLE);
			
			// MOSI10 (PB0)
			PCU_ConfigureDirection(PB, 0, PCU_MODE_ALT_FUNC);
			PCU_ConfigureFunction(PB, 0, PCU_ALT_FUNCTION_2);
			PCU_ConfigurePullupdown(PB, 0, PCU_PUPD_DISABLE);

			// MISO10 (PB1)	
			PCU_ConfigureDirection(PB, 1, PCU_MODE_ALT_FUNC);
			PCU_ConfigureFunction(PB, 1, PCU_ALT_FUNCTION_2);
			PCU_ConfigurePullupdown(PB, 1, PCU_PUPD_PULL_UP);
			
			// Uses USART10 as SPI20
			PORT_ACCESS_EN();
			PCU1->SPI2PMR |= PCU1_SPI2PMR_SPI20_PRM_Msk;
			PORT_ACCESS_DIS();
			
		} else if (Remap == REMAP_USART11) {
			// USART11 PIN FUNCTION SETTING
			// SS11 (PD5)
			PCU_ConfigureDirection(PD, 5, PCU_MODE_ALT_FUNC);
			PCU_ConfigureFunction(PD, 5, PCU_ALT_FUNCTION_2);
			PCU_ConfigurePullupdown(PD, 5, PCU_PUPD_DISABLE);

			// SCK11 (PD4)   
			PCU_ConfigureDirection(PD, 4, PCU_MODE_ALT_FUNC);
			PCU_ConfigureFunction(PD, 4, PCU_ALT_FUNCTION_2);
			PCU_ConfigurePullupdown(PD, 4, PCU_PUPD_DISABLE);
			
			// MOSI11 (PD2)
			PCU_ConfigureDirection(PD, 2, PCU_MODE_ALT_FUNC);
			PCU_ConfigureFunction(PD, 2, PCU_ALT_FUNCTION_2);
			PCU_ConfigurePullupdown(PD, 2, PCU_PUPD_DISABLE);

			// MISO11 (PD3)	
			PCU_ConfigureDirection(PD, 3, PCU_MODE_ALT_FUNC);
			PCU_ConfigureFunction(PD, 3, PCU_ALT_FUNCTION_2);
			PCU_ConfigurePullupdown(PD, 3, PCU_PUPD_PULL_UP);
			
			// SPI21 --> USART11
			PORT_ACCESS_EN();
			PCU1->SPI2PMR |= PCU1_SPI2PMR_SPI21_PRM_Msk;
			PORT_ACCESS_DIS();
		}
	// Disable SPI2n Pin Remap
	} else {
		if (Remap == REMAP_USART10)	{
			SPI_PortInit(SPI20);
			
			PORT_ACCESS_EN();
			PCU1->SPI2PMR &= ~PCU1_SPI2PMR_SPI20_PRM_Msk;
			PORT_ACCESS_DIS();
		} else if (Remap == REMAP_USART11) {
			SPI_PortInit(SPI21);
			
			PORT_ACCESS_EN();
			PCU1->SPI2PMR &= ~PCU1_SPI2PMR_SPI21_PRM_Msk;
			PORT_ACCESS_DIS();
		}
	}
}

/* --------------------------------- End Of File ------------------------------ */

