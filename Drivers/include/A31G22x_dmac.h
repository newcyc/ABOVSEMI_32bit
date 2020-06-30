/***************************************************************************//**
* @file     A31G22x_dmac.h
* @brief    Contains all macro definitions and function prototypes support
*           for DMA Controller driver on A31G22x
* @author   AE Team, ABOV Semiconductor Co., Ltd.
* @version  V0.0.1
* @date     30. Jul. 2018
*
* Copyright(C) 2018, ABOV Semiconductor
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

#ifndef _A31G22x_DMAC_H_
#define _A31G22x_DMAC_H_

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
* Included File
*******************************************************************************/
#include "A31G22x.h"
#include "aa_types.h"


/*******************************************************************************
* Public Macro
*******************************************************************************/



/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief  DMAC peripheral enumerated definition
 */
typedef enum {
	DMAC_PERIPHERAL_IDLE 		= 0, 	/*!< Idle */
	DMAC_PERIPHERAL_UART0_RX, 		 	/*!< DMA of UART0 Rx */
	DMAC_PERIPHERAL_UART0_TX, 			/*!< DMA of UART0 Tx */
	DMAC_PERIPHERAL_UART1_RX, 			/*!< DMA of UART1 Rx */
	DMAC_PERIPHERAL_UART1_TX,			/*!< DMA of UART1 Tx */
	DMAC_PERIPHERAL_CRC, 				/*!< DMA of CRC (SRAM Only) */
	DMAC_PERIPHERAL_RESERVED0, 			/*!< Reserved */
	DMAC_PERIPHERAL_RESERVED1, 			/*!< Reserved */
	DMAC_PERIPHERAL_RESERVED2, 			/*!< Reserved */
	DMAC_PERIPHERAL_SPI20_RX, 			/*!< DMA of SPI20 Rx */
	DMAC_PERIPHERAL_SPI20_TX, 			/*!< DMA of SPI20 Tx */
	DMAC_PERIPHERAL_SPI21_RX, 			/*!< DMA of SPI21 Rx */
	DMAC_PERIPHERAL_SPI21_TX, 			/*!< DMA of SPI21 Tx */
	DMAC_PERIPHERAL_ADC, 				/*!< DMA of ADC */
	DMAC_PERIPHERAL_DAC,  				/*!< DMA of DAC */
	DMAC_PERIPHERAL_RESERVED3, 			/*!< Reserved */
	DMAC_PERIPHERAL_RESERVED4, 			/*!< Reserved */
	DMAC_PERIPHERAL_USART10_RX, 		/*!< DMA of USART10 Rx */
	DMAC_PERIPHERAL_USART10_TX, 		/*!< DMA of USART10 Rx */
	DMAC_PERIPHERAL_USART11_RX, 		/*!< DMA of USART11 Rx */
	DMAC_PERIPHERAL_USART11_TX, 		/*!< DMA of USART11 Rx */
	DMAC_PERIPHERAL_USART12_RX, 		/*!< DMA of USART12 Rx */
	DMAC_PERIPHERAL_USART12_TX, 		/*!< DMA of USART12 Rx */
	DMAC_PERIPHERAL_USART13_RX, 		/*!< DMA of USART13 Rx */
	DMAC_PERIPHERAL_USART13_TX, 		/*!< DMA of USART13 Rx */
} DMAC_PERIPHERAL_Type;

/**
 * @brief  DMAC transfer width enumerated definition
 */
typedef enum {
	DMAC_WIDTH_BYTE      = 0x00UL, /*!< Byte size transfer (8-bit) */
	DMAC_WIDTH_HALF_WORD = 0x01UL, /*!< Half word size transfer (16-bit) */
	DMAC_WIDTH_WORD      = 0x02UL  /*!< Word size transfer (32-bit) */
} DMAC_WIDTH_Type;

/**
 * @brief  DMAC transfer width enumerated definition
 */
typedef enum {
	DMAC_DIRECTION_TX = 0x00UL, /*!< From memory to peripheral */
	DMAC_DIRECTION_RX = 0x01UL  /*!< From peripheral to memory */
} DMAC_DIRECTION_Type;

/**
 * @brief  DMAC Channel configuration structure type definition
 */
typedef struct {
	DMAC_PERIPHERAL_Type Peripheral; /*!< Peripheral selection */
	DMAC_WIDTH_Type Width; /*!< Transfer width */
	DMAC_DIRECTION_Type Direction; /*!< Transfer direction */
	uint32_t PeripheralAddress; /*!< Peripheral address */
} DMAC_CFG_Type;

/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void DMAC_Init(DMAC_Type *pDMACx, DMAC_CFG_Type *pConfig);
void DMAC_DeInit(DMAC_Type *pDMACx);
uint32_t DMAC_GetTransferCount(DMAC_Type *pDMACx);
void DMAC_SetTransferCount(DMAC_Type *pDMACx, uint32_t Count);
uint32_t DMAC_GetMemoryAddress(DMAC_Type *pDMACx);
void DMAC_SetMemoryAddress(DMAC_Type *pDMACx, uint32_t Address);
FlagStatus DMAC_GetStatus(DMAC_Type *pDMACx);
void DMAC_Start(DMAC_Type *pDMACx);
void DMAC_Stop(DMAC_Type *pDMACx);
void DMAC_ClearBuffer(DMAC_Type *pDMACx);

#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_DMAC_H_ */
/* --------------------------------- End Of File ------------------------------ */
