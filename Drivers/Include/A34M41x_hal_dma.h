/**********************************************************************
* @file		A34M41x_dma.h
* @brief	Contains all macro definitions and function prototypes
* 			support for DMA firmware library on A34M41x
* @version	1.0
* @date		
* @author ABOV Application2 team
*
* Copyright(C)  2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_DMA_H_
#define _A34M41x_DMA_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	uint32_t	transcnt;
	uint32_t	perisel;
	uint32_t	bussize;
	uint32_t	dirsel;
} DMA_CFG_Type;

extern enum
{
	CHANNEL_IDLE = 0,
	UART0_RX,
	UART0_TX,
	UART1_RX,
	UART1_TX,
	UART2_RX,
	UART2_TX,
	UART3_RX,
	UART3_TX,
	UART4_RX,
	UART4_TX,
	UART5_RX,
	UART5_TX,
	SPI0_RX,
	SPI0_TX,
	SPI1_RX,
	SPI1_TX,
	SPI2_RX,
	SPI2_TX,
	ADC0_RX,
	ADC1_RX,
	ADC2_RX,
	AES128_RX,
	AES128_TX,
	CRC_TX
} DMA_PERISEL;



/* Public Macros -------------------------------------------------------------- */
/*********************************************************************//**
 * DMA Control	defines
 **********************************************************************/
#define DMA_CR_BYTE_TRANS			(0<<2)
#define DMA_CR_HWORD_TRANS			(1<<2)
#define DMA_CR_WORD_TRANS			(2<<2)

#define DMA_CR_DIR_MEM_TO_PERI		(0<<1)
#define DMA_CR_DIR_PERI_TO_MEM		(1<<1)


/*********************************************************************//**
 * DMA Control	defines
 **********************************************************************/
#define DMA_SR_EOT					(1<<7)



/* Public Functions ----------------------------------------------------------- */
void HAL_DMA_Init(void);
HAL_Status_Type HAL_DMA_Cmd(DMA_Type* DMAx, DMA_CFG_Type *dma_cfg);
HAL_Status_Type HAL_DMA_Start(DMA_Type* DMAx);
HAL_Status_Type HAL_DMA_Stop(DMA_Type* DMAx);
uint32_t HAL_DMA_GetStatus(DMA_Type* DMAx);
HAL_Status_Type HAL_DMA_SetPAR(DMA_Type* DMAx, uint32_t peri_addr);
HAL_Status_Type HAL_DMA_SetMAR(DMA_Type* DMAx, uint32_t mem_addr);


#ifdef __cplusplus
}
#endif


#endif /* _A34M41x_DMA_H_ */

/* --------------------------------- End Of File ------------------------------ */
