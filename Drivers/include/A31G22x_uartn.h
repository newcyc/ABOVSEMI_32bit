/***************************************************************************//**
* @file     A31G22x_uartn.h
* @brief    Contains all macro definitions and function prototypes support
*           for UARTn driver on A31G22x
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

#ifndef _A31G22x_UART_H_
#define _A31G22x_UART_H_

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
/** UART time-out definitions in case of using Read() and Write function
 * with Blocking Flag mode
 */
#define UART_BLOCKING_TIMEOUT		(0xFFFFFFFFUL)

#define UART_IIR_IID_RLS			((0x03UL << UART_IIR_IID_Pos)) /*!<Interrupt identification: Receive line status*/
#define UART_IIR_IID_RDA			((0x02UL << UART_IIR_IID_Pos)) /*!<Interrupt identification: Receive data available*/
#define UART_IIR_IID_THRE			((0x01UL << UART_IIR_IID_Pos)) /*!<Interrupt identification: THRE interrupt*/


/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief UART Parity selection enumerated definitions
 */
typedef enum {
	UART_PARITY_NONE, /*!< No parity */
	UART_PARITY_ODD,  /*!< Odd parity */
	UART_PARITY_EVEN, /*!< Even parity */
	UART_PARITY_SP_1, /*!< Forced "1" stick parity */
	UART_PARITY_SP_0  /*!< Forced "0" stick parity */
} UART_PARITY_Type;

/**
 * @brief UART Stop bit selection enumerated definitions
 */
typedef enum {
	UART_STOPBIT_1 = 0x00, /*!< UART 1 Stop bit */
	UART_STOPBIT_2 = 0x01  /*!< UART 1.5 or 2 Stop bit */
} UART_STOPBIT_Type;

/**
 * @brief UART Data Length selection enumerated definitions
 */
typedef enum {
	UART_DATA_LENGTH_5 = 0x00, /*!< 5-bit data length */
	UART_DATA_LENGTH_6 = 0x01, /*!< 6-bit data length */
	UART_DATA_LENGTH_7 = 0x02, /*!< 7-bit data length */
	UART_DATA_LENGTH_8 = 0x03  /*!< 8-bit data length */
} UART_DATA_LENGTH_Type;

/**
 * @brief  Interrupt source enumerated definition
 */
typedef enum {
	UART_INTERRUPT_DRIE = UART_IER_DRIE_Pos,     /*!< Data Receive Interrupt */
	UART_INTERRUPT_THREIE = UART_IER_THREIE_Pos, /*!< Transmit Holding Register Empty Interrupt */
	UART_INTERRUPT_RLSIE = UART_IER_RLSIE_Pos,   /*!< Receiver Line Status Interrupt */
	UART_INTERRUPT_TXEIE = UART_IER_TXEIE_Pos,   /*!< Transmit Register Empty Interrupt */
	UART_INTERRUPT_DRXIEN = UART_IER_DRXIEN_Pos, /*!< DMA Receive done Interrupt Enable */
	UART_INTERRUPT_DTXIEN = UART_IER_DTXIEN_Pos  /*!< DMA Transmit done Interrupt Enable */
} UART_INTERRUPT_Type;


/**
 * @brief  UART configuration structure definition
 */
typedef struct {
	uint32_t BaudRate; /*!< UART baud rate */
	/* LCR */
	UART_PARITY_Type Parity; /*!< Parity selection */
	UART_STOPBIT_Type StopBit; /*!< stop bits */
	UART_DATA_LENGTH_Type DataLength; /*!< Data Length */
	/* IDTR */
	FunctionalState SB_MultiSampling; /*!< Start bit multi sampling */
	FunctionalState DB_MultiSampling; /*!< Data bit multi sampling */
	uint32_t WaitTime; /*!< Wait time value between 2 continuous transmits. 0 ~ 7 */
} UART_CFG_Type;

/**
 * @brief  UART data control configuration structure definition
 */
typedef struct {
	/* DCR */
	FunctionalState LocalLoopback; /*!< Local Loopback Test Mode */
	FunctionalState InverseReceive; /*!< Receive Data Inversion */
	FunctionalState InverseTransmit; /*!< Transmit Data Inversion */
} UART_DATA_CFG_Type;


/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void UART_Init(UART_Type *pUARTx, UART_CFG_Type *pConfig);
void UART_DeInit(UART_Type* pUARTx);
void UART_GetDefaultConfig(UART_CFG_Type *pConfig);
void UART_ConfigureDataControl(UART_Type *pUARTx, UART_DATA_CFG_Type *pConfig);
void UART_ConfigureInterrupt(UART_Type *pUARTx, UART_INTERRUPT_Type InterruptSource, FunctionalState InterruptEnable);
void UART_ForceBreak(UART_Type* pUARTx, FunctionalState BreakEnable);
uint32_t UART_GetLineStatus(UART_Type* pUARTx);
FlagStatus UART_CheckBusy(UART_Type *pUARTx);
void UART_SendByte(UART_Type* pUARTx, uint8_t Data);
uint8_t UART_ReceiveByte(UART_Type* pUARTx);
uint32_t UART_Send(UART_Type *pUARTx, uint8_t *pTxBuffer, uint32_t BufferLength, TRANSFER_BLOCK_Type Flag);
uint32_t UART_Receive(UART_Type *pUARTx, uint8_t *pRxBuffer, uint32_t BufferLength, TRANSFER_BLOCK_Type Flag);


#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_UART_H_ */
/* --------------------------------- End Of File ------------------------------ */
