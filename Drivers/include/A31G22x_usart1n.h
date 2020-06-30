/***************************************************************************//**
* @file     A31G22x_usart1n.h
* @brief    Contains all macro definitions and function prototypes support
*           for USART1n driver on A31G22x
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

#ifndef _A31G22x_USART1N_H_
#define _A31G22x_USART1N_H_

#ifdef __cplusplus
extern "C" {
#endif


//------------------------------- Includes ----------------------------
#include "A31G22x.h"
#include "aa_types.h"


/** USART time-out definitions in case of using Read() and Write function
 * with Blocking Flag mode
 */
#define USART_BLOCKING_TIMEOUT		(0xFFFFFFFFUL)


/* Private Macros ------------------------------------------------------------- */
/* --------------------- BIT DEFINITIONS -------------------------------------- */
/**********************************************************************
 * Macro defines for Macro defines for USART interrupt enable register
 **********************************************************************/
#define USART_IER_WAKEINT_EN		(0x01UL << 2) /*!< WAKE Interrupt enable*/
#define USART_IER_RXCINT_EN			(0x01UL << 3) /*!< RXC Interrupt enable*/
#define USART_IER_TXCINT_EN			(0x01UL << 4) /*!< TXC interrupt enable*/
#define USART_IER_DRE_EN			(0x01UL << 5) /*!< DRE interrupt enable */
#define USART_IER_BITMASK			(0x3CUL) /*!< USART interrupt enable register bit mask */

/**********************************************************************
 * Macro defines for Macro defines for USART interrupt status register
 **********************************************************************/
#define USART_ST_DMATXF				(0x01UL << 9) /*!<Interrupt identification: DMA Tx Complete */
#define USART_ST_DMARXF				(0x01UL << 8) /*!<Interrupt identification: DMA Rx Complete */
#define USART_ST_DRE				(0x01UL << 7) /*!<Interrupt identification: Tx Buffer Busy*/
#define USART_ST_TXC				(0x01UL << 6) /*!<Interrupt identification: Tx Complete*/
#define USART_ST_RXC				(0x01UL << 5) /*!<Interrupt identification: Rx Complete*/
#define USART_ST_WAKE				(0x01UL << 4) /*!<Interrupt identification: Wake*/
#define USART_ST_RTOF				(0x01UL << 3) /*!<Interrupt identification: Receive Time-out */
#define USART_ST_DOR				(0x01UL << 2) /*!<Interrupt identification: Data OverRun */
#define USART_ST_FE					(0x01UL << 1) /*!<Interrupt identification: Frame Error */
#define USART_ST_PE					(0x01UL << 0) /*!<Interrupt identification: Parity Error */
#define USART_ST_BITMASK			(0x3FFUL)

#define USART_CR2_USTRX8			(0x01UL << 0)
#define USART_CR2_USTTX8			(0x01UL << 1)
#define USART_CR2_USTSB				(0x01UL << 2)
#define USART_CR2_FXCH				(0x01UL << 3)
#define USART_CR2_USTSSEN			(0x01UL << 4)
#define USART_CR2_DISSCK			(0x01UL << 5)
#define USART_CR2_LOOPS				(0x01UL << 6)
#define USART_CR2_MASTER			(0x01UL << 7)
#define USART_CR2_DBLS				(0x01UL << 8)
#define USART_CR2_USTEN				(0x01UL << 9)
#define USART_CR2_BFREN				(0x01UL << 10)
#define USART_CR2_RTOEN				(0x01UL << 11)
#define USART_CR2_RTOIE				(0x01UL << 12)
#define USART_CR2_DMARXIE			(0x01UL << 13)
#define USART_CR2_DMATXIE			(0x01UL << 14)
#define USART_CR2_BITMASK			(0x7FF)

#define USART_RTO_RTOMASK			(0xFFFFFF)

#define USART_IP_INDEX_MAX			(4)

/***********************************************************************
 * @brief USART enumeration
**********************************************************************/
/**
 * @brief USART Databit type definitions
 */
typedef enum {
	USART_UART_MODE = 0,			/*!< UART mode */
	USART_SYNC_MODE,				/*!< Syncronous Mode mode*/
	USART_SPI_MODE = 3				/*!< USART 9 bit data mode */
} USART_OPMODE_Type;

typedef enum {
	USART_SPI_LSB = 0,				/*!< SPI LSB First */
	USART_SPI_MSB,					/*!< SPI MSB First */
} USART_SPI_ORDER_Type;

typedef enum {
	USART_SPI_TX_RISING = 0,		/*!< Txd Change : Rising / Rxd Change : Falling */
	USART_SPI_TX_FALLING,			/*!< Txd Change : Falling / Rxd Change : Rising */
} USART_ACK_Type;

typedef enum {
	USART_SPI_TX_LEADEDGE_SAMPLE = 0,	/*!< Leading edge : Sample / Trailing edge : Setup */
	USART_SPI_TX_LEADEDGE_SETUP,		/*!< Leading edge : Setup / Trailing edge : Sample */
} USART_EDGE_Type;



typedef enum {
	USART_DATABIT_5 = 0,			/*!< USART 5 bit data mode */
	USART_DATABIT_6,				/*!< USART 6 bit data mode */
	USART_DATABIT_7,				/*!< USART 7 bit data mode */
	USART_DATABIT_8,				/*!< USART 8 bit data mode */
	USART_DATABIT_9 = 7				/*!< USART 9 bit data mode */
} USART_DATABIT_Type;

/**
 * @brief USART Stop bit type definitions
 */
typedef enum {
	USART_STOPBIT_1 = (0),			/*!< USART 1 Stop Bits Select */
	USART_STOPBIT_2					/*!< USART 2 Stop Bits Select */
} USART_STOPBIT_Type;

/**
 * @brief USART Parity type definitions
 */
typedef enum {
	USART_PARITY_NONE = 0,			/*!< No parity */
	USART_PARITY_EVEN = 2,			/*!< Even parity */
	USART_PARITY_ODD = 3			/*!< Odd parity */
} USART_PARITY_Type;

/*********************************************************************
* @brief USART Interrupt Type definitions
**********************************************************************/
/**
 * @brief USART Data Control type definition
 */
typedef enum {
	USART_CONTROL_USTRX8 = 0,
	USART_CONTROL_USTTX8,
	USART_CONTROL_USTSB,
	USART_CONTROL_FXCH,
	USART_CONTROL_USTSSEN,
	USART_CONTROL_DISSCK,
	USART_CONTROL_LOOPS,
	USART_CONTROL_MASTER,
	USART_CONTROL_DBLS,
	USART_CONTROL_USTEN
} USART_CONTROL_Type;

typedef enum {
	USART_STATUS_PE = 0,
	USART_STATUS_FE,
	USART_STATUS_DOR,
	USART_STATUS_RTOF,
	USART_STATUS_WAKE,
	USART_STATUS_RXC,
	USART_STATUS_TXC,
	USART_STATUS_DRE,
	USART_STATUS_DMARXF,
	USART_STATUS_DMATXF
} USART_STATUS_Type;


typedef enum {
	USART_INTCFG_WAKE = 0,			/*!< Wake-up Interrupt enable*/
	USART_INTCFG_RXC,				/*!< Receive Complete Interrupt enable*/
	USART_INTCFG_TXC,				/*!< Transmit Complete line status interrupt enable*/
	USART_INTCFG_DRE,				/*!< Data Register Empty interrupt */
	USART_INTCFG_RTO,				/*!< Receive Time-out interrupt */
} USART_INT_Type;



typedef struct {
  uint32_t Baud_rate;				/**< USART baud rate */
  USART_OPMODE_Type Mode;
  USART_SPI_ORDER_Type Order;
  USART_ACK_Type ACK;
  USART_EDGE_Type Edge;
  USART_DATABIT_Type Databits;		/**< Number of data bits, should be:
									 - USART_DATABIT_5: USART 5 bit data mode
									 - USART_DATABIT_6: USART 6 bit data mode
									 - USART_DATABIT_7: USART 7 bit data mode
									 - USART_DATABIT_8: USART 8 bit data mode
									 - USART_DATABIT_9: USART 9 bit data mode
									*/
  USART_PARITY_Type Parity;			/**< Parity selection, should be:
									 - USART_PARITY_NONE: No parity
									 - USART_PARITY_ODD: Odd parity
									 - USART_PARITY_EVEN: Even parity
									 - USART_PARITY_SP_1: Forced "1" stick parity
									 - USART_PARITY_SP_0: Forced "0" stick parity
									*/
  USART_STOPBIT_Type Stopbits;		/**< Number of stop bits, should be:
									 - USART_STOPBIT_1: USART 1 Stop Bits Select
									 - USART_STOPBIT_2: USART 2 Stop Bits Select
									*/
} USART_CFG_Type;


void USART_Init(USART_Type *USTx, USART_CFG_Type *USART_ConfigStruct);
void USART_DeInit(USART_Type* USTx);
void USART_UART_Mode_ConfigStructInit(USART_CFG_Type *USART_InitStruct);
void USART_USRT_Mode_ConfigStructInit(USART_CFG_Type *USART_InitStruct);
void USART_SPI_Mode_ConfigStructInit(USART_CFG_Type *USART_InitStruct);
void USART_SendByte(USART_Type* USTx, uint8_t Data);
uint8_t USART_ReceiveByte(USART_Type* USTx);

uint32_t USART_Send(USART_Type *USTx, uint8_t *txbuf, uint32_t buflen);
uint32_t USART_Receive(USART_Type *USTx, uint8_t *rxbuf, uint32_t buflen);

void USART_IntConfig(USART_Type *USTx, USART_INT_Type USTIntCfg, FunctionalState NewState);
uint16_t USART_GetStatus(USART_Type* USTx);
void USART_ClearStatus(USART_Type* USTx, USART_STATUS_Type Status);
FlagStatus USART_CheckBusy(USART_Type *USTx);
void USART_DataControlConfig(USART_Type *USTx, USART_CONTROL_Type Mode, FunctionalState NewState);
void USART_Enable(USART_Type* USTx, FunctionalState state);
void USART_ReceiveTimeOut(USART_Type* USTx, FunctionalState State, FunctionalState Intr, uint32_t TimeOut);
void USART_DmaConfig(USART_Type* USTx, uint32_t DmaInterrupType, FunctionalState en);

#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_USART1N_H_ */
/* --------------------------------- End Of File ------------------------------ */
