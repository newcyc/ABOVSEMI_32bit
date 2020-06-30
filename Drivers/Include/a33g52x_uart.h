/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_uart.h
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
*
* @ Description 
*   ABOV Semiconductor is supplying this software for use with A33G52x
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

#ifndef __UART_H__
#define __UART_H__

#include "A33G52x.h"
#include <stdint.h>




//=======================================================================
// D A T A    S T R U C T U R E
//=======================================================================
typedef struct {
	uint32_t 	Baudrate;
	uint8_t		DataBits;
	uint8_t		Invert; 
	uint8_t		StopBits;
	uint8_t		Parity;
	uint32_t		Reference_clock; 
} UART_CONFIG; 	



#define UART_NO_PARITY					(0)
#define UART_ODD_PARITY				(1)
#define UART_EVEN_PARITY				(2)




//=======================================================================
// Definitions
//=======================================================================
#define N_UART							(4)






//==========================================================================
// 	UnRBR, UnTHR, UnDLL
//		
//				UART0			@ address = 0x4000_0B00
//				UART1			@ address = 0x4000_0B40
//				UART2			@ address = 0x4000_0B80
//				UART3			@ address = 0x4000_0BC0
//
//==========================================================================







//==========================================================================
// 	UnIER, UnDLM
//		
//				UART0			@ address = 0x4000_0B04
//				UART1			@ address = 0x4000_0B44
//				UART2			@ address = 0x4000_0B84
//				UART3			@ address = 0x4000_0BC4
//
//==========================================================================
#define UnIER_TEMTIE					(0x0001UL<<3)
#define UnIER_RLSIE						(0x0001UL<<2)
#define UnIER_THREIE					(0x0001UL<<1)
#define UnIER_DRIE						(0x0001UL<<0)

#define UnIER_INTR_MASK				(0x000FUL<<0)



//==========================================================================
// 	UnIIR (RO)
//	
//				UART0			@ address = 0x4000_0B08
//				UART1			@ address = 0x4000_0B48
//				UART2			@ address = 0x4000_0B88
//				UART3			@ address = 0x4000_0BC8
//
//==========================================================================
#define UnIIR_FIFO_NON_FIFO_MODE		(0x0000UL<<6)
#define UnIIR_FIFO_FIFO_MODE			(0x0003UL<<6)
#define UnIIR_FIFO_MASK					(0x0003UL<<6)

#define UnIIR_XMITE 						(0x0001UL<<4)

#define UnIIR_FID						(0x0001UL<<3)

#define UnIIR_IID_THR_EMPTY			(0x0001UL<<1)
#define UnIIR_IID_RBR_READY			(0x0002UL<<1)
#define UnIIR_IID_RCV_LINE_STATUS		(0x0003UL<<1)

#define UnIIR_IPEN_NO_PENDING			(0x0001UL<<0)
#define UnIIR_IPEN_PENDING				(0x0000UL<<0)

#define UnIIR_INTR_MASK 				(0x00CFUL<<0)
#define UnIIR_INTR_MASK_NO_FIFO		(0x0017UL<<0)
#define UnIIR_INTR_BASIC_MASK			(0x0007UL<<0)




//==========================================================================
// 	UnFCR (WO)
//	
//				UART0			@ address = 0x4000_0B08
//				UART1			@ address = 0x4000_0B48
//
//==========================================================================
#define UnFCR_FIFODEPTH_1_BYTE		(0x0000UL<<6)
#define UnFCR_FIFODEPTH_4_BYTES		(0x0001UL<<6)
#define UnFCR_FIFODEPTH_8_BYTES		(0x0002UL<<6)
#define UnFCR_FIFODEPTH_14_BYTES		(0x0003UL<<6)
#define UnFCR_FIFODEPTH_MASK			(0x0003UL<<6)

#define UnFCR_FCR2						(0x0001UL<<2)
#define UnFCR_FCR1						(0x0001UL<<1)
#define UnFCR_FIFOEN					(0x0001UL<<0)





//==========================================================================
// 	UnLCR
//	
//				UART0			@ address = 0x4000_0B0C
//				UART1			@ address = 0x4000_0B4C
//				UART2			@ address = 0x4000_0B8C
//				UART3			@ address = 0x4000_0BCC
//
//==========================================================================
#define UnLCR_DLAB						(0x0001UL<<7)
#define UnLCR_BREAK						(0x0001UL<<6)
#define UnLCR_STICKP					(0x0001UL<<5)
#define UnLCR_PARITY					(0x0001UL<<4)

#define UnLCR_PEN						(0x0001UL<<3)

#define UnLCR_STOPBIT					(0x0001UL<<2)
#define UnLCR_STOPBIT_1_STOPBIT		(0x0000UL<<2)
#define UnLCR_STOPBIT_2_STOPBITS		(0x0001UL<<2)

#define UnLCR_DLEN_5_BITS				(0x0000UL<<0)
#define UnLCR_DLEN_6_BITS				(0x0001UL<<0)
#define UnLCR_DLEN_7_BITS				(0x0002UL<<0)
#define UnLCR_DLEN_8_BITS				(0x0003UL<<0)
#define UnLCR_DLEN_MASK				(0x0003UL<<0)





//==========================================================================
// 	UnLSR
//	
//				UART0			@ address = 0x4000_0B14
//				UART1			@ address = 0x4000_0B54
//				UART2			@ address = 0x4000_0B94
//				UART3			@ address = 0x4000_0BD4
//
//==========================================================================
#define UnLSR_FIFOE						(0x0001UL<<7)
#define UnLSR_TEMT						(0x0001UL<<6)
#define UnLSR_THRE						(0x0001UL<<5)
#define UnLSR_BI							(0x0001UL<<4)

#define UnLSR_FE						(0x0001UL<<3)
#define UnLSR_PE						(0x0001UL<<2)
#define UnLSR_OE						(0x0001UL<<1)
#define UnLSR_DR						(0x0001UL<<0)




//=======================================================================
// MSR
//
//				UART0			@ address = 0x4000_0B18
//				UART1			@ address = 0x4000_0B58
//				UART2			@ address = 0x4000_0B98
//				UART3			@ address = 0x4000_0BD8
//
//=======================================================================





//=======================================================================
// SCR
//
//				UART0			@ address = 0x4000_0B1C
//				UART1			@ address = 0x4000_0B5C
//				UART2			@ address = 0x4000_0B9C
//				UART3			@ address = 0x4000_0BDC
//
//=======================================================================




//=======================================================================
// BFR
//
//				UART0			@ address = 0x4000_0B24
//				UART1			@ address = 0x4000_0B64
//				UART2			@ address = 0x4000_0BA4
//				UART3			@ address = 0x4000_0BE4
//
//=======================================================================


//=======================================================================
//DTR 
//
//				UART0			@ address = 0x4000_0B30
//				UART1			@ address = 0x4000_0B70
//				UART2			@ address = 0x4000_0BB0
//				UART3			@ address = 0x4000_0BF0
//
//=======================================================================
#define UnDTR_SMS 						(0x0001UL<<7)
#define UnDTR_DMS 						(0x0001UL<<6)
#define UnDTR_RXINV						(0x0001UL<<5)
#define UnDTR_TXINV 					(0x0001UL<<4)
#define UnDTR_WAITVAL(n)				(((n)&0x07UL)<<0)



//==================================================================
// definitions
//==================================================================
#define UART_MAX_RX_BUFFER					(100)
#define UART_MAX_TX_BUFFER					(100)

#define UART_CHANNEL_SUPPORTED				(1)
#define UART_CHANNEL_NOT_SUPPORTED			(0)




//------------------------------------------------------------------------------------
// definitions for TX/RX state  
//------------------------------------------------------------------------------------
#define UART_RX_STATE_IDLE					(0)
#define UART_RX_STATE_RECEIVE				(1)


#define UART_TX_STATE_IDLE					(0)
#define UART_TX_STATE_TRANSMIT				(1)


//------------------------------------------------------------------------------------
// definitions for read/write function 
//------------------------------------------------------------------------------------
#define UART_TX_BUFFER_SUCCESS				(0)
#define UART_TX_BUFFER_ERROR_WRONG_CHANNEL	(1)
#define UART_TX_BUFFER_ERROR_WAIT_TIMEOUT	(2)

#define UART_RX_BUFFER_SUCCESS				(0)
#define UART_RX_BUFFER_ERROR_WRONG_CHANNEL	(1)
#define UART_RX_BUFFER_ERROR_EMPTY			(2)


//------------------------------------------------------------------------------------
// definitions for UART interrupts
//------------------------------------------------------------------------------------
#define UART_INTR_RX						(UnIER_DRIE)
#define UART_INTR_TX						(UnIER_THREIE)
#define UART_INTR_LINE						(UnIER_RLSIE)
#define UART_INTR_TEMT						(UnIER_TEMTIE)





//==================================================================
// structures
//==================================================================
typedef struct {
	uint16_t		RxState; 
	uint16_t		TxState; 
	uint16_t		RxBuffer_HeadIndex;
	uint16_t 	RxBuffer_TailIndex;
	uint16_t		TxBuffer_HeadIndex;
	uint16_t 	TxBuffer_TailIndex;
	uint8_t		RxBuffer[UART_MAX_RX_BUFFER];
	uint8_t		TxBuffer[UART_MAX_TX_BUFFER]; 
} UART_BUFFER; 

//==========================================================================
// 
//		F U N C T I O N    D E C L A R A T I O N S 
//
//==========================================================================
void UART_ConfigureGPIO (UART_Type * UARTx); 
void UART_Init (UART_Type * UARTx, UART_CONFIG *config);
UART_Type * UART_Get_Object (int uart_no);
void UART_SetBaudrate (UART_Type * UARTx, uint32_t baudrate, uint32_t prescaler, uint32_t ref_clk); 
void UART_ConfigureInterrupt (UART_Type * UARTx, uint32_t intr_mask, uint32_t enable); 

void _UART_Init (int uart_no);
void UART_InitBuffer (int uart_no);
UART_BUFFER* UART_GetBufferBaseAddr (int uart_no, int *pResult);

uint8_t UART_WriteBuffer (int uart_no, uint8_t *p_data, uint32_t data_count); 
void UART_Enable_Tx_Interrupt(UART_Type * UARTx, uint8_t status);
int UART_GetChar (int uart_no); 
uint8_t UART_ReadBuffer (int uart_no, int *p_status); 
///////////////////////////////////////////////////////////////////////////////

void UART0_Transmit_Receive_ISR(void); 
void UART0_ReceiveData_ISR (void); 
void UART0_TransmitData_ISR (void); 

void UART1_Transmit_Receive_ISR(void); 
void UART1_ReceiveData_ISR (void); 
void UART1_TransmitData_ISR (void); 

void UART2_Transmit_Receive_ISR(void); 
void UART2_ReceiveData_ISR (void); 
void UART2_TransmitData_ISR (void); 

void UART3_Transmit_Receive_ISR(void); 
void UART3_ReceiveData_ISR (void); 
void UART3_TransmitData_ISR (void); 

#endif 

