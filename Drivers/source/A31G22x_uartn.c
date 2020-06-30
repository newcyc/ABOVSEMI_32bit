/***************************************************************************//**
* @file     A31G22x_uartn.c
* @brief    Contains all functions support for UARTn driver on A31G22x
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

/*******************************************************************************
* Included File
*******************************************************************************/
#include "A31G22x_scu.h"
#include "A31G22x_uartn.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER2_UART0_ENABLE_PERI			(0x01UL << SCU_PER2_UART0_Pos)
#define SCU_PCER2_UART0_ENABLE_CLOCK		(0x01UL << SCU_PCER2_UART0_Pos)

#define SCU_PER2_UART1_ENABLE_PERI			(0x01UL << SCU_PER2_UART1_Pos)
#define SCU_PCER2_UART1_ENABLE_CLOCK		(0x01UL << SCU_PCER2_UART1_Pos)

#define UART_IER_MASK_VALUE					(0x01UL)
#define UART_LSR_MASK_VALUE					(UART_LSR_TEMT_Msk | UART_LSR_THRE_Msk \
											| UART_LSR_BI_Msk | UART_LSR_FE_Msk \
											| UART_LSR_PE_Msk | UART_LSR_OE_Msk \
											| UART_LSR_DR_Msk)


/*******************************************************************************
* Private Typedef
*******************************************************************************/


/*******************************************************************************
* Private Variable
*******************************************************************************/


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/
static void UART_SetDivisors(UART_Type *pUARTx, uint32_t BaudRate);


/*******************************************************************************
* Public Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Initialize UART (Universal Asynchronous Receiver/Transmitter) peripheral
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      pConfig : Pointer contains configuration of UART
* @return     None
*******************************************************************************/
void UART_Init(UART_Type *pUARTx, UART_CFG_Type *pConfig)
{
	volatile uint32_t Reg32;

	if (pUARTx == UART0) {
		// Disable UART0 peripheral & clock
		SCU->PER2 &= ~SCU_PER2_UART0_Msk;
		SCU->PCER2 &= ~SCU_PCER2_UART0_Msk;

		// Enable UART0 peripheral & clock
		SCU->PER2 |= SCU_PER2_UART0_ENABLE_PERI;
		SCU->PCER2 |= SCU_PCER2_UART0_ENABLE_CLOCK;
	} else {
		// Disable UART1 peripheral & clock
		SCU->PER2 &= ~SCU_PER2_UART1_Msk;
		SCU->PCER2 &= ~SCU_PCER2_UART1_Msk;

		// Enable UART1 peripheral & clock
		SCU->PER2 |= SCU_PER2_UART1_ENABLE_PERI;
		SCU->PCER2 |= SCU_PCER2_UART1_ENABLE_CLOCK;
	}

	// Dummy reading
	while (pUARTx->LSR & UART_LSR_DR_Msk) {
		Reg32 = pUARTx->RBR;
	}

	// Wait for current transmit complete
	while (!(pUARTx->LSR & UART_LSR_THRE_Msk));

	// Disable interrupt
	pUARTx->IER = 0;

	// Set LCR, DCR to default state
	pUARTx->LCR = 0;
	pUARTx->DCR = 0;

	// Dummy reading
	Reg32 = pUARTx->LSR;
	Reg32 = pUARTx->IIR;

	// Set baud rate and fraction counter
	UART_SetDivisors(pUARTx, (pConfig->BaudRate));

	// Set line control register
	Reg32 = 0x00UL
		| ((0x00UL << UART_LCR_BREAK_Pos) & UART_LCR_BREAK_Msk)
		| ((pConfig->StopBit << UART_LCR_STOPBIT_Pos) & UART_LCR_STOPBIT_Msk)
		| ((pConfig->DataLength << UART_LCR_DLEN_Pos) & UART_LCR_DLEN_Msk)
		;

	if (pConfig->Parity != UART_PARITY_NONE) {
		if ((pConfig->Parity == UART_PARITY_SP_0) || (pConfig->Parity == UART_PARITY_SP_1)) {
			Reg32 |= (0x01UL << UART_LCR_STICKP_Pos);
		}

		if ((pConfig->Parity == UART_PARITY_EVEN) || (pConfig->Parity == UART_PARITY_SP_0)) {
			Reg32 |= (0x01UL << UART_LCR_PARITY_Pos);
		}

		Reg32 |= (0x01UL << UART_LCR_PEN_Pos);
	}

	pUARTx->LCR = Reg32;

	// Set inter-frame delay time register
	pUARTx->IDTR = 0x00UL
		| ((pConfig->SB_MultiSampling << UART_IDTR_SMS_Pos) & UART_IDTR_SMS_Msk)
		| ((pConfig->DB_MultiSampling << UART_IDTR_DMS_Pos) & UART_IDTR_DMS_Msk)
		| ((pConfig->WaitTime << UART_IDTR_WAITVAL_Pos) & UART_IDTR_WAITVAL_Msk)
		;
}

/***************************************************************************//**
* @brief      De-Initialize UART peripheral
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @return     None
*******************************************************************************/
void UART_DeInit(UART_Type* pUARTx)
{
	if (pUARTx == UART0) {
		// Disable UART0 peripheral & clock
		SCU->PER2 &= ~SCU_PER2_UART0_Msk;
		SCU->PCER2 &= ~SCU_PCER2_UART0_Msk;
	} else {
		// Disable UART1 peripheral & clock
		SCU->PER2 &= ~SCU_PER2_UART1_Msk;
		SCU->PCER2 &= ~SCU_PCER2_UART1_Msk;
	}
}

/***************************************************************************//**
* @brief      Get default configuration of UARTn
*              - 38400 bps
*              - None Parity
*              - 8-bit data length
*              - 1 Stop bit
*              - Disable start bit multi sampling
*              - Disable data bit multi sampling
*              - 0 wait time
*
* @param      pConfig : Pointer is initialized by default configuration of UART
* @return     None
*******************************************************************************/
void UART_GetDefaultConfig(UART_CFG_Type *pConfig)
{
	// Line control register
	pConfig->BaudRate = 38400;
	pConfig->Parity = UART_PARITY_NONE;
	pConfig->StopBit = UART_STOPBIT_1;
	pConfig->DataLength = UART_DATA_LENGTH_8;

	// Inter-frame delay time register
	pConfig->SB_MultiSampling = DISABLE;
	pConfig->DB_MultiSampling = DISABLE;
	pConfig->WaitTime = 0;
}

/***************************************************************************//**
* @brief      Configure Data Control mode for UART peripheral
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      pConfig : Pointer contains data control configuration of UART
* @return     None
*******************************************************************************/
void UART_ConfigureDataControl(UART_Type *pUARTx, UART_DATA_CFG_Type *pConfig)
{
	volatile uint32_t Reg32;

	Reg32 = 0x00UL
		| ((pConfig->LocalLoopback << UART_DCR_LBON_Pos) & UART_DCR_LBON_Msk)
		| ((pConfig->InverseReceive << UART_DCR_RXINV_Pos) & UART_DCR_RXINV_Msk)
		| ((pConfig->InverseTransmit << UART_DCR_TXINV_Pos) & UART_DCR_TXINV_Msk)
		;
}

/***************************************************************************//**
* @brief      Configure UART interrupt
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      InterruptSource : Interrupt source selected, should be
*              - UART_INTERRUPT_DRIE : Data Receive Interrupt
*              - UART_INTERRUPT_THREIE : Transmit Holding Register Empty Interrupt
*              - UART_INTERRUPT_RLSIE : Receiver Line Status Interrupt
*              - UART_INTERRUPT_TXEIE : Transmit Register Empty Interrupt
*              - UART_INTERRUPT_DRXIEN :  DMA Receive done Interrupt Enable
*              - UART_INTERRUPT_DTXIEN : DMA Transmit done Interrupt Enable
* @param      InterruptEnable :
*              - ENABLE : Enable interrupt
*              - DISABLE : Disable interrupt
* @return     None
*******************************************************************************/
void UART_ConfigureInterrupt(UART_Type *pUARTx, UART_INTERRUPT_Type InterruptSource, FunctionalState InterruptEnable)
{
	volatile uint32_t Reg32;

	Reg32 = pUARTx->IER;
	Reg32 &= ~(UART_IER_MASK_VALUE << InterruptSource);
	Reg32 |= (InterruptEnable << InterruptSource);
	pUARTx->IER = Reg32;
}

/***************************************************************************//**
* @brief      Set force BREAK character on UART line, output pin(pUARTx TXD) is
*             forced to logic 0.
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      BreakEnable :
*              - ENABLE : Enable interrupt
*              - DISABLE : Disable interrupt
* @return     None
*******************************************************************************/
void UART_ForceBreak(UART_Type* pUARTx, FunctionalState BreakEnable)
{
	volatile uint32_t Reg32;

	Reg32 = pUARTx->LCR;
	Reg32 &= ~UART_LCR_BREAK_Msk;
	Reg32 |= (BreakEnable << UART_LCR_BREAK_Pos);
	pUARTx->LCR = Reg32;
}

/***************************************************************************//**
* @brief      Get current value of Line Status register in UART peripheral  
*             Note) The return value of this function must be ANDed with each member in
*               UART_LS_Type enumeration to determine current flag status
*               corresponding to each Line status type. Because some flags in
*               Line Status register will be cleared after reading, the next reading
*               Line Status register could not be correct. So this function used to
*               read Line status register in one time only, then the return value
*               used to check all flags.
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @return     Current value of line status register in UART peripheral.
*******************************************************************************/
uint32_t UART_GetLineStatus(UART_Type* pUARTx)
{
	return (pUARTx->LSR & UART_LSR_MASK_VALUE);
}

/***************************************************************************//**
* @brief      Check whether if UART is busy or not
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @return     SET : UART is busy
*             RESET : UART is not busy
*******************************************************************************/
FlagStatus UART_CheckBusy(UART_Type *pUARTx)
{
	return ((pUARTx->LSR & UART_LSR_TEMT_Msk) == 0) ? SET : RESET;
}

/***************************************************************************//**
* @brief      Transmit a single data through UART peripheral
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      Data : Data to transmit (8-bit)
* @return     None
*******************************************************************************/
void UART_SendByte(UART_Type* pUARTx, uint8_t Data)
{
	pUARTx->THR = (uint32_t)Data;
}

/***************************************************************************//**
* @brief      Receive a single data from UART peripheral
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @return     Data received (8-bit)
*******************************************************************************/
uint8_t UART_ReceiveByte(UART_Type* pUARTx)
{
	return (uint8_t)(pUARTx->RBR);
}

/***************************************************************************//**
* @brief      Send a block of data via UART peripheral  
*              Note) when using UART in BLOCKING mode, a time-out condition is used
*              via defined symbol UART_BLOCKING_TIMEOUT.
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      pUARTx : UART peripheral selected, should be
* @param      pTxBuffer : Pointer to Tx buffer
* @param      BufferLength : Length of Tx buffer
* @param      Flag : Flag mode, should be
*              - NONE_BLOCKING
*              - BLOCKING
* @return     Number of bytes sent
*******************************************************************************/
uint32_t UART_Send(UART_Type *pUARTx, uint8_t *pTxBuffer, uint32_t BufferLength, TRANSFER_BLOCK_Type Flag)
{
	uint32_t TimeOut;
	uint32_t ByteCount;

	ByteCount = 0;
	while (BufferLength) {
		if (Flag == BLOCKING) {
			// Wait for THR empty with timeout
			TimeOut = UART_BLOCKING_TIMEOUT;
		} else {
			TimeOut = 0;
		}

		while ((pUARTx->LSR & UART_LSR_THRE_Msk) == 0) {
			if (TimeOut == 0) {
				return ByteCount;
			} else {
				TimeOut--;
			}
		}

		UART_SendByte(pUARTx, (*pTxBuffer++));
		BufferLength--;
		ByteCount++;
	}

	return ByteCount;
}

/***************************************************************************//**
* @brief      Receive a block of data via UART peripheral  
*              Note) When using UART in BLOCKING mode, a time-out condition is used
*              via defined symbol UART_BLOCKING_TIMEOUT.
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      pRxBuffer : Pointer to Rx buffer
* @param      BufferLength : Length of Rx buffer
* @param      Flag : Flag mode, should be
*              - NONE_BLOCKING
*              - BLOCKING
* @return     Number of bytes received
*******************************************************************************/
uint32_t UART_Receive(UART_Type *pUARTx, uint8_t *pRxBuffer, uint32_t BufferLength, TRANSFER_BLOCK_Type Flag)
{
	uint32_t TimeOut;
	uint32_t ByteCount;

	ByteCount = 0;
	while (BufferLength) {
		if (Flag == BLOCKING) {
			TimeOut = UART_BLOCKING_TIMEOUT;
		} else {
			TimeOut = 0;
		}

		while ((pUARTx->LSR & UART_LSR_DR_Msk) == 0) {
			if (TimeOut == 0) {
				return ByteCount;
			} else {
				TimeOut--;
			}
		}

		(*pRxBuffer++) = UART_ReceiveByte(pUARTx);
		BufferLength--;
		ByteCount++;
	}

	return ByteCount;
}


/*******************************************************************************
* Private Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Determines best dividers to get a target baud rate
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      BaudRate : Target baud rate.
* @return     None
*******************************************************************************/
static void UART_SetDivisors(UART_Type *pUARTx, uint32_t BaudRate)
{
	uint32_t BDR;
	uint32_t BFR;
	uint32_t Remainder;
	uint32_t Numerator;
	uint32_t Denominator;

	// bdr = (PCLK / 2) / (16 * baudrate)
	Numerator = SystemCoreClock / 2;
	Denominator = 16 * BaudRate;

	BDR = Numerator / Denominator;

	// Remainder = Numerator - BDR x Denominator
	// Fraction = Remainder / Denominator
	// BFR = INT(Fraction * 256) => (Remainder * 256) / Denominator
	Remainder = Numerator - (BDR * Denominator);
	BFR = (Remainder << 8) / Denominator;

	pUARTx->BDR = (BDR & UART_BDR_BDR_Msk);
	pUARTx->BFR = (BFR & UART_BFR_BFR_Msk);
}

/* --------------------------------- End Of File ------------------------------ */
