/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_uart.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : July, 2017
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


#include "A33G52x.h"
#include "a33g52x_uart.h"
#include "a33g52x_pcu.h"
#include "a33g52x_nvic.h"
#include "system_A33G52x.h"

#define USE_BFR
//#define USE_TEMT_INTR

//#define USE_BAUDRATE_1200
//#define USE_BAUDRATE_2400
//#define USE_BAUDRATE_4800
//#define USE_BAUDRATE_9600

//#define USE_BAUDRATE_19200
#define USE_BAUDRATE_38400
//#define USE_BAUDRATE_57600
//#define USE_BAUDRATE_115200

//#define USE_BAUDRATE_230400
//#define USE_BAUDRATE_460800
//#define USE_BAUDRATE_921600


///////////////	structure ///////////////	
UART_BUFFER		sUart0Buffer; 
UART_BUFFER		sUart1Buffer; 
UART_BUFFER		sUart2Buffer; 
UART_BUFFER		sUart3Buffer; 


/**
*********************************************************************************************************
* @ Name : UART_ConfigureGPIO 
*
* @ Parameters
*		- uart : UART0 ~ UART3
*
*
*********************************************************************************************************
*/
void UART_ConfigureGPIO (UART_Type * UARTx)
{

	if (UARTx == UART0)
	{
		//------------------------------------------------------------------------------------
		// UART0 
		//
		//				PC8				RXD0
		//				PC9				TXD0
		//
		//				PCMR			@ address = 0x4000_0240
		//				PCCR			@ address = 0x4000_0244
		//				PCPCR			@ address = 0x4000_0248
		//
		//------------------------------------------------------------------------------------
		PCU_ConfigureFunction (PCC, PIN_8, PC8_MUX_RXD0);
		PCU_ConfigureFunction (PCC, PIN_9, PC9_MUX_TXD0); 

		PCU_Set_Direction_Type (PCC, PIN_8, PnCR_INPUT_LOGIC); 
		PCU_Set_Direction_Type (PCC, PIN_9, PnCR_OUTPUT_PUSH_PULL); 

		PCU_ConfigurePullup_Pulldown (PCC, PIN_8, 0, PnPCR_PULLUPDOWN_DISABLE); 
		PCU_ConfigurePullup_Pulldown (PCC, PIN_9, 0, PnPCR_PULLUPDOWN_DISABLE); 
	}
	else if (UARTx == UART1)
	{

		//------------------------------------------------------------------------------------
		// UART1 
		//
		//				PD12			RXD1
		//				PD13			TXD1
		//
		//				PDMR			@ address = 0x4000_0260
		//				PDCR			@ address = 0x4000_0264
		//				PDPCR			@ address = 0x4000_0268
		//
		//------------------------------------------------------------------------------------
		PCU_ConfigureFunction (PCD, PIN_12, PD12_MUX_RXD1);
		PCU_ConfigureFunction (PCD, PIN_13, PD13_MUX_TXD1); 	

		PCU_Set_Direction_Type (PCD, PIN_12, PnCR_INPUT_LOGIC); 
		PCU_Set_Direction_Type (PCD, PIN_13, PnCR_OUTPUT_PUSH_PULL); 	

		PCU_ConfigurePullup_Pulldown (PCD, PIN_12, 0, PnPCR_PULLUPDOWN_DISABLE); 
		PCU_ConfigurePullup_Pulldown (PCD, PIN_13, 0, PnPCR_PULLUPDOWN_DISABLE); 

	}
	else if (UARTx == UART2)
	{

		//------------------------------------------------------------------------------------
		// UART2 
		//
		//				PC10 			RXD2
		//				PC11 			TXD2
		//
		//				PCMR			@ address = 0x4000_0240
		//				PCCR			@ address = 0x4000_0244
		//				PCPCR			@ address = 0x4000_0248
		//
		//------------------------------------------------------------------------------------
		PCU_ConfigureFunction (PCC, PIN_10, PC10_MUX_RXD2);
		PCU_ConfigureFunction (PCC, PIN_11, PC11_MUX_TXD2); 		

		PCU_Set_Direction_Type (PCC, PIN_10, PnCR_INPUT_LOGIC); 
		PCU_Set_Direction_Type (PCC, PIN_11, PnCR_OUTPUT_PUSH_PULL); 	

		PCU_ConfigurePullup_Pulldown (PCC, PIN_10, 0, PnPCR_PULLUPDOWN_DISABLE); 
		PCU_ConfigurePullup_Pulldown (PCC, PIN_11, 0, PnPCR_PULLUPDOWN_DISABLE); 
	}
	else if (UARTx == UART3)
	{

		//------------------------------------------------------------------------------------
		// UART3
		//
		//				PE6 				RXD3
		//				PE7 				TXD3
		//
		//				PEMR			@ address = 0x4000_0280
		//				PECR			@ address = 0x4000_0284
		//				PEPCR			@ address = 0x4000_0288
		//
		//------------------------------------------------------------------------------------
		PCU_ConfigureFunction (PCE, PIN_6, PE6_MUX_RXD3);
		PCU_ConfigureFunction (PCE, PIN_7, PE7_MUX_TXD3); 

		PCU_Set_Direction_Type (PCE, PIN_6, PnCR_INPUT_LOGIC); 
		PCU_Set_Direction_Type (PCE, PIN_7, PnCR_OUTPUT_PUSH_PULL); 	

		PCU_ConfigurePullup_Pulldown (PCE, PIN_6, 0, PnPCR_PULLUPDOWN_DISABLE); 
		PCU_ConfigurePullup_Pulldown (PCE, PIN_7, 0, PnPCR_PULLUPDOWN_DISABLE); 
	}

}


/**
*********************************************************************************************************
* @ Name : UART_Init 
*
* @ Parameters
*		- uart : UART0 ~ UART3
*		- config->Baudrate : 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
*		   config->DataBits : 5, 6, 7, 8
*		   config->StopBits : 1, 2 (1.5 & 2 stop bits)
*          config->Parity : UART_NO_PARITY, UART_ODD_PARITY, UART_EVEN_PARITY
*		   config->reference_clock : reference clock for baudrate
*		   config->Invert : 0, 1
*
*********************************************************************************************************
*/
void UART_Init (UART_Type * UARTx, UART_CONFIG *config)
{
	uint32_t		mode; 
	uint32_t		reg_val; 

	//------------------------------------------------------------------------------
	// reset 
	//------------------------------------------------------------------------------


	//------------------------------------------------------------------------------
	// DTR
	//
	//				
	//------------------------------------------------------------------------------
#ifdef USE_UART_3_SAMPLING
	reg_val = (UnDTR_SMS|UnDTR_DMS); 
#else 
	reg_val = 0;
#endif 

	if (config->Invert == 1)
		reg_val |= (UnDTR_RXINV|UnDTR_TXINV); 

	UARTx->DTR = reg_val;

	//------------------------------------------------------------------------------
	// Line Control 
	//
	//				U0LCR			@ address = 0x4000_0B0C
	//				U1LCR			@ address = 0x4000_0B4C
	//				U2LCR			@ address = 0x4000_0B8C
	//				U3LCR			@ address = 0x4000_0BCC
	//				
	//------------------------------------------------------------------------------
	// data bits
	switch (config->DataBits)
	{
	case 5:
		mode = UnLCR_DLEN_5_BITS;
		break; 

	case 6:
		mode = UnLCR_DLEN_6_BITS;
		break; 

	case 7:
		mode = UnLCR_DLEN_7_BITS;
		break;

	case 8:
	default:
		mode = UnLCR_DLEN_8_BITS;
		break;
	}

	// stop bit
	if (config->StopBits == 1)
	{
		mode |= UnLCR_STOPBIT_1_STOPBIT; 
	}
	else if (config->StopBits == 2)
	{
		mode |= UnLCR_STOPBIT_2_STOPBITS;
	}


	// parity 
	if (config->Parity == UART_ODD_PARITY)
	{
		mode |= UnLCR_PEN; 
	}
	else if (config->Parity == UART_EVEN_PARITY)
	{
		mode |= (UnLCR_PEN|UnLCR_PARITY); 
	}

	UARTx->LCR = mode;


	//------------------------------------------------------------------------------
	// baudrate
	//
	//				U0DLL			@ address = 0x4000_0B00
	//				U1DLL			@ address = 0x4000_0B40
	//				U2DLL			@ address = 0x4000_0B80
	//				U3DLL			@ address = 0x4000_0BC0
	//
	//				U0BFR			@ address = 0x4000_0B24
	//				U1BFR			@ address = 0x4000_0B64
	//				U2BFR			@ address = 0x4000_0BA4
	//				U3BFR			@ address = 0x4000_0BE4
	//
	//------------------------------------------------------------------------------
	UART_SetBaudrate(UARTx, config->Baudrate, 2, config->Reference_clock);
	

	
	//------------------------------------------------------------------------------
	// FCR
	//
	//				16C450, 16C550
	//------------------------------------------------------------------------------
	UARTx->FCR = 0;		// 16C450 mode 


	//------------------------------------------------------------------------------
	// SCR
	//
	//
	//				U0SCR			@ address = 0x4000_0B1C
	//				U1SCR			@ address = 0x4000_0B5C
	//				U2SCR			@ address = 0x4000_0B9C
	//				U3SCR			@ address = 0x4000_0BDC
	//
	//------------------------------------------------------------------------------
	UARTx->SCR = 0;

}

void _UART_Init (int uart_no)
{

	UART_Type			*UARTx;
	UART_CONFIG		uart_config; 
	uint32_t					intr; 
	NVIC_IntrConfig		nvic_config; 

	

	//------------------------------------------------------------------------------------------
	// check uart_no
	//------------------------------------------------------------------------------------------

	if ((uart_no < 0) || (uart_no > 3)) return; 


	//------------------------------------------------------------------------------------------
	// get object 
	//------------------------------------------------------------------------------------------
	UARTx = UART_Get_Object(uart_no); 



	//------------------------------------------------------------------------------------------
	// configure GPIO 
	//------------------------------------------------------------------------------------------
	UART_ConfigureGPIO(UARTx); 



	//------------------------------------------------------------------------------------------
	// buffer setting 
	//------------------------------------------------------------------------------------------	
	UART_InitBuffer (uart_no); 	

	

	//------------------------------------------------------------------------------------------
	// mode setting 
	//------------------------------------------------------------------------------------------	
#ifdef USE_BAUDRATE_1200
	uart_config.Baudrate = 1200;
#elif defined (USE_BAUDRATE_2400)
	uart_config.Baudrate = 2400;
#elif defined (USE_BAUDRATE_4800)
	uart_config.Baudrate = 4800;
#elif defined (USE_BAUDRATE_9600)
	uart_config.Baudrate = 9600;
#elif defined (USE_BAUDRATE_19200)
	uart_config.Baudrate = 19200;
#elif defined (USE_BAUDRATE_38400)
	uart_config.Baudrate = 38400;
#elif defined (USE_BAUDRATE_57600)
	uart_config.Baudrate = 57600;

#elif defined (USE_BAUDRATE_115200)
	uart_config.Baudrate = 115200;
#elif defined (USE_BAUDRATE_230400)
	uart_config.Baudrate = 230400;
#elif defined (USE_BAUDRATE_460800)
	uart_config.Baudrate = 460800;
#elif defined (USE_BAUDRATE_921600)
	uart_config.Baudrate = 921600;
#endif 


	uart_config.DataBits = 8; 
	uart_config.StopBits = 1; 
	uart_config.Parity = 0; 
	uart_config.Reference_clock = SystemPeriClock; 
	uart_config.Invert = 0; 
	UART_Init (UARTx, &uart_config); 



	//------------------------------------------------------------------------------
	// interrupt setting (peripheral)
	//
	//					
	//------------------------------------------------------------------------------	
	intr = (UART_INTR_RX|UART_INTR_TX|UART_INTR_LINE); 
//	UART_ConfigureInterrupt (UARTx, intr, INTR_ENABLE); 
	UART_ConfigureInterrupt (UARTx, intr, 1); 


	//------------------------------------------------------------------------------
	// interrupt setting (interrupt module)
	//
	//					IRQ_UART0 = 38
	//					IRQ_UART1 = 39
	//					IRQ_UART2 = 40
	//					IRQ_UART3 = 41
	//
	//------------------------------------------------------------------------------		
	nvic_config.nIRQ_Number = (IRQ_UART0+uart_no); 
	nvic_config.Preemption_Priority= PRIO_UART0_PREEMPTION; 
	nvic_config.Subpriority= PRIO_UART0_SUBPRIORITY; 
	nvic_config.IntrEnable = INTR_ENABLE; 
	NVIC_ConfigureInterrupt (NVIC, &nvic_config); 
}


/**
*********************************************************************************************************
* @ Name : UART_SetBaudrate 
*
* @ Parameters
*		- uart : UART0 ~ UART3
*		- baudrate : 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
*		- prescaler : 2
*		- ref_clk : reference clock for baudrate 
*
*
*********************************************************************************************************
*/
void UART_SetBaudrate (UART_Type * UARTx, uint32_t baudrate, uint32_t prescaler, uint32_t ref_clk)
{
	uint32_t		reg_val;
	uint32_t		numerator; 
	uint32_t		denominator; 
	uint32_t		dlm, dll; 
	uint32_t		q;
#ifdef USE_BFR	
	uint32_t		d_product_f;
	uint32_t		bfr;
#endif


	//------------------------------------------------------------------------------
	// numerator & denominator 
	//------------------------------------------------------------------------------
	numerator = ref_clk; 
	denominator = 16 * prescaler * baudrate; 


	//------------------------------------------------------------------------------
	// 	numerator 
	//	----------- = quotient + fraction
	//	denominator
	//------------------------------------------------------------------------------
	q = numerator / denominator; 



	//------------------------------------------------------------------------------
	//	DLM, DLL
	//------------------------------------------------------------------------------
	dlm = (q >> 8) & 0x00FF; 
	dll = (q & 0x00FF); 
	

	//------------------------------------------------------------------------------
	// 	numerator 		= PCLK
	//
	//	denominator 		= (16 * 2 * baudrate)
	//
	//
	//	numerator / denominator 	= quotient + fraction
	//
	//	(numerator / denominator) x 256  = quotient x 256 + BFR + remnant
	//
	//	INT (numerator / denominator) x 256) = quotient x 256 + BFR
	//
	//	numerator x 256 = quotient x 256 x denominator + BFR x denominator
	//
	//	numerator x 256 - quotient x 256 x denominator = BFR x denominator
	//
	//	(numerator - quotient x denominator) x 256 = BFR x denominator <-- equation
	//
	//	(numerator - quotient x denominator) x 256 / denominator = BFR 
	//
	//
	//------------------------------------------------------------------	
#ifdef USE_BFR
	d_product_f = numerator - (q * denominator); 
	bfr = (d_product_f << 8) / denominator; 
#endif

	//------------------------------------------------------------------------------
	// set baudrate 
	//
	//				U0LCR			@ address = 0x4000_0B0C
	//				U1LCR			@ address = 0x4000_0B4C
	//				U2LCR			@ address = 0x4000_0B8C
	//				U3LCR			@ address = 0x4000_0BCC
	//
	//				U0DLL			@ address = 0x4000_0B00
	//				U1DLL			@ address = 0x4000_0B40
	//				U2DLL			@ address = 0x4000_0B80
	//				U3DLL			@ address = 0x4000_0BC0
	//
	//				U0BFR			@ address = 0x4000_0B24
	//				U1BFR			@ address = 0x4000_0B64
	//				U2BFR			@ address = 0x4000_0BA4
	//				U3BFR			@ address = 0x4000_0BE4

	//------------------------------------------------------------------------------
	reg_val = UARTx->LCR;
	UARTx->LCR = (reg_val | UnLCR_DLAB);
	
	UARTx->RBR_THR_DLL = dll;
	UARTx->IER_DLM = dlm;
	
#ifdef USE_BFR	
	UARTx->BFR = bfr;
#endif 
	
	reg_val = UARTx->LCR;
	UARTx->LCR = (reg_val  & ~UnLCR_DLAB);

}




/**
*********************************************************************************************************
* @ Name : UART_ConfigureInterrupt 
*
* @ Parameters
*		- uart : UART0 ~ UART3
*		- intr_mask : UnIER_DRIE, UnIER_THREIE, UnIER_RLSIE, UnIER_TEMTIE
*		- enable : INTR_ENABLE, INTR_DISABLE
*
*
*********************************************************************************************************
*/
void UART_ConfigureInterrupt (UART_Type * UARTx, uint32_t intr_mask, uint32_t enable)
{
	volatile uint32_t 	intr_status;
	volatile uint32_t 	reg_val;
	


	//------------------------------------------------------------------------------
	// disable interrupt 
	//
	//				U0IER			@ address = 0x4000_0B04
	//				U1IER			@ address = 0x4000_0B44
	//				U2IER			@ address = 0x4000_0B84
	//				U3IER			@ address = 0x4000_0BC4
	//
	//------------------------------------------------------------------------------
	reg_val = UARTx->IER_DLM;
	reg_val = (reg_val & ~UnIER_INTR_MASK);
	
	UARTx->IER_DLM = reg_val;


	//------------------------------------------------------------------------------
	// clear interrupt flag 
	//
	//				U0IER			@ address = 0x4000_0B04
	//				U1IER			@ address = 0x4000_0B44
	//				U2IER			@ address = 0x4000_0B84
	//				U3IER			@ address = 0x4000_0BC4
	//
	//------------------------------------------------------------------------------
	//
	//				line status		read LSR 
	//				RBR available		read RBR or IIR
	//				THR empty		write THR or read IIR
	//
	//------------------------------------------------------------------------------
	intr_status = UARTx->IIR;

	if (intr_status & UnIIR_IID_RCV_LINE_STATUS)
	{
		reg_val = UARTx->LCR;
	}

	if (intr_status & UnIIR_IID_RBR_READY)
	{
		reg_val = UARTx->RBR_THR_DLL;
	}


	
	//------------------------------------------------------------------------------
	// enable interrupt flag 
	//
	//				U0IER			@ address = 0x4000_0B04
	//				U1IER			@ address = 0x4000_0B44
	//				U2IER			@ address = 0x4000_0B84
	//				U3IER			@ address = 0x4000_0BC4
	//
	//------------------------------------------------------------------------------
//	if (enable == INTR_ENABLE)
	if (enable == 1)		
	{
		reg_val = UARTx->IER_DLM;
		reg_val = reg_val | (intr_mask & ~(UnIER_TEMTIE|UnIER_THREIE));
		UARTx->IER_DLM = reg_val;
	}

	
}

/**
*********************************************************************************************************
* @ Name : UART_Get_Object
*
* @ Parameter
*		- uart_no : 0~3
*
* @ Return 
*
*********************************************************************************************************
*/
UART_Type * UART_Get_Object (int uart_no)
{

	UART_Type *	p_obj; 


	switch (uart_no)
	{
	case 0: 
		p_obj = UART0; 
		break; 
		
	case 1: 
		p_obj = UART1; 
		break; 

	case 2: 
		p_obj = UART2; 
		break; 
		
	case 3: 
		p_obj = UART3; 
		break; 		

	default:
		p_obj = (UART_Type *) 0; 
		break; 
	}


	return (p_obj); 

}

/**
*********************************************************************************************************
* @ Name : UART_InitBuffer
*
* @ Parameter
*		- uart_no : 0~3
*
*
*********************************************************************************************************
*/
void UART_InitBuffer (int uart_no)
{
	UART_BUFFER		*pUartBuffer; 
	int				i, result; 
	
	//------------------------------------------------------------------------------
	// get base address of buffer 
	//------------------------------------------------------------------------------
	pUartBuffer = UART_GetBufferBaseAddr (uart_no, &result);

	if (result == UART_CHANNEL_NOT_SUPPORTED)
	{
		return;
	}

	//------------------------------------------------------------------------------
	// init buffer variables
	//------------------------------------------------------------------------------
	pUartBuffer->RxState = UART_RX_STATE_IDLE;
	pUartBuffer->TxState = UART_TX_STATE_IDLE;
	
	pUartBuffer->RxBuffer_HeadIndex = 0; 
	pUartBuffer->RxBuffer_TailIndex = 0;
	pUartBuffer->TxBuffer_HeadIndex = 0; 
	pUartBuffer->TxBuffer_TailIndex = 0; 

	for (i=0; i<UART_MAX_RX_BUFFER; i++)
	{
		pUartBuffer->RxBuffer[i] = 0; 
	}

	for (i=0; i<UART_MAX_TX_BUFFER; i++)
	{
		pUartBuffer->RxBuffer[i] = 0; 
	}	


}

/**
*********************************************************************************************************
* @ Name : UART_GetBufferBaseAddr
*
* @ Parameter
*		- uart_no : 0~3
*		- pResult
*
* @ Return 
*		- buffer 
*
*
*********************************************************************************************************
*/
UART_BUFFER* UART_GetBufferBaseAddr (int uart_no, int *pResult)
{
	UART_BUFFER 	*pUartBuffer; 
	int					result = UART_CHANNEL_SUPPORTED; 

	switch (uart_no)
	{
	case 0:
		pUartBuffer = &sUart0Buffer; 
		break; 

	case 1:
		pUartBuffer = &sUart1Buffer; 
		break; 		

	case 2:
		pUartBuffer = &sUart2Buffer; 
		break; 

	case 3:
		pUartBuffer = &sUart3Buffer; 
		break; 	


	default:
		pUartBuffer = (UART_BUFFER *) 0; 
		result = UART_CHANNEL_NOT_SUPPORTED; 
	}

	*pResult = result; 

	return (pUartBuffer); 
}



/**
*********************************************************************************************************
* @ Name : UART_WriteBuffer 
*
* @ Parameters
*		- uart : 0~3
*
* @ Results
*
*
*********************************************************************************************************
*/
uint8_t UART_WriteBuffer (int uart_no, uint8_t *p_data, uint32_t data_count)
{

	UART_Type			*UARTx; 
	UART_BUFFER		*pUartBuffer; 
	int						status; 
#if 0
	uint32_t					tx_state; 
#endif
	uint32_t					i; 


	//------------------------------------------------------------------------------
	// get buffer address 
	//------------------------------------------------------------------------------	
	pUartBuffer = UART_GetBufferBaseAddr(uart_no, &status);
	while(pUartBuffer->TxBuffer_HeadIndex != pUartBuffer->TxBuffer_TailIndex) {}
	if (status == UART_CHANNEL_NOT_SUPPORTED)
	{
		return (UART_TX_BUFFER_ERROR_WRONG_CHANNEL); 
	}



	//------------------------------------------------------------------------------
	// wait until the previous transmission is completed
	//
	//		why does UnSCR is used
	//
	//		When the optimizatio level is raised above O0, any repeated access of a variable in SRAM memory 
	//		space is allowed one time, irrespective of whether the keyword 'volatile' is used or not. 
	//
	//		So, it is mandatory to refer to the peripheral memory space for such a repeated operation. 
	//------------------------------------------------------------------------------	
	UARTx = UART_Get_Object(uart_no); 
#if 0
	do {
		tx_state = UARTx->SCR; 
	} while (tx_state); 

	UARTx->SCR = 1;								// mark that the UART is in use
#else


	for (i=0; i<0x100000; i++)
	{
		//if (pUartBuffer->TxState == UART_TX_STATE_IDLE) break; 
		if (pUartBuffer->TxBuffer_HeadIndex == pUartBuffer->TxBuffer_TailIndex) break; 

	}

	if (i == 0x100000)
	{
		return (UART_TX_BUFFER_ERROR_WAIT_TIMEOUT); 
	}
	
#endif 


	//------------------------------------------------------------------------------
	// buffer <-- data 
	//------------------------------------------------------------------------------
	for (i=0; i<data_count; i++)
	{
		pUartBuffer->TxBuffer[i] = *(p_data+i); 
	}

	pUartBuffer->TxBuffer_HeadIndex = 0; 
	pUartBuffer->TxBuffer_TailIndex = data_count; 


	//------------------------------------------------------------------------------
	// update state 
	//------------------------------------------------------------------------------
	pUartBuffer->TxState = UART_TX_STATE_TRANSMIT; 

	
	//------------------------------------------------------------------------------
	// enable TX Interrupt
	//------------------------------------------------------------------------------
	UART_Enable_Tx_Interrupt (UARTx, 1); 

	return (UART_TX_BUFFER_SUCCESS); 

}



///**
//*********************************************************************************************************
//* @ Name : UART_Enable_TX_Interrupt 
//*
//* @ Parameters
//*		- UART Type : UARTx
//*
//* @ Results
//*
//*
//*********************************************************************************************************
//*/
void UART_Enable_Tx_Interrupt(UART_Type * UARTx, uint8_t status)
{
	uint32_t reg_val;
	
#ifdef USE_TEMT_INTR
	do {
		if(status == 1)
		{
			reg_val = UARTx->IER_DLM;
			reg_val &= ~(UnIER_TEMTIE|UnIER_THREIE);
			reg_val |= (UnIER_TEMTIE|UnIER_THREIE);
			UARTx->IER_DLM = reg_val;
		}
		else
		{
			reg_val = UARTx->IER_DLM;
			reg_val &= ~(UnIER_TEMTIE|UnIER_THREIE);
			UARTx->IER_DLM;
		}
	}	while(0);
#else
	do{
		if(status == 1)
		{
			reg_val =  UARTx->IER_DLM;
			reg_val |= UnIER_THREIE;
			UARTx->IER_DLM = reg_val;
		}
		else
		{
			reg_val =  UARTx->IER_DLM;
			reg_val &= ~(UnIER_THREIE);
			UARTx->IER_DLM = reg_val;
		}	
	} while(0);
#endif
}



/**
*********************************************************************************************************
* @ Name : UART_GetChar 
*
* @ Parameter
*		- uart_no : 0~3
*
* @ Results
*		- 8-bit data other than "-1"		= success
*		-"-1"						= error 
*
*
* @ Description
*		This function mediates cgetchar() and UART_ReadBuffer(). UART_ReadBuffer is a lower-level function which reads directly
*		an UART data from the UART buffer. 
*
*		UART_GetChar() manipulates the received data. 
*
*			(1) converts the data from 8-bit data to 32-bit data
*			(2) returns "-1" if error occurs.
*
*
*********************************************************************************************************
*/

int UART_GetChar (int uart_no)
{
	int			status;
	int			ch;

	ch = UART_ReadBuffer (uart_no, &status);

	if (status != UART_RX_BUFFER_SUCCESS) ch = -1; 

	return (ch); 

}


/**
*********************************************************************************************************
* @ Name : UART_ReadBuffer 
*
* @ Parameters
*		- uart : 0~3
*
* @ Results
*		[return]
*		- 8-bit unsigned data			= success
*		- 0							= fail (it is qualified by *p_status)
*		
*
*		[*p_status]
*		- UART_RX_BUFFER_SUCCESS					= success 
*		- UART_RX_BUFFER_ERROR_EMPTY				= no available data
*		- UART_RX_BUFFER_ERROR_WRONG_CHANNEL		= wrong UART number
*
*
*********************************************************************************************************
*/
uint8_t UART_ReadBuffer (int uart_no, int *p_status)
{

	UART_BUFFER 		*pUartBuffer; 
	int 				status; 

	uint8_t				uart_data; 


	//------------------------------------------------------------------------------
	// get buffer address 
	//------------------------------------------------------------------------------	
	pUartBuffer = UART_GetBufferBaseAddr(uart_no, &status);
	if (status == UART_CHANNEL_NOT_SUPPORTED)
	{
		return (UART_RX_BUFFER_ERROR_WRONG_CHANNEL); 
	}


	//------------------------------------------------------------------------------
	// fetch data 
	//------------------------------------------------------------------------------
	if (pUartBuffer->RxBuffer_TailIndex == pUartBuffer->RxBuffer_HeadIndex)
	{
		uart_data = 0; 
		*p_status = UART_RX_BUFFER_ERROR_EMPTY; 
	}
	else 
	{
		uart_data = pUartBuffer->RxBuffer[pUartBuffer->RxBuffer_HeadIndex++]; 
		*p_status = UART_RX_BUFFER_SUCCESS; 
	}



	//------------------------------------------------------------------------------
	// adjust 
	//------------------------------------------------------------------------------
	if (pUartBuffer->RxBuffer_HeadIndex >= UART_MAX_RX_BUFFER)
	{
		pUartBuffer->RxBuffer_HeadIndex = 0; 
	}


	//------------------------------------------------------------------------------
	// return 
	//------------------------------------------------------------------------------
	return (uart_data); 

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UART0_Transmit_Receive_ISR(void)
{
#ifdef USE_TEMT_INTR	
	uint32_t					reg_val;
#endif
	volatile uint32_t		intr_status;
	volatile uint32_t		line_status; 



	//------------------------------------------------------------------------------
	// get interrupt identification
	//------------------------------------------------------------------------------
	intr_status = UART0->IIR;


	//------------------------------------------------------------------------------
	// line interrupt 
	//------------------------------------------------------------------------------
	if ((intr_status & UnIIR_INTR_BASIC_MASK) == UnIIR_IID_RCV_LINE_STATUS)
	{
		line_status = UART0->LSR;
	}
	

	//------------------------------------------------------------------------------
	// RX interrupt 
	//------------------------------------------------------------------------------
	if (intr_status & UnIIR_IID_RBR_READY)
	{
		UART0_ReceiveData_ISR(); 
	}


	//------------------------------------------------------------------------------
	// TX interrupt - THRE
	//------------------------------------------------------------------------------
	if (intr_status & UnIIR_IID_THR_EMPTY)
	{
		UART0_TransmitData_ISR(); 
	}


	//------------------------------------------------------------------------------
	// TX interrupt - TEMT
	//------------------------------------------------------------------------------	
#ifdef USE_TEMT_INTR
	if (intr_status & UnIIR_XMITE)
	{
		reg_val = UART0->IER_DLM;
		reg_val &= ~(UnIER_TEMTIE|UnIER_THREIE);
		UART0->IER_DLM = reg_val;

		sUart0Buffer.TxBuffer_HeadIndex = 0; 
		sUart0Buffer.TxBuffer_TailIndex = 0; 
		sUart0Buffer.TxState = UART_TX_STATE_IDLE; 
		UART0->SCR = 0;
	}
#endif 


}

void UART0_ReceiveData_ISR (void)
{
	uint8_t					rcv_data;
	volatile uint32_t		line_status; 
	uint16_t					next_index;
	
	uint32_t					reg_val;


	//------------------------------------------------------------------------------
	// get UART data 
	//------------------------------------------------------------------------------
	rcv_data = UART0->RBR_THR_DLL;


	//------------------------------------------------------------------------------
	// check line error again 
	//------------------------------------------------------------------------------
	reg_val = UART0->LSR;
	if( (reg_val &  UnLSR_BI) == 1 || (reg_val &  UnLSR_PE) == 1 || (reg_val & UnLSR_OE) == 1 )
	{
		line_status = UART0->LSR;
	}
	else 
	{
		//--------------------------------------------------------------------------
		// get next index
		//--------------------------------------------------------------------------
		next_index = sUart0Buffer.RxBuffer_TailIndex + 1; 
		if (next_index >= UART_MAX_RX_BUFFER)
		{
			next_index = 0; 
		}

		//--------------------------------------------------------------------------
		// check if RX buffer is full, and ten store data if buffer space available
		//--------------------------------------------------------------------------
		if (next_index == sUart0Buffer.RxBuffer_HeadIndex)
		{
			return; 
		}
		else 
		{
			sUart0Buffer.RxBuffer[sUart0Buffer.RxBuffer_TailIndex] = rcv_data; 
			sUart0Buffer.RxBuffer_TailIndex = next_index; 
		}

	}

}


void UART0_TransmitData_ISR (void)
{
	uint32_t						reg_val;
	volatile uint32_t			line_status;
	uint8_t						send_data; 



	//------------------------------------------------------------------------------
	// check line error again 
	//------------------------------------------------------------------------------
	reg_val = UART0->LSR;
	if( (reg_val &  UnLSR_BI) == UnLSR_BI || (reg_val &  UnLSR_PE) == UnLSR_PE || (reg_val & UnLSR_OE) == UnLSR_OE )
	{
		line_status = UART0->LSR;
	}

	//------------------------------------------------------------------------------
	// Technical Comments
	//
	//		LSR:TEMT			Transmitter Empty (bit 6)
	//
	//		LSR:THRE			Transmit Holding Register Empty (bit 5)
	//
	//	
	//		When THRE is "1", UART interrupt is invoked. The condition of TEMT doesn't invoke 
	//		UART interrupt. 
	//
	//------------------------------------------------------------------------------
	reg_val = UART0->LSR;
	
	if ( (reg_val & UnLSR_THRE) == UnLSR_THRE)
	{
		if (sUart0Buffer.TxBuffer_HeadIndex < sUart0Buffer.TxBuffer_TailIndex)
		{
			//----------------------------------------------------------------------
			// have a data to transmit
			//----------------------------------------------------------------------
			send_data = sUart0Buffer.TxBuffer[sUart0Buffer.TxBuffer_HeadIndex++];
			UART0->RBR_THR_DLL = send_data;
		}
		else
		{
			//----------------------------------------------------------------------
			// transmission done
			//----------------------------------------------------------------------
		#ifdef USE_TEMT_INTR

			reg_val = UART0->IER_DLM;
			reg_val |= UnIER_TEMTIE;
			UART0->IER_DLM = reg_val;
		#else 
			UART_Enable_Tx_Interrupt(UART0, 0);

			sUart0Buffer.TxBuffer_HeadIndex = 0; 
			sUart0Buffer.TxBuffer_TailIndex = 0; 
			sUart0Buffer.TxState = UART_TX_STATE_IDLE; 
			UART0->SCR = 0;
		#endif 

		}

	}

	
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UART1_Transmit_Receive_ISR(void)
{
#ifdef USE_TEMT_INTR	
	uint32_t					reg_val;
#endif
	volatile uint32_t		intr_status;
	volatile uint32_t		line_status; 


	//------------------------------------------------------------------------------
	// get interrupt identification
	//------------------------------------------------------------------------------
	intr_status = UART1->IIR;

	
	//------------------------------------------------------------------------------
	// line interrupt 
	//------------------------------------------------------------------------------
	if ((intr_status & UnIIR_INTR_BASIC_MASK) == UnIIR_IID_RCV_LINE_STATUS)
	{
		line_status = UART1->LSR;
//		intr_status = UART1->IIR;
	}
	

	//------------------------------------------------------------------------------
	// RX interrupt 
	//------------------------------------------------------------------------------
	if (intr_status == UnIIR_IID_RBR_READY)
	{
		UART1_ReceiveData_ISR(); 
	}


	//------------------------------------------------------------------------------
	// TX interrupt - THRE 
	//------------------------------------------------------------------------------
	if (intr_status == UnIIR_IID_THR_EMPTY)
	{
		UART1_TransmitData_ISR(); 
	}

	//------------------------------------------------------------------------------
	// TX interrupt - TEMT
	//------------------------------------------------------------------------------	
#ifdef USE_TEMT_INTR
	if (intr_status & UnIIR_XMITE)
	{
		reg_val = UART1->IER_DLM;
		reg_val &= ~(UnIER_TEMTIE|UnIER_THREIE);
		UART1->IER_DLM = reg_val;

		sUart1Buffer.TxBuffer_HeadIndex = 0; 
		sUart1Buffer.TxBuffer_TailIndex = 0; 
		sUart1Buffer.TxState = UART_TX_STATE_IDLE; 
		UART1->SCR = 0;
	}
#endif 	
	

}

void UART1_ReceiveData_ISR (void)
{
	uint32_t				reg_val;
	uint8_t				rcv_data;
	volatile uint32_t		line_status; 
	uint16_t				next_index; 


	//------------------------------------------------------------------------------
	// get UART data 
	//------------------------------------------------------------------------------
	rcv_data = UART1->RBR_THR_DLL;


	//------------------------------------------------------------------------------
	// check line error again 
	//------------------------------------------------------------------------------
	reg_val = UART1->LSR;
	if( (reg_val &  UnLSR_BI) == UnLSR_BI || (reg_val &  UnLSR_PE) == UnLSR_PE || (reg_val & UnLSR_OE) == UnLSR_OE )
	{
		line_status = UART1->LSR;
	}
	else 
	{
		//--------------------------------------------------------------------------
		// get next index
		//--------------------------------------------------------------------------
		next_index = sUart1Buffer.RxBuffer_TailIndex + 1; 
		if (next_index >= UART_MAX_RX_BUFFER)
		{
			next_index = 0; 
		}

		//--------------------------------------------------------------------------
		// check if RX buffer is full, and ten store data if buffer space available
		//--------------------------------------------------------------------------
		if (next_index == sUart1Buffer.RxBuffer_HeadIndex)
		{
			return; 
		}
		else 
		{
			sUart1Buffer.RxBuffer[sUart1Buffer.RxBuffer_TailIndex] = rcv_data; 
			sUart1Buffer.RxBuffer_TailIndex = next_index; 
		}

	}

}


void UART1_TransmitData_ISR (void)
{
	uint32_t						reg_val;
	volatile uint32_t			line_status;
	uint8_t						send_data; 



	//------------------------------------------------------------------------------
	// check line error again 
	//------------------------------------------------------------------------------
	reg_val = UART1->LSR;
	if( (reg_val &  UnLSR_BI) == UnLSR_BI || (reg_val &  UnLSR_PE) == UnLSR_PE || (reg_val & UnLSR_OE) == UnLSR_OE )
	{
		line_status = UART1->LSR;
	}
	
	



	//------------------------------------------------------------------------------
	// Technical Comments
	//
	//		LSR:TEMT			Transmitter Empty (bit 6)
	//
	//		LSR:THRE			Transmit Holding Register Empty (bit 5)
	//
	//	
	//		When THRE is "1", UART interrupt is invoked. The condition of TEMT doesn't invoke 
	//		UART interrupt. 
	//
	//------------------------------------------------------------------------------
	reg_val = UART1->LSR;
	
	if ( (reg_val & UnLSR_THRE) == UnLSR_THRE)
	{
		if (sUart1Buffer.TxBuffer_HeadIndex < sUart1Buffer.TxBuffer_TailIndex)
		{
			//----------------------------------------------------------------------
			// have a data to transmit
			//----------------------------------------------------------------------
			send_data = sUart1Buffer.TxBuffer[sUart1Buffer.TxBuffer_HeadIndex++];
			UART1->RBR_THR_DLL = send_data;
		}
		else
		{
			//----------------------------------------------------------------------
			// transmission done
			//----------------------------------------------------------------------
		#ifdef USE_TEMT_INTR
			reg_val = UART1->IER_DLM;
			reg_val |= UnIER_TEMTIE;
			UART1->IER_DLM = reg_val;
		#else 
			UART_Enable_Tx_Interrupt(UART1, 0);

			sUart1Buffer.TxBuffer_HeadIndex = 0; 
			sUart1Buffer.TxBuffer_TailIndex = 0; 
			sUart1Buffer.TxState = UART_TX_STATE_IDLE; 
			UART1->SCR = 0;
		#endif 
		}

	}

	
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UART2_Transmit_Receive_ISR(void)
{
#ifdef USE_TEMT_INTR
	uint32_t					reg_val;
#endif
	volatile uint32_t		intr_status;
	volatile uint32_t		line_status; 


	//------------------------------------------------------------------------------
	// get interrupt identification
	//------------------------------------------------------------------------------
	intr_status = UART2->IIR;


	//------------------------------------------------------------------------------
	// line interrupt 
	//------------------------------------------------------------------------------
	if ((intr_status & UnIIR_INTR_BASIC_MASK) == UnIIR_IID_RCV_LINE_STATUS)
	{
		line_status = UART2->LSR;
//		intr_status = UART2->IIR;
	}


	//------------------------------------------------------------------------------
	// RX interrupt 
	//------------------------------------------------------------------------------
	if (intr_status == UnIIR_IID_RBR_READY)
	{
		UART2_ReceiveData_ISR(); 
	}


	//------------------------------------------------------------------------------
	// TX interrupt - THRE 
	//------------------------------------------------------------------------------
	if (intr_status == UnIIR_IID_THR_EMPTY)
	{
		UART2_TransmitData_ISR(); 
	}

	//------------------------------------------------------------------------------
	// TX interrupt - TEMT
	//------------------------------------------------------------------------------	
	
#ifdef USE_TEMT_INTR
	if (intr_status & UnIIR_XMITE)
	{
		reg_val = UART2->IER_DLM;
		reg_val &= ~(UnIER_TEMTIE|UnIER_THREIE);
		UART2->IER_DLM = reg_val;

		sUart2Buffer.TxBuffer_HeadIndex = 0; 
		sUart2Buffer.TxBuffer_TailIndex = 0; 
		sUart2Buffer.TxState = UART_TX_STATE_IDLE; 
		UART2->SCR = 0;
	}
#endif 	


}

void UART2_ReceiveData_ISR (void)
{
	uint32_t				reg_val;
	uint8_t				rcv_data;
	volatile uint32_t		line_status; 
	uint16_t				next_index; 




	//------------------------------------------------------------------------------
	// get UART data 
	//------------------------------------------------------------------------------
	rcv_data = UART2->RBR_THR_DLL;


	//------------------------------------------------------------------------------
	// check line error again 
	//------------------------------------------------------------------------------
	reg_val = UART2->LSR;
	if( (reg_val &  UnLSR_BI) == UnLSR_BI || (reg_val &  UnLSR_PE) == UnLSR_PE || (reg_val & UnLSR_OE) == UnLSR_OE )
	{
		line_status = UART2->LSR;
	}
	else 
	{
		//--------------------------------------------------------------------------
		// get next index
		//--------------------------------------------------------------------------
		next_index = sUart2Buffer.RxBuffer_TailIndex + 1; 
		if (next_index >= UART_MAX_RX_BUFFER)
		{
			next_index = 0; 
		}

		//--------------------------------------------------------------------------
		// check if RX buffer is full, and ten store data if buffer space available
		//--------------------------------------------------------------------------
		if (next_index == sUart2Buffer.RxBuffer_HeadIndex)
		{
			return; 
		}
		else 
		{
			sUart2Buffer.RxBuffer[sUart2Buffer.RxBuffer_TailIndex] = rcv_data; 
			sUart2Buffer.RxBuffer_TailIndex = next_index; 
		}

	}

}


void UART2_TransmitData_ISR (void)
{
	uint32_t						reg_val;
	volatile uint32_t			line_status;
	uint8_t						send_data; 



	//------------------------------------------------------------------------------
	// check line error again 
	//------------------------------------------------------------------------------
	reg_val = UART2->LSR;
	if( (reg_val &  UnLSR_BI) == UnLSR_BI || (reg_val &  UnLSR_PE) == UnLSR_PE || (reg_val & UnLSR_OE) == UnLSR_OE )
	{
		line_status = UART2->LSR;
	}


	//------------------------------------------------------------------------------
	// Technical Comments
	//
	//		LSR:TEMT			Transmitter Empty (bit 6)
	//
	//		LSR:THRE			Transmit Holding Register Empty (bit 5)
	//
	//	
	//		When THRE is "1", UART interrupt is invoked. The condition of TEMT doesn't invoke 
	//		UART interrupt. 
	//
	//------------------------------------------------------------------------------
	reg_val = UART2->LSR;
	
	if ( (reg_val & UnLSR_THRE) == UnLSR_THRE)
	{
		if (sUart2Buffer.TxBuffer_HeadIndex < sUart2Buffer.TxBuffer_TailIndex)
		{
			//----------------------------------------------------------------------
			// have a data to transmit
			//----------------------------------------------------------------------
			send_data = sUart2Buffer.TxBuffer[sUart2Buffer.TxBuffer_HeadIndex++];
			UART2->RBR_THR_DLL = send_data;
		}
		else
		{
			//----------------------------------------------------------------------
			// transmission done
			//----------------------------------------------------------------------
		#ifdef USE_TEMT_INTR
			reg_val = UART2->IER_DLM;
			reg_val |= UnIER_TEMTIE;
			UART2->IER_DLM = reg_val;
		#else 
			UART_Enable_Tx_Interrupt(UART2, 0);

			sUart2Buffer.TxBuffer_HeadIndex = 0; 
			sUart2Buffer.TxBuffer_TailIndex = 0; 
			sUart2Buffer.TxState = UART_TX_STATE_IDLE;
			UART2->SCR = 0;
		#endif 

		}

	}

	
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UART3_Transmit_Receive_ISR(void)
{
#ifdef USE_TEMT_INTR
	uint32_t					reg_val;
#endif
	volatile uint32_t		intr_status;
	volatile uint32_t		line_status; 


	//------------------------------------------------------------------------------
	// get interrupt identification
	//------------------------------------------------------------------------------
	intr_status = UART3->IIR;

#if 0
	if (g_UART3_debug_index < 40)
	{
		g_UART3_debug_buffer[g_UART3_debug_index++] = intr_status; 
	}
#endif  



	//------------------------------------------------------------------------------
	// line interrupt 
	//------------------------------------------------------------------------------
	if ((intr_status & UnIIR_INTR_BASIC_MASK) == UnIIR_IID_RCV_LINE_STATUS)
	{
		line_status = UART3->LSR;
		intr_status = UART3->IIR;
	}


	//------------------------------------------------------------------------------
	// RX interrupt 
	//------------------------------------------------------------------------------
	if (intr_status & UnIIR_IID_RBR_READY)
	{
		UART3_ReceiveData_ISR(); 
	}


	//------------------------------------------------------------------------------
	// TX interrupt - THRE
	//------------------------------------------------------------------------------
	if (intr_status & UnIIR_IID_THR_EMPTY)
	{
		UART3_TransmitData_ISR(); 
	}


	//------------------------------------------------------------------------------
	// TX interrupt - TEMT
	//------------------------------------------------------------------------------	
#ifdef USE_TEMT_INTR
	if (intr_status & UnIIR_XMITE)
	{
		reg_val = UART3->IER_DLM;
		reg_val &= ~(UnIER_TEMTIE|UnIER_THREIE);
		UART3->IER_DLM = reg_val;

		sUart3Buffer.TxBuffer_HeadIndex = 0; 
		sUart3Buffer.TxBuffer_TailIndex = 0; 
		sUart3Buffer.TxState = UART_TX_STATE_IDLE; 
		UART3->SCR = 0;
	}
#endif 





}

void UART3_ReceiveData_ISR (void)
{
	uint8_t				rcv_data;
	volatile uint32_t		line_status; 
	uint16_t				next_index;
	uint32_t				reg_val;


	//------------------------------------------------------------------------------
	// get UART data 
	//------------------------------------------------------------------------------
	rcv_data = UART3->RBR_THR_DLL;



	//------------------------------------------------------------------------------
	// check line error again 
	//------------------------------------------------------------------------------
	reg_val = UART3->LSR;
	if( (reg_val &  UnLSR_BI) == UnLSR_BI || (reg_val &  UnLSR_PE) == UnLSR_PE || (reg_val & UnLSR_OE) == UnLSR_OE )
	{
		line_status = UART3->LSR;
	}
	
	else 
	{
		//--------------------------------------------------------------------------
		// get next index
		//--------------------------------------------------------------------------
		next_index = sUart3Buffer.RxBuffer_TailIndex + 1; 
		if (next_index >= UART_MAX_RX_BUFFER)
		{
			next_index = 0; 
		}

		//--------------------------------------------------------------------------
		// check if RX buffer is full, and ten store data if buffer space available
		//--------------------------------------------------------------------------
		if (next_index == sUart3Buffer.RxBuffer_HeadIndex)
		{
			return; 
		}
		else 
		{
			sUart3Buffer.RxBuffer[sUart3Buffer.RxBuffer_TailIndex] = rcv_data; 
			sUart3Buffer.RxBuffer_TailIndex = next_index; 
		}

	}

}


void UART3_TransmitData_ISR (void)
{
	uint32_t						reg_val;
	volatile uint32_t			line_status;
	uint8_t						send_data; 


	//------------------------------------------------------------------------------
	// check line error again 
	//------------------------------------------------------------------------------
	reg_val = UART3->LSR;
	if( (reg_val &  UnLSR_BI) == UnLSR_BI || (reg_val &  UnLSR_PE) == UnLSR_PE || (reg_val & UnLSR_OE) == UnLSR_OE )
	{
		line_status = UART3->LSR;
	}
	



	//------------------------------------------------------------------------------
	// Technical Comments
	//
	//		LSR:TEMT			Transmitter Empty (bit 6)
	//
	//		LSR:THRE			Transmit Holding Register Empty (bit 5)
	//
	//	
	//		When THRE is "1", UART interrupt is invoked. The condition of TEMT doesn't invoke 
	//		UART interrupt. 
	//
	//------------------------------------------------------------------------------
	reg_val = UART3->LSR;
	
	if ( (reg_val & UnLSR_THRE) == UnLSR_THRE)
	{
		if (sUart3Buffer.TxBuffer_HeadIndex < sUart3Buffer.TxBuffer_TailIndex)
		{
			//----------------------------------------------------------------------
			// have a data to transmit
			//----------------------------------------------------------------------
			send_data = sUart3Buffer.TxBuffer[sUart3Buffer.TxBuffer_HeadIndex++];
			UART3->RBR_THR_DLL = send_data;
		}
		else
		{
			//----------------------------------------------------------------------
			// transmission done
			//----------------------------------------------------------------------
		#ifdef USE_TEMT_INTR
			reg_val = UART3->IER_DLM;
			reg_val |= UnIER_TEMTIE;
			UART3->IER_DLM = reg_val;
		#else 
			UART_Enable_Tx_Interrupt(UART3, 0);

			sUart3Buffer.TxBuffer_HeadIndex = 0; 
			sUart3Buffer.TxBuffer_TailIndex = 0; 
			sUart3Buffer.TxState = UART_TX_STATE_IDLE; 
			UART3->SCR=0;
		#endif 
		}

	}

	
}
