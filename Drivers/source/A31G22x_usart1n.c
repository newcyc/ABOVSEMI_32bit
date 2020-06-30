/***************************************************************************//**
* @file     A31G22x_usart1n.c
* @brief    Contains all functions support for USART dirver on A31G22x
* @author   AE Team, ABOV Semiconductor Co., Ltd.
* @version  V0.0.1
* @date     08. May. 2019
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

/* Includes ------------------------------------------------------------------- */
#include "A31G22x_scu.h"
#include "A31G22x_usart1n.h"


uint32_t UsartBaseClock; 

/*********************************************************************
 * @brief		Determines best dividers to get a target clock rate
 * @param[in]	USARTx	Pointer to selected USART peripheral, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @param[in]	baudrate Desired USART baud rate.
 * @return 		None
 **********************************************************************/
void usart_set_divisors(USART_Type *USARTx, uint32_t baudrate)
{
	uint32_t numerator; 
	uint32_t denominator; 
	uint32_t bdr;
	uint32_t bfr;

	//------------------------------------------------------------------------------
	// numerator & denominator 
	// 
	// bdr = (UsartBaseClock) / n / baudrate  - 1
	//------------------------------------------------------------------------------
	numerator = UsartBaseClock; 
	denominator = baudrate; 

	bdr = numerator / denominator - 1; 

	USARTx->BDR= (uint16_t)(bdr&0xffff);
	
	// Calcuated Fraction point values
	bfr = (((UsartBaseClock * 256) / (16 * bdr)) & 0xFF);
	USARTx->BFR = (bfr & USART_BFR_BFC_Msk);
	
}

/* USART Init/DeInit functions -------------------------------------------------*/
/********************************************************************
 * @brief		Initializes the USART1x peripheral according to the specified
 *               parameters in the USART_ConfigStruct.
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @param[in]	USART_ConfigStruct Pointer to a USART_CFG_Type structure
 *              that contains the configuration information for the
 *              specified USART peripheral.
 * @return 		None
 *********************************************************************/
void USART_Init(USART_Type *USARTx, USART_CFG_Type *USART_ConfigStruct)
{
	uint32_t tmp ;

	if(USARTx == USART10)
	{
		SCU->PER2&=~(1<<8);
		SCU->PCER2&=~(1<<8);		
		/* Set up peripheral clock for USART10 module */
		SCU->PER2|=(1<<8);
		SCU->PCER2|=(1<<8);	
	}
	else if(USARTx == USART11)
	{
		SCU->PER2&=~(1<<9);
		SCU->PCER2&=~(1<<9);		
		/* Set up peripheral clock for USART11 module */
		SCU->PER2|=(1<<9);
		SCU->PCER2|=(1<<9);
	}
	else if(USARTx == USART12)
	{
		SCU->PER2&=~(1<<10);
		SCU->PCER2&=~(1<<10);		
		/* Set up peripheral clock for USART12 module */
		SCU->PER2|=(1<<10);
		SCU->PCER2|=(1<<10);	
	}
	else if(USARTx == USART13)
	{
		SCU->PER2&=~(1<<11);
		SCU->PCER2&=~(1<<11);		
		/* Set up peripheral clock for USART13 module */
		SCU->PER2|=(1<<11);
		SCU->PCER2|=(1<<11);	
	}

	USARTx->CR2 = (1<<9);     // EN     
	
    if(USART_ConfigStruct->Mode == USART_UART_MODE){
		tmp = 0
			| ((USART_ConfigStruct->Mode & 0x3) << 14)
			| ((USART_ConfigStruct->Parity & 0x3) << 12)
			| ((USART_ConfigStruct->Databits & 0x7) << 9)
			| (1 << 1)  //Tx Enable
			| (1 << 0)  //Rx Enable
			; 
		USARTx->CR1 = tmp;

		tmp = 0
			|(1<<9)  // EN
			| ((USART_ConfigStruct->Stopbits & 0x1)<<2)
			;
		USARTx->CR2 = tmp;
			
        UsartBaseClock=SystemCoreClock >> 4;  //     UsartBaseClock = SystemCoreClock / 16
		usart_set_divisors(USARTx, USART_ConfigStruct->Baud_rate);
   
    }
	else if(USART_ConfigStruct->Mode == USART_SYNC_MODE){
		tmp = 0
			| ((USART_ConfigStruct->Mode & 0x3) << 14)
			| ((USART_ConfigStruct->Parity & 0x3) << 12)
			| ((USART_ConfigStruct->Databits & 0x7) << 9)
			| (1 << 1)  //Tx Enable
			| (1 << 0)  //Rx Enable
			; 
		USARTx->CR1 = tmp;

		tmp = 0
			|(1<<9)  // EN
			| ((USART_ConfigStruct->Stopbits & 0x1)<<2)
			;
		USARTx->CR2 = tmp;
			
        UsartBaseClock=SystemCoreClock >> 1;  //     UsartBaseClock = SystemCoreClock / 2
		usart_set_divisors(USARTx, USART_ConfigStruct->Baud_rate);
    }
	else if(USART_ConfigStruct->Mode == USART_SPI_MODE){ 
		tmp = 0
			| ((USART_ConfigStruct->Mode & 0x3) << 14)
			| ((USART_ConfigStruct->Databits & 0x7) << 9)
			| ((USART_ConfigStruct->Order & 0x1) << 8)
			| ((USART_ConfigStruct->ACK & 0x1) << 7)
			| ((USART_ConfigStruct->Edge & 0x1)  << 6)
			| (1 << 1)  //Tx Enable
			| (1 << 0)  //Rx Enable
			; 
		USARTx->CR1 = tmp;
		
		USARTx->CR2 &= ~(1<<3);   // FXCHn reset
	    USARTx->CR2 |= (1<<3);    // FXCHn

		UsartBaseClock=SystemCoreClock >> 1;
		usart_set_divisors(USARTx, USART_ConfigStruct->Baud_rate);
		
		//Dummy Read
//		tmp = USART_ReceiveByte(USARTx);
//		tmp = USART_ReceiveByte(USARTx);
    }
}


/*********************************************************************
 * @brief		De-initializes the USARTx peripheral registers to their
 *              default reset values.
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @return 		None
 **********************************************************************/
void USART_DeInit(USART_Type* USARTx)
{
	if(USARTx == USART10)
	{
		SCU->PER2&=~(1<<8);
		SCU->PCER2&=~(1<<8);		
	}
	else if(USARTx == USART11)
	{
		SCU->PER2&=~(1<<9);
		SCU->PCER2&=~(1<<9);		
	}
	else if(USARTx == USART12)
	{
		SCU->PER2&=~(1<<10);
		SCU->PCER2&=~(1<<10);		
	}
	else if(USARTx == USART13)
	{
		SCU->PER2&=~(1<<11);
		SCU->PCER2&=~(1<<11);		
	}
}


/*********************************************************************
 * @brief		USARTx enable control
 *              default reset values.
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 *              state ENABLE / DISABLE
 * @return 		None
 **********************************************************************/
void USART_Enable(USART_Type* USARTx, FunctionalState state)
{
    if(state == ENABLE){
        USARTx->CR2 |= (1<<9);     // EN
    }else{
        USARTx->CR2 &= ~(1<<9);     // EN
    }
}


/******************************************************************************
 * @brief		Fills each USART_InitStruct member with its default value:
 * 					- 38400 bps
 * 					- 8-bit data
 * 					- 1 Stopbit
 * 					- None Parity
 * @param[in]	USART_InitStruct Pointer to a USART_CFG_Type structure which will
 * 				be initialized.
 * @return		None
 *******************************************************************************/
void USART_UART_Mode_ConfigStructInit(USART_CFG_Type *USART_InitStruct)
{
    USART_InitStruct->Mode = USART_UART_MODE;
	USART_InitStruct->Baud_rate = 38400;
	USART_InitStruct->Databits = USART_DATABIT_8;
	USART_InitStruct->Parity = USART_PARITY_NONE;
	USART_InitStruct->Stopbits = USART_STOPBIT_1;
}

/******************************************************************************
 * @brief		Fills each USART_InitStruct member with its default value:
 * @param[in]	USART_InitStruct Pointer to a USART_CFG_Type structure which will
 * 				be initialized.
 * @return		None
 *******************************************************************************/
void USART_SPI_Mode_ConfigStructInit(USART_CFG_Type *USART_InitStruct)
{
    USART_InitStruct->Mode = USART_SPI_MODE;
	USART_InitStruct->Baud_rate = 38400;
	USART_InitStruct->Databits = USART_DATABIT_8;
    
    //only SPI
    USART_InitStruct->Order = USART_SPI_LSB;
#if 0 // CPOLn : 0, CPHAn : 0 : (Error Case at G314)
    USART_InitStruct->ACK = USART_SPI_TX_RISING;
    USART_InitStruct->Edge = USART_SPI_TX_LEADEDGE_SAMPLE;
#endif
	
#if 0 // CPOLn : 0, CPHAn : 1
    USART_InitStruct->ACK = USART_SPI_TX_RISING;
    USART_InitStruct->Edge = USART_SPI_TX_LEADEDGE_SETUP;
#endif
    
#if 0 // CPOLn : 1, CPHAn : 0 (Error Case at G314)
    USART_InitStruct->ACK = USART_SPI_TX_FALLING;
    USART_InitStruct->Edge = USART_SPI_TX_LEADEDGE_SAMPLE;
#endif
    
#if 1 // CPOLn : 1, CPHAn : 1
    USART_InitStruct->ACK = USART_SPI_TX_FALLING;
    USART_InitStruct->Edge = USART_SPI_TX_LEADEDGE_SETUP;
#endif
    
}

/******************************************************************************
 * @brief		Fills each USART_InitStruct member with its default value:
 * 					- 38400 bps
 * 					- 8-bit data
 * 					- 1 Stopbit
 * 					- None Parity
 * @param[in]	USART_InitStruct Pointer to a USART_CFG_Type structure which will
 * 				be initialized.
 * @return		None
 *******************************************************************************/
void USART_USRT_Mode_ConfigStructInit(USART_CFG_Type *USART_InitStruct)
{
    USART_InitStruct->Mode = USART_SYNC_MODE;
	USART_InitStruct->Baud_rate = 38400;
	USART_InitStruct->Databits = USART_DATABIT_8;
	USART_InitStruct->Parity = USART_PARITY_NONE;
	USART_InitStruct->Stopbits = USART_STOPBIT_1;   
}

/* USART Send/Recieve functions -------------------------------------------------*/
/**********************************************************************
 * @brief		Transmit a single data through USART peripheral
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * @param[in]	Data	Data to transmit (must be 8-bit long)
 * @return 		None
 **********************************************************************/
void USART_SendByte(USART_Type* USARTx, uint8_t Data)
{
	USARTx->DR = Data;
}

/**********************************************************************
 * @brief		Receive a single data from USART peripheral
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * @return 		Data received
 **********************************************************************/
uint8_t USART_ReceiveByte(USART_Type* USARTx)
{
	return (USARTx->DR);
}

/**********************************************************************
 * @brief		Send a block of data via USART peripheral
 * @param[in]	USARTx	Selected USART peripheral used to send data, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @param[in]	txbuf 	Pointer to Transmit buffer
 * @param[in]	buflen 	Length of Transmit buffer
 * @return 		Number of bytes sent.
 *
 **********************************************************************/
uint32_t USART_Send(USART_Type *USARTx, uint8_t *txbuf, uint32_t buflen)
{
	uint32_t bToSend, bSent;
	uint8_t *pChar = txbuf;

	bToSend = buflen;
	bSent = 0;
	while (bToSend){          
		while(!(USARTx->ST & USART_ST_DRE)){ }

		USART_SendByte(USARTx, (*pChar));

		pChar++;
		bToSend--;
		bSent++;
	}
	
	return bSent;
}

/*********************************************************************
 * @brief		Receive a block of data via USART peripheral
 * @param[in]	USARTx	Selected USART peripheral used to send data,
 * 				should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @param[out]	rxbuf 	Pointer to Received buffer
 * @param[in]	buflen 	Length of Received buffer
 * @param[in] 	flag 	Flag mode, should be:
 * 					- NONE_BLOCKING
 * 					- BLOCKING
 * @return 		Number of bytes received
 **********************************************************************/
uint32_t USART_Receive(USART_Type *USARTx, uint8_t *rxbuf, uint32_t buflen)
{
	uint32_t bToRecv, bRecv;
	uint8_t *pChar = rxbuf;

	bToRecv = buflen;
	bRecv = 0;
	while (bToRecv) {
		while(!(USARTx->ST & USART_ST_RXC)){ }
		
		(*pChar++) = USART_ReceiveByte(USARTx);
		bRecv++;
		bToRecv--;
	}
	return bRecv;
}


/********************************************************************
 * @brief 		Enable or disable specified USART interrupt.
 * @param[in]	USARTx	USART peripheral selected, should be
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @param[in]	USTIntCfg	Specifies the interrupt flag,
 * 				should be one of the following:
 * 					- USART_INTCFG_DR 	:DR Interrupt enable
 * 					- USART_INTCFG_TXC 	:TXC Interrupt enable
 * 					- USART_INTCFG_RXC 	:RXC interrupt enable
 * 					- USART_INTCFG_WAKE :WAKE Interrupt enable
 * @param[in]	NewState New state of specified USART interrupt type,
 * 				should be:
 * 					- ENALBE	:Enable this USART interrupt type.
 * 					- DISALBE	:Disable this USART interrupt type.
 * @return 		None
 *********************************************************************/
void USART_IntConfig(USART_Type *USARTx, USART_INT_Type USTIntCfg, FunctionalState NewState)
{
	uint32_t tmp=0;
	
    
	switch(USTIntCfg){
		case USART_INTCFG_WAKE :
			tmp = USART_IER_WAKEINT_EN;
			break;
		case USART_INTCFG_RXC:
			tmp = USART_IER_RXCINT_EN;
			break;
		case USART_INTCFG_TXC:
			tmp = USART_IER_TXCINT_EN;
			break;
		case USART_INTCFG_DRE:
			tmp = USART_IER_DRE_EN;
			break;
	}
    
	if (NewState == ENABLE)
	{
		USARTx->CR1 |= tmp;
	}
	else
	{
		USARTx->CR1 &= ~(tmp & USART_IER_BITMASK) ;
	}
}


/*********************************************************************
 * @brief 		Get current value of Line Status register in USART peripheral.
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @return		Current value of Status register in USART peripheral.
 *********************************************************************/
uint16_t USART_GetStatus(USART_Type* USARTx)
{
	return ((USARTx->ST) & USART_ST_BITMASK);
}

/*********************************************************************
 * @brief 		Clear Status register in USART peripheral.
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 *********************************************************************/
void USART_ClearStatus(USART_Type* USARTx, USART_STATUS_Type Status)
{
    uint32_t tmp;
    
    switch(Status){
        case USART_STATUS_WAKE:
            tmp = USART_ST_WAKE;
            break;
        case USART_STATUS_RXC:
            tmp = USART_ST_RXC;
            break;
        case USART_STATUS_TXC:
            tmp = USART_ST_TXC;
            break;
        case USART_STATUS_DRE:
            tmp = USART_ST_DRE;
            break;
        case USART_STATUS_RTOF:
            tmp = USART_ST_RTOF;
            break;
        case USART_STATUS_DMARXF:
            tmp = USART_ST_DMARXF;
            break;
        case USART_STATUS_DMATXF:
            tmp = USART_ST_DMATXF;
            break;			
        default:
            return;
    }    
    
    USARTx->ST = tmp;
}

/**********************************************************************
 * @brief		Check whether if USART is busy or not
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @return		RESET if USART is not busy, otherwise return SET.
 **********************************************************************/
FlagStatus USART_CheckBusy(USART_Type *USARTx)
{
	if (USARTx->ST & USART_ST_DRE){
		return RESET;
	} else {
		return SET;
	}
}

/*********************************************************************
 * @brief		Configure Data Control mode for USART peripheral
 * @param[in]	USARTx
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral 
 * @param[in]	Data Control mode, should be:
 * 					- USART_CR2_USTEN     :Activate USARTn Block by supplying.
 * 					- USART_CR2_DBLS      :Selects receiver sampling rate. (only UART mode)
 * 					- USART_CR2_MASTER    :Selects master or slave in SPIn or Synchronous mode and controls the direction of SCKn pin.
 * 					- USART_CR2_LOOPS     :Control the Loop Back mode of USARTn for test mode.
 * 					- USART_CR2_DISSCK    :In synchronous mode operation, selects the waveform of SCKn output.
 * 					- USART_CR2_USTSSEN   :This bit controls the SSn pin operation. (only SPI mode)
 * 					- USART_CR2_FXCH      :SPIn port function exchange control bit. (only SPI mode)
 * 					- USART_CR2_USTSB     :Selects the length of stop bit in Asynchronous or Synchronous mode.
 * 					- USART_CR2_USTTX8    :The ninth bit of data frame in Asynchronous or Synchronous mode of operation. Write this bit first before loading the DR register.
 * 					- USART_CR2_USTRX8    :The ninth bit of data frame in Asynchronous or Synchronous mode of operation. Read this bit first before reading the receive buffer (only UART mode)
 * @param[in]	NewState New State of this mode, should be:
 * 					- ENABLE	:Enable this mode.
					- DISABLE	:Disable this mode.
 * @return none
 **********************************************************************/
void USART_DataControlConfig(USART_Type *USARTx, USART_CONTROL_Type Mode, FunctionalState NewState)
{
	uint16_t tmp=0;
    
	switch(Mode){
	case USART_CONTROL_USTRX8:
		tmp = USART_CR2_USTRX8;
		break;
	case USART_CONTROL_USTTX8:
		tmp = USART_CR2_USTTX8;
		break;
	case USART_CONTROL_USTSB:
		tmp = USART_CR2_USTSB;
		break;
	case USART_CONTROL_FXCH:
		tmp = USART_CR2_FXCH;
		break;	
	case USART_CONTROL_USTSSEN:
		tmp = USART_CR2_USTSSEN;
		break;	
	case USART_CONTROL_DISSCK:
		tmp = USART_CR2_DISSCK;
		break;	
	case USART_CONTROL_LOOPS:
		tmp = USART_CR2_LOOPS;
		break;	
	case USART_CONTROL_MASTER:
		tmp = USART_CR2_MASTER;
		break;	
	case USART_CONTROL_DBLS:
		tmp = USART_CR2_DBLS;
		break;	
	case USART_CONTROL_USTEN:
		tmp = USART_CR2_USTEN;
		break;
	default:
		break;
	}

	if (NewState == ENABLE)
	{
		USARTx->CR2 |= tmp;
	}
	else
	{
		USARTx->CR2 &= ~(tmp & USART_CR2_BITMASK) ;
	}
}

/**********************************************************************
 * @brief		Check whether if USART is busy or not
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral
 * @param[in]	USARTx	USART Receive Time-out function, should be:
 * 					- DISABLE	:Disable USART Receive Time-out
 * 					- ENABLE	:Enable USART Receive Time-out
 * @param[in]	USARTx	USART Receive Time-out Interrupt, should be:
 * 					- DISABLE	:Disable USART Receive Time-out interrupt
 * 					- ENABLE	:Enable USART Receive Time-out interrupt
 * @param[in]	USARTx	USART Receive Time-out value, should be:
 * 					- 0x0 ~ 0xFFFFFF
 *
 * @return		none
 **********************************************************************/
void USART_ReceiveTimeOut(USART_Type* USTx, FunctionalState State, FunctionalState Intr, uint32_t TimeOut)
{
	uint32_t Reg32;

	Reg32 = USTx->CR2;
	
	if (State == ENABLE) {
		Reg32 |= USART_CR2_RTOEN;
	} else {
		Reg32 &= ~(USART_CR2_RTOEN);
	}
	
	if (Intr == ENABLE) {
		Reg32 |= USART_CR2_RTOIE;
	} else {
		Reg32 &= ~(USART_CR2_RTOIE);
	}
	
	USTx->CR2 = Reg32; // Update the settings of receive time out
	
	USTx->RTO = (TimeOut & USART_RTO_RTOMASK); // Update the time-out value
		
}

/**********************************************************************
 * @brief		Check whether if USART is busy or not
 * @param[in]	USARTx	USART peripheral selected, should be:
 * 					- USART10	:USART10 peripheral
 * 					- USART11	:USART11 peripheral
 * 					- USART12	:USART12 peripheral
 * 					- USART13	:USART13 peripheral
 * @param[in]	DmaTx	USART Tx DMA Interrupt, should be:
 * 					- DISABLE	:Disable USART Tx Interrupt
 * 					- ENABLE	:Enable USART Tx Interrupt
 * @param[in]	DmaRx	USART Rx DMA Interrupt, should be:
 * 					- DISABLE	:Disable USART Rx Interrupt
 * 					- ENABLE	:Enable USART Rx Interrupt
 * @return		none
 **********************************************************************/
void USART_DmaConfig(USART_Type* USTx, uint32_t DmaInterrupType, FunctionalState en)
{
	uint32_t Reg32;

	Reg32 = USTx->CR2;

	if (DmaInterrupType == USART_CR2_DMATXIE) {
		if (en == ENABLE) {
			Reg32 |= USART_CR2_DMATXIE;
		} else {
			Reg32 &= ~(USART_CR2_DMATXIE);
		}		
	}
	
	else if (DmaInterrupType == USART_CR2_DMARXIE) {
		if (en == ENABLE) {
			Reg32 |= USART_CR2_DMARXIE;
		} else {
			Reg32 &= ~(USART_CR2_DMARXIE);
		}	
	}
	
	USTx->CR2 = Reg32; // Update the settings of USART1n DMA Intererupt
}

/* --------------------------------- End Of File ------------------------------ */
