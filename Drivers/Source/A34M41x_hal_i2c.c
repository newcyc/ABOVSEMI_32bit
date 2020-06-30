/**********************************************************************
* @file		A34M41x_i2c.c
* @brief	Contains all functions support for I2C firmware library
* 			on A34M41x
* @version	1.1
* @date	
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_i2c.h"
#include "A34M41x_hal_libcfg.h"



/* Private Types -------------------------------------------------------------- */
#define BLOCKING_TIMEOUT (0x000FFFFFUL)

/**
 * @brief I2C device configuration structure type
 */
typedef struct
{
	union {
		I2C_M_SETUP_Type	txrx_setup_master; 						/* Transmission setup */
		I2C_S_SETUP_Type	txrx_setup_slave; 						/* Transmission setup */
	};
	int32_t		dir;								/* Current direction phase, 0 - write, 1 - read */
} I2C_CFG_T;


/* Private Variables ---------------------------------------------------------- */
/**
 * @brief II2C driver data for I2C0, I2C1
 */
static I2C_CFG_T i2cdat[2];

static Bool I2C_MasterComplete[2];
static Bool I2C_SlaveComplete[2];

/********************************************************************//**
 * @brief		Convert from I2C peripheral to number
 * @param[in]	I2Cx I2C peripheral selected, should be:
 * 					- IC	:I2C0 peripheral
 * @return 		I2C number or error code, could be:
 * 					- 0		:I2C0
 * 					- 1		:I2C1
 * 					- (-1)	:Error
 *********************************************************************/
static int32_t I2C_getNum(I2C_Type *I2Cx){
	
	if (I2Cx == I2C0) {
		return (0);
	}
	else if (I2Cx == I2C1) {
		return (1);
	}
	
	return (-1);
}

/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief		Initializes the I2Cx peripheral with specified parameter.
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @param[in]	clockrate Target clock rate value to initialized I2C
 * 				peripheral (Hz)
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_I2C_Init(I2C_Type *I2Cx, uint32_t clockrate)
{
	/* Check I2Cx  handle */
         if(I2Cx == NULL)
        {
            return HAL_ERROR;
         }

	if (I2Cx == I2C0)
	{
		SYST_ACCESS_EN();
		
		/* Set up clock for I2C0 module */
		SCU->PER2&=~(1<<4);
		SCU->PCER2&=~(1<<4);
		
		SCU->PER2|=(1<<4);
		SCU->PCER2|=(1<<4);
		
		SYST_ACCESS_DIS();
	}
	else if (I2Cx == I2C1)
	{
		SYST_ACCESS_EN();
		
		/* Set up clock for I2C0 module */
		SCU->PER2&=~(1<<5);
		SCU->PCER2&=~(1<<5);
		
		SCU->PER2|=(1<<5);
		SCU->PCER2|=(1<<5);
		
		SYST_ACCESS_DIS();
	}

	
	I2Cx->CR = (1<<5);  // SOFTRST
	I2Cx->CR = (1<<4) | (3<<8);  // INTEN

	I2Cx->SCLL = (SystemPeriClock / clockrate)/2 -2;  // freq = PCLK / ((SCLL+2) + (SCLH+3)) 
	I2Cx->SCLH = (SystemPeriClock / clockrate)/2 -3; //  ex)100k = 72M / ((358+2) + (357+3)) ,if PCLK:72Mhz.
	I2Cx->SDH = 1;
	
	I2Cx->CR |= (1<<3) ;            // ACKEN	
	return HAL_OK;
}


/**********************************************************************
 * @brief		De-initializes the I2C peripheral registers to their
 *                  default reset values.
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_I2C_DeInit(I2C_Type* I2Cx)
{
	/* Check I2Cx  handle */
         if(I2Cx == NULL)
        {
            return HAL_ERROR;
         }
	/* Disable I2C control */
	if (I2Cx == I2C0)
	{
		SYST_ACCESS_EN();
		
		/* Set up clock for I2C0 module */
		SCU->PER2&=~(1<<4);
		SCU->PCER2&=~(1<<4);
		
		SYST_ACCESS_DIS();
	}
	else if (I2Cx == I2C1)
	{
		SYST_ACCESS_EN();
		
		/* Set up clock for I2C0 module */
		SCU->PER2&=~(1<<5);
		SCU->PCER2&=~(1<<5);
		
		SYST_ACCESS_DIS();
	}
	return HAL_OK;
}

/*********************************************************************//**
 * @brief 		Enable/Disable interrupt for I2C peripheral
 * @param[in]	I2Cx	I2C peripheral selected, should be:
 * 					- IC	:I2C0 peripheral
 * @param[in]	NewState	New State of I2C peripheral interrupt in NVIC core
 * 				should be:
 * 					- ENABLE: enable interrupt for this I2C peripheral
 * 					- DISABLE: disable interrupt for this I2C peripheral
 * @return 		None
 **********************************************************************/
void  HAL_I2C_ConfigInterrupt(I2C_Type *I2Cx, Bool NewState)
{
	if (NewState)
	{
		if(I2Cx == I2C0)
		{
			NVIC_ClearPendingIRQ(I2C0_IRQn);
			NVIC_EnableIRQ(I2C0_IRQn);
		}
		else if(I2Cx == I2C1)
		{
			NVIC_ClearPendingIRQ(I2C1_IRQn);
			NVIC_EnableIRQ(I2C1_IRQn);
		}
	}
	else
	{
		if(I2Cx == I2C0)
		{
			NVIC_DisableIRQ(I2C0_IRQn);
		}
		else if(I2Cx == I2C1)
		{
			NVIC_DisableIRQ(I2C1_IRQn);
		}
	}
	
    return;
}

/*********************************************************************//**
 * @brief 		General Master Interrupt handler for I2C peripheral
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_I2C_Master_IRQHandler_IT(I2C_Type  *I2Cx)
{
	int32_t tmp;
	I2C_M_SETUP_Type *txrx_setup;
	uint32_t status;

	/* Check I2Cx  handle */
         if(I2Cx == NULL)
        {
            return HAL_ERROR;
         }
	tmp = I2C_getNum(I2Cx);
	txrx_setup = (I2C_M_SETUP_Type *)&i2cdat[tmp].txrx_setup_master;

	status = I2Cx->SR;
	switch(status){
		case 0x87: // transmit mode - addr ACK
			if(txrx_setup->tx_count < txrx_setup->tx_length){
				I2Cx->DR = txrx_setup->tx_data[txrx_setup->tx_count];
				txrx_setup->tx_count++;
			}
			else {
				I2Cx->CR= 0
				|(3<<8)
				|(1<<4)     // INTEN 
				|(1<<1);	// STOP
			}
			break;
				
		case 0x47: // transmit mode - data ACK 
			if(txrx_setup->tx_count < txrx_setup->tx_length){
				I2Cx->DR = txrx_setup->tx_data[txrx_setup->tx_count];
				txrx_setup->tx_count++;
			}
			else {
				if(txrx_setup->rx_count < txrx_setup->rx_length){	
					I2Cx->DR = ((txrx_setup->sl_addr7bit << 1) | 0x01);  // 0:write, 1:read
					I2Cx->CR |= (1<<0);	// reSTART						
				}
				else {
					I2Cx->CR=0
					|(3<<8)
					|(1<<4)     // INTEN
					|(1<<1);	// STOP
				}
			}				
			break;
				
		case 0x85: // receive mode - addr ACK
			if(txrx_setup->rx_count < txrx_setup->rx_length){
				if ((txrx_setup->rx_length > 1) && (txrx_setup->rx_count < (txrx_setup->rx_length - 1))){
					__NOP();
				}
				else {
					I2Cx->CR=(1<<4) | (3<<8);	// disable ACKEN 
				}
			}
			else{
				I2Cx->CR= 0
				|(3<<8)
				|(1<<4)     // INTEN 
				|(1<<1);	// STOP				
			}
			break;
				
		case 0x45: // receive mode - data ACK 
			if(txrx_setup->rx_count < txrx_setup->rx_length){
				txrx_setup->rx_data[txrx_setup->rx_count] = I2Cx->DR;
				txrx_setup->rx_count++;
					
				if ((txrx_setup->rx_length > 1) && (txrx_setup->rx_count < (txrx_setup->rx_length - 1))){
					__NOP();
				}
				else {
					I2Cx->CR=(1<<4) | (3<<8);	// disable ACKEN 
				}
			}
			break;
			
		case 0x44: // receive mode - data NOACK 
			if(txrx_setup->rx_count < txrx_setup->rx_length){
				txrx_setup->rx_data[txrx_setup->rx_count] = I2Cx->DR;
				txrx_setup->rx_count++;
					
				I2Cx->CR=0
				|(3<<8)				
				|(1<<4) 
				|(1<<1);	// INTEN + STOP  
			}
			break;			

//		case 0x20: // receive mode				
//		case 0x22: // transmit mode - stop receive 
//			I2Cx->CR=0
//			|(3<<8)			
//			|(1<<4) 
//			|(1<<3);	// ACKEN				
//			goto s_int_end;
			
		default:
			if (status & 0x08) { // mastership lost
				
			}
			break;
	}

	
//s_int_end:
	if((status == 0x20) || (status == 0x22))
	{
			I2Cx->CR=0
			|(3<<8)			
			|(1<<4) 
			|(1<<3);	// ACKEN
			I2Cx->SR=0xff; // flag clear and SCL go to HIGH
			// Disable interrupt
			HAL_I2C_ConfigInterrupt(I2Cx, FALSE);
			I2C_MasterComplete[tmp] = TRUE;
	}
	else
	{
			I2Cx->SR=0xff; // flag clear and SCL go to HIGH
//			return;
	}
	
//	I2Cx->SR=0xff; // flag clear and SCL go to HIGH
	// Disable interrupt
//	I2C_IntCmd(I2Cx, FALSE);
//	I2C_MasterComplete[tmp] = TRUE;
	return HAL_OK;

}

/*********************************************************************//**
 * @brief 		General Slave Interrupt handler for I2C peripheral
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_I2C_Slave_IRQHandler_IT(I2C_Type  *I2Cx)
{
	int32_t tmp;
	I2C_S_SETUP_Type *txrx_setup;
	uint32_t status;

	/* Check I2Cx  handle */
         if(I2Cx == NULL)
        {
            return HAL_ERROR;
         }
	tmp = I2C_getNum(I2Cx);
	txrx_setup = (I2C_S_SETUP_Type *)&i2cdat[tmp].txrx_setup_slave;

	status = I2Cx->SR;
	switch(status){
		case 0x15: // receive mode - slave select + ACK
		case 0x45: // receive mode - data ACK 			
			if ((txrx_setup->rx_count < txrx_setup->rx_length) && (txrx_setup->rx_data != NULL))
			{
				txrx_setup->rx_data[txrx_setup->rx_count]=I2Cx->DR;
				txrx_setup->rx_count++;
			}	
			break;
	
//		case 0x20: // receive mode				
//		case 0x22: // transmit mode - stop receive 
//			goto s_int_end;

		case 0x17: // transmit mode - slave select + ACK
		case 0x46: // transmit mode - data NOACK 										
		case 0x47: // transmit mode - data ACK 			
			if ((txrx_setup->tx_count < txrx_setup->tx_length) && (txrx_setup->tx_data != NULL))
			{
				I2Cx->DR = txrx_setup->tx_data[txrx_setup->tx_count];
				txrx_setup->tx_count++;
			}			
			break;
	}
	
	if((status == 0x20) || (status == 0x22))
	{
			I2Cx->SR=0xff; // flag clear and SCL go to HIGH
			// Disable interrupt
			HAL_I2C_ConfigInterrupt(I2Cx, FALSE);
			I2C_SlaveComplete[tmp] = TRUE;
	}
	else
	{
			I2Cx->SR=0xff; // flag clear and SCL go to HIGH
//			return;
	}
//s_int_end:
//	I2Cx->SR=0xff; // flag clear and SCL go to HIGH
//	// Disable interrupt
//	I2C_IntCmd(I2Cx, FALSE);
//	I2C_SlaveComplete[tmp] = TRUE;
         return HAL_OK;


}

/**********************************************************************
 * @brief 		wait and return status in master mode
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @return 		return status 
 **********************************************************************/
int32_t HAL_I2C_MWait(I2C_Type *I2Cx)
{
	uint32_t tmp;
	int32_t ret = 0;
	
	I2Cx->SR=0xFF;
	while(1)
	{
		if((I2Cx->CR & 0x80) != 0) break;
	}

	tmp = I2Cx->SR;
	switch(tmp)
	{
		// Transmitter mode
		case 0x87: ret = TRANS_MODE; break;		
		// Receive mode
		case 0x85: ret = RECEIVE_MODE; break;
		// Transed Data
		case 0x47: ret = TRANS_DATA; break;
		// Received Data
		case 0x44:
		case 0x45: ret = RECEIVE_DATA; break;
		default:
			if(tmp&0x08) ret = LOST_BUS;		// lost
			else if(tmp&0x20) ret = STOP_DECT;		// stop
			else
			{
				ret = -1;
			}
		break;
	}

	return ret;
}

/**********************************************************************
 * @brief 		Transmit and Receive data in master mode
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @param[in]	TransferCfg		Pointer to a I2C_M_SETUP_Type structure that
 * 								contains specified information about the
 * 								configuration for master transfer.
 * @param[in]	Opt				a I2C_TRANSFER_OPT_Type type that selected for
 * 								interrupt or polling mode.
 * @return 		SUCCESS or ERROR
 **********************************************************************/ 
Status HAL_I2C_MasterTransferData(I2C_Type* I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt)
{
	int32_t tmp;
	uint32_t exitflag;
	int32_t Ret;

	// Reset I2C setup value to default state
	TransferCfg->tx_count = 0;
	TransferCfg->rx_count = 0;

	while(I2Cx->SR&0x04);	// busy check
	
	if (Opt == I2C_TRANSFER_POLLING)
	{
		/* First Start condition -------------------------------------------------------------- */
		// Reset I2C setup value to default state
		TransferCfg->tx_count = 0;
		TransferCfg->rx_count = 0;
		
		if(TransferCfg->tx_count < TransferCfg->tx_length){
			I2Cx->DR = (TransferCfg->sl_addr7bit << 1); //write
			
			// Start command		
			I2Cx->CR|=(1<<0);	// START	
			Ret = HAL_I2C_MWait(I2Cx);
			
			if ((Ret != TRANS_MODE)){
				I2Cx->CR|=(1<<1);	// STOP
				HAL_I2C_MWait(I2Cx);
				I2Cx->SR=0xFF;
				I2Cx->CR = 0
				|(3<<8)				
				|(1<<4)   // INTEN
				|(1<<3) ; // ACKEN
				return ERROR; 
			}
			
			exitflag=1;
			while(exitflag){
				if(TransferCfg->tx_count < TransferCfg->tx_length)
				{
					I2Cx->DR = TransferCfg->tx_data[TransferCfg->tx_count];
					TransferCfg->tx_count++;					
					Ret = HAL_I2C_MWait(I2Cx);
					
					if ((Ret != TRANS_DATA)){
						I2Cx->CR|=(1<<1);	// STOP
						HAL_I2C_MWait(I2Cx);
						I2Cx->SR=0xFF;
						I2Cx->CR = 0
						|(3<<8)						
						|(1<<4)   // INTEN
						|(1<<3) ; // ACKEN
						return ERROR; 
					}				
				}
				else 
				{
					if(TransferCfg->rx_count >= TransferCfg->rx_length)
					{
						I2Cx->CR|=(1<<1);	// STOP
						HAL_I2C_MWait(I2Cx);
						I2Cx->SR=0xFF;
						I2Cx->CR = 0
						|(3<<8)						
						|(1<<4)   // INTEN
						|(1<<3) ; // ACKEN
						return SUCCESS;
					}
					else {
						exitflag=0;
					}
				}
			}
		}
		
		if(TransferCfg->rx_count < TransferCfg->rx_length){
			I2Cx->DR = ((TransferCfg->sl_addr7bit << 1) | 0x01); 
			I2Cx->CR|=(1<<0);	// START			
			Ret = HAL_I2C_MWait(I2Cx);
			
			if ((Ret != RECEIVE_MODE)){
				I2Cx->CR|=(1<<1);	// STOP			
				HAL_I2C_MWait(I2Cx);
				I2Cx->SR=0xFF;
				I2Cx->CR = 0
				|(3<<8)				
				|(1<<4)   // INTEN
				|(1<<3) ; // ACKEN
				return ERROR; 
			}
			
			exitflag=1;
			while(exitflag){		
				if ((TransferCfg->rx_length > 1) && (TransferCfg->rx_count < (TransferCfg->rx_length - 1))){
					Ret = HAL_I2C_MWait(I2Cx);
				
					if ((Ret != RECEIVE_DATA)){
						I2Cx->CR|=(1<<1);	// STOP
						HAL_I2C_MWait(I2Cx);
						I2Cx->SR=0xFF;
						I2Cx->CR = 0
						|(3<<8)						
						|(1<<4)   // INTEN
						|(1<<3) ; // ACKEN
						return ERROR; 
					}
				}
				else {  // the next byte is the last byte, send NACK instead.
					I2Cx->CR&=(1<<4);	// INTEN
					Ret = HAL_I2C_MWait(I2Cx);
				
					if ((Ret != RECEIVE_DATA)){
						I2Cx->CR|=(1<<1);	// STOP
						HAL_I2C_MWait(I2Cx);
						I2Cx->SR=0xFF;
						I2Cx->CR = 0
						|(3<<8)						
						|(1<<4)   // INTEN
						|(1<<3) ; // ACKEN
						return ERROR; 
					}						
				}
				TransferCfg->rx_data[TransferCfg->rx_count]=I2Cx->DR;
				TransferCfg->rx_count++;
				if(TransferCfg->rx_count == TransferCfg->rx_length){
					exitflag=0;					
				}
			}
			
			I2Cx->CR|=(1<<1);	// STOP
			HAL_I2C_MWait(I2Cx);
			I2Cx->SR=0xFF;
			I2Cx->CR = 0
			|(3<<8)			
			|(1<<4)   // INTEN
			|(1<<3) ; // ACKEN
			return SUCCESS;			
		}
	}
	else if (Opt == I2C_TRANSFER_INTERRUPT)
	{
		tmp = I2C_getNum(I2Cx);
		I2C_MasterComplete[tmp] = FALSE;
		// Setup tx_rx data, callback and interrupt handler
		i2cdat[tmp].txrx_setup_master = *TransferCfg;

		// Set direction phase, write first
		i2cdat[tmp].dir = 0;

		/* First Start condition -------------------------------------------------------------- */
		HAL_I2C_ConfigInterrupt(I2Cx, TRUE);
		if(TransferCfg->tx_count < TransferCfg->tx_length){
			I2Cx->DR = (TransferCfg->sl_addr7bit << 1); //write
		}
		else if(TransferCfg->rx_count < TransferCfg->rx_length){
			I2Cx->DR = ((TransferCfg->sl_addr7bit << 1) | 0x01); 
		}
		
		// Start command		
		I2Cx->CR|=(1<<0);	// START	

		return (SUCCESS);
	}

	return ERROR;
}

/**********************************************************************
 * @brief 		wait and return status in slave mode
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @return 		return status 
 **********************************************************************/
int32_t HAL_I2C_SWait(I2C_Type *I2Cx)
{
	uint32_t tmp;
	int32_t ret = 0;
	
	I2Cx->SR=0xFF;
	while(1)
	{
		if((I2Cx->CR & 0x80) != 0) break;
	}

	tmp = I2Cx->SR;	
	
	switch(tmp)
	{
		// Receive mode
		case 0x15:
		case 0x95: ret = RECEIVE_MODE; break;
		// Transmitter mode
		case 0x17:
		case 0x97: ret = TRANS_MODE; break;
		// Received Data
		case 0x45: ret = RECEIVE_DATA; break;
		// Transed Data
		case 0x47: ret = TRANS_DATA; break;
		default:
			if(tmp&0x08) ret = LOST_BUS;		// lost
			else if(tmp&0x20) ret = STOP_DECT;		// stop
			else
			{
				ret = -1;
			}
		break;
	}

	return ret;
}

/*********************************************************************//**
 * @brief 		Receive and Transmit data in slave mode
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @param[in]	TransferCfg		Pointer to a I2C_S_SETUP_Type structure that
 * 								contains specified information about the
 * 								configuration for master transfer.
 * @param[in]	Opt				I2C_TRANSFER_OPT_Type type that selected for
 * 								interrupt or polling mode.
 * @return 		SUCCESS or ERROR
 **********************************************************************/
Status HAL_I2C_SlaveTransferData(I2C_Type* I2Cx, I2C_S_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt)
{
	int32_t tmp;
	int32_t Ret;

	// Reset I2C setup value to default state
	TransferCfg->tx_count = 0;
	TransferCfg->rx_count = 0;

	// Polling option
	if (Opt == I2C_TRANSFER_POLLING)
	{
		while (1)
		{
			Ret = HAL_I2C_SWait(I2Cx);	// Start
			switch(Ret)
			{				
				case RECEIVE_MODE:
				case RECEIVE_DATA:					
					if ((TransferCfg->rx_count < TransferCfg->rx_length) && (TransferCfg->rx_data != NULL))
					{
						TransferCfg->rx_data[TransferCfg->rx_count]=I2Cx->DR;
						TransferCfg->rx_count++;
					}				
					break;
				case TRANS_MODE:
				case TRANS_DATA:
					if ((TransferCfg->tx_count < TransferCfg->tx_length) && (TransferCfg->tx_data != NULL))
					{
						I2Cx->DR = TransferCfg->tx_data[TransferCfg->tx_count];
						TransferCfg->tx_count++;
					}
					break;
				case STOP_DECT:
					goto s_end_stage;
				case 0: break;
				default:
					goto s_error;
			}			
		}			

s_end_stage:
		I2Cx->SR=0xFF;
		return SUCCESS;

s_error:
		I2Cx->SR=0xFF;
		return ERROR;
	}

	else if (Opt == I2C_TRANSFER_INTERRUPT)
	{
		tmp = I2C_getNum(I2Cx);
		I2C_SlaveComplete[tmp] = FALSE;
		// Setup tx_rx data, callback and interrupt handler
		i2cdat[tmp].txrx_setup_slave = *TransferCfg;

		// Set direction phase, read first
		i2cdat[tmp].dir = 1;

		HAL_I2C_ConfigInterrupt(I2Cx, TRUE);

		return (SUCCESS);
	}

	return ERROR;
}

/********************************************************************//**
 * @brief		Transmit an array of bytes in Master mode
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @param[in]	TransferCfg   Pointer to a I2C_M_SETUP_Type structure that
 * 								contains specified information about the
 * 								configuration for master transfer.
 * @param[in]	Opt				a I2C_TRANSFER_OPT_Type type that selected for
 * 								interrupt or polling mode.
 * @return 		SUCCESS or ERROR
 *********************************************************************/
Status	HAL_I2C_Master_Transmit(I2C_Type* I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt)
{
	TransferCfg->rx_data = NULL;
	TransferCfg->rx_length = 0;
	TransferCfg->tx_count = 0;
	TransferCfg->rx_count = 0;

	return HAL_I2C_MasterTransferData(I2Cx, TransferCfg, Opt);
}

/********************************************************************//**
 * @brief		Receive an array of bytes in Master mode
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @param[in]	TransferCfg   Pointer to a I2C_M_SETUP_Type structure that
 * 								contains specified information about the
 * 								configuration for master transfer.
 * @param[in]	Opt				a I2C_TRANSFER_OPT_Type type that selected for
 * 								interrupt or polling mode.
 * @return 		SUCCESS or ERROR
 *********************************************************************/
Status HAL_I2C_Master_Receive(I2C_Type* I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt)
{
	TransferCfg->tx_data = NULL;
	TransferCfg->tx_length = 0;
	TransferCfg->tx_count = 0;
	TransferCfg->rx_count = 0;

	return HAL_I2C_MasterTransferData(I2Cx, TransferCfg, Opt);
}

/********************************************************************//**
 * @brief		Receive an array of bytes in Slave mode
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @param[in]	TransferCfg   Pointer to a I2C_S_SETUP_Type structure that
 * 								contains specified information about the
 * 								configuration for slave transfer.
 * @param[in]	Opt				a I2C_TRANSFER_OPT_Type type that selected for
 * 								interrupt or polling mode.
 * @return 		SUCCESS or ERROR
 *********************************************************************/
Status HAL_I2C_Slave_Receive(I2C_Type* I2Cx, I2C_S_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt)
{
	TransferCfg->tx_data = NULL;
	TransferCfg->tx_length = 0;
	TransferCfg->tx_count = 0;
	TransferCfg->rx_count = 0;

	return HAL_I2C_SlaveTransferData(I2Cx, TransferCfg, Opt);
}

/*********************************************************************//**
 * @brief		Set Own slave address in I2C peripheral corresponding to
 * 				parameter specified in OwnSlaveAddrConfigStruct.
 * @param[in]	I2Cx	I2C peripheral selected, should be
 * 					- IC	:I2C0 peripheral
 * @param[in]	SlaveAddr_7bit : own slave address 
 * @param[in]	GeneralCallState : 
 *                     - ENABLE
 *                     - DISABLE
 * @return 		None
 **********************************************************************/
HAL_Status_Type HAL_I2C_Slave_SetAddress(I2C_Type *I2Cx, uint8_t SlaveAddr_7bit, uint8_t GeneralCallState)
{
	/* Check I2Cx  handle */
         if(I2Cx == NULL)
        {
            return HAL_ERROR;
         }
	I2Cx->SAR = (((uint32_t)(SlaveAddr_7bit << 1)) | ((GeneralCallState == ENABLE) ? 0x01 : 0x00))& I2C_I2ADR_BITMASK;
	return HAL_OK;
}

/*********************************************************************//**
 * @brief 		Get status of Master Transfer
 * @param[in]	I2Cx	I2C peripheral selected, should be:
 * 					- IC	:I2C0 peripheral
 * @return 		Master transfer status, could be:
 * 					- TRUE		:master transfer completed
 * 					- FALSE 	:master transfer have not completed yet
 **********************************************************************/
uint32_t HAL_I2C_Master_GetState(I2C_Type *I2Cx)
{
	uint32_t retval, tmp;
	
	tmp = I2C_getNum(I2Cx);
	retval = I2C_MasterComplete[tmp];
	I2C_MasterComplete[tmp] = FALSE;
	return retval;
}

/*********************************************************************//**
 * @brief 		Get status of Slave Transfer
 * @param[in]	I2Cx	I2C peripheral selected, should be:
 * 					- IC	:I2C0 peripheral
 * @return 		Complete status, could be: TRUE/FALSE
 **********************************************************************/
uint32_t HAL_I2C_Slave_GetState(I2C_Type *I2Cx)
{
	uint32_t retval, tmp;
	
	tmp = I2C_getNum(I2Cx);
	retval = I2C_SlaveComplete[tmp];
	I2C_SlaveComplete[tmp] = FALSE;
	return retval;
}


/* --------------------------------- End Of File ------------------------------ */
