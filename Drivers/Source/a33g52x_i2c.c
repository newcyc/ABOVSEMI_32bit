/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_i2c.c
*
* @ Author : Application Team, ABOV Semiconductor Co., Ltd
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
*
* @ Description
*   ABOV Semiconductor is supplying this software for use with HART-m310
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



#include <stdint.h>
#include "A33G52x.h"
#include "a33g52x_i2c.h"
#include "a33g52x_pcu.h"
#include "a33g52x_nvic.h"


uint8_t	g_I2C_TargetAddr;

int		g_I2C0_index;
uint32_t	g_I2C0_status[40];

int		g_I2C1_index;
uint32_t	g_I2C1_status[40];


extern uint32_t	g_I2C0_count;
extern uint32_t	g_I2C1_count;


I2C_BUFFER		sI2C0_Master_Buffer;
I2C_BUFFER		sI2C0_Slave_Buffer;


I2C_BUFFER		sI2C1_Master_Buffer;
I2C_BUFFER		sI2C1_Slave_Buffer;


//================================================================================
// I2C PORT (no differentiation between master and slave)
//
//
//				first index			i2c_no
//				second index			SCL, SDA
//				third index			port base address, pin no, mux data
//
//================================================================================
uint32_t 	g_I2C_PORT[2][2][3] = {

	// I2C0
	{
		{PCB_BASE, PIN_14, PB14_MUX_SCL0}, 		// SCL0
		{PCB_BASE, PIN_15, PB15_MUX_SDA0}		// SDA0
	},


	// I2C1
	{
		{PCD_BASE, PIN_14, PD14_MUX_SCL1}, 		// SCL1
		{PCD_BASE, PIN_15, PD15_MUX_SDA1}		// SDA1
	}

};


/**
*********************************************************************************************************
* @ Name : I2C_ConfigureGPIO
*
* @ Parameters
*		- i2c : I2C0, I2C1
*
*
*********************************************************************************************************
*/
void I2C_ConfigureGPIO (I2C_Type * const i2c)
{
	int 			i2c_no;

	PCU_Type		*scl_port_addr, *sda_port_addr;
	uint32_t			scl_pin_no, sda_pin_no;
	uint32_t			scl_mux_info, sda_mux_info;



	//--------------------------------------------------------------------------------
	// get i2c_no
	//--------------------------------------------------------------------------------
	if (i2c == I2C0) i2c_no = 0;
	else if (i2c == I2C1) i2c_no = 1;



	//--------------------------------------------------------------------------------
	// setting info
	//--------------------------------------------------------------------------------
	scl_port_addr = (PCU_Type *) g_I2C_PORT[i2c_no][0][0]; 			// SCL, port base address
	scl_pin_no = g_I2C_PORT[i2c_no][0][1]; 							// SCL, pin no
	scl_mux_info = g_I2C_PORT[i2c_no][0][2]; 						// SCL, mux info

	sda_port_addr = (PCU_Type *) g_I2C_PORT[i2c_no][1][0]; 		// SDA, port base address
	sda_pin_no = g_I2C_PORT[i2c_no][1][1]; 						// SDA, pin no
	sda_mux_info = g_I2C_PORT[i2c_no][1][2]; 						// SDA, mux info



	//--------------------------------------------------------------------------------
	// setting
	//--------------------------------------------------------------------------------
	PCU_ConfigureFunction (scl_port_addr, scl_pin_no, scl_mux_info);
	PCU_ConfigureFunction (sda_port_addr, sda_pin_no, sda_mux_info);

	PCU_Set_Direction_Type (scl_port_addr, scl_pin_no, PnCR_OUTPUT_OPEN_DRAIN);
	PCU_Set_Direction_Type (sda_port_addr, sda_pin_no, PnCR_OUTPUT_OPEN_DRAIN);

	PCU_ConfigurePullup_Pulldown(scl_port_addr, scl_pin_no, 0, PnPCR_PULLUPDOWN_DISABLE);
	PCU_ConfigurePullup_Pulldown(sda_port_addr, sda_pin_no, 0, PnPCR_PULLUPDOWN_DISABLE);

}




/**
*********************************************************************************************************
* @ Name : I2C_Init
*
* @ Parameters
*		- i2c : I2C0, I2C1
*		- master_slave : I2C_MASTER, I2C_SLAVE
*		- p_config
*			# slave_addr : ~ 0x7F
*			# general_call : I2C_GENERAL_CALL_ENABLE, I2C_GENERAL_CALL_DISABLE
*			# scl_low_duration: 0x0000 ~ 0xFFFF
*			# scl_high_duration: 0x0000 ~ 0xFFFF
*			# sda_hold_duration: 0x0000 ~ 0x7FFF
*			# interval : 0~3
*
* @ Return
*		none
*
*
*********************************************************************************************************
*/
void I2C_Init (I2C_Type * const i2c, int master_slave, I2C_CONFIG * p_config)
{
	uint32_t			reg_val;
	volatile int		delay;



	//----------------------------------------------------------------------------------------
	// software reset
	//
	//				@ IC0CR = 0x4000_0A14
	//				@ IC1CR = 0x4000_0A94
	//
	//----------------------------------------------------------------------------------------
	i2c->CR = ICnCR_SOFTRST;

	for (delay=0; delay<5; delay++);

	i2c->CR = 0;


	//----------------------------------------------------------------------------------------
	// SCL low duration
	//
	//				@ IC0SCLL = 0x4000_0A18
	//				@ IC1SCLL = 0x4000_0A98
	//
	//----------------------------------------------------------------------------------------
	reg_val = (p_config->scl_low_duration & ICnSCLL_SCLL_MASK);
	i2c->SCLL = reg_val;



	//----------------------------------------------------------------------------------------
	// SCL high duration
	//
	//				@ IC0SCLH = 0x4000_0A1C
	//				@ IC1SCLH = 0x4000_0A9C
	//
	//----------------------------------------------------------------------------------------
	reg_val = (p_config->scl_high_duration & ICnSCLH_SCLH_MASK);
	i2c->SCLH = reg_val;



	//----------------------------------------------------------------------------------------
	// SDA hold duration
	//
	//				@ IC0SDH = 0x4000_0A20
	//				@ IC1SDH = 0x4000_0AA0
	//
	//----------------------------------------------------------------------------------------
	reg_val = (p_config->sda_hold_duration & ICnSDH_SDH_MASK);
	i2c->SDH = reg_val;


	//----------------------------------------------------------------------------------------
	// ICnSAR
	//
	//				@ IC0SAR = 0x4000_0A0C
	//				@ IC1SAR = 0x4000_0A8C
	//
	//----------------------------------------------------------------------------------------
	reg_val = ((p_config->slave_addr<<1) & 0xFE);
	if (p_config->general_call == I2C_GENERAL_CALL_ENABLE) reg_val |= ICnSAR_GCEN;

	i2c->SAR = reg_val;




	//----------------------------------------------------------------------------------------
	// ICnCR
	//
	//				@ IC0CR = 0x4000_0A14
	//				@ IC1CR = 0x4000_0A94
	//
	//----------------------------------------------------------------------------------------
	reg_val = 0;


	reg_val = ICnCR_INTERVAL_VAL(p_config->interval);


	reg_val |= ICnCR_ACKEN;
	i2c->CR = reg_val;

}




/**
*********************************************************************************************************
* @ Name : I2C_ConfigureInterrupt
*
* @ Parameters
*		- i2c : I2C0, I2C1
*		- intr_mask : ICnCR_INTEN
*		- enable : INTR_ENABLE, INTR_DISABLE
*
*
*********************************************************************************************************
*/
void I2C_ConfigureInterrupt (I2C_Type * const i2c, uint32_t intr_mask, uint32_t enable)
{
	uint32_t		reg_val;

	//----------------------------------------------------------------------------------------
	// disable interrupt
	//
	//				@ IC0CR = 0x4000_0A14
	//				@ IC1CR = 0x4000_0A94
	//
	//----------------------------------------------------------------------------------------
	reg_val = i2c->CR;
	reg_val &= ~(ICnCR_INTEN);
	i2c->CR = reg_val;


	//----------------------------------------------------------------------------------------
	// clear interrupt flag
	//
	//				@ IC0CR = 0x4000_0A08
	//				@ IC1CR = 0x4000_0A88
	//
	//----------------------------------------------------------------------------------------
	 i2c->SR = 0xFF;



	//----------------------------------------------------------------------------------------
	// enable interrupt
	//
	//				@ IC0CR = 0x4000_0A14
	//				@ IC1CR = 0x4000_0A94
	//
	//----------------------------------------------------------------------------------------
	if (enable == INTR_ENABLE)
	{
		reg_val = i2c->CR;
		reg_val |= (intr_mask & ICnCR_INTEN);
		i2c->CR = reg_val;
	}


}



////////////////////////////////////////////////////////////////////////////////////////////////////
/**
************************************************************************************************************
* @ Name: _I2C_Init
*
* @ Parameter
*		- i2c_no : 0, 1
*		- master_slave : I2C_MASTER, I2C_SLAVE
*
*		- p_config
*			# slave_addr			= slave addr (0x01~0x7F, unshifted address)
*			# general_call 		= I2C_GENERAL_CALL_ENABLE, I2C_GENERAL_CALL_DISABLE
*			# scl_low_duration		=
*			# scl_high_duration	=
*			# sda_hold_duration	=
*			# interval			= 0~3
*
* @ return
*		0		success
*		1 		error
*
************************************************************************************************************
*/
int _I2C_Init (int i2c_no, int master_slave, I2C_CONFIG * p_config)
{

	I2C_Type				* i2c;
	NVIC_IntrConfig			nvic_config;


	//------------------------------------------------------------------------------------------
	// check i2c_no
	//------------------------------------------------------------------------------------------
	if ((i2c_no < 0) || (i2c_no > 1)) return (1);



	//------------------------------------------------------------------------------------------
	// get object
	//------------------------------------------------------------------------------------------
	i2c = I2C_Get_Object (i2c_no);


	//------------------------------------------------------------------------------------------
	// configure GPIO
	//------------------------------------------------------------------------------------------
	I2C_ConfigureGPIO(i2c);


	//------------------------------------------------------------------------------------------
	// buffer setting
	//------------------------------------------------------------------------------------------
	I2C_InitBuffer (i2c_no, master_slave);


	//------------------------------------------------------------------------------------------
	// mode setting
	//------------------------------------------------------------------------------------------
	I2C_Init (i2c, master_slave, p_config);



	//------------------------------------------------------------------------------------------
	// interrupt (peripheral)
	//------------------------------------------------------------------------------------------
	I2C_ConfigureInterrupt (i2c, ICnCR_INTEN, INTR_ENABLE);


	//------------------------------------------------------------------------------------------
	// interrupt (NVIC)
	//
	//					I2C0		: 36
	//					I2C1		: 37
	//------------------------------------------------------------------------------------------
	nvic_config.nIRQ_Number = (IRQ_I2C0+i2c_no);
	nvic_config.Preemption_Priority = PRIO_I2C0_PREEMPTION;
	//nvic_config.Preemption_Priority = 0;
	nvic_config.Subpriority = PRIO_I2C0_SUBPRIORITY;
	nvic_config.IntrEnable = INTR_ENABLE;
	NVIC_ConfigureInterrupt(NVIC, &nvic_config);


	//------------------------------------------------------------------------------------------
	// enable
	//------------------------------------------------------------------------------------------

	return (0);

}




/**
************************************************************************************************************
* @ Name: I2C_Get_Object
*
* @ Parameter
*		- i2c_no : 0, 1
*
* @ Return
*		I2C object
*
*
************************************************************************************************************
*/
I2C_Type* I2C_Get_Object (int i2c_no)
{

	I2C_Type *	p_obj;

	switch (i2c_no)
	{
	case 0:
		p_obj = I2C0;
		break;

	case 1:
		p_obj = I2C1;
		break;

	default:
		p_obj = (I2C_Type *) 0;
		break;
	}

	return (p_obj);

}



/**
************************************************************************************************************
* @ Name: I2C_InitBuffer
*
* @ Parameter
*		- i2c_no : 0, 1
*		- master_slave : I2C_MASTER, I2C_SLAVE
*
* @ Return
*		0 = success
*		1 = fail
*
*
************************************************************************************************************
*/
int I2C_InitBuffer (int i2c_no, int master_slave)
{

	I2C_BUFFER			*p_I2C_buffer;
	int					i;
	int					result;


	//------------------------------------------------------------------------------------------
	// get base address
	//------------------------------------------------------------------------------------------
	p_I2C_buffer = I2C_Get_BufferAddr (i2c_no, master_slave, &result);


	if (result == I2C_CHANNEL_NOT_SUPPORTED)
	{
		return (1);
	}



	//------------------------------------------------------------------------------------------
	// init buffer variables
	//------------------------------------------------------------------------------------------
	p_I2C_buffer->u8RxState = I2C_RX_STATE_IDLE;
	p_I2C_buffer->u8TxState = I2C_TX_STATE_IDLE;

	p_I2C_buffer->u16RxBuffer_HeadIndex = 0;
	p_I2C_buffer->u16RxBuffer_TailIndex = 0;
	p_I2C_buffer->u16TxBuffer_HeadIndex = 0;
	p_I2C_buffer->u16TxBuffer_TailIndex = 0;

	for (i=0; i<I2C_MAX_RX_BUFFER; i++)
	{
		p_I2C_buffer->u8RxBuffer[i] = 0;
	}


	for (i=0; i<I2C_MAX_TX_BUFFER; i++)
	{
		p_I2C_buffer->u8TxBuffer[i] = 0;
	}


	return (0);

}




/**
************************************************************************************************************
* @ Name: I2C_Get_BufferAddr
*
* @ Parameter
*		- i2c_no : 0, 1
*		- master_slave : I2C_MASTER, I2C_SLAVE
*		- *p_result: I2C_CHANNEL_SUPPORTED, I2C_CHANNEL_NOT_SUPPORTED
*
* @ Return
*		buffer address
*
*
************************************************************************************************************
*/
I2C_BUFFER* I2C_Get_BufferAddr (int i2c_no, int master_slave, int *p_result)
{

	I2C_BUFFER		*p_I2C_buffer;
	int				result = I2C_CHANNEL_SUPPORTED;


	switch (i2c_no)
	{
	case 0:
		if (master_slave == I2C_MASTER) p_I2C_buffer = &sI2C0_Master_Buffer;
		else if (master_slave == I2C_SLAVE) p_I2C_buffer = &sI2C0_Slave_Buffer;
		else result = I2C_CHANNEL_NOT_SUPPORTED;

		break;

	case 1:
		if (master_slave == I2C_MASTER) p_I2C_buffer = &sI2C1_Master_Buffer;
		else if (master_slave == I2C_SLAVE) p_I2C_buffer = &sI2C1_Slave_Buffer;
		else result = I2C_CHANNEL_NOT_SUPPORTED;

		break;


	default:
		result = I2C_CHANNEL_NOT_SUPPORTED;
		break;
	}

	*p_result = result;

	return (p_I2C_buffer);


}




/**
************************************************************************************************************
* @ Name: I2C_Write_Data
*
* @ Parameter
*		- i2c_no : 0, 1
*		- master_slave : I2C_MASTER, I2C_SLAVE
*		- p_write_buf : data buffer to transmit to the counterpart
*		- data_count
*		- restart : 0 (no restart), 1(restart)
*
* @ Return
*		I2C_TX_BUFFER_SUCCESS
*		I2C_TX_BUFFER_ERROR_WRONG_CHANNEL
*		I2C_TX_BUFFER_ERROR_WAIT_TIMEOUT
*
*
************************************************************************************************************
*/
int I2C_Write_Data (int i2c_no, int master_slave, uint8_t *p_write_buf, uint32_t data_count, uint16_t restart)
{

	I2C_Type			*i2c;
	I2C_BUFFER			*p_I2C_buffer;
	int 				i;
	//int 				request_count;
	//int 				read_count;
	int 				result;
	uint32_t			reg_val;



	//------------------------------------------------------------------------------------------
	// get I2C object
	//------------------------------------------------------------------------------------------
	i2c = I2C_Get_Object(i2c_no);

	if (i2c == (I2C_Type *) 0)
	{
		return (I2C_RX_BUFFER_ERROR_WRONG_CHANNEL);
	}



	//------------------------------------------------------------------------------------------
	// get base address
	//------------------------------------------------------------------------------------------
	p_I2C_buffer = I2C_Get_BufferAddr(i2c_no, master_slave, &result);

	if (result == I2C_CHANNEL_NOT_SUPPORTED)
	{
		return (I2C_RX_BUFFER_ERROR_WRONG_CHANNEL);
	}


	//------------------------------------------------------------------------------------------
	// examine if I2C is idle
	//------------------------------------------------------------------------------------------
//	for (i=0; i<10000; i++)
//	{
//		if (p_I2C_buffer->u8TxState == I2C_TX_STATE_IDLE)
//				break;
//	}
//
//	if (i == 10000)
//	{
//		return (I2C_TX_BUFFER_ERROR_WAIT_TIMEOUT);
//	}


	//------------------------------------------------------------------------------------------
	// copy data from write-buffer to SPI-buffer
	//------------------------------------------------------------------------------------------
	for (i=0; i<data_count; i++)
	{

		p_I2C_buffer->u8TxBuffer[i] = *(p_write_buf + i);
	}

	p_I2C_buffer->u16TxBuffer_HeadIndex = 0;
	p_I2C_buffer->u16TxBuffer_TailIndex = data_count;
	p_I2C_buffer->u16Restart = restart;


	//------------------------------------------------------------------------------------------
	// update state & init SCLK count
	//------------------------------------------------------------------------------------------
	if(master_slave == I2C_MASTER)
		p_I2C_buffer->u8TxState = I2C_TX_STATE_TRANSMIT;


	//------------------------------------------------------------------------------------------
	// start transmission
	//------------------------------------------------------------------------------------------
	if (master_slave == I2C_MASTER)
	{

		i2c->DR = I2C_ADDR_RW(g_I2C_TargetAddr, I2C_WR);

		reg_val = i2c->CR;
		reg_val |= (ICnCR_START | ICnCR_ACKEN);
		i2c->CR = reg_val;
	}



	return (I2C_TX_BUFFER_SUCCESS);

}


/**
************************************************************************************************************
* @ Name: I2C_Read_Data_After_Restart
*
* @ Parameter
*		- i2c_no : 0, 1
*		- master_slave : I2C_MASTER, I2C_SLAVE
*		- p_write_buf : data buffer to transmit to the counterpart
*		- data_count
*		- restart : 0 (no restart), 1(restart)
*
* @ Return
*		I2C_TX_BUFFER_SUCCESS
*		I2C_TX_BUFFER_ERROR_WRONG_CHANNEL
*		I2C_TX_BUFFER_ERROR_WAIT_TIMEOUT
*
*
************************************************************************************************************
*/
int I2C_Read_Data_After_Restart (int i2c_no, int master_slave, uint8_t *p_write_buf, uint32_t send_count, uint8_t *p_read_buf, uint32_t *p_rcv_count, uint16_t restart)
{

	I2C_Type			*i2c;
	I2C_BUFFER			*p_I2C_buffer;
	int 				i;
//	int 				request_count;
	int 				read_count;
	int 				result;

	uint32_t			reg_val;


	//------------------------------------------------------------------------------------------
	// get I2C object
	//------------------------------------------------------------------------------------------
	i2c = I2C_Get_Object(i2c_no);

	if (i2c == (I2C_Type *) 0)
	{
		return (I2C_RX_BUFFER_ERROR_WRONG_CHANNEL);
	}



	//------------------------------------------------------------------------------------------
	// get base address
	//------------------------------------------------------------------------------------------
	p_I2C_buffer = I2C_Get_BufferAddr(i2c_no, master_slave, &result);

	if (result == I2C_CHANNEL_NOT_SUPPORTED)
	{
		return (I2C_RX_BUFFER_ERROR_WRONG_CHANNEL);
	}


	//------------------------------------------------------------------------------------------
	// examine if I2C is idle
	//------------------------------------------------------------------------------------------
	for (i=0; i<10000; i++)
	{
		if (p_I2C_buffer->u8TxState == I2C_TX_STATE_IDLE) break;
	}

	if (i == 10000)
	{
		return (I2C_TX_BUFFER_ERROR_WAIT_TIMEOUT);
	}


	//------------------------------------------------------------------------------------------
	// copy data from write-buffer to SPI-buffer
	//------------------------------------------------------------------------------------------
	for (i=0; i<send_count; i++)
	{

		p_I2C_buffer->u8TxBuffer[i] = *(p_write_buf + i);
	}

	p_I2C_buffer->u16TxBuffer_HeadIndex = 0;
	p_I2C_buffer->u16TxBuffer_TailIndex = send_count;
	p_I2C_buffer->u16RxBuffer_HeadIndex = 0;
	p_I2C_buffer->u16RxBuffer_TailIndex = *p_rcv_count;
	p_I2C_buffer->u16Restart = restart;


	//------------------------------------------------------------------------------------------
	// update state & init SCLK count
	//------------------------------------------------------------------------------------------
	if(master_slave == I2C_MASTER)
	{
		p_I2C_buffer->u8TxState = I2C_TX_STATE_TRANSMIT;
		p_I2C_buffer->u8RxState = I2C_RX_STATE_RECEIVE;
	}


	//------------------------------------------------------------------------------------------
	// start transmission
	//------------------------------------------------------------------------------------------
	if (master_slave == I2C_MASTER)
	{

		i2c->DR = I2C_ADDR_RW(g_I2C_TargetAddr, I2C_WR);

		reg_val = i2c->CR;
		reg_val |= (ICnCR_START|ICnCR_ACKEN);
		i2c->CR = reg_val;
	}



	//################################################################
	//#
	//#		O P T I O N A L 		B E G I N
	//#
	//#		The following code may be spined off into another function.
	//#
	//################################################################

	while (p_I2C_buffer->u8RxState != I2C_RX_STATE_IDLE);

	//------------------------------------------------------------------------------------------
	// get received data
	//------------------------------------------------------------------------------------------
//	request_count = *p_rcv_count;
	read_count = p_I2C_buffer->u16RxBuffer_HeadIndex;


	for (i=0; i<p_I2C_buffer->u16RxBuffer_HeadIndex; i++)
	{
		*(p_read_buf + i) = p_I2C_buffer->u8RxBuffer[i];

	}

	for (; i<p_I2C_buffer->u16RxBuffer_TailIndex; i++)
	{
		*(p_read_buf + i) = 0;

	}



	*p_rcv_count = read_count;


	return (I2C_TX_BUFFER_SUCCESS);

}



/**
************************************************************************************************************
* @ Name: I2C_Read_Data
*
* @ Parameter
*		- i2c_no : 0, 1
*		- master_slave : I2C_MASTER, I2C_SLAVE
*		- p_read_buf : data buffer to transmit to the counterpart
*		- p_data_count
*
* @ Return
*		I2C_RX_BUFFER_SUCCESS
*		I2C_RX_BUFFER_ERROR_WRONG_CHANNEL
*		I2C_RX_BUFFER_ERROR_WAIT_TIMEOUT

*
*
************************************************************************************************************
*/
int I2C_Read_Data (int i2c_no, int master_slave, uint8_t * p_read_buf, uint32_t * p_data_count)
{

	I2C_Type			*i2c;
	I2C_BUFFER			*p_I2C_buffer;

	int					i;
	int					request_count;
	int					read_count;
	int					result;

	uint32_t				reg_val;


	//------------------------------------------------------------------------------------------
	// get I2C object
	//------------------------------------------------------------------------------------------
	i2c = I2C_Get_Object(i2c_no);

	if (i2c == (I2C_Type *) 0)
	{
		return (I2C_RX_BUFFER_ERROR_WRONG_CHANNEL);
	}

	//------------------------------------------------------------------------------------------
	// get base address
	//------------------------------------------------------------------------------------------
	p_I2C_buffer = I2C_Get_BufferAddr(i2c_no, master_slave, &result);

	if (result == I2C_CHANNEL_NOT_SUPPORTED)
	{
		return (I2C_RX_BUFFER_ERROR_WRONG_CHANNEL);
	}



	if (master_slave == I2C_MASTER)
	{

		//--------------------------------------------------------------------------------------
		// wait until MASTER READ is available
		//--------------------------------------------------------------------------------------
		for (i=0; i<0x10000; i++)
		{
			if (p_I2C_buffer->u8RxState == I2C_RX_STATE_IDLE) break;
		}


		if (i == 0x10000)
		{
			return (I2C_RX_BUFFER_ERROR_WAIT_TIMEOUT);
		}



		//-----------------------------------------------------------------------------------
		// init buffer
		//-----------------------------------------------------------------------------------
		p_I2C_buffer->u16RxBuffer_HeadIndex = 0;
		p_I2C_buffer->u16RxBuffer_TailIndex = *p_data_count;

		for (i=0; i<*p_data_count; i++)
		{
			p_I2C_buffer->u8RxBuffer[i] = 0;
		}




		//-----------------------------------------------------------------------------------
		// update state
		//-----------------------------------------------------------------------------------
		p_I2C_buffer->u8RxState = I2C_RX_STATE_RECEIVE;


		//----------------------------------------------------------------------------------
		// start transmission
		//----------------------------------------------------------------------------------
		i2c->DR = I2C_ADDR_RW(g_I2C_TargetAddr, I2C_RD);

		reg_val = i2c->CR;
		reg_val |= (ICnCR_START|ICnCR_ACKEN);
		i2c->CR = reg_val;


		//################################################################
		//#
		//#		O P T I O N A L 		B E G I N
		//#
		//#		The following code may be spined off into another function.
		//#
		//################################################################

		while (p_I2C_buffer->u8RxState != I2C_RX_STATE_IDLE);

		//------------------------------------------------------------------------------------------
		// get received data
		//------------------------------------------------------------------------------------------
		request_count = *p_data_count;
		read_count = 0;

		p_I2C_buffer->u16RxBuffer_HeadIndex = 0;

		for (i=0; i<request_count; i++)
		{
			if (p_I2C_buffer->u16RxBuffer_HeadIndex != p_I2C_buffer->u16RxBuffer_TailIndex)
			{

				*(p_read_buf + i) = p_I2C_buffer->u8RxBuffer[p_I2C_buffer->u16RxBuffer_HeadIndex++];

				if (p_I2C_buffer->u16RxBuffer_HeadIndex >= I2C_MAX_RX_BUFFER)
					p_I2C_buffer->u16RxBuffer_HeadIndex = 0;

				read_count++;
			}
			else
			{
				*(p_read_buf + i) = 0;
			}
		}

		*p_data_count = read_count;

	}
	else if (master_slave == I2C_SLAVE)
	{

		//------------------------------------------------------------------------------------------
		// get received data
		//------------------------------------------------------------------------------------------
		request_count = *p_data_count;
		read_count = 0;

		for (i=0; i<request_count; i++)
		{
			if (p_I2C_buffer->u16RxBuffer_HeadIndex != p_I2C_buffer->u16RxBuffer_TailIndex)
			{

				*(p_read_buf + i) = p_I2C_buffer->u8RxBuffer[p_I2C_buffer->u16RxBuffer_HeadIndex++];

				if (p_I2C_buffer->u16RxBuffer_HeadIndex >= I2C_MAX_RX_BUFFER)
					p_I2C_buffer->u16RxBuffer_HeadIndex = 0;

				read_count++;
			}
			else
			{
				*(p_read_buf + i) = 0;
			}

		}


		*p_data_count = read_count;

	}


	return (I2C_TX_BUFFER_SUCCESS);

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
************************************************************************************************************
* @ Name: I2C0_Master_Transmit_Receive_ISR
*
* @ Parameter
*		none
*
* @ Function
*		interrupt handler
*
************************************************************************************************************
*/
void I2C0_Master_Transmit_Receive_ISR(void)
{
	I2C_Type		* i2c = I2C0;
	I2C_BUFFER		* i2c_buffer = &sI2C0_Master_Buffer;

	uint32_t			status, status2;
	uint8_t			send_data, rcv_data;
	uint32_t			reg_val;
	volatile int		delay;


	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	status = i2c->SR;
	status2 = i2c->CR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C0_index < 40) g_I2C0_status[g_I2C0_index++] = (status2 | ((g_I2C0_count & 0xFF)<<24));




	//======================================================================
	//
	//	ADDR MODE DONE
	//
	//======================================================================
	if ((status & ICnSR_GCALL) == ICnSR_GCALL)
	{

		//-------------------------------------------------------------------------------------
		// ADDR SENT & NAK -> go to STOP
		//
		//		In ADDR mode, there is a possibility for the master to receive NAK, irrespective of WRITE or READ transaction.
		//-------------------------------------------------------------------------------------
		if ((status & ICnSR_RXACK)==0)
		{
			reg_val = i2c->CR;
			reg_val &= ~ICnCR_START;
			reg_val |= ICnCR_STOP;
			i2c->CR = reg_val;

		}
		//-------------------------------------------------------------------------------------
		// ADDR SENT & ACK in write action > load 1st DATA to transmit
		//-------------------------------------------------------------------------------------
		else if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
				i2c->DR = send_data;
			}
			else
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;
			}

		}
		//-------------------------------------------------------------------------------------
		// ADDR SENT & ACK in read action
		//-------------------------------------------------------------------------------------
		else
		{
			/* If only one data is needed in RCV transaction, the master will send NAK to the slave after receiving one data. */
			if ((i2c_buffer->u16RxBuffer_HeadIndex + 1) == i2c_buffer->u16RxBuffer_TailIndex)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_ACKEN;
				i2c->CR = reg_val;
			}
		}

	}
	//======================================================================
	//
	//	TEND MODE (Transmission can be Transfer or Receive)
	//
	//======================================================================
	else if ((status & ICnSR_TEND) == ICnSR_TEND)
	{

		//--------------------------------------------------------------------------------------
		// WRITE transaction
		//--------------------------------------------------------------------------------------
		 if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			//-------------------------------------------------------------------------------------
			// In WRITE transaction, there is a possibility for the master to receive NAK in DATA phase.
			//-------------------------------------------------------------------------------------
			if ((status & ICnSR_RXACK)==0)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;

			}
			else if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
				i2c->DR = send_data;
			}
			else if (i2c_buffer->u16TxBuffer_HeadIndex == i2c_buffer->u16TxBuffer_TailIndex)
			{
				//----------------------------------------------------------------------------------
				// 	Now, all data are transmitted.
				// 	What to do next?
				//
				//	Send STOP or RESTART
				//----------------------------------------------------------------------------------
				if (i2c_buffer->u16Restart == 0)
				{
					reg_val = i2c->CR;
					reg_val &= ~ICnCR_START;
					reg_val |= ICnCR_STOP;
					i2c->CR = reg_val;

				}
				else
				{
					for (delay=0; delay<10; delay++);

					i2c->DR = I2C_ADDR_RW(g_I2C_TargetAddr, I2C_RD);

					reg_val = i2c->CR;
					reg_val |= (ICnCR_START|ICnCR_ACKEN);
					i2c->CR = reg_val;


				}


			}

		}
		//--------------------------------------------------------------------------------------
		// READ transaction
		//--------------------------------------------------------------------------------------
		else
		{
			//----------------------------------------------------------------------------------
			// save the received data
			//----------------------------------------------------------------------------------
			rcv_data = (uint8_t) (i2c->DR) & 0xFF;


			if (i2c_buffer->u16RxBuffer_HeadIndex < i2c_buffer->u16RxBuffer_TailIndex)
			{
				i2c_buffer->u8RxBuffer[i2c_buffer->u16RxBuffer_HeadIndex++] = rcv_data;
			}


			//----------------------------------------------------------------------------------
			// next action ?  transmit ACK, NAK, or STOP
			//----------------------------------------------------------------------------------
			if ((i2c_buffer->u16RxBuffer_HeadIndex + 1) == i2c_buffer->u16RxBuffer_TailIndex)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_ACKEN;
				i2c->CR = reg_val;
			}
			else if (i2c_buffer->u16RxBuffer_HeadIndex == i2c_buffer->u16RxBuffer_TailIndex)
			{
				// send STOP signal at the next phase
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;
			}

		}


	}
	//======================================================================
	//
	//	STOP MODE
	//
	//======================================================================
	else if ((status & ICnSR_STOP) == ICnSR_STOP)
	{

		//i2c_buffer->u16TxBuffer_HeadIndex = 0;
		//i2c_buffer->u16TxBuffer_TailIndex = 0;
		i2c_buffer->u8TxState = I2C_TX_STATE_IDLE;

		//i2c_buffer->u16RxBuffer_HeadIndex = 0;
		//i2c_buffer->u16RxBuffer_TailIndex = 0;
		i2c_buffer->u8RxState = I2C_RX_STATE_IDLE;


	}



	//------------------------------------------------------------------------------------------
	// clear interrupt flags
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	//for (delay=0; delay<300; delay++);
	i2c->SR = 0xFF;


	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	status = i2c->SR;
	status2 = i2c->CR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C0_index < 40) g_I2C0_status[g_I2C0_index++] = (status2 | ((g_I2C0_count & 0xFF)<<24));


}





/**
************************************************************************************************************
* @ Name: I2C0_Master_Transmit_Receive_ISR2
*
* @ Parameter
*		This ISR supports RESTART transaction.
*
* @ Function
*		interrupt handler
*
************************************************************************************************************
*/
void I2C0_Master_Transmit_Receive_ISR2(void)
{
	I2C_Type		* i2c = I2C0;
	I2C_BUFFER		* i2c_buffer = &sI2C0_Master_Buffer;

	uint32_t			status;
	uint8_t			send_data, rcv_data;
	uint32_t			reg_val;
	volatile int		delay;


	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	status = i2c->SR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C0_index < 40) g_I2C0_status[g_I2C0_index++] = (status | ((g_I2C0_count & 0xFF)<<24));




	//======================================================================
	//
	//	ADDR MODE DONE
	//
	//======================================================================
	if ((status & ICnSR_GCALL) == ICnSR_GCALL)
	{

		//-------------------------------------------------------------------------------------
		// ADDR SENT & NAK -> go to STOP
		//
		//		In ADDR mode, there is a possibility for the master to receive NAK, irrespective of WRITE or READ transaction.
		//-------------------------------------------------------------------------------------
		if ((status & ICnSR_RXACK)==0)
		{
			reg_val = i2c->CR;
			reg_val &= ~ICnCR_START;
			reg_val |= ICnCR_STOP;
			i2c->CR = reg_val;

		}
		//-------------------------------------------------------------------------------------
		// ADDR SENT & ACK in write action > load 1st DATA to transmit
		//-------------------------------------------------------------------------------------
		else if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
				i2c->DR = send_data;
			}
			else
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;
			}

		}
		//-------------------------------------------------------------------------------------
		// ADDR SENT & ACK in read action
		//-------------------------------------------------------------------------------------
		else
		{
			/* If only one data is needed in RCV transaction, the master will send NAK to the slave after receiving one data. */
			if ((i2c_buffer->u16RxBuffer_HeadIndex + 1) == i2c_buffer->u16RxBuffer_TailIndex)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_ACKEN;
				i2c->CR = reg_val;
			}
		}


	}
	//======================================================================
	//
	//	TEND MODE (Transmission can be Transfer or Receive)
	//
	//======================================================================
	else if ((status & ICnSR_TEND) == ICnSR_TEND)
	{


		//--------------------------------------------------------------------------------------
		// WRITE transaction
		//--------------------------------------------------------------------------------------
		 if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			//-------------------------------------------------------------------------------------
			// In WRITE transaction, there is a possibility for the master to receive NAK in DATA phase.
			//-------------------------------------------------------------------------------------
			if ((status & ICnSR_RXACK)==0)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;

			}
			else if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
				i2c->DR = send_data;
			}
			else if (i2c_buffer->u16TxBuffer_HeadIndex == i2c_buffer->u16TxBuffer_TailIndex)
			{
				//----------------------------------------------------------------------------------
				// 	Now, all data are transmitted.
				// 	What to do next?
				//
				//	Send STOP or RESTART
				//----------------------------------------------------------------------------------
				if (i2c_buffer->u16Restart == 0)
				{
					reg_val = i2c->CR;
					reg_val &= ~ICnCR_START;
					reg_val |= ICnCR_STOP;
					i2c->CR = reg_val;

				}
				else
				{
					for (delay=0; delay<10; delay++);

					i2c->DR = I2C_ADDR_RW(g_I2C_TargetAddr, I2C_RD);

					reg_val = i2c->CR;
					reg_val |= (ICnCR_START|ICnCR_ACKEN);
					i2c->CR = reg_val;

				}


			}

		}
		//--------------------------------------------------------------------------------------
		// READ transaction
		//--------------------------------------------------------------------------------------
		else
		{
			//----------------------------------------------------------------------------------
			// save the received data
			//----------------------------------------------------------------------------------
			rcv_data = (uint8_t) (i2c->DR & 0xFF);

			if (i2c_buffer->u16RxBuffer_HeadIndex < i2c_buffer->u16RxBuffer_TailIndex)
			{
				i2c_buffer->u8RxBuffer[i2c_buffer->u16RxBuffer_HeadIndex++] = rcv_data;
			}


			//----------------------------------------------------------------------------------
			// next action ?  transmit ACK, NAK, or STOP
			//----------------------------------------------------------------------------------
			if ((i2c_buffer->u16RxBuffer_HeadIndex + 1) == i2c_buffer->u16RxBuffer_TailIndex)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_ACKEN;
				i2c->CR = reg_val;
			}
			else if (i2c_buffer->u16RxBuffer_HeadIndex == i2c_buffer->u16RxBuffer_TailIndex)
			{
				// send STOP signal at the next phase
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR= reg_val;
			}

		}

	}
	//======================================================================
	//
	//	STOP MODE
	//
	//======================================================================
	else if ((status & ICnSR_STOP) == ICnSR_STOP)
	{

		//i2c_buffer->u16TxBuffer_HeadIndex = 0;
		//i2c_buffer->u16TxBuffer_TailIndex = 0;
		i2c_buffer->u8TxState = I2C_TX_STATE_IDLE;

		//i2c_buffer->u16RxBuffer_HeadIndex = 0;
		//i2c_buffer->u16RxBuffer_TailIndex = 0;
		i2c_buffer->u8RxState = I2C_RX_STATE_IDLE;

	}




	//------------------------------------------------------------------------------------------
	// clear interrupt flags
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	i2c->SR = 0xFF;



	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	status = i2c->SR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C0_index < 40) g_I2C0_status[g_I2C0_index++] = (status | ((g_I2C0_count & 0xFF)<<24));


}



/**
************************************************************************************************************
* @ Name: I2C0_Slave_Transmit_Receive_ISR
*
* @ Parameter
*		none
*
* @ Function
*		interrupt handler
*
************************************************************************************************************
*/
void I2C0_Slave_Transmit_Receive_ISR(void)
{

	I2C_Type		* i2c = I2C0;
	I2C_BUFFER		* i2c_buffer = &sI2C0_Slave_Buffer;

	uint32_t			status;
	uint16_t			next_index;
	uint8_t			send_data, rcv_data;
	//uint32_t			reg_val;


	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC1SR = 0x4000_0A88
	//------------------------------------------------------------------------------------------
	status = i2c->SR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C0_index < 40) g_I2C0_status[g_I2C0_index++] = (status | ((g_I2C0_count & 0xFF)<<24));




	//======================================================================
	//
	//	ADDR MODE DONE
	//
	//======================================================================
	if ((status & ICnSR_SSEL) == ICnSR_SSEL)
	{

		if (status & ICnSR_TMOD)
		{
			i2c_buffer->u8RxState = I2C_RX_STATE_IDLE;
			i2c_buffer->u8TxState = I2C_TX_STATE_TRANSMIT;
		}
		else
		{
			i2c_buffer->u8TxState = I2C_TX_STATE_IDLE;
			i2c_buffer->u8RxState = I2C_RX_STATE_RECEIVE;
		}


		//--------------------------------------------------------------------------------------
		// WRITE transaction ("master-read" transaction)
		//--------------------------------------------------------------------------------------
		if (status & ICnSR_TMOD)
		{
			//-------------------------------------------------------------------------------
			// transmit mode
			//-------------------------------------------------------------------------------

			if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
			}
			else
			{
				send_data = 0;
				i2c_buffer->u16TxBuffer_HeadIndex = 0;
				i2c_buffer->u16TxBuffer_TailIndex = 0;
			}

			i2c->DR = send_data;
		}

	}
	//======================================================================
	//
	//	TEND MODE DONE
	//
	//======================================================================
	else if ((status & ICnSR_TEND) == ICnSR_TEND)
	{

		//--------------------------------------------------------------------------------------
		// WRITE transaction ("master-read" transaction)
		//--------------------------------------------------------------------------------------
		if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
			}
			else
			{
				send_data = 0;
				i2c_buffer->u16TxBuffer_HeadIndex = 0;
				i2c_buffer->u16TxBuffer_TailIndex = 0;
			}
			i2c->DR = send_data;


		}
		//--------------------------------------------------------------------------------------
		// READ transaction ("master-write" transaction)
		//--------------------------------------------------------------------------------------
		else
		{
			rcv_data = (uint8_t) (i2c->DR & 0xFF);

			next_index = i2c_buffer->u16RxBuffer_TailIndex + 1;
			if (next_index >= I2C_MAX_RX_BUFFER)
			{
				next_index = 0;
			}

			if (next_index != i2c_buffer->u16RxBuffer_HeadIndex)
			{
				i2c_buffer->u8RxBuffer[i2c_buffer->u16RxBuffer_TailIndex] = rcv_data;
				i2c_buffer->u16RxBuffer_TailIndex = next_index;

			}

		}




	}

	//======================================================================
	//
	//	STOP MODE
	//
	//======================================================================
	if (status & ICnSR_STOP)
	{
		i2c_buffer->u8RxState = I2C_RX_STATE_IDLE;
		i2c_buffer->u8TxState = I2C_TX_STATE_IDLE;
	}




	//------------------------------------------------------------------------------------------
	// clear interrupt flags
	//
	//				@ IC1SR = 0x4000_0A88
	//------------------------------------------------------------------------------------------
	i2c->SR = 0xFF;



	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC1SR = 0x4000_0A88
	//------------------------------------------------------------------------------------------
	status = i2c->SR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C0_index < 40) g_I2C0_status[g_I2C0_index++] = (status | ((g_I2C0_count & 0xFF)<<24));

}





/**
************************************************************************************************************
* @ Name: I2C1_Master_Transmit_Receive_ISR
*
* @ Parameter
*		none
*
* @ Function
*		interrupt handler
*
************************************************************************************************************
*/



void I2C1_Master_Transmit_Receive_ISR(void)
{
	I2C_Type		* i2c = I2C1;
	I2C_BUFFER		* i2c_buffer = &sI2C1_Master_Buffer;

	uint32_t			status;
	uint8_t			send_data, rcv_data;
	uint32_t			reg_val;
	volatile int		delay;


	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	status = i2c->SR;

	for (delay=0; delay<250; delay++);

	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C1_index < 40) g_I2C1_status[g_I2C1_index++] = (status | ((g_I2C1_count & 0xFF)<<24));




	//======================================================================
	//
	//	ADDR MODE DONE
	//
	//======================================================================
	if ((status & ICnSR_GCALL) == ICnSR_GCALL)
	{

		//-------------------------------------------------------------------------------------
		// ADDR SENT & NAK -> go to STOP
		//
		//		In ADDR mode, there is a possibility for the master to receive NAK, irrespective of WRITE or READ transaction.
		//-------------------------------------------------------------------------------------
		if ((status & ICnSR_RXACK)==0)
		{
			reg_val = i2c->CR;
			reg_val &= ~ICnCR_START;
			reg_val |= ICnCR_STOP;
			i2c->CR = reg_val;

		}
		//-------------------------------------------------------------------------------------
		// ADDR SENT & ACK in write action > load 1st DATA to transmit
		//-------------------------------------------------------------------------------------
		else if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
				i2c->DR = send_data;
			}
			else
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;
			}

		}
		//-------------------------------------------------------------------------------------
		// ADDR SENT & ACK in read action
		//-------------------------------------------------------------------------------------
		else
		{
			/* If only one data is needed in RCV transaction, the master will send NAK to the slave after receiving one data. */
			if ((i2c_buffer->u16RxBuffer_HeadIndex + 1) == i2c_buffer->u16RxBuffer_TailIndex)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_ACKEN;
				i2c->CR = reg_val;
			}
		}

	}
	//======================================================================
	//
	//	TEND MODE (Transmission can be Transfer or Receive)
	//
	//======================================================================
	else if ((status & ICnSR_TEND) == ICnSR_TEND)
	{

		//--------------------------------------------------------------------------------------
		// WRITE transaction
		//--------------------------------------------------------------------------------------
		 if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			//-------------------------------------------------------------------------------------
			// In WRITE transaction, there is a possibility for the master to receive NAK in DATA phase.
			//-------------------------------------------------------------------------------------
			if ((status & ICnSR_RXACK)==0)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;
			}
			else if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
				i2c->DR = send_data;
			}
			else if (i2c_buffer->u16TxBuffer_HeadIndex == i2c_buffer->u16TxBuffer_TailIndex)
			{
				//----------------------------------------------------------------------------------
				// 	Now, all data are transmitted.
				// 	What to do next?
				//
				//	Send STOP or RESTART
				//----------------------------------------------------------------------------------
				if (i2c_buffer->u16Restart == 0)
				{
					reg_val = i2c->CR;
					reg_val &= ~ICnCR_START;
					reg_val |= ICnCR_STOP;
					i2c->CR = reg_val;

				}
				else
				{
					for (delay=0; delay<10; delay++);

					i2c->DR = I2C_ADDR_RW(g_I2C_TargetAddr, I2C_RD);

					reg_val = i2c->CR;
					reg_val |= (ICnCR_START|ICnCR_ACKEN);
					i2c->CR = reg_val;

				}


			}

		}
		//--------------------------------------------------------------------------------------
		// READ transaction
		//--------------------------------------------------------------------------------------
		else
		{
			//----------------------------------------------------------------------------------
			// save the received data
			//----------------------------------------------------------------------------------
			rcv_data = (uint8_t) (i2c->DR & 0xFF);

			if (i2c_buffer->u16RxBuffer_HeadIndex < i2c_buffer->u16RxBuffer_TailIndex)
			{
				i2c_buffer->u8RxBuffer[i2c_buffer->u16RxBuffer_HeadIndex++] = rcv_data;
			}


			//----------------------------------------------------------------------------------
			// next action ?  transmit ACK, NAK, or STOP
			//----------------------------------------------------------------------------------
			if ((i2c_buffer->u16RxBuffer_HeadIndex + 1) == i2c_buffer->u16RxBuffer_TailIndex)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_ACKEN;
				i2c->CR = reg_val;
			}
			else if (i2c_buffer->u16RxBuffer_HeadIndex == i2c_buffer->u16RxBuffer_TailIndex)
			{
				// send STOP signal at the next phase
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;
			}

		}


	}
	//======================================================================
	//
	//	STOP MODE
	//
	//======================================================================
	else if ((status & ICnSR_STOP) == ICnSR_STOP)
	{

		//i2c_buffer->u16TxBuffer_HeadIndex = 0;
		//i2c_buffer->u16TxBuffer_TailIndex = 0;
		i2c_buffer->u8TxState = I2C_TX_STATE_IDLE;

		//i2c_buffer->u16RxBuffer_HeadIndex = 0;
		//i2c_buffer->u16RxBuffer_TailIndex = 0;
		i2c_buffer->u8RxState = I2C_RX_STATE_IDLE;


	}



	//------------------------------------------------------------------------------------------
	// clear interrupt flags
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	i2c->SR = 0xFF;



	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	status = i2c->SR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C1_index < 40) g_I2C1_status[g_I2C1_index++] = (status | ((g_I2C1_count & 0xFF)<<24));

	//for (delay=0; delay<250; delay++);


}








/**
************************************************************************************************************
* @ Name: I2C1_Master_Transmit_Receive_ISR2
*
* @ Parameter
*		This ISR supports RESTART transaction.
*
* @ Function
*		interrupt handler
*
************************************************************************************************************
*/
void I2C1_Master_Transmit_Receive_ISR2(void)
{
	I2C_Type		* i2c = I2C1;
	I2C_BUFFER		* i2c_buffer = &sI2C1_Master_Buffer;

	uint32_t			status;
	uint8_t			send_data, rcv_data;
	uint32_t			reg_val;
	volatile int		delay;


	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	status = i2c->SR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C1_index < 40) g_I2C1_status[g_I2C1_index++] = (status | ((g_I2C1_count & 0xFF)<<24));




	//======================================================================
	//
	//	ADDR MODE DONE
	//
	//======================================================================
	if ((status & ICnSR_GCALL) == ICnSR_GCALL)
	{

		//-------------------------------------------------------------------------------------
		// ADDR SENT & NAK -> go to STOP
		//
		//		In ADDR mode, there is a possibility for the master to receive NAK, irrespective of WRITE or READ transaction.
		//-------------------------------------------------------------------------------------
		if ((status & ICnSR_RXACK)==0)
		{
			reg_val = i2c->CR;
			reg_val &= ~ICnCR_START;
			reg_val |= ICnCR_STOP;
			i2c->CR = reg_val;
		}
		//-------------------------------------------------------------------------------------
		// ADDR SENT & ACK in write action > load 1st DATA to transmit
		//-------------------------------------------------------------------------------------
		else if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
				i2c->DR = send_data;
			}
			else
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;
			}

		}
		//-------------------------------------------------------------------------------------
		// ADDR SENT & ACK in read action
		//-------------------------------------------------------------------------------------
		else
		{
			/* If only one data is needed in RCV transaction, the master will send NAK to the slave after receiving one data. */
			if ((i2c_buffer->u16RxBuffer_HeadIndex + 1) == i2c_buffer->u16RxBuffer_TailIndex)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_ACKEN;
				i2c->CR = reg_val;
			}
		}


	}
	//======================================================================
	//
	//	TEND MODE (Transmission can be Transfer or Receive)
	//
	//======================================================================
	else if ((status & ICnSR_TEND) == ICnSR_TEND)
	{


		//--------------------------------------------------------------------------------------
		// WRITE transaction
		//--------------------------------------------------------------------------------------
		 if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			//-------------------------------------------------------------------------------------
			// In WRITE transaction, there is a possibility for the master to receive NAK in DATA phase.
			//-------------------------------------------------------------------------------------
			if ((status & ICnSR_RXACK)==0)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->CR = reg_val;

			}
			else if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
				i2c->DR = send_data;
			}
			else if (i2c_buffer->u16TxBuffer_HeadIndex == i2c_buffer->u16TxBuffer_TailIndex)
			{
				//----------------------------------------------------------------------------------
				// 	Now, all data are transmitted.
				// 	What to do next?
				//
				//	Send STOP or RESTART
				//----------------------------------------------------------------------------------
				if (i2c_buffer->u16Restart == 0)
				{
					reg_val = i2c->CR;
					reg_val &= ~ICnCR_START;
					reg_val |= ICnCR_STOP;
					i2c->CR = reg_val;
				}
				else
				{
					for (delay=0; delay<10; delay++);
					i2c->DR = I2C_ADDR_RW(g_I2C_TargetAddr, I2C_RD);

					reg_val = i2c->CR;
					reg_val |= (ICnCR_START|ICnCR_ACKEN);
					i2c->CR = reg_val;

				}


			}

		}
		//--------------------------------------------------------------------------------------
		// READ transaction
		//--------------------------------------------------------------------------------------
		else
		{
			//----------------------------------------------------------------------------------
			// save the received data
			//----------------------------------------------------------------------------------
			rcv_data = (uint8_t) (i2c->DR & 0xFF);

			if (i2c_buffer->u16RxBuffer_HeadIndex < i2c_buffer->u16RxBuffer_TailIndex)
			{
				i2c_buffer->u8RxBuffer[i2c_buffer->u16RxBuffer_HeadIndex++] = rcv_data;
			}


			//----------------------------------------------------------------------------------
			// next action ?  transmit ACK, NAK, or STOP
			//----------------------------------------------------------------------------------
			if ((i2c_buffer->u16RxBuffer_HeadIndex + 1) == i2c_buffer->u16RxBuffer_TailIndex)
			{
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_ACKEN;
				i2c->DR = reg_val;
			}
			else if (i2c_buffer->u16RxBuffer_HeadIndex == i2c_buffer->u16RxBuffer_TailIndex)
			{
				// send STOP signal at the next phase
				reg_val = i2c->CR;
				reg_val &= ~ICnCR_START;
				reg_val |= ICnCR_STOP;
				i2c->DR = reg_val;

			}

		}

	}
	//======================================================================
	//
	//	STOP MODE
	//
	//======================================================================
	else if ((status & ICnSR_STOP) == ICnSR_STOP)
	{

		//i2c_buffer->u16TxBuffer_HeadIndex = 0;
		//i2c_buffer->u16TxBuffer_TailIndex = 0;
		i2c_buffer->u8TxState = I2C_TX_STATE_IDLE;

		//i2c_buffer->u16RxBuffer_HeadIndex = 0;
		//i2c_buffer->u16RxBuffer_TailIndex = 0;
		i2c_buffer->u8RxState = I2C_RX_STATE_IDLE;

	}




	//------------------------------------------------------------------------------------------
	// clear interrupt flags
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	i2c->SR = 0xFF;



	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC0SR = 0x4000_0A08
	//------------------------------------------------------------------------------------------
	status =i2c->SR;


	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C1_index < 40) g_I2C1_status[g_I2C1_index++] = (status | ((g_I2C1_count & 0xFF)<<24));


}




/**
************************************************************************************************************
* @ Name:I2C1_Slave_Transmit_Receive_ISR
*
* @ Parameter
*		none
*
* @ Function
*		interrupt handler
*
************************************************************************************************************
*/
void I2C1_Slave_Transmit_Receive_ISR(void)
{

	I2C_Type		* i2c = I2C1;
	I2C_BUFFER		* i2c_buffer = &sI2C1_Slave_Buffer;

	uint32_t			status;
	uint16_t			next_index;
	uint8_t			send_data, rcv_data;
	//uint32_t			reg_val;


	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC1SR = 0x4000_0A88
	//------------------------------------------------------------------------------------------
	status = i2c->SR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C1_index < 40) g_I2C1_status[g_I2C1_index++] = (status | ((g_I2C1_count & 0xFF)<<24));




	//======================================================================
	//
	//	ADDR MODE DONE
	//
	//======================================================================
	if ((status & ICnSR_SSEL) == ICnSR_SSEL)
	{

		if (status & ICnSR_TMOD)
		{
			i2c_buffer->u8RxState = I2C_RX_STATE_IDLE;
			i2c_buffer->u8TxState = I2C_TX_STATE_TRANSMIT;
		}
		else
		{
			i2c_buffer->u8TxState = I2C_TX_STATE_IDLE;
			i2c_buffer->u8RxState = I2C_RX_STATE_RECEIVE;
		}


		//--------------------------------------------------------------------------------------
		// WRITE transaction ("master-read" transaction)
		//--------------------------------------------------------------------------------------
		if (status & ICnSR_TMOD)
		{
			//-------------------------------------------------------------------------------
			// transmit mode
			//-------------------------------------------------------------------------------

			if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
			}
			else
			{
				send_data = 0;
				i2c_buffer->u16TxBuffer_HeadIndex = 0;
				i2c_buffer->u16TxBuffer_TailIndex = 0;
			}

			i2c->DR =send_data;
		}

	}
	//======================================================================
	//
	//	TEND MODE DONE
	//
	//======================================================================
	else if ((status & ICnSR_TEND) == ICnSR_TEND)
	{

		//--------------------------------------------------------------------------------------
		// WRITE transaction ("master-read" transaction)
		//--------------------------------------------------------------------------------------
		if ((status & ICnSR_TMOD) == ICnSR_TMOD)
		{

			if (i2c_buffer->u16TxBuffer_HeadIndex < i2c_buffer->u16TxBuffer_TailIndex)
			{
				send_data = i2c_buffer->u8TxBuffer[i2c_buffer->u16TxBuffer_HeadIndex++];
			}
			else
			{
				send_data = 0;
				i2c_buffer->u16TxBuffer_HeadIndex = 0;
				i2c_buffer->u16TxBuffer_TailIndex = 0;
			}
			i2c->DR = send_data;


		}
		//--------------------------------------------------------------------------------------
		// READ transaction ("master-write" transaction)
		//--------------------------------------------------------------------------------------
		else
		{
			rcv_data = (uint8_t) (i2c->DR & 0xFF);

			next_index = i2c_buffer->u16RxBuffer_TailIndex + 1;
			if (next_index >= I2C_MAX_RX_BUFFER)
			{
				next_index = 0;
			}

			if (next_index != i2c_buffer->u16RxBuffer_HeadIndex)
			{
				i2c_buffer->u8RxBuffer[i2c_buffer->u16RxBuffer_TailIndex] = rcv_data;
				i2c_buffer->u16RxBuffer_TailIndex = next_index;

			}

		}




	}

	//======================================================================
	//
	//	STOP MODE
	//
	//======================================================================
	if (status & ICnSR_STOP)
	{
		i2c_buffer->u8RxState = I2C_RX_STATE_IDLE;
		i2c_buffer->u8TxState = I2C_TX_STATE_IDLE;
	}




	//------------------------------------------------------------------------------------------
	// clear interrupt flags
	//
	//				@ IC1SR = 0x4000_0A88
	//------------------------------------------------------------------------------------------
	i2c->SR = 0xFF;


	//------------------------------------------------------------------------------------------
	// get status
	//
	//				@ IC1SR = 0x4000_0A88
	//------------------------------------------------------------------------------------------
	status = i2c->SR;



	//------------------------------------------------------------------------------------------
	// monitor
	//------------------------------------------------------------------------------------------
	if (g_I2C1_index < 40) g_I2C1_status[g_I2C1_index++] = (status | ((g_I2C1_count & 0xFF)<<24));

}



