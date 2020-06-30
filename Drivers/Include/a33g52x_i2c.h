/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_i2c.h
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

#include "A33G52x.h"



//==========================================================================
//
//	D E F I N A T I O N S
//
//==========================================================================
#define I2C_MASTER								(1)
#define I2C_SLAVE								   (0)



//==========================================================================
//
//	S T R U C T U R E S 
//
//==========================================================================

//==========================================================================
//

//
//==========================================================================
#define I2C_GENERAL_CALL_ENABLE				(1)
#define I2C_GENERAL_CALL_DISABLE				(0)


typedef struct {

	uint16_t				slave_addr; 
	uint16_t				general_call; 
	uint16_t				scl_low_duration; 
	uint16_t				scl_high_duration;
	uint16_t				sda_hold_duration; 
	uint16_t				interval; 
	
} I2C_CONFIG;





//==========================================================================
// 	ICnDR
//		
//				@ IC0DR = 0x4000_0A00
//				@ IC1DR = 0x4000_0A80
//
//==========================================================================






//==========================================================================
// 	ICnSR
//	
//				@ IC0SR = 0x4000_0A08
//				@ IC1SR = 0x4000_0A88
//
//==========================================================================
//
//				GCALL			GCALL flag is also used to indicate the completion of ADDR mode.
//
//==========================================================================
#define ICnSR_GCALL								(0x0001UL<<7)
#define ICnSR_TEND								(0x0001UL<<6)
#define ICnSR_STOP								(0x0001UL<<5)
#define ICnSR_SSEL								(0x0001UL<<4)
#define ICnSR_MLOST								(0x0001UL<<3)
#define ICnSR_BUSY								(0x0001UL<<2)
#define ICnSR_TMOD								(0x0001UL<<1)
#define ICnSR_RXACK								(0x0001UL<<0)



//==========================================================================
// 	ICnSAR
//		
//				@ IC0SAR = 0x4000_0A0C
//				@ IC1SAR = 0x4000_0A8C
//
//==========================================================================
#define ICnSAR_SVAD_VAL(n)						(((n)&0x7FUL)<<1)
#define ICnSAR_GCEN								(0x0001UL<<0)




//==========================================================================
// ICnCR
//
//				@ IC0CR = 0x4000_0A14
//				@ IC1CR = 0x4000_0A94
//
//==========================================================================
#define ICnCR_INTERVAL_VAL(n)					(((n)&0x03UL)<<8)
#define ICnCR_INTERVAL_MASK 					(0x0003UL<<8)

#define ICnCR_IIF								(0x0001UL<<7)
#define ICnCR_SOFTRST							(0x0001UL<<5)
#define ICnCR_INTEN								(0x0001UL<<4)
#define ICnCR_ACKEN								(0x0001UL<<3)
#define ICnCR_STOP								(0x0001UL<<1)
#define ICnCR_START								(0x0001UL<<0)




//==========================================================================
// ICnSCLL
//
//				@ IC0SCLL = 0x4000_A018
//
//==========================================================================
#define ICnSCLL_SCLL_VAL(n)						(((n)&0xFFFFUL)<<0)
#define ICnSCLL_SCLL_MASK						(0xFFFFUL<<0)



//==========================================================================
// ICnSCLH
//
//				@ IC0SCLH = 0x4000_A01C
//
//==========================================================================
#define ICnSCLH_SCLH_VAL(n)						(((n)&0xFFFFUL)<<0)
#define ICnSCLH_SCLH_MASK						(0xFFFFUL<<0)




//==========================================================================
// ICnSCDH
//
//				@ IC0SDH = 0x4000_A020
//
//==========================================================================
#define ICnSDH_SDH_VAL(n)						(((n)&0x7FFFUL)<<0)
#define ICnSDH_SDH_MASK						(0x7FFFUL<<0)



//============================================================================
// 	
//	D E F I N A T I O N S 
//
//============================================================================
#define I2C_RD									(0x01)
#define I2C_WR 									(0x00)
#define I2C_ADDR_RW(addr, rw)					(((addr)<<1)|((rw)&0x01))




//============================================================================
// 	
//	D E F I N A T I O N S 
//
//============================================================================
#define I2C_MAX_RX_BUFFER					(20)
#define I2C_MAX_TX_BUFFER					(20)

#define I2C_CHANNEL_SUPPORTED				(1)
#define I2C_CHANNEL_NOT_SUPPORTED			(0)



//------------------------------------------------------------------------------------
// definitions for TX/RX state  
//------------------------------------------------------------------------------------
#define I2C_RX_STATE_IDLE					(0)
#define I2C_RX_STATE_RECEIVE				(1)


#define I2C_TX_STATE_IDLE					(0)
#define I2C_TX_STATE_TRANSMIT				(1)




//------------------------------------------------------------------------------------
// definitions for read/write function 
//------------------------------------------------------------------------------------
#define I2C_TX_BUFFER_SUCCESS				(0)
#define I2C_TX_BUFFER_ERROR_WRONG_CHANNEL	(1)
#define I2C_TX_BUFFER_ERROR_WAIT_TIMEOUT	(2)

#define I2C_RX_BUFFER_SUCCESS				(0)
#define I2C_RX_BUFFER_ERROR_WRONG_CHANNEL	(1)
#define I2C_RX_BUFFER_ERROR_WAIT_TIMEOUT	(2)





#define I2C_TXDATA_END_MARK					(0xFFFFFFFF)


//============================================================================
// 	
//	S T R U C T U R E S
//
//============================================================================
typedef struct {
	uint16_t		u8RxState;
	uint16_t		u8TxState;
	uint16_t		u16Restart;
	
	uint16_t		u16RxBuffer_HeadIndex;
	uint16_t 		u16RxBuffer_TailIndex;
	
	uint16_t		u16TxBuffer_HeadIndex;
	uint16_t 		u16TxBuffer_TailIndex;
	
	uint8_t		u8RxBuffer[I2C_MAX_RX_BUFFER];
	uint8_t		u8TxBuffer[I2C_MAX_TX_BUFFER];


} I2C_BUFFER;




//============================================================================
// 	
//	E X T E R N    V A R I A B L E S 
//
//============================================================================
extern uint8_t				g_I2C_TargetAddr;

extern int 					g_I2C0_index;
extern uint32_t			g_I2C0_status[40];

extern int 					g_I2C1_index;
extern uint32_t			g_I2C1_status[40];


extern I2C_BUFFER		sI2C0_Master_Buffer;
extern I2C_BUFFER		sI2C0_Slave_Buffer;

extern I2C_BUFFER		sI2C1_Master_Buffer;
extern I2C_BUFFER		sI2C1_Slave_Buffer;






//==========================================================================
// 
//		F U N C T I O N    D E C L A R A T I O N S 
//
//==========================================================================
void I2C_ConfigureGPIO (I2C_Type * const i2c);
void I2C_Init (I2C_Type * const i2c, int master_slave, I2C_CONFIG * p_config);
void I2C_ConfigureInterrupt (I2C_Type * const i2c, uint32_t intr_mask, uint32_t enable);




////////////////////////////////////////////////////////////////////////////////////////////////////////////
int _I2C_Init (int i2c_no, int master_slave, I2C_CONFIG * p_config);
I2C_Type* I2C_Get_Object (int i2c_no);
int I2C_InitBuffer (int i2c_no, int master_slave);
I2C_BUFFER* I2C_Get_BufferAddr (int i2c_no, int master_slave, int *p_result);


int I2C_Write_Data (int i2c_no, int master_slave, uint8_t *p_write_buf, uint32_t data_count, uint16_t restart);
int I2C_Read_Data_After_Restart (int i2c_no, int master_slave, uint8_t *p_write_buf, uint32_t send_count, uint8_t *p_read_buf, uint32_t *p_rcv_count, uint16_t restart);
int I2C_Read_Data (int i2c_no, int master_slave, uint8_t * p_read_buf, uint32_t * p_data_count);


void I2C0_Master_Transmit_Receive_ISR(void);
void I2C0_Master_Transmit_Receive_ISR2(void);
void I2C0_Slave_Transmit_Receive_ISR(void);
void I2C1_Master_Transmit_Receive_ISR(void);
void I2C1_Slave_Transmit_Receive_ISR(void);

