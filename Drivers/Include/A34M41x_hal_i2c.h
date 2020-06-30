/**********************************************************************
* @file		A34M41x_i2c.h
* @brief	Contains all macro definitions and function prototypes
* 			support for I2C firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_I2C_H_
#define _A34M41x_I2C_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

#if defined(__ARMCC_VERSION)
  #pragma anon_unions
#endif

/** I2C Slave Address registers bit mask */
#define I2C_I2ADR_BITMASK			((0xFF))

/* I2C state handle return values */
#define RECEIVE_MODE		1
#define TRANS_MODE			2
#define RECEIVE_DATA		3
#define TRANS_DATA			4
#define LOST_BUS				5
#define STOP_DECT			6


/* Public Types --------------------------------------------------------------- */
/**
 * @brief Master transfer setup data structure definitions
 */
typedef struct
{
  uint32_t          sl_addr7bit;				/**< Slave address in 7bit mode */
  uint8_t*          tx_data;					/**< Pointer to Transmit data - NULL if data transmit is not used */
  uint32_t          tx_length;					/**< Transmit data length - 0 if data transmit is not used*/
  uint32_t          tx_count;					/**< Current Transmit data counter */
  uint8_t*          rx_data;					/**< Pointer to Receive data - NULL if data receive is not used */
  uint32_t          rx_length;					/**< Receive data length - 0 if data receive is not used */
  uint32_t          rx_count;					/**< Current Receive data counter */
} I2C_M_SETUP_Type;

/**
 * @brief Slave transfer setup data structure definitions
 */
typedef struct
{
  uint8_t*          tx_data;					/**< Pointer to transmit data - NULL if data transmit is not used */
  uint32_t          tx_length;					/**< Transmit data length - 0 if data transmit is not used */
  uint32_t          tx_count;					/**< Current transmit data counter	*/
  uint8_t*          rx_data;					/**< Pointer to receive data - NULL if data received is not used */
  uint32_t          rx_length;					/**< Receive data length - 0 if data receive is not used */
  uint32_t          rx_count;					/**< Current receive data counter */
} I2C_S_SETUP_Type;

/**
 * @brief Transfer option type definitions
 */
typedef enum {
	I2C_TRANSFER_POLLING = 0,		/**< Transfer in polling mode */
	I2C_TRANSFER_INTERRUPT			/**< Transfer in interrupt mode */
} I2C_TRANSFER_OPT_Type;


/* Public Functions ----------------------------------------------------------- */

/* I2C Init/DeInit functions ---------- */
HAL_Status_Type HAL_I2C_Init(I2C_Type *I2Cx, uint32_t clockrate);
HAL_Status_Type HAL_I2C_DeInit(I2C_Type* I2Cx);

/* I2C transfer data functions -------- */
Status HAL_I2C_MasterTransferData(I2C_Type* I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
Status HAL_I2C_SlaveTransferData(I2C_Type *I2Cx, I2C_S_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
uint32_t HAL_I2C_Master_GetState(I2C_Type *I2Cx);
uint32_t HAL_I2C_Slave_GetState(I2C_Type *I2Cx);

HAL_Status_Type HAL_I2C_Master_IRQHandler_IT(I2C_Type  *I2Cx);
HAL_Status_Type HAL_I2C_Slave_IRQHandler_IT (I2C_Type  *I2Cx);

Status HAL_I2C_Master_Transmit(I2C_Type* I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
Status HAL_I2C_Master_Receive(I2C_Type* I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
Status HAL_I2C_Slave_Receive(I2C_Type* I2Cx, I2C_S_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);

HAL_Status_Type HAL_I2C_Slave_SetAddress(I2C_Type *I2Cx, uint8_t SlaveAddr_7bit, uint8_t GeneralCallState);

/* I2C Interrupt handler functions ------*/
void  HAL_I2C_ConfigInterrupt(I2C_Type *I2Cx, Bool NewState);

#ifdef __cplusplus
}
#endif

#endif /* _A34M41x_I2C_H_ */

/* --------------------------------- End Of File ------------------------------ */
