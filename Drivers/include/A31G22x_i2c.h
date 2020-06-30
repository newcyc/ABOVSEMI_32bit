/***************************************************************************//**
* @file     A31G22x_i2c.h
* @brief    Contains all macro definitions and function prototypes support
*           for I2C driver on A31G22x
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

#ifndef _A31G22x_I2C_H_
#define _A31G22x_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------- */
#include "A31G22x.h"
#include "aa_types.h"

/** I2C Slave Address registers bit mask */
#define I2C_I2ADR_BITMASK			(0xFF)

/* I2C state handle return values */
#define RECEIVE_MODE				(1)
#define TRANS_MODE					(2)
#define RECEIVE_DATA				(3)
#define TRANS_DATA					(4)
#define LOST_BUS					(5)
#define STOP_DECT					(6)

#define I2C_ST_SLT_FLAG				(0x01UL << I2C_ST_SLT_Pos)
#define I2C_SLTCR_SLTEN				(0x01UL << I2C_SLTCR_SLTEN_Pos)
#define I2C_SLTCR_SLTINT			(0x01UL << I2C_SLTCR_SLTINT_Pos)

#define I2C_MBCR_SCL_OUTPUT			(0x01UL << I2C_MBCR_SCLO_Pos)
#define I2C_MBCR_SDA_OUTPUT			(0x01UL << I2C_MBCR_SDAO_Pos)
#define I2C_MBCR_SCL_MANUAL			(0x01UL << I2C_MBCR_SCLMCE_Pos)
#define I2C_MBCR_SDA_MANUAL			(0x01UL << I2C_MBCR_SDAMCE_Pos)
#define I2C_MBCR_CONTROL_MASK		(0x0FUL)

#define I2C_MBCR_STATUS_MASK		(0x300UL)
#define I2C_MBCR_STATUS_SCL			(0x01UL << I2C_MBCR_SCLS_Pos)
#define I2C_MBCR_STATUS_SDA			(0x01UL << I2C_MBCR_SDAS_Pos)

/* Public Types --------------------------------------------------------------- */
/**
 * @brief Master transfer setup data structure definitions
 */
typedef struct
{
	uint32_t sl_addr7bit;			/**< Slave address in 7bit mode */
	uint8_t *tx_data;				/**< Pointer to Transmit data - NULL if data transmit is not used */
	uint32_t tx_length;				/**< Transmit data length - 0 if data transmit is not used*/
	uint32_t tx_count;				/**< Current Transmit data counter */
	uint8_t *rx_data;				/**< Pointer to Receive data - NULL if data receive is not used */
	uint32_t rx_length;				/**< Receive data length - 0 if data receive is not used */
	uint32_t rx_count;				/**< Current Receive data counter */
} I2C_M_SETUP_Type;

/**
 * @brief Slave transfer setup data structure definitions
 */
typedef struct
{
	uint8_t *tx_data;				/**< Pointer to transmit data - NULL if data transmit is not used */
	uint32_t tx_length;				/**< Transmit data length - 0 if data transmit is not used */
	uint32_t tx_count;				/**< Current transmit data counter	*/
	uint8_t *rx_data;				/**< Pointer to receive data - NULL if data received is not used */
	uint32_t rx_length;				/**< Receive data length - 0 if data receive is not used */
	uint32_t rx_count;				/**< Current receive data counter */
} I2C_S_SETUP_Type;

/**
 * @brief Transfer option type definitions
 */
typedef enum {
	I2C_TRANSFER_POLLING = 0,		/**< Transfer in polling mode */
	I2C_TRANSFER_INTERRUPT			/**< Transfer in interrupt mode */
} I2C_TRANSFER_OPT_Type;

typedef enum {
	I2C_SCL,
	I2C_SDA
} I2C_BUS_Type;

typedef enum {
	I2C_OUTPUT_LOW,
	I2C_OUTPUT_HIGH_OPENDRAIN
} I2C_BUS_OUTPUT_Type;

typedef enum {
	I2C_STATUS_LOW,
	I2C_STATUS_OPENDRAIN
} I2C_BUS_STATUS_Type;

/* Public Functions ----------------------------------------------------------- */
/* I2C Init/DeInit functions ---------- */
void I2C_Init(I2C_Type *I2Cx, uint32_t clockrate);
void I2C_DeInit(I2C_Type* I2Cx);

/* I2C transfer data functions -------- */
Status I2C_MasterTransferData(I2C_Type *I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
Status I2C_SlaveTransferData(I2C_Type *I2Cx, I2C_S_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
uint32_t I2C_MasterTransferComplete(I2C_Type *I2Cx);
uint32_t I2C_SlaveTransferComplete(I2C_Type *I2Cx);

void I2C_Interrupt_MasterHandler(I2C_Type  *I2Cx);
void I2C_Interrupt_SlaveHandler (I2C_Type  *I2Cx);

Status	I2C_MasterTransmitData(I2C_Type* I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
Status	I2C_MasterReceiveData(I2C_Type* I2Cx, I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
Status	I2C_SlaveReceiveData(I2C_Type* I2Cx, I2C_S_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);

void I2C_SetOwnSlaveAddr0(I2C_Type *I2Cx, uint8_t SlaveAddr_7bit, uint8_t GeneralCallState);
void I2C_SetOwnSlaveAddr1(I2C_Type *I2Cx, uint8_t SlaveAddr_7bit, uint8_t GeneralCallState);

/* I2C Interrupt handler functions ------*/
void I2C_IntCmd(I2C_Type *I2Cx, Bool NewState);

/* I2C SCL Low Timeout functions ------*/
void I2C_SetSclLowTimeout(I2C_Type *I2Cx, FunctionalState Enable, FunctionalState Interrupt, uint32_t SclPeriod);
void I2C_ClearSclLowTimeoutFlag(I2C_Type *I2Cx);

/* I2C SCL, SDA Manual Bus Control Functions ------*/
void I2C_SetManualBus(I2C_Type *I2Cx, I2C_BUS_Type I2CBus, I2C_BUS_OUTPUT_Type output, FunctionalState ManualControl);
I2C_BUS_STATUS_Type I2C_GetBusStatus(I2C_Type *I2Cx, I2C_BUS_Type I2CBus);

#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_I2C_H_ */
/* --------------------------------- End Of File ------------------------------ */
