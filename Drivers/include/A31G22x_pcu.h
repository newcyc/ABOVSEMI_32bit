/***************************************************************************//**
* @file     A31G22x_pcu.h
* @brief    Contains all macro definitions and function prototypes support
*           for PCU(Port Control Unit) driver on A31G22x
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

#ifndef _A31G22x_PCU_H_
#define _A31G22x_PCU_H_

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
* Included File
*******************************************************************************/
#include "A31G22x.h"
#include "aa_types.h"


/*******************************************************************************
* Public Macro
*******************************************************************************/
#define PORT_ACCESS_EN()			do { PCU2->PORTEN = 0x15; PCU2->PORTEN = 0x51; } while(0)
#define PORT_ACCESS_DIS()			do { PCU2->PORTEN = 0x00; } while(0)

#define PCU_ISR_BIT_WIDTH				(0x02UL)
#define PCU_ISR_BIT_MASK				(0x03UL)


/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief  PCU alternative function selection enumerated definition
 */
typedef enum {
	PCU_ALT_FUNCTION_0 = 0x00UL, /*!< Alternative function 0 */
	PCU_ALT_FUNCTION_1 = 0x01UL, /*!< Alternative function 1 */
	PCU_ALT_FUNCTION_2 = 0x02UL, /*!< Alternative function 2 */
	PCU_ALT_FUNCTION_3 = 0x03UL, /*!< Alternative function 3 */
	PCU_ALT_FUNCTION_4 = 0x04UL  /*!< Alternative function 4 */
} PCU_ALT_FUNCTION_Type;

/*
 * @brief  PCU mode selection enumerated definition
 */
typedef enum {
	PCU_MODE_INPUT = 0x00UL, /*!< Input mode */
	PCU_MODE_PUSH_PULL = 0x01UL, /*!< Push-pull output mode */
	PCU_MODE_ALT_FUNC = 0x02UL, /*!< Alternative function mode */
	PCU_MODE_OPEN_DRAIN = 0x03UL /*!< Open-drain output mode */
} PCU_MODE_Type;

/*
 * @brief  PCU pull-up/down selection enumerated definition
 */
typedef enum {
	PCU_PUPD_DISABLE = 0x00UL, /*!< Disable pull-up/down */
	PCU_PUPD_PULL_UP = 0x01UL, /*!< Enable pull-up */
	PCU_PUPD_PULL_DOWN = 0x02UL, /*!< Enable pull-down */
} PCU_PUPD_Type;

/*
 * @brief  PCU interrupt mode selection enumerated definition
 */
typedef enum {
	PCU_INTERRUPT_MODE_DISABLE = 0x00UL, /*!< Disable interrupt */
	PCU_INTERRUPT_MODE_LEVEL = 0x01UL, /*!< Level interrupt */
	PCU_INTERRUPT_MODE_EDGE = 0x03UL /*!< Edge interrupt */
} PCU_INTERRUPT_MODE_Type;

/*
 * @brief  PCU interrupt control selection enumerated definition
 */
typedef enum {
	PCU_INTERRUPT_CTRL_PROHIBIT = 0x00UL, /*!< Prohibit external interrupt */
	PCU_INTERRUPT_CTRL_EDGE_FALLING = 0x01UL, /*!< Falling edge interrupt */
	PCU_INTERRUPT_CTRL_EDGE_RISING = 0x02UL, /*!< Rising edge interrupt */
	PCU_INTERRUPT_CTRL_EDGE_BOTH = 0x03UL, /*!< Both edge interrupt */
	PCU_INTERRUPT_CTRL_LEVEL_LOW = PCU_INTERRUPT_CTRL_EDGE_FALLING, /*!< Low level interrupt */
	PCU_INTERRUPT_CTRL_LEVEL_HIGH = PCU_INTERRUPT_CTRL_EDGE_RISING /*!< High level interrupt */
} PCU_INTERRUPT_CTRL_Type;

/*
 * @brief  PCU ICOM status selection enumerated definition
 */
typedef enum {
	PCU_ICOM_STATUS_FLOATING = 0x00UL, /*!< Output low */
	PCU_ICOM_STATUS_HIGH_CURRENT_LOW = 0x01UL, /*!< Output high (open-drain) */
} PCU_ICOM_STATUS_Type;


/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void PCU_ConfigureDirection(PORT_Type *pPCUx, uint32_t PortNumber, PCU_MODE_Type Mode);
void PCU_ConfigureFunction(PORT_Type *pPCUx, uint32_t PortNumber, PCU_ALT_FUNCTION_Type Function);
void PCU_ConfigurePullupdown(PORT_Type *pPCUx, uint32_t PortNumber, PCU_PUPD_Type PUPD);
void PCU_ConfigureOutDataMask(PORT_Type *pPCUx, uint32_t PortNumber, FunctionalState Mask);
void PCU_ConfigureDebounce(PORT_Type *pPCUx, uint32_t PortNumber, FunctionalState Debounce);
void PCU_ConfigureICOM(uint32_t ICOM_Number, PCU_ICOM_STATUS_Type Status);
void PCU_EnableICOM(FunctionalState Output);
void GPIO_ConfigureInterrupt(PORT_Type *pPCUx, uint32_t PortNumber, PCU_INTERRUPT_MODE_Type Mode, PCU_INTERRUPT_CTRL_Type Control);
uint32_t GPIO_GetStatus(PORT_Type *pPCUx);
void GPIO_ClearStatus(PORT_Type *pPCUx, uint32_t Status);
void GPIO_SetValue(PORT_Type *pPCUx, uint16_t BitValue);
void GPIO_ClearValue(PORT_Type *pPCUx, uint32_t BitValue);
uint32_t GPIO_ReadValue(PORT_Type *pPCUx);
void GPIO_WriteValue(PORT_Type *pPCUx, uint32_t Value);


#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_PCU_H_ */
/* --------------------------------- End Of File ------------------------------ */
