/***************************************************************************//**
* @file     debug_frmwrk.h
* @brief    Contains some utilities that used for debugging through UART
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

#ifndef _DEBUG_FRMWRK_H_
#define _DEBUG_FRMWRK_H_

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
#define _DBC(x)	 					DEBUG_PutChar(x)
#define _DBG(x)						DEBUG_PutString(x)
#define _DBG_(x)					DEBUG_PutStringCRLF(x)
#define _DBD8(x)					DEBUG_PutDec(x, DEBUG_DATA_CHAR)
#define _DBD16(x)					DEBUG_PutDec(x, DEBUG_DATA_SHORT)
#define _DBD32(x)					DEBUG_PutDec(x, DEBUG_DATA_INT)
#define _DBH8(x)					DEBUG_PutHex(x, DEBUG_DATA_CHAR)
#define _DBH16(x)					DEBUG_PutHex(x, DEBUG_DATA_SHORT)
#define _DBH32(x)					DEBUG_PutHex(x, DEBUG_DATA_INT)
#define _DG							DEBUG_GetChar()

//#define DEBUG_INTERFACE_PERI		(DEBUG_INTERFACE_PERI_USART10)
#define DEBUG_INTERFACE_PERI		(DEBUG_INTERFACE_PERI_UART1)

/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief Debug Data type
 */
typedef enum {
	DEBUG_DATA_CHAR, /*!< 8-bit */
	DEBUG_DATA_SHORT, /*!< 16-bit */
	DEBUG_DATA_INT, /*!< 32-bit */
	DEBUG_DATA_MAX
} DEBUG_DATA_Type;

/**
 * @brief  Debug Interface enumerated definition
 */
typedef enum {
	DEBUG_INTERFACE_PERI_UART0,
	DEBUG_INTERFACE_PERI_UART1,
	DEBUG_INTERFACE_PERI_USART10,
	DEBUG_INTERFACE_PERI_USART11,
	DEBUG_INTERFACE_PERI_USART12,
	DEBUG_INTERFACE_PERI_USART13,
	DEBUG_INTERFACE_PERI_MAX
} DEBUG_INTERFACE_PERI_Type;


/*******************************************************************************
* Exported Public Function
*******************************************************************************/
extern void cprintf(const    char *pFormat, ...);
extern void DEBUG_Init(DEBUG_INTERFACE_PERI_Type Peri);
extern void DEBUG_DeInit(void);
extern void DEBUG_PutChar(uint8_t Ch);
extern void DEBUG_PutString(const void *pString);
extern void DEBUG_PutStringCRLF(const void *pString);
extern void DEBUG_PutDec(uint32_t Number, DEBUG_DATA_Type DataType);
extern void DEBUG_PutHex(uint32_t Number, DEBUG_DATA_Type DataType);
extern uint8_t DEBUG_GetChar(void);


#ifdef __cplusplus
}
#endif

#endif /* _DEBUG_FRMWRK_H_ */
/* --------------------------------- End Of File ------------------------------ */
