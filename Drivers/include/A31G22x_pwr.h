/***************************************************************************//**
* @file     A31G22x_pwr.h
* @brief    Contains all macro definitions and function prototypes support
*           for power control driver on A31G22x
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

#ifndef _A31G22x_PWR_H_
#define _A31G22x_PWR_H_

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
#define SCULV_LVICR_LVIEN_DISABLE					(0x00UL << 7)
#define SCULV_LVICR_LVIEN_ENABLE					(0x01UL << 7)
	
#define SCULV_LVICR_LVINTEN_DISABLE					(0x00UL << 5)
#define SCULV_LVICR_LVINTEN_ENABLE					(0x01UL << 5)	

#define SCULV_LVICR_LVIFLAG							(0x01UL << SCULV_LVICR_LVIFLAG_Pos)

#define SCULV_LVRCR_LVREN_ENABLE					(0x00UL << SCULV_LVRCR_LVREN_Pos)
#define SCULV_LVRCR_LVREN_DISABLE				(0x55UL << SCULV_LVRCR_LVREN_Pos)

#define SCULV_LVRCNFIG_WTIDKY						(0x72A5UL << SCULV_LVRCNFIG_WTIDKY_Pos)
#define SCULV_LVRCNFIG_LVRENM_ENABLE			(0x00UL << SCULV_LVRCNFIG_LVRENM_Pos)
#define SCULV_LVRCNFIG_LVRENM_DISABLE			(0xAAUL << SCULV_LVRCNFIG_LVRENM_Pos)

/*******************************************************************************
* Public Typedef
*******************************************************************************/
/**
 * @brief  LVI voltage threshold enumerated definition
 */
typedef enum {
	LVI_VOLTAGE_160 = 0x00UL, /*!< 1.60V */
	LVI_VOLTAGE_169 = 0x01UL, /*!< 1.69V */
	LVI_VOLTAGE_178 = 0x02UL, /*!< 1.78V */
	LVI_VOLTAGE_190 = 0x03UL, /*!< 1.90V */
	LVI_VOLTAGE_199 = 0x04UL, /*!< 1.99V */
	LVI_VOLTAGE_212 = 0x05UL, /*!< 2.12V */
	LVI_VOLTAGE_230 = 0x06UL, /*!< 2.30V */
	LVI_VOLTAGE_247 = 0x07UL, /*!< 2.47V */
	LVI_VOLTAGE_267 = 0x08UL, /*!< 2.67V */
	LVI_VOLTAGE_304 = 0x09UL, /*!< 3.04V */
	LVI_VOLTAGE_318 = 0x0AUL, /*!< 3.18V */
	LVI_VOLTAGE_359 = 0x0BUL, /*!< 3.59V */
	LVI_VOLTAGE_372 = 0x0CUL, /*!< 3.72V */
	LVI_VOLTAGE_403 = 0x0DUL, /*!< 4.03V */
	LVI_VOLTAGE_420 = 0x0EUL, /*!< 4.20V */
	LVI_VOLTAGE_448 = 0x0FUL  /*!< 4.48V */
} LVI_VOLTAGE_Type;

/**
 * @brief  LVR voltage threshold enumerated definition
 */
typedef enum {
	LVR_VOLTAGE_448 = 0x00UL, /*!< 4.48V */
	LVR_VOLTAGE_420 = 0x01UL, /*!< 4.20V */
	LVR_VOLTAGE_403 = 0x02UL, /*!< 4.03V */
	LVR_VOLTAGE_372 = 0x03UL, /*!< 3.72V */
	LVR_VOLTAGE_359 = 0x04UL, /*!< 3.59V */
	LVR_VOLTAGE_318 = 0x05UL, /*!< 3.18V */
	LVR_VOLTAGE_304 = 0x06UL, /*!< 3.04V */
	LVR_VOLTAGE_267 = 0x07UL, /*!< 2.67V */
	LVR_VOLTAGE_247 = 0x08UL, /*!< 2.47V */
	LVR_VOLTAGE_230 = 0x09UL, /*!< 2.30V */
	LVR_VOLTAGE_212 = 0x0AUL, /*!< 2.12V */
	LVR_VOLTAGE_199 = 0x0BUL, /*!< 1.99V */
	LVR_VOLTAGE_190 = 0x0CUL, /*!< 1.90V */
	LVR_VOLTAGE_178 = 0x0DUL, /*!< 1.78V */
	LVR_VOLTAGE_169 = 0x0EUL, /*!< 1.69V */
	LVR_VOLTAGE_160 = 0x0FUL  /*!< 1.60V */
} LVR_VOLTAGE_Type;


typedef enum {
	LVI_DISABLE,	
	LVI_ENABLE
} LVI_EN_Type;

typedef enum {
	LVI_INTR_DISABLE,	
	LVI_INTR_ENABLE	
} LVI_INTR_Type;

typedef enum {
	LVR_ENABLE = 0x00,	
	LVR_DISABLE = 0x55,	
} LVR_EN_Type;

typedef enum {
	LVR_CONFIG_MASTER = 0x00,	
	LVR_CONFIG_LVREN = 0xAA
} LVR_CONFIG_Type;

/**
 * @brief  LVI configuration structure definition
 */
typedef struct {
	/* LVICR */
	LVI_EN_Type EnableIndicator;
	LVI_INTR_Type EnableInterrupt;
	uint32_t LVI_Voltage;
} LVI_CFG_Type;

/**
 * @brief  LVR configuration structure definition
 */
typedef struct {
	/* LVRCNFIG */
	LVR_EN_Type EnableReset;
	LVR_CONFIG_Type EnableOperation;
	uint32_t LVR_Voltage;
} LVR_CFG_Type;

/*******************************************************************************
* Exported Public Function
*******************************************************************************/
void LVI_Init(LVI_CFG_Type *pConfig);
FlagStatus LVI_GetInterruptStatus(void);
void LVI_ClearInterruptStatus(void);

void LVR_Init(LVR_CFG_Type *pConfig);

void PWR_EnterSleep(void);
void PWR_EnterDeepSleep(void);

void PWR_AlwaysOnLSIForDeepSleep(uint8_t LSI, uint8_t BGR, uint8_t VDC);


#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_PWR_H_ */
/* --------------------------------- End Of File ------------------------------ */
