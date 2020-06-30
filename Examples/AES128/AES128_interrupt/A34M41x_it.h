/**********************************************************************
* @file		A34M41x_it.h
* @brief	Contains all macro definitions and function prototypes
* 			support for PCU firmware library
* @version	1.0
* @date		
* @author	ABOV M team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __A34M41x_IT_H
#define __A34M41x_IT_H

/* Includes ------------------------------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif
	
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);


void AES128_IRQHandler(void);

	
#ifdef __cplusplus
}
#endif

#endif /* __A34M41x_IT_H */

