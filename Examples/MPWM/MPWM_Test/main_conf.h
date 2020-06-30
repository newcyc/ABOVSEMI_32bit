/**********************************************************************
* @file		main_conf.h
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
#ifndef __A34M41x_CONF_H
#define __A34M41x_CONF_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "string.h"
#include "A34M41x_hal_pcu.h"
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_adc.h"
#include "A34M41x_hal_cfmc.h"
#include "A34M41x_hal_libcfg.h"
#include "A34M41x_hal_uart.h"
#include "A34M41x_debug_frmwrk.h"
#include "A34M41x_hal_timer.h"
#include "A34M41x_it.h"
#include "A34M41x_hal_mpwm.h"
#include "slib.h"


#ifdef __cplusplus
extern "C"
{
#endif	
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/* Initialize all port */
extern void Port_Init(void); 
/* Configure the system clock to 48 MHz */
extern void SystemClock_Config(void);
#ifdef __cplusplus
}
#endif


#endif /* __A34M41x_CONF_H */


