/**********************************************************************
* @file		A34M41x_mpwm.h
* @brief	Contains all macro definitions and function prototypes
* 			support for MPWM firmware library on A34M41x
* @version	1.0
* @date		
* @author ABOV Application 3 team
*
* Copyright(C)  2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_MPWM_H_
#define _A34M41x_MPWM_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

	

typedef enum {
	PH_U = 0,
	PH_V,
	PH_W
} MPWM_PH_VAL;

typedef enum {
	MPWM_MAIN_CAP_CNT,
	MPWM_RISING_CAP_VAL,
	MPWM_FALLING_CAP_VAL,
	MPWM_SUB_CAP_VAL
} MPWM_CAP_MODE;


typedef enum {
	MPWM_Ext_STOP = 0,
	MPWM_Ext_START
} MPWM_Ext_OPERATEVAL;


typedef enum {
	MPWM_Ext_HALT = 0,
	MPWM_Ext_CONT
} MPWM_Ext_PAUSE;


typedef enum {
	RISING_CAP_VAL = 0,
	FALLING_CAP_VAL,
	SUB_CAP_VAL
} MPWM_CAP_SEL;


typedef enum {
	SUBCAP_FALLING = 0,
	SUBCAP_RISING
} MPWM_SCAP_EDGE;


	
/* Public Functions ----------------------------------------------------------- */
/* MPWM Init/DeInit functions --------------------------------------------------*/
HAL_Status_Type HAL_MPWM_Init(MPWM_Type *MPWMx);
HAL_Status_Type HAL_MPWM_DeInit(MPWM_Type* MPWMx);

HAL_Status_Type HAL_MPWM_Start(MPWM_Type* MPWMx, FunctionalState NewState);

HAL_Status_Type HAL_MPWM_SetOutput(MPWM_Type* MPWMx, uint32_t Data);
HAL_Status_Type HAL_MPWM_SetFOutput(MPWM_Type* MPWMx, uint32_t Data);

HAL_Status_Type HAL_MPWM_SetProtCtrl(MPWM_Type* MPWMx, uint32_t Data, uint32_t ch);
HAL_Status_Type HAL_MPWM_SetProtStat(MPWM_Type* MPWMx, uint32_t Data, uint32_t ch);
	
HAL_Status_Type HAL_MPWM_IntConfig(MPWM_Type* MPWMx, uint32_t MPIntCfg, FunctionalState NewState);

HAL_Status_Type HAL_MPWM_Cmd(MPWM_Type* MPWMx, uint32_t motorb, uint32_t update, uint32_t tup, uint32_t bup, uint32_t mchmode, uint32_t updown);

HAL_Status_Type HAL_MPWM_SetPeriod(MPWM_Type* MPWMx, uint32_t period);
HAL_Status_Type HAL_MPWM_SetUDuty(MPWM_Type* MPWMx, uint32_t udutyH, uint32_t udutyL);
HAL_Status_Type HAL_MPWM_SetVDuty(MPWM_Type* MPWMx, uint32_t vdutyH, uint32_t vdutyL);
HAL_Status_Type HAL_MPWM_SetWDuty(MPWM_Type* MPWMx, uint32_t wdutyH, uint32_t wdutyL);

uint32_t HAL_MPWM_GetCounter(MPWM_Type *MPWMx);

HAL_Status_Type HAL_MPWM_SetDeadTime(MPWM_Type* MPWMx, uint32_t dten, uint32_t pshrt, uint32_t clk, uint32_t clkdata, uint32_t dirsel);

HAL_Status_Type HAL_MPWM_SetATR(MPWM_Type* MPWMx, uint32_t atudt, uint32_t atmod, uint32_t atcnt, uint32_t ch, uint32_t trgsrc);


/* Extend Mode MPWM Functions ------------------------------------------------ */
HAL_Status_Type HAL_MPWM_ExtStartCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_Ext_OPERATEVAL OperSel);
HAL_Status_Type HAL_MPWM_ExtEnableCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, uint32_t irq_n, FunctionalState NewStatus);
HAL_Status_Type HAL_MPWM_ExtPauseCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_Ext_PAUSE PauSel);

HAL_Status_Type HAL_MPWM_ExtSetPeriod(MPWM_Type* MPWMx, MPWM_PH_VAL pha_val, uint32_t period);

uint32_t HAL_MPWM_ExtGetCounter(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val);

HAL_Status_Type HAL_MPWM_ExtSetDeadTime(MPWM_Type* MPWMx, MPWM_PH_VAL pha_val, uint32_t dten, uint32_t pshrt, uint32_t clk, uint32_t rclkdata, uint32_t fclkdata, uint32_t dirsel);

HAL_Status_Type HAL_MPWM_ExtCaptureCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_SCAP_EDGE ScapEdge, FunctionalState NewStatus);
HAL_Status_Type HAL_MPWM_ExtClearCaptureCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_CAP_MODE CapMode);
uint32_t HAL_MPWM_ExtGetCaptureCNT(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val);
uint32_t HAL_MPWM_ExtGetCaptureVal(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_CAP_SEL CapSel);

HAL_Status_Type HAL_MPWM_ExtInit(MPWM_Type *MPWMx);



#ifdef __cplusplus
}
#endif


#endif /* _A34M41x_MPWM_H_ */

/* --------------------------------- End Of File ------------------------------ */
