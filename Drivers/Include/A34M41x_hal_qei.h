/**********************************************************************
* @file		A34M41x_qei.h
* @brief	Contains all macro definitions and function prototypes
* 			support for fmc firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_QEI_H_
#define _A34M41x_QEI_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

	
//	QEI Mode Register
#define QEInMR_INXGATE_PASS_ALL						(0x0F)
#define QEInMR_INXGATE_PASS_PHA0_PHB0				(0x08)
#define QEInMR_INXGATE_PASS_PHA0_PHB1				(0x04)
#define QEInMR_INXGATE_PASS_PHA1_PHB1				(0x02)
#define QEInMR_INXGATE_PASS_PHA1_PHB0				(0x01)

#define QEInMR_QDVEL_ON								(0x01<<7)
#define QEInMR_DIRI_ON								(0x01<<6)
#define QEInMR_DIRPC_ON								(0x01<<5)
#define QEInMR_QDRST_ON								(0x01<<4)
#define QEInMR_QDCAP_ON								(0x01<<3)
#define QEInMR_QDSIG_ON								(0x01<<2)
#define QEInMR_QDSWAP_ON							(0x01<<1)
#define QEInMR_QDMOD_ON								(0x01<<0)


// QEI Control Register
#define QEInCON_INVI_ON								(0x01<<8)
#define QEInCON_RESV_ON								(0x01<<2)
#define QEInCON_RESI_ON								(0x01<<1)
#define QEInCON_RESP_ON								(0x01<<0)



// QEI Status Register
#define QEInSR_DIR_REVERSE							(0x00<<1)
#define QEInSR_DIR_FORWARD							(0x01<<1)
#define QEInSR_ERROR_CHK							(0x01<<0)


// QEI Interrupt Enable Register
#define QEInIER_VELC_ON								(0x01<<10)
#define QEInIER_VELT_ON								(0x01<<9)
#define QEInIER_IDX_ON								(0x01<<8)

#define QEInIER_MAX_ON								(0x01<<7)
#define QEInIER_POS2_ON								(0x01<<6)
#define QEInIER_POS1_ON								(0x01<<5)
#define QEInIER_POS0_ON								(0x01<<4)

#define QEInIER_ENCLK_ON							(0x01<<3)
#define QEInIER_ERR_ON								(0x01<<2)
#define QEInIER_DIR_ON								(0x01<<1)
#define QEInIER_INX_ON								(0x01<<0)


// QEI Interrupt Status Register
#define QEInISR_VELC_FLAG							(0x01<<10)
#define QEInISR_VELT_FLAG							(0x01<<9)
#define QEInISR_IDX_FLAG							(0x01<<8)

#define QEInISR_MAX_FLAG							(0x01<<7)
#define QEInISR_POS2_FLAG							(0x01<<6)
#define QEInISR_POS1_FLAG							(0x01<<5)
#define QEInISR_POS0_FLAG							(0x01<<4)

#define QEInISR_ENCLK_FLAG							(0x01<<3)
#define QEInISR_ERR_FLAG							(0x01<<2)
#define QEInISR_DIR_FLAG							(0x01<<1)
#define QEInISR_INX_FLAG							(0x01<<0)

#define QEInISR_INT_CLR_MASK						(0x7FF)


// QEI Interrupt Status Clear Register
#define QEInISCR_CLEAR_VELC							(0x01<<10)
#define QEInISCR_CLEAR_VELT							(0x01<<9)
#define QEInISCR_CLEAR_IDX							(0x01<<8)

#define QEInISCR_CLEAR_MAX							(0x01<<7)
#define QEInISCR_CLEAR_POS2							(0x01<<6)
#define QEInISCR_CLEAR_POS1							(0x01<<5)
#define QEInISCR_CLEAR_POS0							(0x01<<4)

#define QEInISCR_CLEAR_ENCLK						(0x01<<3)
#define QEInISCR_CLEAR_ERR							(0x01<<2)
#define QEInISCR_CLEAR_DIR							(0x01<<1)
#define QEInISCR_CLEAR_INX							(0x01<<0)


	
/* Private macros ------------------------------------------------------------- */
HAL_Status_Type HAL_QEI_IDXGatingCmd (QEI_Type *QEIn, uint8_t gate_set_val);
HAL_Status_Type HAL_QEI_ModeCmd (QEI_Type *QEIn, uint32_t ModeVal, FunctionalState NewStatus);
HAL_Status_Type HAL_QEI_FunctionCmd (QEI_Type *QEIn, uint32_t FuncVal, FunctionalState NewStatus);
uint32_t HAL_QEI_GetStatus(QEI_Type *QEIn);
HAL_Status_Type HAL_QEI_SetPositionCNT(QEI_Type *QEIn, uint32_t PosVal);
HAL_Status_Type HAL_QEI_SetMaximumCNT(QEI_Type *QEIn, uint32_t MaxVal);
HAL_Status_Type HAL_QEI_SetCompareCNT(QEI_Type *QEIn, uint32_t Cmp0Val, uint32_t Cmp1Val, uint32_t Cmp2Val);
HAL_Status_Type HAL_QEI_SetIndexCNT(QEI_Type *QEIn, uint32_t IndexVal);
HAL_Status_Type HAL_QEI_SetIndexCompareCNT(QEI_Type *QEIn, uint32_t IdxCmpVal);
HAL_Status_Type HAL_QEI_SetVelocityReload(QEI_Type *QEIn, uint32_t VelReloadVal);
HAL_Status_Type HAL_QEI_SetVelocityTimer(QEI_Type *QEIn, uint32_t VelTimerVal);
HAL_Status_Type HAL_QEI_SetVelocityPulse(QEI_Type *QEIn, uint32_t VelPulseVal);
HAL_Status_Type HAL_QEI_SetVelocityCapture(QEI_Type *QEIn, uint32_t VelCapVal);
HAL_Status_Type HAL_QEI_SetVelocityCompare(QEI_Type *QEIn, uint32_t VelCmpVal);
HAL_Status_Type HAL_QEI_InterruptCmd(QEI_Type *QEIn, uint32_t InterruptVal, FunctionalState NewStatus);
uint32_t QEI_GetInterruptStatus(QEI_Type *QEIn);
HAL_Status_Type HAL_QEI_ClearInterruptFlag(QEI_Type *QEIn, uint32_t FlagBit);
HAL_Status_Type HAL_QEI_ModeInit(QEI_Type *QEIn, uint8_t index_gate_val, uint32_t ModeVal);
uint32_t QEI_getPositionCNT(QEI_Type *QEIn);
uint32_t HAL_QEI_GetInterruptStatus(QEI_Type *QEIn);

/*
 * @brief 	PCU port mode enumerate definition
 */

	
/* Public Functions ----------------------------------------------------------- */

	
	

/* Public Functions ----------------------------------------------------------- */

	


#ifdef __cplusplus
}
#endif


#endif /* end _A34M41x_QEI_H_ */

/* --------------------------------- End Of File ------------------------------ */
