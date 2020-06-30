/**********************************************************************
* @file		A34M41x_qei.c
* @brief	Contains all functions support for fmc firmware library
* 			on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_qei.h"
#include "A34M41x_hal_libcfg.h"
#include "A34M41x_hal_scu.h"



/* QEI ------------------------------------------------------------------------------ */

/**********************************************************************
 * @brief		QEI Index gating configuration
 * @param[in]	QEIn QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Indexing
 * 				 	- QEInMR_INXGATE_PASS_ALL 		(0x0F)
 * 				 	- QEInMR_INXGATE_PASS_PHA0_PHB0 	(0x08)
 * 				 	- QEInMR_INXGATE_PASS_PHA0_PHB1 	(0x04)
 * 				 	- QEInMR_INXGATE_PASS_PHA1_PHB1 	(0x02)
 * 				 	- QEInMR_INXGATE_PASS_PHA1_PHB0 	(0x01)
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_IDXGatingCmd (QEI_Type *QEIn, uint8_t gate_set_val)
{
	uint32_t		reg_val;

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	reg_val = QEIn->MR;
	
	reg_val &= ~(0x0F<<8);
	
	reg_val |= ((gate_set_val & 0x0F)<<8);
	
	QEIn->MR = reg_val;	
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Mode Selection
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	ModeVal	QEI Mode Selection Bit
 * 				 	- QEInMR_QDVEL_ON 				(0x01<<7)
 * 				 	- QEInMR_DIRI_ON 					(0x01<<6)
 * 				 	- QEInMR_DIRPC_ON 				(0x01<<5)
 * 				 	- QEInMR_QDRST_ON 				(0x01<<4)
 * 				 	- QEInMR_QDCAP_ON 				(0x01<<3)
 * 				 	- QEInMR_QDSIG_ON 				(0x01<<2)
 * 				 	- QEInMR_QDSWAP_ON 				(0x01<<1)
 * 				 	- QEInMR_QDMOD_ON 				(0x01<<0)
 * @param[in]	NewStatus
 * 					- DISABLE		:0
 * 					- ENABLE 		:1
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_ModeCmd (QEI_Type *QEIn, uint32_t ModeVal, FunctionalState NewStatus)
{
	uint32_t		reg_val;
	
	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }
	
	reg_val = QEIn->MR;
	
	reg_val &= ~(ModeVal & 0xFF);
	
	if (NewStatus == ENABLE)
	{
		reg_val |= (ModeVal & 0xFF);
	}
	
	QEIn->MR = reg_val;
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Function Command
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in] FuncVal QEI Function Selection Bit
 * 				 	- QEInCON_INVI_ON 				(0x01<<8)
 * 				 	- QEInCON_RESV_ON 				(0x01<<2)
 * 				 	- QEInCON_RESI_ON 				(0x01<<1)
 * 				 	- QEInCON_RESP_ON 				(0x01<<0)
 * @param[in]	NewStatus
 * 					- DISABLE		:0
 * 					- ENABLE 		:1
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_FunctionCmd (QEI_Type *QEIn, uint32_t FuncVal, FunctionalState NewStatus)
{
	uint32_t		reg_val;

	
	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }
	reg_val = QEIn->CON;
	
	reg_val &= ~(FuncVal & 0x107);
	
	if (NewStatus == ENABLE)
	{
		reg_val |= (FuncVal & 0x107);
	}
	
	QEIn->CON = reg_val;
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Get Status
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @return		QEI Status (Direction[1], Error[0] bit)
 *
 **********************************************************************/
uint32_t HAL_QEI_GetStatus(QEI_Type *QEIn)
{
	return (QEIn->SR);
}


/**********************************************************************
 * @brief		QEI Set Position Counter
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Position Setting value
 * 				 	- 0x0 ~ 0xFFFFFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetPositionCNT(QEI_Type *QEIn, uint32_t PosVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->POS = PosVal;
	return HAL_OK;
}

/**********************************************************************
 * @brief		QEI Get Position Counter
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @return	QEIn Position Counter Value
 *
 **********************************************************************/
uint32_t HAL_QEI_getPositionCNT(QEI_Type *QEIn)
{
	return QEIn->POS;
}

/**********************************************************************
 * @brief		QEI Set Maximum counter value
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Maximum counter value Bit
 * 				 	- 0x0 ~ 0xFFFFFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetMaximumCNT(QEI_Type *QEIn, uint32_t MaxVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->MAX = MaxVal;
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Set Compare counter value
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	Cmp0Val	QEI Compare counter 0 value Bit
 * 				 	- 0x0 ~ 0xFFFFFFFF
 * @param[in]	Cmp1Val	QEI Compare counter 1 value Bit
 * 				 	- 0x0 ~ 0xFFFFFFFF
 * @param[in]	Cmp2Val	QEI Compare counter 2 value Bit
 * 				 	- 0x0 ~ 0xFFFFFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetCompareCNT(QEI_Type *QEIn, uint32_t Cmp0Val, uint32_t Cmp1Val, uint32_t Cmp2Val)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->CMP0 = Cmp0Val;
	QEIn->CMP1 = Cmp1Val;
	QEIn->CMP2 = Cmp2Val;
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Set Index counter value
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Index counter value Bit
 * 				 	- 0x0 ~ 0xFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetIndexCNT(QEI_Type *QEIn, uint32_t IndexVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->IDX = (IndexVal & 0xFFFF);
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Set Index compare counter value
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Index Compare counter value Bit
 * 				 	- 0x0 ~ 0xFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetIndexCompareCNT(QEI_Type *QEIn, uint32_t IdxCmpVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->CMPI = (IdxCmpVal & 0xFFFF);
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Set Velocity Reload
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Velocity Reload value Bit
 * 				 	- 0x0 ~ 0xFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetVelocityReload(QEI_Type *QEIn, uint32_t VelReloadVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->VLR = (VelReloadVal & 0xFFFF);
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Set Velocity timer
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Velocity timer value Bit
 * 				 	- 0x0 ~ 0xFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetVelocityTimer(QEI_Type *QEIn, uint32_t VelTimerVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->VLT = (VelTimerVal & 0xFFFF);
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Set Velocity Pulse Counter
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Velocity Pulse Counter value Bit
 * 				 	- 0x0 ~ 0xFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetVelocityPulse(QEI_Type *QEIn, uint32_t VelPulseVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->VLP = (VelPulseVal & 0xFFFF);
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Set Velocity Capture
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Velocity Capture value Bit
 * 				 	- 0x0 ~ 0xFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetVelocityCapture(QEI_Type *QEIn, uint32_t VelCapVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->VLC = (VelCapVal & 0xFFFF);
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Set Velocity Compare
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Velocity Compare value Bit
 * 				 	- 0x0 ~ 0xFFFF
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_SetVelocityCompare(QEI_Type *QEIn, uint32_t VelCmpVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->VLCOM = (VelCmpVal & 0xFFFF);
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Interrupt Enable
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Interrupt Selection
 * 				 	- QEInIER_VELC_ON 		(0x01<<10)
 * 				 	- QEInIER_VELT_ON 		(0x01<<9)
 * 				 	- QEInIER_IDX_ON 			(0x01<<8)
 * 				 	- QEInIER_MAX_ON 			(0x01<<7)
 * 				 	- QEInIER_POS2_ON 		(0x01<<6)
 * 				 	- QEInIER_POS1_ON 		(0x01<<5)
 * 				 	- QEInIER_POS0_ON 		(0x01<<4)
 * 				 	- QEInIER_ENCLK_ON 		(0x01<<3)
 * 				 	- QEInIER_ERR_ON 			(0x01<<2)
 * 				 	- QEInIER_DIR_ON 			(0x01<<1)
 * 				 	- QEInIER_INX_ON 			(0x01<<0)
 * @param[in]	QEI Interrupt Enable Bit
 * 				Enable/Disable
 *
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_InterruptCmd(QEI_Type *QEIn, uint32_t InterruptVal, FunctionalState NewStatus)
{
	uint32_t		reg_val;

	
	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }
		 
	reg_val = QEIn->IER;
	
	reg_val &= ~(InterruptVal & 0x7FF);
	
	if (NewStatus == ENABLE)
	{
		reg_val |= (InterruptVal & 0x7FF);
	}
	
	QEIn->IER = reg_val;
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Get Interrupt Status
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @return		Interrupt Status Value
 *
 **********************************************************************/
uint32_t HAL_QEI_GetInterruptStatus(QEI_Type *QEIn)
{
	return (QEIn->ISR);
}


/**********************************************************************
 * @brief		QEI Clear Interrupt Flag
 * @param[in]	QEIn	QEI peripheral selected, should be:
 * 				 	- QEIn :QEI0~1 peripheral
 * @param[in]	QEI Clear Interrupt Flag Bit Selection
 * 				 	- QEInIER_VELC_FLAG		(0x01<<10)
 * 				 	- QEInIER_VELT_FLAG 		(0x01<<9)
 * 				 	- QEInIER_IDX_FLAG 		(0x01<<8)
 * 				 	- QEInIER_MAX_FLAG		(0x01<<7)
 * 				 	- QEInIER_POS2_FLAG 		(0x01<<6)
 * 				 	- QEInIER_POS1_FLAG 		(0x01<<5)
 * 				 	- QEInIER_POS0_FLAG 		(0x01<<4)
 * 				 	- QEInIER_ENCLK_FLAG 		(0x01<<3)
 * 				 	- QEInIER_ERR_FLAG		(0x01<<2)
 * 				 	- QEInIER_DIR_FLAG		(0x01<<1)
 * 				 	- QEInIER_INX_FLAG		(0x01<<0)
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_ClearInterruptFlag(QEI_Type *QEIn, uint32_t FlagBit)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	QEIn->ISCR = (FlagBit & 0x7FF);
	return HAL_OK;
}


/**********************************************************************
 * @brief		QEI Mode Init
 * @param[in]	QEI Mode Indexing
 * 				 	- QEInMR_INXGATE_PASS_ALL 		(0x0F)
 * 				 	- QEInMR_INXGATE_PASS_PHA0_PHB0 	(0x08)
 * 				 	- QEInMR_INXGATE_PASS_PHA0_PHB1 	(0x04)
 * 				 	- QEInMR_INXGATE_PASS_PHA1_PHB1 	(0x02)
 * 				 	- QEInMR_INXGATE_PASS_PHA1_PHB0 	(0x01)
 * @param[in]	ModeVal	QEI Mode Selection Bit
 * 				 	- QEInMR_QDVEL_ON 				(0x01<<7)
 * 				 	- QEInMR_DIRI_ON 					(0x01<<6)
 * 				 	- QEInMR_DIRPC_ON 				(0x01<<5)
 * 				 	- QEInMR_QDRST_ON 				(0x01<<4)
 * 				 	- QEInMR_QDCAP_ON 				(0x01<<3)
 * 				 	- QEInMR_QDSIG_ON 				(0x01<<2)
 * 				 	- QEInMR_QDSWAP_ON 				(0x01<<1)
 * 				 	- QEInMR_QDMOD_ON 				(0x01<<0)
 * @return		HAL Status
 *
 **********************************************************************/
HAL_Status_Type HAL_QEI_ModeInit(QEI_Type *QEIn, uint8_t index_gate_val, uint32_t ModeVal)
{

	/* Check QEI  handle */
         if(QEIn == NULL)
        {
            return HAL_ERROR;
        }

	SYST_ACCESS_EN();
	
	if (QEIn == QEI0)
	{
		SCU->PER1 &= ~(1 << 28);
		SCU->PCER1 &= ~(1 << 28);
		
		SCU->PER1 |= (1 << 28);
		SCU->PCER1 |= (1 << 28);
	}
	else if (QEIn == QEI1)
	{
		SCU->PER1 &= ~(1 << 29);
		SCU->PCER1 &= ~(1 << 29);
		
		SCU->PER1 |= (1 << 29);
		SCU->PCER1 |= (1 << 29);
	}
	
	SYST_ACCESS_EN();
	
	// Index Gating Value Setting
	HAL_QEI_IDXGatingCmd(QEIn, index_gate_val);
	
	// QEI Mode Selection
	HAL_QEI_ModeCmd(QEIn, ModeVal, ENABLE);
	return HAL_OK;
}



/* --------------------------------- End Of File ------------------------------ */

