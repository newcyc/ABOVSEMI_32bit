/**********************************************************************
* @file		A34M41x_frt.c
* @brief	Contains all functions support for FRT(Free Running TImer) firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/
/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_frt.h"
#include "A34M41x_hal_libcfg.h"
#include "A34M41x_hal_scu.h"
#include "A34M41x_debug_frmwrk.h"	

/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief		Initialization register related FRT function at SCU
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 *                  - MODE_v : Mode Selection Bit
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_Init(FRT_Type *FRTx, uint8_t MODE_v)
{
         /* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
		 
	SYST_ACCESS_EN();
	
	if(FRTx == FRT0)
	{		
		SCU->PER1 &= ~(1 << 6); 
		SCU->PCER1 &= ~(1 << 6);
		
		SCU->PER1 |= (1 << 6);			// FRT0 Enable
		SCU->PCER1 |= (1 << 6);			// FRT0 Clock Enable
	}
	else if(FRTx == FRT1)
	{
		SCU->PER1 &= ~(1 << 7); 
		SCU->PCER1 &= ~(1 << 7);
		
		SCU->PER1 |= (1 << 7);			// FRT1 Enable
		SCU->PCER1 |= (1 << 7);			// FRT1 Clock Enable
	}
	
	SYST_ACCESS_DIS();

	HAL_FRT_ModeSelection(FRTx, MODE_v);
	
	/* Register Reset */
	HAL_FRT_ClearCounter(FRTx);
	return HAL_OK;

}

/*********************************************************************
 * @brief		Set Clock Source Type of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @param[in]	Clock Source:
 *					- SCU_MCCR_CSEL_LSI 
 *					- SCU_MCCR_CSEL_LSE  
 *					- SCU_MCCR_CSEL_MCLK 
 *					- SCU_MCCR_CSEL_HSI 
 *					- SCU_MCCR_CSEL_HSE 
 *					- SCU_MCCR_CSEL_PLL 
 * @param[in]	Clock Divider: 
 *					- 0x0~0xFF
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_ClockSource(FRT_Type *FRTx, uint8_t ClkSource, uint8_t ClkDivider)
{
	 /* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
		 
	SYST_ACCESS_EN();
	
	if (FRTx == FRT0)
	{
		SCU->MCCR6 &= ~(0x07<<8);
		SCU->MCCR6 &= ~(0xFF<<0);
		
		SCU->MCCR6 |= (((ClkSource&0x07)<<8) | ((ClkDivider&0xFF)<<0));
	}
	else if (FRTx == FRT1)
	{
		SCU->MCCR6 &= ~(0x07<<24);
		SCU->MCCR6 &= ~(0xFF<<16);
		
		SCU->MCCR6 |= (((ClkSource&0x07)<<24) | ((ClkDivider&0xFF)<<16));
	}
	
	SYST_ACCESS_DIS();
	return HAL_OK;
}

/*********************************************************************
 * @brief		Get Clock Source Type of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @return 		Clock Source Type
 **********************************************************************/
uint8_t HAL_FRT_GetClockSource(FRT_Type *FRTx)
{
	uint8_t ClkSource;
	
	/* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
	
	if (FRTx == FRT0)
	{
		ClkSource = ((SCU->MCCR6&(0x07<<8))>>8);
	}
	else if (FRTx == FRT1)
	{
		ClkSource = ((SCU->MCCR6&(0x07<<24))>>24);
	}
	
	
	return ClkSource;
}


/*********************************************************************
 * @brief		Set FRT Operation 
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @param[in]	isset	Enable or Disable
 *					- 1		:Enable 
 *					- 0		:Disable
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_Run(FRT_Type *FRTx, uint8_t isset)
{
	/* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
		 
	if(isset)
	{
		FRTx->CTRL |= FRT_CTRL_FRT_ON;		// FRT Enable
	}
	else
	{
		FRTx->CTRL &= ~FRT_CTRL_FRT_ON;		// FRT Disable
	}
	return HAL_OK;
}

/*********************************************************************
 * @brief		Set FRT Match Interrupt
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @param[in]	isset	Enable or Disable
 *					- 1		:Enable 
 *					- 0		:Disable
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_MatchInterrupt(FRT_Type *FRTx, uint8_t isset)
{
	/* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
		 
	if(isset)
	{
		FRTx->CTRL |= FRT_CTRL_MATCHIE;		// FRT Match Interrupt Enable
	}
	else
	{
		FRTx->CTRL &= ~FRT_CTRL_MATCHIE;	// FRT Match Interrupt Disable
	}
	return HAL_OK;
}

/*********************************************************************
 * @brief		Set FRT OVERFLOW Interrupt Mode
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @param[in]	isset	Enable or Disable
 *					- 1		:Enable 
 *					- 0		:Disable
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_OverflowInterrupt(FRT_Type *FRTx, uint8_t isset)
{
	/* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
		 
	if(isset)		
	{
		FRTx->CTRL |= FRT_CTRL_OVFIE;		// FRT Overflow Interrupt Enable
	}
	else
	{
		FRTx->CTRL &= ~FRT_CTRL_OVFIE;		// FRT Overflow Interrupt Disable
	}
	return HAL_OK;
}


/*********************************************************************
 * @brief		Clear Status of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @param[in]	StatusType	Status Type
 *					- FRT_STAT_OVFI
					- FRT_STAT_MATCHI
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_ClearStatus(FRT_Type *FRTx, uint16_t StatusType)
{
	/* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
		 
	FRTx->STAT = StatusType;
	return HAL_OK;
}

/*********************************************************************
 * @brief		Get Status of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @return 		8-bit Status
 **********************************************************************/
uint16_t HAL_FRT_GetStatus(FRT_Type *FRTx)
{	
	return FRTx->STAT;			// Get Status Register Value
	
}


/*********************************************************************
 * @brief		Get Counter Value of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @return 		32-bit Count Value (uint32_t)
 **********************************************************************/
uint32_t HAL_FRT_GetCounterVal(FRT_Type *FRTx)
{
	uint32_t CountValue = 0;
    
	CountValue = FRTx->CNT;				// Get FRTx Counter Value

	return CountValue;
}

/*********************************************************************
 * @brief		Clear Counter Value of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_ClearCounter(FRT_Type *FRTx)
{
	uint32_t regtemp;
	
	/* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }	
	regtemp = (0x00); 
	FRTx->CNT = regtemp;			// Clear FRTx Counter Register Value
//	_DBG("FRTx Counter Value is Cleard.\r\n");
        return HAL_OK;

}

/*********************************************************************
 * @brief		Set Match Counter Value of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @param[in]	MCNT_v	FRT Counter Value Bit
 *
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_SetMatchCounter(FRT_Type *FRTx, uint32_t MCNT_v)
{
	uint32_t	regtemp;

	/* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
	regtemp = MCNT_v;
	FRTx->MCNT = regtemp;
	return HAL_OK;
}

/*********************************************************************
 * @brief		Get Match Counter Value of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @return 		32-bit Match Counter Value
 **********************************************************************/
uint32_t HAL_FRT_GetMatchCounter(FRT_Type *FRTx)
{
	uint32_t MatchCounterValue;

	MatchCounterValue = FRTx->MCNT;
	
	return MatchCounterValue;
}

/*********************************************************************
 * @brief		Mode Selection of FRT
 * @param[in]	FRTx	Pointer to selected FRT peripheral, should be:
 * 					- FRT	:FRT peripheral
 * @param[in]	MODE_v	FRT Mode Selection Bit
 * 					- FRT_CTRL_MODE_FREE_RUN	(0<<1)
 * 					- FRT_CTRL_MODE_MATCH		(1<<1)	
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_FRT_ModeSelection(FRT_Type *FRTx, uint8_t MODE_v)
{
	uint32_t	regtemp;
	
	/* Check FRT  handle */
         if(FRTx == NULL)
        {
            return HAL_ERROR;
         }
	regtemp = FRTx->CTRL;
	regtemp &= ~(0x01<<1);
	regtemp |= (MODE_v&(1<<1));
		
	FRTx->CTRL = regtemp;
	return HAL_OK;
}




/* --------------------------------- End Of File ------------------------------ */

