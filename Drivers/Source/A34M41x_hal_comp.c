/**********************************************************************
* @file		A34M41x_comp.c
* @brief	Contains all functions support for ADC firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV M team
*
* Copyright(C) 2015, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_comp.h"
#include "A34M41x_hal_scu.h"
#include "A34M41x_debug_frmwrk.h"


/***********************************************************************************************
* @brief		Configure comparator peripheral and clock reference/input voltage
* @param[in]	COMPx		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
* @param[in]	NewState		should be :
* 						- DISABLE= 0
* 						- ENABLE	= 1							
* @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_Init(COMP_Type *COMPx, FunctionalState NewState)
{
 /* Check COMP handle */
	 if(COMPx == NULL)
	{
			return HAL_ERROR;
	 }
	//Comparator peripheral & Clock Enable
	SYST_ACCESS_EN();
	
	SCU->PER2 &= ~(1<<28);//COMP enable
	SCU->PCER2 &= ~(1<<28); //COMP clock enable 
	
	if(NewState == ENABLE)
	{
		SCU->PER2 |= (1<<28);//COMP enable
		SCU->PCER2 |= (1<<28); //COMP clock enable 
	}
	
	SYST_ACCESS_DIS();
	return HAL_OK;
}

/***********************************************************************************************
 * @brief		Get Comparator Interrupt Status
 * @param[in]	COMPx 		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
 * @return	STAT register value
 ***********************************************************************************************/
uint32_t HAL_COMP_GetStatus_IT(COMP_Type *COMPx)
{
	return COMPx->STAT;
}

/***********************************************************************************************
 * @brief		Clear Comparator interrupt status 
 * @param[in]	COMPx 		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
 * @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_ClearStatus_IT(COMP_Type *COMPx, uint32_t Type)
{
	 /* Check COMP handle */
	 if(COMPx == NULL)
	{
		return HAL_ERROR;
	 }
	COMPx->STAT |= Type;
	return HAL_OK;
}

/***********************************************************************************************
 * @brief		Clear All Comparator interrupt status
 * @param[in]	COMPx COMP peripheral selected, should be :
			- COMP : COMP0~3 peripheral
 * @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_ClearStatusAll_IT(COMP_Type *COMPx)
{
		/* Check COMP handle */
	 if(COMPx == NULL)
	{
			return HAL_ERROR;
	 }
	COMPx->STAT |= COMP_STAT_BITMASK;
	return HAL_OK;
}

/***********************************************************************************************
 * @brief		Configure comparator hysteris 
* @param[in]	COMPx 		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
* @param[in]	hyssel		comparator hysterisis select, should be :
						- COMP_CONF_HYSSEL_5MV	= 0,   					
						- COMP_CONF_HYSSEL_20MV	= 1
* @param[in]	NewState	 	should be :
* 						- DISABLE= 0
* 						- ENABLE	= 1
 * @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_ConfigHysterisis(COMP_Type *COMPx, uint32_t hyssel, FunctionalState NewState)
{
	 /* Check COMP handle */
	 if(COMPx == NULL)
	{
			return HAL_ERROR;
	 }
	COMPx->CONF &= ~(COMP_CONF_HYSEN_BITMASK | COMP_CONF_HYSSEL_BITMASK);
	
	if(NewState == ENABLE)
	{
		COMPx->CONF |= (hyssel | COMP_CONF_HYSEN);
	}
	return HAL_OK;
}


/***********************************************************************************************
* @brief		Configure comparator reference/input voltage
* @param[in]	COMPx		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
* @param[in]	refsel		Comparator Reference(input -) Selection bit, should be :
						- COMP_CONF_INPSEL_CP0A, COMP_CONF_INPSEL_CP1A,
						   COMP_CONF_INPSEL_CP2, COMP_CONF_INPSEL_CP3 = 0,   					
						- COMP_CONF_INPSEL_CP0B,COMP_CONF_INPSEL_CP1B =1
						- COMP_CONF_INPSEL_CP0C, COMP_CONF_INPSEL_CP1C = 2
* @param[in]	inputsel		Comparator Input(input +) Selection bit, should be :
						- COMP_CONF_INNSEL_CREF0, COMP_CONF_INNSEL_CREF1,
						   COMP_CONF_INNSEL_CREF2, COMP_CONF_INNSEL_CREF3 = 0  					
						- COMP_CONF_INNSEL_BRG1V = 1		 							
* @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_ConfigInputLevel(COMP_Type *COMPx, uint32_t refsel, uint32_t inputsel)
{
	 /* Check COMP handle */
	 if(COMPx == NULL)
	{
			return HAL_ERROR;
	 }
	COMPx->CONF &=  ~(COMP_CONF_INPSEL_BITMASK | COMP_CONF_INNSEL_BITMASK);
	COMPx->CONF |= inputsel | refsel;
	return HAL_OK;
}

/***********************************************************************************************
* @brief		Enable/disable Comparator operate
* @param[in]	COMPx		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
* @param[in]	NewState	 	should be :
* 						- DISABLE= 0
* 						- ENABLE	= 1
 * @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_Cmd(COMP_Type *COMPx,  FunctionalState NewState)
{
		/* Check COMP handle */
	 if(COMPx == NULL)
	{
			return HAL_ERROR;
	 }
	COMPx->CTRL &= ~(COMP_CTRL_COMPEN);
	
	if(NewState == ENABLE)
	{
		COMPx->CTRL |= COMP_CTRL_COMPEN;
	}
	return HAL_OK;
}

/***********************************************************************************************
 * @brief		Set Comparator Debounce 
 * @param[in]	COMPx		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
 * @param[in]	dbnccounter	Debounce time base counter, should be in range from 0 to 0xF
 * @return		HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_ConfigDebounce(COMP_Type *COMPx, uint32_t dbnccounter)
{
	 /* Check COMP handle */
	 if(COMPx == NULL)
	{
			return HAL_ERROR;
	 }
	COMPx->CONF &=  ~(COMP_CONF_FLTSEL_BITMASK);
	COMPx->CONF |=  dbnccounter <<24 ;
	return HAL_OK;
}

/***********************************************************************************************
* @brief		Enable/disable Comparator Interrupt
* @param[in]	COMPx		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
* @param[in]	NewState	 	should be :
* 						- DISABLE = 0
* 						- ENABLE	= 1
* @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_InterruptCmd(COMP_Type *COMPx, FunctionalState NewState)
{
		/* Check COMP handle */
	 if(COMPx == NULL)
	{
			return HAL_ERROR;
	 }
	COMPx->CTRL &= ~COMP_CTRL_COMPINTEN;	
	if(NewState == ENABLE)
	{
		COMPx->CTRL |= COMP_CTRL_COMPINTEN;
	}
	return HAL_OK;
}

/***********************************************************************************************
* @brief		Configure comparator interrupt 
* @param[in]	COMPx		COMP peripheral selected, should be :
						- COMP : COMP0~3 peripheral
* @param[in]	ipol			Comparator interrupt polarity(level mode), should be :
						- COMP_CONF_INTPOL_LOW		= 0 					
						- COMP_CONF_INTPOL_HIGH		= 1
* @param[in]	mode		Comparator Interrupt mode select, should be :   		
						- COMP_CONF_INTTYPE_LEVEL	= 1		     			
						- COMP_CONF_INTTYPE_EDGE	= 2		     		
						- COMP_CONF_INTTYPE_BOTH_EDGE		= 3
 * @return	HAL Status
 ***********************************************************************************************/
HAL_Status_Type HAL_COMP_ConfigInterrupt(COMP_Type *COMPx, uint32_t ipol, uint32_t mode)
{
		/* Check COMP handle */
	 if(COMPx == NULL)
	{
			return HAL_ERROR;
	 }
	COMPx->CONF &= ~(COMP_CONF_INTPOL_BITMASK | COMP_CONF_INTTYPE_BITMASK);
	COMPx->CONF |= ipol;
	COMPx->CONF |= mode;
	return HAL_OK;
}
/* --------------------------------- End Of File ------------------------------ */

