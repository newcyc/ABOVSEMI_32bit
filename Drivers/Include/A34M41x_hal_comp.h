/**********************************************************************
* @file		A34M41x_comp.h
* @brief	Contains all macro definitions and function prototypes
* 			support for UART firmware library on A34M41x
* @version	1.0
* @date		
* @author ABOV M team
*
* Copyright(C)  2015, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_COMP_H_
#define _A34M41x_COMP_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
//#include "def.h"
#include "A34M41x_aa_types.h"


#ifdef __cplusplus
extern "C"
{
#endif


/* Private Macros ------------------------------------------------------------- */

/* --------------------- BIT DEFINITIONS -------------------------------------- */
/**********************************************************************
 * Macro defines for Macro defines for Comparator confiure register
 **********************************************************************/
#define COMP_CONF_INPSEL_CP0A   		((uint32_t)(0))   		
#define COMP_CONF_INPSEL_CP1A   		((uint32_t)(0))   
#define COMP_CONF_INPSEL_CP2   		((uint32_t)(0))   		
#define COMP_CONF_INPSEL_CP3   		((uint32_t)(0))   

#define COMP_CONF_INPSEL_CP0B   		((uint32_t)(1<<0))   		
#define COMP_CONF_INPSEL_CP1B   		((uint32_t)(1<<0))   	

#define COMP_CONF_INPSEL_CP0C  		((uint32_t)(2<<0))	
#define COMP_CONF_INPSEL_CP1C  		((uint32_t)(2<<0)) 
	
#define COMP_CONF_INPSEL_BITMASK 		((uint32_t)(3<<0))   		
	
#define COMP_CONF_INNSEL_CREF0    		((uint32_t)(0<<4))   
#define COMP_CONF_INNSEL_CREF1    		((uint32_t)(0<<4))   
#define COMP_CONF_INNSEL_CREF2    		((uint32_t)(0<<4))   
#define COMP_CONF_INNSEL_CREF3    		((uint32_t)(0<<4))   

#define COMP_CONF_INNSEL_BRG1V    		((uint32_t)(1<<4))   

#define COMP_CONF_INNSEL_BITMASK    		((uint32_t)(3<<4)) 

#define COMP_CONF_INTTYPE_NOTUSE    		((uint32_t)(0<<8))   
#define COMP_CONF_INTTYPE_LEVEL    		((uint32_t)(1<<8))   
#define COMP_CONF_INTTYPE_EDGE    		((uint32_t)(2<<8))   
#define COMP_CONF_INTTYPE_BOTH_EDGE    		((uint32_t)(3<<8))
	
#define COMP_CONF_INTTYPE_BITMASK    		((uint32_t)(3<<8))   

#define COMP_CONF_INTPOL_LOW    		((uint32_t)(0<<10))   
#define COMP_CONF_INTPOL_HIGH    		((uint32_t)(1<<10))

#define COMP_CONF_INTPOL_BITMASK    		((uint32_t)(1<<10))
	
#define COMP_CONF_HYSSEL_5MV     		((uint32_t)(0<<16))   	
#define COMP_CONF_HYSSEL_20MV     		((uint32_t)(1<<16))   	
#define COMP_CONF_HYSSEL_BITMASK     		((uint32_t)(1<<16))   	

#define COMP_CONF_HYSDISEN	((uint32_t)(0<<20))   	
#define COMP_CONF_HYSEN	((uint32_t)(1<<20))   	
#define COMP_CONF_HYSEN_BITMASK     		((uint32_t)(1<<20))   

#define COMP_CONF_FLTSEL(val)			((uint32_t)((val&0xF)<<24uL))
#define COMP_CONF_FLTSEL_BITMASK	((uint32_t)(0xF<<24uL))

#define COMP_CONF_BITMASK		((uint32_t)(0x0F110733uL))		

/**********************************************************************
 * Macro defines for Macro defines for Comparator control register
 **********************************************************************/
#define COMP_CTRL_COMPEN   		((uint32_t)(1<<0))   		
#define COMP_CTRL_COMPDISEN 		((uint32_t)(0<<0))   		
#define COMP_CTRL_COMPEN_MASK 		((uint32_t)(1<<0))   	

#define COMP_CTRL_COMPINTEN   		((uint32_t)(1<<8))   		
#define COMP_CTRL_COMPDISINTEN 		((uint32_t)(0<<8))   		
#define COMP_CTRL_COMPINTEN_MASK 		((uint32_t)(1<<8))   		

#define COMP_CTRL_BITMASK		((uint32_t)(0x101))		

/**********************************************************************
 * Macro defines for Macro defines for Comparator Interrupt status register
 **********************************************************************/
#define COMP_STAT_COMPINTF		((uint8_t)(1<<8)) 	
#define COMP_STAT_COMPINTF_BITMASK		((uint8_t)(1<<8)) 	
#define COMP_STAT_COMPFLAG		((uint8_t)(1<<0))
#define COMP_STAT_COMPFLAG_BITMASK	((uint8_t)(1<<0)
#define COMP_STAT_BITMASK	((uint32_t)(0x101)) 

/* Public Types --------------------------------------------------------------- */
/***********************************************************************
 * @brief Comparator enumeration
**********************************************************************/

/* Public Functions ----------------------------------------------------------- */
/* Comparator interrupt status get/clear functions --------------------------------------------------*/
uint32_t HAL_COMP_GetStatus_IT(COMP_Type *COMPx);
HAL_Status_Type HAL_COMP_ClearStatus_IT(COMP_Type *COMPx, uint32_t Type);
HAL_Status_Type HAL_COMP_ClearStatusAll_IT(COMP_Type *COMPx);

/* Comparator configure functions -------------------------------------------------*/
HAL_Status_Type HAL_COMP_ConfigHysterisis(COMP_Type *COMPx, uint32_t hyssel, FunctionalState NewState);
HAL_Status_Type HAL_COMP_ConfigInputLevel(COMP_Type *COMPx, uint32_t refsel, uint32_t inputsel);
HAL_Status_Type HAL_COMP_ConfigDebounce(COMP_Type *COMPx, uint32_t dbnccounter);
HAL_Status_Type HAL_COMP_ConfigInterrupt(COMP_Type *COMPx, uint32_t ipol, uint32_t mode);
HAL_Status_Type HAL_COMP_Init(COMP_Type *COMPx, FunctionalState NewState);

/* Comparator operate functions -------------------------------------------------*/
HAL_Status_Type HAL_COMP_InterruptCmd(COMP_Type *COMPx, FunctionalState NewState);
HAL_Status_Type HAL_COMP_Cmd(COMP_Type *COMPx,  FunctionalState NewState);

#ifdef __cplusplus
}
#endif


#endif /* _COMP_H_ */

/* --------------------------------- End Of File ------------------------------ */
