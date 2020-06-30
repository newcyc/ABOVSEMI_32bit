/**********************************************************************
* @file		A34M41x_timer.c
* @brief	Contains all functions support for timer firmware library
* 			on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application2 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Peripheral group ----------------------------------------------------------- */

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_timer.h"
#include "A34M41x_hal_libcfg.h"


/* Public Functions ----------------------------------------------------------- */
/**********************************************************************
 * @brief 		Initial Timer/Counter device
 * 				 	Set Clock frequency for Timer
 * 					Set initial configuration for Timer
 * @param[in]	TIMERx  Timer selection, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @param[in]	TimerCounterMode Timer counter mode, should be:
 * 					- PERIODIC_MODE			:Timer mode
 * 					- PWM_MODE	:Counter rising mode
 * 					- ONESHOT_MODE	:Counter falling mode
 * 					- CAPTURE_MODE		:Counter on both edges
 * @param[in]	TIMER_ConfigStruct pointer to TIM_TIMERCFG_Type
 * 				that contains the configuration information for the
 *                    specified Timer peripheral.
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_TIMER_Init(TIMER_Type *TIMERx, TIMER_MODE_OPT TimerCounterMode, void *TIMER_ConfigStruct)
{
	TIMER_PERIODICCFG_Type *pTimeCfg;
	TIMER_PWMCFG_Type *pPwmOneshotCfg;
	TIMER_CAPTURECFG_Type *pCaptureCfg;
	uint16_t reg_val16;

	/* Check TIMER  handle */
         if(TIMERx == NULL)
        {
            return HAL_ERROR;
        }
		 
	//set power
	SYST_ACCESS_EN();
	
	if (TIMERx== TIMER0)
	{
		SCU->PER1&=~(1<<16);
		SCU->PCER1&=~(1<<16);
		
		/* Set up peripheral clock for TIMER0 module */
		SCU->PER1|=(1<<16);
		SCU->PCER1|=(1<<16);
	}
	else if (TIMERx== TIMER1)
	{
		SCU->PER1&=~(1<<17);
		SCU->PCER1&=~(1<<17);
		
		/* Set up peripheral clock for TIMER1 module */		
		SCU->PER1|=(1<<17);
		SCU->PCER1|=(1<<17);
	}
	else if (TIMERx== TIMER2)
	{
		SCU->PER1&=~(1<<18);
		SCU->PCER1&=~(1<<18);
		
		/* Set up peripheral clock for TIMER2 module */	
		SCU->PER1|=(1<<18);
		SCU->PCER1|=(1<<18);
	}
	else if (TIMERx== TIMER3)
	{
		SCU->PER1&=~(1<<19);
		SCU->PCER1&=~(1<<19);
		
		/* Set up peripheral clock for TIMER3 module */			
		SCU->PER1|=(1<<19);
		SCU->PCER1|=(1<<19);
	}
	else if (TIMERx== TIMER4)
	{
		SCU->PER1&=~(1<<20);
		SCU->PCER1&=~(1<<20);
		
		/* Set up peripheral clock for TIMER4 module */			
		SCU->PER1|=(1<<20);
		SCU->PCER1|=(1<<20);
	}
	else if (TIMERx== TIMER5)
	{
		SCU->PER1&=~(1<<21);
		SCU->PCER1&=~(1<<21);
		
		/* Set up peripheral clock for TIMER5 module */			
		SCU->PER1|=(1<<21);
		SCU->PCER1|=(1<<21);
	}
	else if (TIMERx== TIMER6)
	{
		SCU->PER1&=~(1<<22);
		SCU->PCER1&=~(1<<22);
		
		/* Set up peripheral clock for TIMER6 module */			
		SCU->PER1|=(1<<22);
		SCU->PCER1|=(1<<22);
	}
	else if (TIMERx== TIMER7)
	{
		SCU->PER1&=~(1<<23);
		SCU->PCER1&=~(1<<23);
		
		/* Set up peripheral clock for TIMER7 module */			
		SCU->PER1|=(1<<23);
		SCU->PCER1|=(1<<23);
	}
	else if (TIMERx== TIMER8)
	{
		SCU->PER1&=~(1<<24);
		SCU->PCER1&=~(1<<24);
		
		/* Set up peripheral clock for TIMER8 module */			
		SCU->PER1|=(1<<24);
		SCU->PCER1|=(1<<24);
	}
	else if (TIMERx== TIMER9)
	{
		SCU->PER1&=~(1<<25);
		SCU->PCER1&=~(1<<25);
		
		/* Set up peripheral clock for TIMER9 module */			
		SCU->PER1|=(1<<25);
		SCU->PCER1|=(1<<25);
	}
	
	SYST_ACCESS_DIS();
	

	if (TimerCounterMode == PERIODIC_MODE)
	{
		pTimeCfg = (TIMER_PERIODICCFG_Type *)TIMER_ConfigStruct;
		reg_val16 = 0
			|TimerCounterMode
//			|(1<<12)		
			|(1<<11)	// OUTPUT
			|TIMER_CR1_CKSEL_SET(pTimeCfg->CkSel) 
			|TIMER_CR1_STARTLVL_SET(pTimeCfg->StartLevel)
			|TIMER_CR1_ADCTRGEN_SET(pTimeCfg->AdcTrgEn)
			;
		TIMERx->CR1 = reg_val16;
		TIMERx->PRS = (pTimeCfg->Prescaler & TIMER_PRS_MASK);
		TIMERx->GRA = pTimeCfg->GRA;
		TIMERx->GRB = pTimeCfg->GRB;		
		TIMERx->CNT = pTimeCfg->Cnt;
	}
	else if ((TimerCounterMode == PWM_MODE) || (TimerCounterMode == ONESHOT_MODE))
	{
		pPwmOneshotCfg = (TIMER_PWMCFG_Type *)TIMER_ConfigStruct;
		reg_val16 = 0
			|TimerCounterMode
			|(1<<11)	// OUTPUT
			|TIMER_CR1_CKSEL_SET(pPwmOneshotCfg->CkSel) 
			|TIMER_CR1_STARTLVL_SET(pPwmOneshotCfg->StartLevel)
			|TIMER_CR1_ADCTRGEN_SET(pPwmOneshotCfg->AdcTrgEn)
//			| (1<<12)
			;
		TIMERx->CR1 = reg_val16;
		TIMERx->PRS = (pPwmOneshotCfg->Prescaler & TIMER_PRS_MASK);
		TIMERx->GRA = pPwmOneshotCfg->GRA;		
		TIMERx->GRB = pPwmOneshotCfg->GRB;	
		TIMERx->CNT = pPwmOneshotCfg->Cnt;		
	}
	else if (TimerCounterMode == CAPTURE_MODE)
	{
		pCaptureCfg = (TIMER_CAPTURECFG_Type *)TIMER_ConfigStruct;
		reg_val16 = 0
			|TimerCounterMode
			|(0<<11)	// INPUT
			|TIMER_CR1_CLRMD_SET(pCaptureCfg->ClrMode)		
			|TIMER_CR1_CKSEL_SET(pCaptureCfg->CkSel) 
			|TIMER_CR1_ADCTRGEN_SET(pCaptureCfg->AdcTrgEn)
			;
		TIMERx->CR1 = reg_val16;
		TIMERx->PRS = (pCaptureCfg->Prescaler & TIMER_PRS_MASK);
	}
	TIMERx->CR2 = 0x2; //  timer counter clear and disable timer 	
	return HAL_OK;
}

/**********************************************************************
 * @brief 		Close Timer/Counter device
 * @param[in]	TIMx  Pointer to timer device, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_TIMER_DeInit (TIMER_Type *TIMERx)
{

	/* Check TIMER  handle */
         if(TIMERx == NULL)
        {
            return HAL_ERROR;
        }

	// Disable timer/counter
	TIMERx->CR2 = 0x00;
	return HAL_OK;
}

/**********************************************************************
 * @brief 		Clear Timer Status
 * @param[in]	TIMERx  Timer selection, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @param[in]	status clear value
 * 					- TIMER_SR_OVF			(1<<0)
 * 					- TIMER_SR_MFB			(1<<1)
 * 					- TIMER_SR_MFA			(1<<2)
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_TIMER_ClearStatus(TIMER_Type *TIMERx, uint8_t value)
{

	/* Check TIMER  handle */
         if(TIMERx == NULL)
        {
            return HAL_ERROR;
        }

	TIMERx->SR = value;
	return HAL_OK;
}

/**********************************************************************
 * @brief 		Get Timer Status
 * @param[in]	TIMERx  Timer selection, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @return 		Value of status register
 **********************************************************************/
uint8_t HAL_TIMER_GetStatus(TIMER_Type *TIMERx)
{
	return (TIMERx->SR);
}

/**********************************************************************
 * @brief	 	Start/Stop Timer/Counter device
 * @param[in]	TIMERx Pointer to timer device, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @param[in]	NewState
 * 					- ENABLE  	:Set timer enable
 * 					- DISABLE 	:Disable timer
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_TIMER_Cmd(TIMER_Type *TIMERx, FunctionalState NewState)
{

	/* Check TIMER  handle */
         if(TIMERx == NULL)
        {
            return HAL_ERROR;
        }

	if (NewState == ENABLE) {
		TIMERx->CR2 =  TIMER_ENABLE;	
	}
	else {
		TIMERx->CR2 &= ~TIMER_ENABLE;
		TIMERx->CR2 =  TIMER_CLEAR;
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief 		Clear Timer/Counter device,
 * @param[in]	TIMERx Pointer to timer device, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_TIMER_ClearCounter(TIMER_Type *TIMERx)
{

	/* Check TIMER  handle */
         if(TIMERx == NULL)
        {
            return HAL_ERROR;
        }

	TIMERx->CR2 |= TIMER_CLEAR;
	return HAL_OK;
}

/**********************************************************************
 * @brief 		Update value
 * @param[in]	TIMERx Pointer to timer device, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @param[in]	CountCh, should be: 0=GRA,1=GRB,2=CNT
 * @param[in]	MatchValue		updated match value
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_TIMER_UpdateCountValue(TIMER_Type *TIMERx, uint8_t CountCh, uint16_t Value)
{

	/* Check TIMER  handle */
         if(TIMERx == NULL)
        {
            return HAL_ERROR;
        }

	switch(CountCh) {
	case 0:
		TIMERx->GRA = Value;
		break;
	case 1:
		TIMERx->GRB = Value;
		break;
	case 2:
		TIMERx->CNT = Value;
		break;	
	}
	return HAL_OK;
}

/*********************************************************************//**
 * @brief 		Read value of capture register in timer/counter device
 * @param[in]	TIMERx Pointer to timer device, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @param[in]	CountCh, should be: 0=GRA,1=GRB,2=CNT
 * @return 		Value of count register
 **********************************************************************/
uint16_t HAL_TIMER_GetCountValue(TIMER_Type *TIMERx, uint8_t CountCh)
{

	/* Check TIMER  handle */
         if(TIMERx == NULL)
        {
            return HAL_ERROR;
        }

	switch(CountCh){
		case 0: return TIMERx->GRA ;
		case 1:	return TIMERx->GRB ;
		case 2:	return TIMERx->CNT;
	}
	return 0;
}

/********************************************************************
 * @brief 		Enable or disable TIMER interrupt.
 * @param[in]	TIMERx Pointer to timer device, should be:
 * 					- TIMER0	:TIMER0 peripheral
 * 					- TIMER1	:TIMER1 peripheral
 * 					- TIMER2	:TIMER2 peripheral
 * 					- TIMER3	:TIMER3 peripheral
 * 					- TIMER4	:TIMER4 peripheral
 * 					- TIMER5	:TIMER5 peripheral
 * 					- TIMER6	:TIMER6 peripheral
 * 					- TIMER7	:TIMER7 peripheral
 * 					- TIMER8	:TIMER8 peripheral
 * 					- TIMER9	:TIMER9 peripheral
 * @param[in]	UARTIntCfg	Specifies the interrupt flag,
 * 				should be one of the following:
 *						- TIMER_INTCFG_OVIE : OVIE Interrupt enable
 *						- TIMER_INTCFG_MBIE : MBIE Interrupt enable
 *						- TIMER_INTCFG_MAIE : MAIE interrupt enable
 * @param[in]	NewState New state of specified timer interrupt type,
 * 				should be:
 * 					- ENALBE	:Enable this interrupt type.
 * 					- DISALBE	:Disable this interrupt type.
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_TIMER_ConfigInterrupt(TIMER_Type *TIMERx, TIMER_INT_Type TIMERIntCfg, FunctionalState NewState)
{
	uint8_t tmp;
	
	/* Check TIMER  handle */
         if(TIMERx == NULL)
        {
            return HAL_ERROR;
        }

	switch(TIMERIntCfg){
		case TIMER_INTCFG_OVIE:
			tmp = TIMER_IER_OVIE;
			break;
		case TIMER_INTCFG_MBIE:
			tmp = TIMER_IER_MBIE;
			break;
		case TIMER_INTCFG_MAIE:
			tmp = TIMER_IER_MAIE;
			break;
	}

	if (NewState == ENABLE)
	{
		TIMERx->IER |= tmp;
	}
	else
	{
		TIMERx->IER &= (~tmp) & TIMER_IER_BITMASK;
	}
	return HAL_OK;
}


/**********************************************************************
 * @brief 		Setting Timer SYNC1-2
 * @param[in]	  	TIMERa		TIMER peripheral A
 * 					- TIMER0~9
 * @param[in]		TIMERb		TIMER peripheral B
 * 					- TIMER0~9
 * @param[in]		CMD			SYNC Command bit
 * 					- TIMER_SYNC_SSYNC		(1<<17)
 * 					- TIMER_SYNC_CSYNC		(1<<16)
 * @param[in]	DLYVAL		TIMER SYNC Delay Value
 *					- 0~255
 *
 *						 SYNCB
 *				TIMERa --------> TIMERb
 *					  |<------->|
 *					  | SYNCDLY |
 *
 * @param[in]	status clear value
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_TIMER_SYNCConfig(TIMER_Type* TIMERa, TIMER_Type* TIMERb, uint32_t CMD, uint32_t DLYVAL)
{
	uint32_t		TIMERb_num;
	uint32_t		TIMERa_num;

	
	/* Check TIMER  handle */
	if(TIMERa == NULL || TIMERb == NULL )
	{
		return HAL_ERROR;
	}
		 
	if (TIMERb == TIMER0)
	{
		TIMERb_num = (1<<20);
	}
	else if (TIMERb == TIMER1)
	{
		TIMERb_num = (1<<21);
	}
	else if (TIMERb == TIMER2)
	{
		TIMERb_num = (1<<22);
	}
	else if (TIMERb == TIMER3)
	{
		TIMERb_num = (1<<23);
	}
	else if (TIMERb == TIMER4)
	{
		TIMERb_num = (1<<24);
	}
	else if (TIMERb == TIMER5)
	{
		TIMERb_num = (1<<25);
	}
	else if (TIMERb == TIMER6)
	{
		TIMERb_num = (1<<26);
	}
	else if (TIMERb == TIMER7)
	{
		TIMERb_num = (1<<27);
	}
	else if (TIMERb == TIMER8)
	{
		TIMERb_num = (1<<28);
	}
	else if (TIMERb == TIMER9)
	{
		TIMERb_num = (1<<29);
	}
	
	if (TIMERa == TIMER0)
	{
		TIMERa_num = 0;
	}
	else if (TIMERa == TIMER1)
	{
		TIMERa_num = 1;
	}
	else if (TIMERa == TIMER2)
	{
		TIMERa_num = 2;
	}
	else if (TIMERa == TIMER3)
	{
		TIMERa_num = 3;
	}
	else if (TIMERa == TIMER4)
	{
		TIMERa_num = 4;
	}
	else if (TIMERa == TIMER5)
	{
		TIMERa_num = 5;
	}
	else if (TIMERa == TIMER6)
	{
		TIMERa_num = 6;
	}
	else if (TIMERa == TIMER7)
	{
		TIMERa_num = 7;
	}
	else if (TIMERa == TIMER8)
	{
		TIMERa_num = 8;
	}
	else if (TIMERa == TIMER9)
	{
		TIMERa_num = 9;
	}
	
	if (TIMERa_num < 5)
	{
		TIMERa->SYNC = 0
		|(TIMERb_num&(0x01F00000))
		|(CMD&(0x00030000))
		|(DLYVAL&(0xFFFF))
		;
		
		TIMERb->SYNC = 0
		|(CMD&(0x00030000))
		;
	}
	else
	{
		TIMERa->SYNC = 0
		|(TIMERb_num&(0x3E000000))
		|(CMD&(0x00030000))
		|(DLYVAL&(0xFFFF))
		;
		
		TIMERb->SYNC = 0
		|(CMD&(0x00030000))
		;
	}
	return HAL_OK;
}




/* --------------------------------- End Of File ------------------------------ */
