/**********************************************************************
* @file		A34M41x_mpwm.c
* @brief	Contains all functions support for MPWM firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application 3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_mpwm.h"
#include "A34M41x_hal_libcfg.h"



/* Public Functions ----------------------------------------------------------- */
/* MPWM Init/DeInit functions -------------------------------------------------*/
/********************************************************************
 * @brief		Initializes the MPWMx peripheral according to the specified
 *               parameters.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_Init(MPWM_Type *MPWMx)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	SYST_ACCESS_EN();
	
	if(MPWMx == MPWM0)
	{
		SCU->PER2&=~(1UL<<16);
		SCU->PCER2&=~(1UL<<16);
		
		/* Set up peripheral clock for MPWM0 module */
		SCU->PER2|=(1UL<<16);
		SCU->PCER2|=(1UL<<16);
	}
	else if(MPWMx == MPWM1)
	{
		SCU->PER2&=~(1UL<<17);
		SCU->PCER2&=~(1UL<<17);
		
		/* Set up peripheral clock for MPWM1 module */
		SCU->PER2|=(1UL<<17);
		SCU->PCER2|=(1UL<<17);
	}
	
	SYST_ACCESS_DIS();
	
// PWMEN bit = '1'
	MPWMx->CR1 = 0
		|(0<<8)                    // IRQN[2:0]
		|(1<<0)                    // PWMEN
		;

	MPWMx->CR2 = 0;         				// PSTART disable 
	MPWMx->OLR = 0;         				// MPWM Output Level 
	MPWMx->FOLR = 0;       				// MPWM Force Output Level
	MPWMx->PCR = 0;        				// Protection 0 Control 
	MPWMx->PSR = (0xca<<8) | 0;        	// Protection 0 Status  
	MPWMx->OCR = 0;        				// Protection 1 Control 
	MPWMx->OSR = (0xac<<8) | 0;        	// Protection 1 Status  
	MPWMx->ATR1 = 0;
	MPWMx->ATR2 = 0;
	MPWMx->ATR3 = 0;
	MPWMx->ATR4 = 0;
	MPWMx->ATR5 = 0;
	MPWMx->ATR6 = 0;	
	return HAL_OK;

}


/*********************************************************************
 * @brief		De-initializes the MPWMx peripheral registers to their
 *              default reset values.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_MPWM_DeInit(MPWM_Type* MPWMx)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->CR1 = 0; // disable PWMEN
	
	SYST_ACCESS_EN();
	
	if (MPWMx == MPWM0)
	{
		SCU->PER2&=~(1UL<<16);
		SCU->PCER2&=~(1UL<<16);
	}
	else if(MPWMx == MPWM1)
	{
		SCU->PER2&=~(1UL<<17);
		SCU->PCER2&=~(1UL<<17);
	}
	
	SYST_ACCESS_DIS();
	return HAL_OK;
}

/*********************************************************************
 * @brief		Enable or Disable PWM start
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	NewState New State of MPWMx peripheral's operation, should be:
 * 					- ENABLE
 * 					- DISABLE
 * @return HAL Status
 **********************************************************************/
HAL_Status_Type HAL_MPWM_Start(MPWM_Type* MPWMx, FunctionalState NewState)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	if (NewState == ENABLE) {
		MPWMx->CR2 = (1UL<<0);
	}
	else 	{
		MPWMx->CR2 = (0UL<<0);
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		Set PWM Output Control data
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	Data	Data to control
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_MPWM_SetOutput(MPWM_Type* MPWMx, uint32_t Data)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->OLR = (Data&0x3f3f);
	return HAL_OK;
}

/**********************************************************************
 * @brief		Set PWM Force Output Control data
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	Data	Data to control
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_MPWM_SetFOutput(MPWM_Type* MPWMx, uint32_t Data)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->FOLR =(Data&0x3f);
	return HAL_OK;
}

/**********************************************************************
 * @brief		Set PWM Protection Control data
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	Data	Data to control
 * @param[in]	ch	channel
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_MPWM_SetProtCtrl(MPWM_Type* MPWMx, uint32_t Data, uint32_t ch)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	if (ch==0){
		MPWMx->PCR = (Data&0xC7C7C7BF);        // Protection 0 Control 		
	}
	else if (ch==1){
		MPWMx->OCR = (Data&0xC7C7C7BF);        // Protection 1 Control 		
	}
	return HAL_OK;
}

/**********************************************************************
 * @brief		Set PWM Protection Status data
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	Data	Data to control
 * @param[in]	ch	channel
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_MPWM_SetProtStat(MPWM_Type* MPWMx, uint32_t Data, uint32_t ch)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	if (ch==0){
		MPWMx->PSR = (0xca<<8) | (Data&0xff);        // Protection 0 Status  		
	}
	else if (ch==1){	
		MPWMx->OSR = (0xac<<8) | (Data&0xff);        // Protection 1 Status  			
	}
	return HAL_OK;
}


/**********************************************************************
 * @brief		Get PWM Counter data
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0/1 peripheral
 * @param[in]	None
 * @return 		Counter Value
 **********************************************************************/
uint32_t HAL_MPWM_GetCounter(MPWM_Type *MPWMx)
{
	return (MPWMx->CNT);
}


/********************************************************************
 * @brief 		Enable or disable specified MPWM interrupt.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	MPIntCfg	Specifies the interrupt src,
 * @param[in]	NewState New state of specified MPWM interrupt type,
 * 				should be:
 * 					- ENALBE	:Enable this MPWM interrupt type.
 * 					- DISALBE	:Disable this MPWM interrupt type.
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_IntConfig(MPWM_Type* MPWMx, uint32_t MPIntCfg, FunctionalState NewState)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	if (NewState == ENABLE)
	{
		MPWMx->IER |= MPIntCfg;
	}
	else
	{
		MPWMx->IER &= (~MPIntCfg) & 0xfff;
	}
	return HAL_OK;
}

/********************************************************************
 * @brief 		Configure PWM Mode setting.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	update			0:duty and period after match, 1:after 2 PWM clocks
 * @param[in]	tup				0:every period match, 1:every period 	
 * @param[in]	bup				0:every period match, 1:e bottom match	
 * @param[in]	mchmode	0:2-ch symmetric, 1:1-ch asymmetric, 2:1-ch symmetric
 * @param[in]	updown		0: PWM UP count, 1:PWM UP/DOWN	 									
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_Cmd(MPWM_Type* MPWMx,uint32_t motorb, uint32_t update, uint32_t tup, uint32_t bup, uint32_t mchmode, uint32_t updown)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->MR = 0
	|((motorb&0x03)<<14)				// MOTORB    0:motor mode, 1:normal mode, 3:Extend mode	
	|((update&0x01)<<7)					// UPDATE    0:duty and period after match, 1:after 2 PWM clocks
	|((tup&0x01)<<5)					// TUP        0:every period match, 1:every period match		
	|((bup&0x01)<<4)					// BUP        0:every period match, 1:every bottom match	
	|((mchmode&0x03)<<1)				// MCHMOD  0:2-ch symmetric, 1:1-ch asymmetric, 2:1-ch symmetric
	|((updown)<<0)						// UPDOWN  0: PWM UP count, 1:PWM UP/DOWN 
	;
	return HAL_OK;
}


/********************************************************************
 * @brief 		Set period data .
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	period			MPWM period data									
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_SetPeriod(MPWM_Type* MPWMx, uint32_t period)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->PRD = (period&0xffff);  // period. it sould be larger than 0x10		
	return HAL_OK;
}

/********************************************************************
 * @brief 		Set duty U high/low data .
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	dutyH			MPWM UdutyH data
 * @param[in]	dutyL			MPWM UdutyL data			 							
 * @return 		None
 *********************************************************************/
HAL_Status_Type HAL_MPWM_SetUDuty(MPWM_Type* MPWMx, uint32_t udutyH, uint32_t udutyL)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->DUH = ((udutyH) &0xffff);  // if using I/O control function, set period data	
	MPWMx->DUL = ((udutyL) &0xffff); 	
	return HAL_OK;
}

/********************************************************************
 * @brief 		Set duty V high/low data .
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	vdutyH			MPWM VdutyH data
 * @param[in]	vdutyL			MPWM VdutyL data			 							
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_SetVDuty(MPWM_Type* MPWMx, uint32_t vdutyH, uint32_t vdutyL)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->DVH = ((vdutyH) &0xffff);  // if using I/O control function, set period data	
	MPWMx->DVL = ((vdutyL) &0xffff); 	
	return HAL_OK;
}

/********************************************************************
 * @brief 		Set duty W high/low data .
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	wdutyH			MPWM WdutyH data
 * @param[in]	wdutyL			MPWM WdutyL data			 							
 * @return 		None
 *********************************************************************/
HAL_Status_Type HAL_MPWM_SetWDuty(MPWM_Type* MPWMx, uint32_t wdutyH, uint32_t wdutyL)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->DWH = ((wdutyH) &0xffff);  // if using I/O control function, set period data	
	MPWMx->DWL = ((wdutyL) &0xffff); 
	return HAL_OK;
}

/********************************************************************
 * @brief 		Set dead time
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	dten			dead time enable 
 * @param[in]	pshrt		Protect short condition 
 * @param[in]	clk			0:PWM CLK /4, 1:PWM CLK /16 
 * @param[in]	clkdata		0x01~0xff dead time value									
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_SetDeadTime(MPWM_Type* MPWMx, uint32_t dten, uint32_t pshrt, uint32_t clk, uint32_t clkdata, uint32_t dirsel)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

 	MPWMx->DTR = 0
	|((dten&0x01)<<15)                 	// DTEN
	|((pshrt&0x01)<<14)                	// PSHRT	
	|((dirsel&0x01)<<13)				// DTMDSEL
	|((clk&0x03)<<8)                   	// DTCLK
	|((clkdata&0xff)<<0)               	// DT
	;
	return HAL_OK;
}

/**********************************************************************
 * @brief		Set PWM ADC Trigger Counter
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @param[in]	atudt	Trigger register update mode
 * @param[in]	atmod	ADC trigger Mode register
 * @param[in]	atcnt	ADC Trigger counter
 * @param[in]	ch	channel
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_MPWM_SetATR(MPWM_Type* MPWMx, uint32_t atudt, uint32_t atmod, uint32_t atcnt, uint32_t ch, uint32_t trgsrc)
{
	uint32_t tempreg;
	
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }
	
	tempreg = 0
	|((trgsrc&0x03)<<22)
	|((atudt&0x01)<<19)
	|((atmod&0x03)<<16)
	|((atmod&0xFFFF)<<0)	
	;
	
	if (ch==1){
		MPWMx->ATR1 = tempreg; 
	}
	else if (ch==2){	
		MPWMx->ATR2 = tempreg; 
	}
	else if (ch==3){	
		MPWMx->ATR3 = tempreg; 
	} 
	else if (ch==4){	
		MPWMx->ATR4 = tempreg; 
	} 
	else if (ch==5){	
		MPWMx->ATR5 = tempreg; 
	} 
	else if (ch==6){	
		MPWMx->ATR6 = tempreg; 
	} 
	return HAL_OK;
}




//==================================================================
//
//		Extend Mode Register
//
//==================================================================

/********************************************************************
 * @brief 		Phase-X Counter Start/Stop.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPWMx : MPWM0, MPWM1
 * @param[in]	Phase-X			U(0), V(1), W(2)
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_ExtStartCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_Ext_OPERATEVAL OperSel)
{
	uint32_t		reg_val, reg_val2;

	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }
	reg_val = MPWMx->CR3;
	reg_val2 = MPWMx->CR4;
	
	if (pha_val == PH_U)
	{
		if (OperSel == MPWM_Ext_STOP)
		{
			reg_val2 |= (1<<1);		// Stop Bit Set
			reg_val &= ~(1<<1);		// START Bit Clear
		}
		else if (OperSel == MPWM_Ext_START)
		{
			reg_val |= (1<<1);
		}
	}
	else if (pha_val == PH_V)
	{
		if (OperSel == MPWM_Ext_STOP)
		{
			reg_val2 |= (1<<9);
			reg_val &= ~(1<<9);
		}
		else if (OperSel == MPWM_Ext_START)
		{
			reg_val |= (1<<9);
		}
	}
	else if (pha_val == PH_W)
	{
		if (OperSel == MPWM_Ext_STOP)
		{
			reg_val2 |= (1<<17);
			reg_val &= ~(1<<17);
		}
		else if (OperSel == MPWM_Ext_START)
		{
			reg_val |= (1<<17);
		}
	}
	
	MPWMx->CR3 = reg_val;
	MPWMx->CR4 = reg_val2;
	return HAL_OK;
}


/********************************************************************
 * @brief 		Phase-X Module Enable & IRQ Period Setting.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPWMx : MPWM0, MPWM1
 * @param[in]	Phase-X			U(0), V(1), W(2)
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_ExtEnableCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, uint32_t irq_n, FunctionalState NewStatus)
{
	uint32_t		reg_val, reg_val2;

	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	reg_val = MPWMx->CR3;
	reg_val2 = MPWMx->CR4;
	
	if (pha_val == PH_U)
	{
		reg_val &= ~((1<<0) | (7<<4));
		reg_val2 &= ~(1<<0);
		
		if (NewStatus == ENABLE)
		{
			reg_val |= ((1<<0) | ((irq_n & 0x07)<<4));
		}
		else if (NewStatus == DISABLE)
		{
			reg_val2 |= (1<<0);
		}
	}
	else if (pha_val == PH_V)
	{
		reg_val &= ~((1<<8) | (7<<12));
		reg_val2 &= ~(1<<8);
		
		if (NewStatus == ENABLE)
		{
			reg_val |= ((1<<8) | ((irq_n & 0x07)<<12));
		}
		else if (NewStatus == DISABLE)
		{
			reg_val2 |= (1<<8);
		}
	}
	else if (pha_val == PH_W)
	{
		reg_val &= ~((1<<16) | (7<<20));
		reg_val2 &= ~(1<<16);
		
		if (NewStatus == ENABLE)
		{
            		reg_val |= ((1<<0) | (1<<16) | ((irq_n & 0x07)<<20)); 			
		}
		else if (NewStatus == DISABLE)
		{
			reg_val2 |= (1<<16);
		}
	}
	
	MPWMx->CR3 = reg_val;
	MPWMx->CR4 = reg_val2;
	return HAL_OK;
}



/********************************************************************
 * @brief 		Phase-X Halt & Continuous Setting.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPWMx : MPWM0, MPWM1
 * @param[in]	Phase-X			U(0), V(1), W(2)
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_ExtPauseCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_Ext_PAUSE PauSel)
{
	uint32_t		reg_val, reg_val2;

	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	
	reg_val = MPWMx->CR3;
	reg_val2 = MPWMx->CR4;
	
	if (pha_val == PH_U)
	{
		reg_val &= ~(1<<3);
		reg_val2 &= ~(1<<3);
		
		if (PauSel == MPWM_Ext_CONT)
		{
			reg_val2 |= (1<<3);
		}
		else if (PauSel == MPWM_Ext_HALT)
		{
			reg_val |= (1<<3);
		}
	}
	else if (pha_val == PH_V)
	{
		reg_val &= ~(1<<11);
		reg_val2 &= ~(1<<11);
		
		if (PauSel == MPWM_Ext_CONT)
		{
			reg_val2 |= (1<<11);
		}
		else if (PauSel == MPWM_Ext_HALT)
		{
			reg_val |= (1<<11);
		}
	}
	else if (pha_val == PH_W)
	{
		reg_val &= ~(1<<19);
		reg_val2 &= ~(1<<19);
		
		if (PauSel == MPWM_Ext_CONT)
		{
			reg_val2 |= (1<<19);
		}
		else if (PauSel == MPWM_Ext_HALT)
		{
			reg_val |= (1<<19);
		}
	}
	
	MPWMx->CR3 = reg_val;
	MPWMx->CR4 = reg_val2;
	return HAL_OK;
}


/********************************************************************
 * @brief 		Phase-X Priod Setting.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPWMx : MPWM0, MPWM1
 * @param[in]	Phase-X			U(0), V(1), W(2)
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_ExtSetPeriod(MPWM_Type* MPWMx, MPWM_PH_VAL pha_val, uint32_t period)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	if (pha_val == PH_U)
	{
		MPWMx->PRDU = (period & 0xFFFF);
	}
	else if (pha_val == PH_V)
	{
		MPWMx->PRDV = (period & 0xFFFF);
	}
	else if (pha_val == PH_W)
	{
		MPWMx->PRDW = (period & 0xFFFF);
	}
	return HAL_OK;
}


/********************************************************************
 * @brief 		Phase-X Get Counter.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPWMx : MPWM0, MPWM1
 * @param[in]	Phase-X			U(0), V(1), W(2)
 * @return 		Counter Value
 *********************************************************************/
uint32_t HAL_MPWM_ExtGetCounter(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	if (pha_val == PH_U)
	{
		return (MPWMx->CNTU);
	}
	else if (pha_val == PH_V)
	{
		return (MPWMx->CNTV);
	}
	else if (pha_val == PH_W)
	{
		return (MPWMx->CNTW);
	}
	
	return 0;
}


/********************************************************************
 * @brief 		Set dead time
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0/1 peripheral
 * @param[in]	dten			dead time enable 
 * @param[in]	pshrt		Protect short condition 
 * @param[in]	clk			0:PWM CLK /4, 1:PWM CLK /16 
 * @param[in]	clkdata		0x01~0xff dead time value									
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_ExtSetDeadTime(MPWM_Type* MPWMx, MPWM_PH_VAL pha_val, uint32_t dten, uint32_t pshrt, uint32_t clk, uint32_t rclkdata, uint32_t fclkdata, uint32_t dirsel)
{
	uint32_t		reg_val;
	
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	
	reg_val = 0
	|((dirsel&0x01)<<31)			// DTMDSEL
	|((dten&0x01)<<23)				// DTEN
	|((pshrt&0x01)<<22)				// PSHRT
	|((clk&0x03)<<16)				// DTCLK
	|((rclkdata&0xff)<<8)			// RDT
	|((fclkdata&0xff)<<0)			// FDT
	;
	
	if (pha_val == PH_U)
	{
		MPWMx->DTRU = reg_val;
	}
	else if (pha_val == PH_V)
	{
		MPWMx->DTRV = reg_val;
	}
	else if (pha_val == PH_W)
	{
		MPWMx->DTRW = reg_val;
	}
	return HAL_OK;
	
}


/********************************************************************
 * @brief 		Capture Counter Enable
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0/1 peripheral
 * @param[in]	MPWM Phase Value
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_ExtCaptureCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_SCAP_EDGE ScapEdge, FunctionalState NewStatus)
{
	uint32_t		reg_val, reg_val2;
	
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	
	if (pha_val == PH_U)
	{
		reg_val = MPWMx->CAPCNTU;
		reg_val2 = MPWMx->SCAPU;
		
		reg_val &= ~(1<<27);
		reg_val2 &= ~(1<<28);
		
		if (NewStatus == ENABLE)
		{
			reg_val |= (1<<27);
			
			if (ScapEdge == SUBCAP_RISING)
			{
				reg_val2 |= (0<<28);
			}
			else if (ScapEdge == SUBCAP_FALLING)
			{
				reg_val2 |= (1<<28);
			}
		}
		
		MPWMx->CAPCNTU = reg_val;
		MPWMx->SCAPU = reg_val2;
	}
	else if (pha_val == PH_V)
	{
		reg_val = MPWMx->CAPCNTV;
		reg_val2 = MPWMx->SCAPV;
		
		reg_val &= ~(1<<27);
		reg_val2 &= ~(1<<28);
		
		if (NewStatus == ENABLE)
		{
			reg_val |= (1<<27);
			
			if (ScapEdge == SUBCAP_RISING)
			{
				reg_val2 |= (0<<28);
			}
			else if (ScapEdge == SUBCAP_FALLING)
			{
				reg_val2 |= (1<<28);
			}
		}
		
		MPWMx->CAPCNTV = reg_val;
		MPWMx->SCAPV = reg_val2;
	}
	else if (pha_val == PH_W)
	{
		reg_val = MPWMx->CAPCNTW;
		reg_val2 = MPWMx->SCAPW;
		
		reg_val &= ~(1<<27);
		reg_val2 &= ~(1<<28);
		
		if (NewStatus == ENABLE)
		{
			reg_val |= (1<<27);
			
			if (ScapEdge == SUBCAP_RISING)
			{
				reg_val2 |= (0<<28);
			}
			else if (ScapEdge == SUBCAP_FALLING)
			{
				reg_val2 |= (1<<28);
			}
		}
		
		MPWMx->CAPCNTW = reg_val;
		MPWMx->SCAPW = reg_val2;
	}
	return HAL_OK;
}


/********************************************************************
 * @brief 		Capture Counter Clear
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0/1 peripheral
 * @param[in]	MPWM Phase Value
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_ExtClearCaptureCmd(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_CAP_MODE CapMode)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	
	if (pha_val == PH_U)
	{
		if (CapMode == MPWM_MAIN_CAP_CNT)
		{
			MPWMx->CAPCNTU |= (1UL<<31);
		}
		else if (CapMode == MPWM_RISING_CAP_VAL)
		{
			MPWMx->RCAPU |= (1UL<<31);
		}
		else if (CapMode == MPWM_FALLING_CAP_VAL)
		{
			MPWMx->FCAPU |= (1UL<<31);
		}
		else if (CapMode == MPWM_SUB_CAP_VAL)
		{
			MPWMx->SCAPU |= (1UL<<31);
		}
	}
	else if (pha_val == PH_V)
	{
		if (CapMode == MPWM_MAIN_CAP_CNT)
		{
			MPWMx->CAPCNTV |= (1UL<<31);
		}
		else if (CapMode == MPWM_RISING_CAP_VAL)
		{
			MPWMx->RCAPV |= (1UL<<31);
		}
		else if (CapMode == MPWM_FALLING_CAP_VAL)
		{
			MPWMx->FCAPV |= (1UL<<31);
		}
		else if (CapMode == MPWM_SUB_CAP_VAL)
		{
			MPWMx->SCAPV |= (1UL<<31);
		}
	}
	else if (pha_val == PH_W)
	{
		if (CapMode == MPWM_MAIN_CAP_CNT)
		{
			MPWMx->CAPCNTW |= (1UL<<31);
		}
		else if (CapMode == MPWM_RISING_CAP_VAL)
		{
			MPWMx->RCAPW |= (1UL<<31);
		}
		else if (CapMode == MPWM_FALLING_CAP_VAL)
		{
			MPWMx->FCAPW |= (1UL<<31);
		}
		else if (CapMode == MPWM_SUB_CAP_VAL)
		{
			MPWMx->SCAPW |= (1UL<<31);
		}
	}
	return HAL_OK;
}		

/********************************************************************
 * @brief 		Get Capture Counter Value
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0/1 peripheral
 * @param[in]	MPWM Phase Value
 * @return 		Capture Counter Value
 *********************************************************************/
uint32_t HAL_MPWM_ExtGetCaptureCNT(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	if (pha_val == PH_U)
	{
		return (MPWMx->CAPCNTU & 0x1FFFF);
	}
	else if (pha_val == PH_V)
	{
		return (MPWMx->CAPCNTV & 0x1FFFF);
	}
	else if (pha_val == PH_W)
	{
		return (MPWMx->CAPCNTW & 0x1FFFF);
	}
	
	return 0;
}


/********************************************************************
 * @brief 		Get Capture Value
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0/1 peripheral
 * @param[in]	MPWM Phase Value, Capture Value Selection
 * @return 		Capture Value
 *********************************************************************/
uint32_t HAL_MPWM_ExtGetCaptureVal(MPWM_Type *MPWMx, MPWM_PH_VAL pha_val, MPWM_CAP_SEL CapSel)
{

	if (pha_val == PH_U)
	{
		if (CapSel == RISING_CAP_VAL)
		{
			return (MPWMx->RCAPU & 0x1FFFF);
		}
		else if (CapSel == FALLING_CAP_VAL)
		{
			return (MPWMx->FCAPU & 0x1FFFF);
		}
		else if (CapSel == SUB_CAP_VAL)
		{
			return (MPWMx->SCAPU & 0x1FFFF);
		}
	}
	else if (pha_val == PH_V)
	{
		if (CapSel == RISING_CAP_VAL)
		{
			return (MPWMx->RCAPV & 0x1FFFF);
		}
		else if (CapSel == FALLING_CAP_VAL)
		{
			return (MPWMx->FCAPV & 0x1FFFF);
		}
		else if (CapSel == SUB_CAP_VAL)
		{
			return (MPWMx->SCAPV & 0x1FFFF);
		}
	}
	else if (pha_val == PH_W)
	{
		if (CapSel == RISING_CAP_VAL)
		{
			return (MPWMx->RCAPW & 0x1FFFF);
		}
		else if (CapSel == FALLING_CAP_VAL)
		{
			return (MPWMx->FCAPW & 0x1FFFF);
		}
		else if (CapSel == SUB_CAP_VAL)
		{
			return (MPWMx->SCAPW & 0x1FFFF);
		}
	}
	
	return 0;
}


/********************************************************************
 * @brief		Initializes the MPWMx peripheral in Extend Mode
 *               parameters.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0/1 peripheral
 * @return 		HAL Status
 *********************************************************************/
HAL_Status_Type HAL_MPWM_ExtInit(MPWM_Type *MPWMx)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	SYST_ACCESS_EN();
	
	if(MPWMx == MPWM0)
	{
		SCU->PER2&=~(1UL<<16);
		SCU->PCER2&=~(1UL<<16);
		
		/* Set up peripheral clock for MPWM0 module */
		SCU->PER2|=(1UL<<16);
		SCU->PCER2|=(1UL<<16);
	}
	else if(MPWMx == MPWM1)
	{
		SCU->PER2&=~(1UL<<17);
		SCU->PCER2&=~(1UL<<17);
		
		/* Set up peripheral clock for MPWM1 module */
		SCU->PER2|=(1UL<<17);
		SCU->PCER2|=(1UL<<17);
	}
	
	SYST_ACCESS_DIS();
	
	// Extend Mode
	MPWMx->MR &= ~(0x03<<14);
	MPWMx->MR |= (0x03<<14);
	
	// PWMEN bit = '1'
	MPWMx->CR3 = 0
	|(0UL<<20)						// Ph-W IRQN[2:0]
	|(1UL<<16)						// Ph-W PWMEN
	|(0UL<<12)						// Ph-V IRQN[2:0]
	|(1UL<<8)							// Ph-V PWMEN
	|(0UL<<4)							// Ph-U IRQN[2:0]
	|(1UL<<0)							// Ph-U PWMEN
	;
	
	MPWMx->CR4 = 0;					// Ph-U, V, W PSTART Disable
	MPWMx->OLR = 0;         			// MPWM Output Level 
	MPWMx->FOLR = 0;       			// MPWM Force Output Level
	MPWMx->PCR = 0;        			// Protection 0 Control 
	MPWMx->PSR = (0xca<<8) | 0;      // Protection 0 Status  
	MPWMx->OCR = 0;        			// Protection 1 Control 
	MPWMx->OSR = (0xac<<8) | 0;      // Protection 1 Status  
	MPWMx->ATR1 = 0;
	MPWMx->ATR2 = 0;
	MPWMx->ATR3 = 0;
	MPWMx->ATR4 = 0;
	MPWMx->ATR5 = 0;
	MPWMx->ATR6 = 0;
	return HAL_OK;

}


/*********************************************************************
 * @brief		De-initializes the MPWMx peripheral registers to their
 *              default reset values.
 * @param[in]	MPWMx	MPWM peripheral selected, should be:
 * 					- MPx	:MPWM0 peripheral
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_MPWM_ExtDeInit(MPWM_Type* MPWMx)
{
	/* Check MPWM  handle */
         if(MPWMx == NULL)
        {
            return HAL_ERROR;
         }

	MPWMx->CR4 = 0
	| (1<<16)
	| (1<<8)
	| (1<<0)
	; 				// disable PWMEN
	
	SYST_ACCESS_EN();
	
	if (MPWMx == MPWM0)
	{
		SCU->PER2&=~(1UL<<16);
		SCU->PCER2&=~(1UL<<16);
	}
	else if(MPWMx == MPWM1)
	{
		SCU->PER2&=~(1UL<<17);
		SCU->PCER2&=~(1UL<<17);
	}
	
	SYST_ACCESS_DIS();
	return HAL_OK;
}



/* --------------------------------- End Of File ------------------------------ */

