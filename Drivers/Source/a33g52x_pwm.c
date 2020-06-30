/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_pwm.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
*
* @ Description 
*   ABOV Semiconductor is supplying this software for use with A33G52x
*   processor. This software contains the confidential and proprietary information
*   of ABOV Semiconductor Co., Ltd ("Confidential Information").
*
*
**************************************************************************************
* DISCLAIMER 
*
* 	THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS  
* 	WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE  
* 	TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* 	DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING  
* 	FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE  
* 	CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.  
*
**************************************************************************************
*/

#include <stdint.h>
#include "A33G52x.h"
#include "a33g52x_pwm.h"
#include "a33g52x_nvic.h"
#include "a33g52x_pcu.h"



//================================================================================
// PWM PORT (no differentiation between master and slave)
//
//			
//				first index			pwm_no 
//				second index			PWMn, PWMBn
//				third index			port base address, pin no, mux data
//
//================================================================================
uint32_t		g_PWM_PORT[8][2][3] = {

	// PWM #0
	{
		{PCD_BASE, PIN_0, PD0_MUX_PWMA0}, 
		{PCE_BASE, PIN_0, PE0_MUX_PWMB0}
	}, 

	// PWM #1
	{
		{PCD_BASE, PIN_1, PD1_MUX_PWMA1}, 
		{PCE_BASE, PIN_1, PE1_MUX_PWMB1}
	}, 

	// PWM #2
	{
		{PCD_BASE, PIN_2, PD2_MUX_PWMA2}, 
		{PCE_BASE, PIN_2, PE2_MUX_PWMB2}
	}, 

	// PWM #3
	{
		{PCD_BASE, PIN_3, PD3_MUX_PWMA3}, 
		{PCE_BASE, PIN_3, PE3_MUX_PWMB3}
	}, 	


	// PWM #4
	{
		{PCD_BASE, PIN_4, PD4_MUX_PWMA4}, 
		{PCE_BASE, PIN_4, PE4_MUX_PWMB4}
	}, 

	// PWM #5
	{
		{PCD_BASE, PIN_5, PD5_MUX_PWMA5}, 
		{PCE_BASE, PIN_5, PE5_MUX_PWMB5}
	}, 

	// PWM #6
	{
		{PCD_BASE, PIN_6, PD6_MUX_PWMA6}, 
		{PCE_BASE, PIN_6, PE6_MUX_PWMB6}
	}, 

	// PWM #7
	{
		{PCD_BASE, PIN_7, PD7_MUX_PWMA7}, 
		{PCE_BASE, PIN_7, PE7_MUX_PWMB7}
	}	

}; 


/**
*********************************************************************************************************
* @ Name : PWM_ConfigureGPIO 
*
* @ Parameters
*		- pwm : PWM0~PWM7
*
*
*********************************************************************************************************
*/
void PWM_ConfigureGPIO (PWM_Type * const pwm)
{

	int 		pwm_no; 


	PCU_Type	*pwm_a_port_addr, *pwm_b_port_addr; 
	uint32_t		pwm_a_pin_no, pwm_b_pin_no; 
	uint32_t		pwm_a_mux_info, pwm_b_mux_info; 


	//--------------------------------------------------------------------------------
	// get pwm_no 
	//--------------------------------------------------------------------------------		
	pwm_no = PWM_Convert_PWM_Into_PWMNo(pwm); 

	
	//--------------------------------------------------------------------------------
	// get info 
	//--------------------------------------------------------------------------------	
	pwm_a_port_addr = (PCU_Type *) g_PWM_PORT[pwm_no][0][0]; 			// PWMA, port base address
	pwm_a_pin_no = g_PWM_PORT[pwm_no][0][1];							// PWMA, pin no
	pwm_a_mux_info = g_PWM_PORT[pwm_no][0][2];							// PWMA, mux info 

	pwm_b_port_addr = (PCU_Type *) g_PWM_PORT[pwm_no][1][0]; 			// PWMB, port base address
	pwm_b_pin_no = g_PWM_PORT[pwm_no][1][1];							// PWMB, pin no
	pwm_b_mux_info = g_PWM_PORT[pwm_no][1][2];							// PWMB, mux info 	


	//--------------------------------------------------------------------------------
	// get info 
	//--------------------------------------------------------------------------------		
	PCU_ConfigureFunction (pwm_a_port_addr, pwm_a_pin_no, pwm_a_mux_info); 
	PCU_Set_Direction_Type(pwm_a_port_addr, pwm_a_pin_no, PnCR_OUTPUT_PUSH_PULL); 
	PCU_ConfigurePullup (pwm_a_port_addr, pwm_a_pin_no, PnPCR_PULLUP_DISABLE); 

	PCU_ConfigureFunction (pwm_b_port_addr, pwm_b_pin_no, pwm_b_mux_info); 
	PCU_Set_Direction_Type(pwm_b_port_addr, pwm_b_pin_no, PnCR_OUTPUT_PUSH_PULL); 
	PCU_ConfigurePullup (pwm_b_port_addr, pwm_b_pin_no, PnPCR_PULLUP_DISABLE); 	

}




/**
*********************************************************************************************************
* @ Name : PWM_Convert_PWM_Into_PWMNo 
*
* @ Parameters
*		- pwm : PWM0~PWM7
*
* @ Return
*		- pwm_no 
*
*********************************************************************************************************
*/
int PWM_Convert_PWM_Into_PWMNo (PWM_Type * const pwm)
{

	int		pwm_no; 

	if (pwm == PWM0) pwm_no = 0; 
	else if (pwm == PWM1) pwm_no = 1; 
	else if (pwm == PWM2) pwm_no = 2; 
	else if (pwm == PWM3) pwm_no = 3; 	
	else if (pwm == PWM4) pwm_no = 4; 
	else if (pwm == PWM5) pwm_no = 5; 
	else if (pwm == PWM6) pwm_no = 6; 
	else if (pwm == PWM7) pwm_no = 7; 		
	else pwm_no = 0; 

	return (pwm_no); 

}




/**
*********************************************************************************************************
* @ Name : PWM_Init 
*
* @ Parameters
*		- pwm : PWM0~PWM7
*		- p_config
*				->sync_mode 	= PWM_SYNC_MODE, PWM_ASYNC_MODE 
*				->invert			= PWM_SIG_INVERT, PWM_SIG_NO_INVERT
*				->clock_select	= PWM_CLKSEL_CLK_DIV_BY_2, PWM_CLKSEL_CLK_DIV_BY_4, 
*								   PWM_CLKSEL_CLK_DIV_BY_8, PWM_CLKSEL_CLK_DIV_BY_16
*				->period			= 0x0000~0xFFFF
*				->compare_val	= 0x0000~0xFFFF
*
*********************************************************************************************************
*/
void PWM_Init (PWM_Type * const pwm, PWM_CONFIG * p_config)
{
	uint32_t			reg_val; 
	

	//------------------------------------------------------------------------------
	// control 
	//
	//				PWM0CTRL		@ address = 0x4000_0700
	//				PWM1CTRL		@ address = 0x4000_0720
	//				PWM2CTRL		@ address = 0x4000_0740
	//				PWM3CTRL		@ address = 0x4000_0760 
	//
	//				PWM4CTRL		@ address = 0x4000_0780
	//				PWM5CTRL		@ address = 0x4000_07A0
	//				PWM6CTRL		@ address = 0x4000_07C0
	//				PWM7CTRL		@ address = 0x4000_07E0 
	//
	//------------------------------------------------------------------------------
	if (p_config->clock_select == PWM_CLKSEL_CLK_DIV_BY_2) reg_val = PWMnCTRL_CKSEL_DIV_BY_2; 
	else if (p_config->clock_select == PWM_CLKSEL_CLK_DIV_BY_4) reg_val = PWMnCTRL_CKSEL_DIV_BY_4; 
	else if (p_config->clock_select == PWM_CLKSEL_CLK_DIV_BY_8) reg_val = PWMnCTRL_CKSEL_DIV_BY_8; 
	else if (p_config->clock_select == PWM_CLKSEL_CLK_DIV_BY_16) reg_val = PWMnCTRL_CKSEL_DIV_BY_16; 

	if (p_config->invert == PWM_SIG_INVERT) reg_val |= PWMnCTRL_INVA; 
	if (p_config->sync_mode == PWM_SYNC_MODE) reg_val |= PWMnCTRL_SYNC; 

	pwm->CTRL = reg_val;



	//------------------------------------------------------------------------------
	// clear count 
	//
	//				PWM0CNT		@ address = 0x4000_0704
	//				PWM1CNT		@ address = 0x4000_0724
	//				PWM2CNT		@ address = 0x4000_0744
	//				PWM3CNT		@ address = 0x4000_0764 
	//
	//				PWM4CNT		@ address = 0x4000_0784
	//				PWM5CNT		@ address = 0x4000_07A4
	//				PWM6CNT		@ address = 0x4000_07C4
	//				PWM7CNT		@ address = 0x4000_07E4 
	//
	//------------------------------------------------------------------------------
	pwm->CNT = 0;



	//------------------------------------------------------------------------------
	// set compare-value
	//
	//				PWM0CMPA		@ address = 0x4000_070C
	//				PWM1CMPA		@ address = 0x4000_072C
	//				PWM2CMPA		@ address = 0x4000_074C
	//				PWM3CMPA		@ address = 0x4000_076C 
	//
	//				PWM4CMPA		@ address = 0x4000_078C
	//				PWM5CMPA		@ address = 0x4000_07AC
	//				PWM6CMPA		@ address = 0x4000_07CC
	//				PWM7CMPA		@ address = 0x4000_07EC 
	//
	//------------------------------------------------------------------------------	
	reg_val = (p_config->compare_val & PWMnCMPA_MASK); 
	pwm->CMP = reg_val;


	//------------------------------------------------------------------------------
	// set compare-value
	//
	//				PWM0PER		@ address = 0x4000_0708
	//				PWM1PER		@ address = 0x4000_0728
	//				PWM2PER		@ address = 0x4000_0748
	//				PWM3PER		@ address = 0x4000_0768 
	//
	//				PWM4PER		@ address = 0x4000_0788
	//				PWM5PER		@ address = 0x4000_07A8
	//				PWM6PER		@ address = 0x4000_07C8
	//				PWM7PER		@ address = 0x4000_07E8 
	//
	//------------------------------------------------------------------------------	
	reg_val = (p_config->period & PWMnPER_MASK); 
	pwm->PER = reg_val;


}



/**
*********************************************************************************************************
* @ Name : PWM_ConfigureInterrupt 
*
* @ Parameters
*		- pwm : PWM0~PWM7
*		- intr_mask : PWMnCTRL_PRIE
*		- enable : INTR_ENABLE, INTR_DISABLE 
*
*
*********************************************************************************************************
*/
void PWM_ConfigureInterrupt (PWM_Type * const pwm, uint32_t intr_mask, uint32_t enable)
{
	uint32_t		reg_val;

	//------------------------------------------------------------------------------
	// disable interrupt 
	//
	//				PWM0CTRL		@ address = 0x4000_0700
	//				PWM1CTRL		@ address = 0x4000_0720
	//				PWM2CTRL		@ address = 0x4000_0740
	//				PWM3CTRL		@ address = 0x4000_0760	
	//
	//				PWM4CTRL		@ address = 0x4000_0780
	//				PWM5CTRL		@ address = 0x4000_07A0
	//				PWM6CTRL		@ address = 0x4000_07C0
	//				PWM7CTRL		@ address = 0x4000_07E0	
	//
	//------------------------------------------------------------------------------
	reg_val = pwm->CTRL;
	reg_val &= ~PWMnCTRL_PRIE;
	pwm->CTRL = reg_val;


	//------------------------------------------------------------------------------
	// clear interrupt flag 
	//
	//				PWM0CTRL		@ address = 0x4000_0700
	//				PWM1CTRL		@ address = 0x4000_0720
	//				PWM2CTRL		@ address = 0x4000_0740
	//				PWM3CTRL		@ address = 0x4000_0760	
	//
	//				PWM4CTRL		@ address = 0x4000_0780
	//				PWM5CTRL		@ address = 0x4000_07A0
	//				PWM6CTRL		@ address = 0x4000_07C0
	//				PWM7CTRL		@ address = 0x4000_07E0	
	//
	//
	//				# PRF (interrupt flag)	write one to clear 
	//
	//------------------------------------------------------------------------------
	reg_val = pwm->CTRL;
	pwm->CTRL = reg_val;


	//------------------------------------------------------------------------------
	// enable interrupt 
	//
	//				PWM0CTRL		@ address = 0x4000_0700
	//				PWM1CTRL		@ address = 0x4000_0720
	//				PWM2CTRL		@ address = 0x4000_0740
	//				PWM3CTRL		@ address = 0x4000_0760	
	//
	//				PWM4CTRL		@ address = 0x4000_0780
	//				PWM5CTRL		@ address = 0x4000_07A0
	//				PWM6CTRL		@ address = 0x4000_07C0
	//				PWM7CTRL		@ address = 0x4000_07E0	
	//
	//------------------------------------------------------------------------------	
	if (enable == INTR_ENABLE)
	{
		reg_val = pwm->CTRL;
		reg_val |= (intr_mask&PWMnCTRL_PRIE);
		pwm->CTRL = reg_val;
	}



}


///////////////////////////////////////////////////////////////////////////////////////////////
/**
*********************************************************************************************************
* @ Name : _PWM_Init
*
* @ Parameter
*		- pwm_no : 0~7
*		- p_PWM_config
*				->sync_mode 	= PWM_SYNC_MODE, PWM_ASYNC_MODE 
*				->invert			= PWM_SIG_INVERT, PWM_SIG_NO_INVERT
*				->clock_select	= PWM_CLKSEL_CLK_DIV_BY_2, PWM_CLKSEL_CLK_DIV_BY_4, 
*								   PWM_CLKSEL_CLK_DIV_BY_8, PWM_CLKSEL_CLK_DIV_BY_16
*				->period			= 0x0000~0xFFFF
*				->compare_val	= 0x0000~0xFFFF

*
*
*********************************************************************************************************
*/
void _PWM_Init (int pwm_no, PWM_CONFIG * p_PWM_config)
{

	PWM_Type 			* pwm; 
	PWMPRS_Type		* pwmprs;  
	NVIC_IntrConfig	 	nvic_config; 



	//--------------------------------------------------------------------------------
	// check pwm_no
	//--------------------------------------------------------------------------------
	if ((pwm_no < 0) || (pwm_no > 7)) return; 



	//--------------------------------------------------------------------------------
	// pwm prescaler 
	//--------------------------------------------------------------------------------
	pwmprs = PWM_Get_Prescaler_Object (pwm_no); 
	pwmprs->PRSn = (PWMPRSn_CLKEN | PWMPRSn_PRESCALERn_VAL(0) );
	

	//--------------------------------------------------------------------------------
	// get object 
	//--------------------------------------------------------------------------------
	pwm = PWM_Get_Object(pwm_no); 


	//--------------------------------------------------------------------------------
	// configure GPIO
	//--------------------------------------------------------------------------------
	PWM_ConfigureGPIO(pwm); 



	//--------------------------------------------------------------------------------
	// mode setting 
	//--------------------------------------------------------------------------------	
	PWM_Init (pwm, p_PWM_config); 



	//------------------------------------------------------------------------------
	// interrupt setting (peripheral)
	//
	//					
	//------------------------------------------------------------------------------	
	PWM_ConfigureInterrupt (pwm, PWMnCTRL_PRIE, INTR_ENABLE); 



	//------------------------------------------------------------------------------
	// interrupt setting (interrupt module)
	//
	//					IRQ_PWM0=24
	//					IRQ_PWM1=25
	//					IRQ_PWM2=26
	//					IRQ_PWM3=27	
	//
	//					IRQ_PWM4=28
	//					IRQ_PWM5=29
	//					IRQ_PWM6=30
	//					IRQ_PWM7=31
	//
	//------------------------------------------------------------------------------		
	nvic_config.nIRQ_Number = (IRQ_PWM0+pwm_no); 
	nvic_config.Preemption_Priority= PRIO_PWM0_PREEMPTION; 
	nvic_config.Subpriority= PRIO_PWM0_SUBPRIORITY; 
	nvic_config.IntrEnable = INTR_ENABLE; 
	NVIC_ConfigureInterrupt (NVIC, &nvic_config); 



}




/**
*********************************************************************************************************
* @ Name : PWM_Get_Object
*
* @ Parameter
*		- pwm_no : 0~7
*
* @ Return 
*		PWM-register-band object 
*
*
*********************************************************************************************************
*/
PWM_Type * PWM_Get_Object (int pwm_no)
{

	PWM_Type	* p_obj; 


	switch (pwm_no)
	{
	case 0:
		p_obj = PWM0; 
		break; 

	case 1:
		p_obj = PWM1; 
		break; 

	case 2:
		p_obj = PWM2; 
		break; 
		
	case 3:
		p_obj = PWM3; 
		break; 
		
	case 4:
		p_obj = PWM4; 
		break; 
		
	case 5:
		p_obj = PWM5; 
		break; 

	case 6:
		p_obj = PWM6; 
		break; 
		
	case 7:
		p_obj = PWM7; 
		break; 
		

	default:
		p_obj = (PWM_Type *) 0; 
		break; 


	}


	return (p_obj); 

}





/**
*********************************************************************************************************
* @ Name : PWM_Get_Prescaler_Object
*
* @ Parameter
*		- pwm_no : 0~7
*
* @ Return 
*		PWM-prescaler object 
*
*
*********************************************************************************************************
*/
PWMPRS_Type * PWM_Get_Prescaler_Object (int pwm_no)
{

	PWMPRS_Type		* pwmprs_obj; 
	

	if ((pwm_no >= 0) && (pwm_no <= 3)) pwmprs_obj = PWMPRS0; 
	else if ((pwm_no >= 4) && (pwm_no <= 7)) pwmprs_obj = PWMPRS1; 
	else pwmprs_obj = PWMPRS0; 


	return (pwmprs_obj); 

}




/**
*********************************************************************************************************
* @ Name : PWM_Start 
*
* @ Parameters
*		- pwm_no : 0~7
*
*
*********************************************************************************************************
*/
void PWM_Start(int pwm_no)
{

	PWM_Type	* pwm; 
	uint32_t			reg_val;


	//--------------------------------------------------------------------------------
	// check pwm_no
	//--------------------------------------------------------------------------------
	if ((pwm_no < 0) || (pwm_no > 7)) return; 


	//--------------------------------------------------------------------------------
	// get object 
	//--------------------------------------------------------------------------------
	pwm = PWM_Get_Object(pwm_no); 


	//--------------------------------------------------------------------------------
	// start 
	//--------------------------------------------------------------------------------
	reg_val = pwm->CTRL;
	reg_val |= PWMnCTRL_STRT;
	pwm->CTRL = reg_val;

	
}







/**
*********************************************************************************************************
* @ Name : PWM_Stop
*
* @ Parameters
*		- pwm_no : 0~7
*
*
*********************************************************************************************************
*/
void PWM_Stop (int pwm_no)
{

	PWM_Type	* pwm;
	uint32_t			reg_val;


	//--------------------------------------------------------------------------------
	// check pwm_no
	//--------------------------------------------------------------------------------
	if ((pwm_no < 0) || (pwm_no > 7)) return; 


	//--------------------------------------------------------------------------------
	// get object 
	//--------------------------------------------------------------------------------
	pwm = PWM_Get_Object(pwm_no); 


	//--------------------------------------------------------------------------------
	// stop
	//--------------------------------------------------------------------------------
	reg_val = pwm->CTRL;
	reg_val &=  ~PWMnCTRL_STRT;
	pwm->CTRL = reg_val;
	

}


