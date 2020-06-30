/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_timer.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
*
* @ Description 
*   ABOV Semiconductor is supplying this software for use with HART-m310
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
#include "a33g52x_timer.h"
#include "a33g52x_pcu.h"
#include "a33g52x_nvic.h"




uint32_t g_TnO_PORT[10][4] = {

	{PCB_BASE, PIN_0, PB0_MUX_T0O, PnCR_OUTPUT_PUSH_PULL}, 		// T0O
	{PCB_BASE, PIN_1, PB1_MUX_T1O, PnCR_OUTPUT_PUSH_PULL}, 		// T1O
	{PCB_BASE, PIN_2, PB2_MUX_T2O, PnCR_OUTPUT_PUSH_PULL}, 		// T2O
	{PCB_BASE, PIN_3, PB3_MUX_T3O, PnCR_OUTPUT_PUSH_PULL}, 		// T3O

	{PCB_BASE, PIN_4, PB4_MUX_T4O, PnCR_OUTPUT_PUSH_PULL}, 		// T4O
	{PCB_BASE, PIN_5, PB5_MUX_T5O, PnCR_OUTPUT_PUSH_PULL}, 		// T5O
	{PCB_BASE, PIN_6, PB6_MUX_T6O, PnCR_OUTPUT_PUSH_PULL}, 		// T6O
	{PCB_BASE, PIN_7, PB7_MUX_T7O, PnCR_OUTPUT_PUSH_PULL}, 		// T7O

	{PCB_BASE, PIN_8, PB8_MUX_T8O, PnCR_OUTPUT_PUSH_PULL}, 		// T8O
	{PCB_BASE, PIN_9, PB9_MUX_T9O, PnCR_OUTPUT_PUSH_PULL}, 		// T9O

}; 


uint32_t	g_TnC_PORT[10][4] = {

	{PCA_BASE, PIN_6, PA6_MUX_T0C, PnCR_INPUT_LOGIC}, 			// T0C
	{PCA_BASE, PIN_7, PA7_MUX_T1C, PnCR_INPUT_LOGIC}, 			// T1C
	{PCA_BASE, PIN_8, PA8_MUX_T2C, PnCR_INPUT_LOGIC}, 			// T2C
	{PCA_BASE, PIN_9, PA9_MUX_T3C, PnCR_INPUT_LOGIC}, 			// T3C

	{PCA_BASE, PIN_10, PA10_MUX_T4C, PnCR_INPUT_LOGIC}, 			// T4C
	{PCA_BASE, PIN_11, PA11_MUX_T5C, PnCR_INPUT_LOGIC}, 			// T5C
	{PCA_BASE, PIN_12, PA12_MUX_T6C, PnCR_INPUT_LOGIC}, 			// T6C
	{PCA_BASE, PIN_13, PA13_MUX_T7C, PnCR_INPUT_LOGIC}, 			// T7C

	{PCA_BASE, PIN_14, PA14_MUX_T8C, PnCR_INPUT_LOGIC}, 			// T8C
	{PCA_BASE, PIN_15, PA15_MUX_T9C, PnCR_INPUT_LOGIC}, 			// T9C	


}; 



/**
*********************************************************************************************************
* @ Name : TIMER_ConfigureGPIO 
*
* @ Parameters
*		- timer : TIMER0~TIMER9
*		- mode : TIMER_MODE_PERIODIC, TIMER_MODE_PWM, TIMER_MODE_ONE_SHOT, TIMER_MODE_CAPTURE
*
*
*********************************************************************************************************
*/
void TIMER_ConfigureGPIO (TIMER_Type * const timer, int mode)
{

	int 			timer_no; 

	PCU_Type		*port_base_addr; 
	uint32_t			pin_no;
	uint32_t			mux_info; 
	uint32_t			dir_type; 


	//--------------------------------------------------------------------------------
	// get timer_no 
	//--------------------------------------------------------------------------------
	timer_no = TIMER_Get_Object_Number(timer); 



	//--------------------------------------------------------------------------------
	// setting info 
	//--------------------------------------------------------------------------------
	if ((mode == TIMER_MODE_PERIODIC) || (mode == TIMER_MODE_PWM) || (mode == TIMER_MODE_ONE_SHOT))
	{

		port_base_addr = (PCU_Type *) g_TnO_PORT[timer_no][0]; 
		pin_no = g_TnO_PORT[timer_no][1]; 
		mux_info = g_TnO_PORT[timer_no][2]; 
		dir_type = g_TnO_PORT[timer_no][3];
		
	}
	else if (mode == TIMER_MODE_CAPTURE)
	{
	
		port_base_addr = (PCU_Type *) g_TnC_PORT[timer_no][0]; 
		pin_no = g_TnC_PORT[timer_no][1]; 
		mux_info = g_TnC_PORT[timer_no][2]; 
		dir_type = g_TnC_PORT[timer_no][3]; 

	}
	

	//--------------------------------------------------------------------------------
	// setting 
	//--------------------------------------------------------------------------------	
	PCU_ConfigureFunction (port_base_addr, pin_no, mux_info); 
	PCU_Set_Direction_Type(port_base_addr, pin_no, dir_type); 
	PCU_ConfigurePullup (port_base_addr, pin_no, PnPCR_PULLUP_DISABLE); 
	

}



/**
*********************************************************************************************************
* @ Name : TIMER_ConfigureExternalSource 
*
* @ Parameters
*		- timer : TIMER0~TIMER9
*
*
*********************************************************************************************************
*/
void TIMER_ConfigureExternalSource (TIMER_Type * const timer)
{

	int 			timer_no; 

	PCU_Type		*port_base_addr; 
	uint32_t			pin_no;
	uint32_t			mux_info; 
	uint32_t			dir_type; 



	//--------------------------------------------------------------------------------
	// get timer_no 
	//--------------------------------------------------------------------------------
	timer_no = TIMER_Get_Object_Number(timer); 



	//--------------------------------------------------------------------------------
	// setting info 
	//--------------------------------------------------------------------------------
	port_base_addr = (PCU_Type *) g_TnC_PORT[timer_no][0]; 
	pin_no = g_TnC_PORT[timer_no][1]; 
	mux_info = g_TnC_PORT[timer_no][2]; 
	dir_type = g_TnC_PORT[timer_no][3]; 
	

	//--------------------------------------------------------------------------------
	// setting 
	//--------------------------------------------------------------------------------	
	PCU_ConfigureFunction (port_base_addr, pin_no, mux_info); 
	PCU_Set_Direction_Type(port_base_addr, pin_no, dir_type); 
	PCU_ConfigurePullup (port_base_addr, pin_no, PnPCR_PULLUP_DISABLE); 
	

}




/**
*********************************************************************************************************
* @ Name : TIMER_Get_Object_Number 
*
* @ Parameters
*		- pwm : TIMER0~TIMER9
*
* @ Return 
*		- timer_no 
*
*
*********************************************************************************************************
*/
int TIMER_Get_Object_Number (TIMER_Type * const timer)
{

	int		timer_no; 

	if (timer == T0) timer_no = 0; 
	else if (timer == T1) timer_no = 1; 
	else if (timer == T2) timer_no = 2; 
	else if (timer == T3) timer_no = 3;
	
	else if (timer == T4) timer_no = 4; 
	else if (timer == T5) timer_no = 5; 
	else if (timer == T6) timer_no = 6; 
	else if (timer == T7) timer_no = 7; 
	
	else if (timer == T8) timer_no = 8; 
	else if (timer == T9) timer_no = 9; 	


	return (timer_no); 

}



/**
*********************************************************************************************************
* @ Name : TIMER_Init
*
* @ Parameters
*		- pwm : TIMER0~TIMER9
*		- mode : TIMER_MODE_PERIODIC, TIMER_MODE_PWM, TIMER_MODE_ONE_SHOT, TIMER_MODE_CAPTURE
*		- p_config
*			# start_level_after_counter_clear = TIMER_START_LEVEL_HIGH, TIMER_START_LEVEL_LOW
*			# clock_select			= TIMER_CLKSEL_TCLK_DIV_BY_2 ~ TIMER_CLKSEL_EXTERNAL_SOURCE
*			# capture_clear_mode	= TIMER_CAPTURE_RISING_EDGE_CLEAR, TIMER_CAPTURE_FALLING_EDGE_CLEAR
*			# GRA				= 16-bit value
*			# GRB				= 16-bit value
*
*********************************************************************************************************
*/
void TIMER_Init (TIMER_Type * const timer, int mode, TIMER_CONFIG * p_config)
{

	uint32_t			reg_val; 
	

	//--------------------------------------------------------------------------------
	// init timer 
	//
	//				@ T0CMD = 0x4000_0C04
	//				@ T1CMD = 0x4000_0C24
	//				@ T2CMD = 0x4000_0C44
	//				@ T3CMD = 0x4000_0C64	
	//
	//				@ T4CMD = 0x4000_0C84
	//				@ T5CMD = 0x4000_0CA4
	//				@ T6CMD = 0x4000_0CC4
	//				@ T7CMD = 0x4000_0CE4	
	//	
	//				@ T8CMD = 0x4000_0D04
	//				@ T9CMD = 0x4000_0D24
	//	
	//
	//--------------------------------------------------------------------------------
	timer->CMD = TnCMD_TCLR;
	timer->CMD = 0;




	//--------------------------------------------------------------------------------
	// mode setting
	//
	//				@ T0CON = 0x4000_0C00
	//				@ T1CON = 0x4000_0C20
	//				@ T2CON = 0x4000_0C40
	//				@ T3CON = 0x4000_0C60	
	//
	//				@ T4CON = 0x4000_0C80
	//				@ T5CON = 0x4000_0CA0
	//				@ T6CON = 0x4000_0CC0
	//				@ T7CON = 0x4000_0CE0	
	//	
	//				@ T8CON = 0x4000_0D00
	//				@ T9CON = 0x4000_0D20	
	//
	//--------------------------------------------------------------------------------
	//
	//				TSTRT, TCS, CAPM, TMODE
	//
	//--------------------------------------------------------------------------------
	reg_val = 0; 


	// TSTRT (start level)

	if (p_config->start_level_after_counter_clear == TIMER_START_LEVEL_HIGH)
		reg_val |= TnCON_TSTRT; 



	// TCS (select clock)
	if (p_config->clock_select == TIMER_CLKSEL_TCLK_DIV_BY_2)
	{
		reg_val |= TnCON_TCS_TCLK_DIV_BY_2; 
	}
	else if (p_config->clock_select == TIMER_CLKSEL_TCLK_DIV_BY_4)
	{
		reg_val |= TnCON_TCS_TCLK_DIV_BY_4; 
	}	
	else if (p_config->clock_select == TIMER_CLKSEL_TCLK_DIV_BY_16)
	{
		reg_val |= TnCON_TCS_TCLK_DIV_BY_16; 
	}	
	else if (p_config->clock_select == TIMER_CLKSEL_TCLK_DIV_BY_64)
	{
		reg_val |= TnCON_TCS_TCLK_DIV_BY_64; 
	}	
	else if (p_config->clock_select == TIMER_CLKSEL_PMUPCSR)
	{
		reg_val |= TnCON_TCS_PMUPCSR; 
	}
	else if (p_config->clock_select == TIMER_CLKSEL_EXTERNAL_SOURCE)
	{
		reg_val |= TnCON_TCS_TnC; 
	}

	
	// TMODE, CAPM (Periodic, PWM, One-shot, Capture (clear mode))
	if (mode == TIMER_MODE_PERIODIC)
	{
		reg_val |= TnCON_TMODE_PERIODIC; 

	}
	else if (mode == TIMER_MODE_PWM)
	{
		reg_val |= TnCON_TMODE_PWM; 

	}
	else if (mode == TIMER_MODE_ONE_SHOT)
	{
		reg_val |= TnCON_TMODE_ONE_SHOT; 
	}
	else if (mode == TIMER_MODE_CAPTURE)
	{
		reg_val |= TnCON_TMODE_CAPTURE; 

		if (p_config->capture_clear_mode == TIMER_CAPTURE_RISING_EDGE_CLEAR) reg_val |= TnCON_CAPM_RISING_EDGE_CLEAR; 
		else if (p_config->capture_clear_mode == TIMER_CAPTURE_FALLING_EDGE_CLEAR) reg_val |= TnCON_CAPM_FALLING_EDGE_CLEAR; 
	}

	timer->CON = reg_val;
	

	
	//--------------------------------------------------------------------------------
	// prescale
	//
	//				@ T0PRS = 0x4000_0C10
	//				@ T1PRS = 0x4000_0C30
	//				@ T2PRS = 0x4000_0C50
	//				@ T3PRS = 0x4000_0C70	
	//
	//				@ T4PRS = 0x4000_0C90
	//				@ T5PRS = 0x4000_0CB0
	//				@ T6PRS = 0x4000_0CD0
	//				@ T7PRS = 0x4000_0CF0		
	//
	//				@ T8PRS = 0x4000_0D10
	//				@ T9PRS = 0x4000_0D30
	//
	//--------------------------------------------------------------------------------
	timer->PRS = p_config->PRS;




	//--------------------------------------------------------------------------------
	// TnGRA
	//
	//				@ T0GRA = 0x4000_0C08
	//				@ T1GRA = 0x4000_0C28
	//				@ T2GRA = 0x4000_0C48
	//				@ T3GRA = 0x4000_0C68
	//
	//				@ T4GRA = 0x4000_0C88
	//				@ T5GRA = 0x4000_0CA8
	//				@ T6GRA = 0x4000_0CC8
	//				@ T7GRA = 0x4000_0CE8
	//
	//				@ T8GRA = 0x4000_0D08
	//				@ T9GRA = 0x4000_0D28	
	//
	//--------------------------------------------------------------------------------
	timer->GRA = p_config->GRA;
	


	//--------------------------------------------------------------------------------
	// TnGRB
	//
	//				@ T0GRB = 0x4000_0C0C
	//				@ T1GRB = 0x4000_0C2C
	//				@ T2GRB = 0x4000_0C4C
	//				@ T3GRB = 0x4000_0C6C
	//
	//				@ T4GRB = 0x4000_0C8C
	//				@ T5GRB = 0x4000_0CAC
	//				@ T6GRB = 0x4000_0CCC
	//				@ T7GRB = 0x4000_0CEC
	//
	//				@ T8GRB = 0x4000_0D0C
	//				@ T9GRB = 0x4000_0D2C	
	//
	//--------------------------------------------------------------------------------
	timer->GRB = p_config->GRB;



	//--------------------------------------------------------------------------------
	// push data into compare-buffer 
	//
	//				@ T0CMD = 0x4000_0C04
	//				@ T1CMD = 0x4000_0C24
	//				@ T2CMD = 0x4000_0C44
	//				@ T3CMD = 0x4000_0C64	
	//
	//				@ T4CMD = 0x4000_0C84
	//				@ T5CMD = 0x4000_0CA4
	//				@ T6CMD = 0x4000_0CC4
	//				@ T7CMD = 0x4000_0CE4	
	//	
	//				@ T8CMD = 0x4000_0D04
	//				@ T9CMD = 0x4000_0D24
	//
	//--------------------------------------------------------------------------------
	timer->CMD = TnCMD_TCLR;


	//--------------------------------------------------------------------------------
	// TnCNT
	//
	//				@ T0CNT = 0x4000_0C14
	//				@ T1CNT = 0x4000_0C34
	//				@ T2CNT = 0x4000_0C54
	//				@ T3CNT = 0x4000_0C74
	//
	//				@ T4CNT = 0x4000_0C94
	//				@ T5CNT = 0x4000_0CB4
	//				@ T6CNT = 0x4000_0CD4
	//				@ T7CNT = 0x4000_0CF4	
	//
	//				@ T8CNT = 0x4000_0D14
	//				@ T9CNT = 0x4000_0D34	
	//
	//--------------------------------------------------------------------------------
	timer->CNT = 0;
	


}



/**
*********************************************************************************************************
* @ Name : TIMER_ConfigureInterrupt
*
* @ Parameters
*		- pwm : TIMER0~TIMER9
*		- intr_mask : TnCON_TOVE, TnCON_TIE1, TnCON_TIE0
*		- enable : INTR_ENABLE, INTR_DISABLE
*
*
*********************************************************************************************************
*/
void TIMER_ConfigureInterrupt (TIMER_Type * const timer, uint32_t intr_mask, uint32_t enable)
{

	uint32_t			reg_val; 

	
	//-----------------------------------------------------------------------------------------
	// disable interrupt
	//
	//				@ T0CON		= 0x4000_0C00
	//				@ T1CON		= 0x4000_0C20
	//				@ T2CON		= 0x4000_0C40
	//				@ T3CON		= 0x4000_0C60
	//
	//				@ T4CON		= 0x4000_0C80
	//				@ T5CON		= 0x4000_0CA0
	//				@ T6CON		= 0x4000_0CC0
	//				@ T7CON		= 0x4000_0CE0
	//
	//				@ T8CON		= 0x4000_0D00
	//				@ T9CON		= 0x4000_0D20
	// 
	//-----------------------------------------------------------------------------------------
	reg_val = timer->CON; 
	reg_val &= ~TnCON_INTR_MASK; 
	timer->CON = reg_val;
	

	//-----------------------------------------------------------------------------------------
	// clear interrupt flag 
	//
	//				@ T0CON		= 0x4000_0C00
	//				@ T1CON		= 0x4000_0C20
	//				@ T2CON		= 0x4000_0C40
	//				@ T3CON		= 0x4000_0C60
	//
	//				@ T4CON		= 0x4000_0C80
	//				@ T5CON		= 0x4000_0CA0
	//				@ T6CON		= 0x4000_0CC0
	//				@ T7CON		= 0x4000_0CE0
	//
	//				@ T8CON		= 0x4000_0D00
	//				@ T9CON		= 0x4000_0D20
	//
	//-----------------------------------------------------------------------------------------
	reg_val = timer->CON;
	reg_val |= TnCON_IFLAG_MASK;
	timer->CON = reg_val;	


	//-----------------------------------------------------------------------------------------
	// enable interrupt
	//
	//				@ T0CON		= 0x4000_0C00
	//				@ T1CON		= 0x4000_0C20
	//				@ T2CON		= 0x4000_0C40
	//				@ T3CON		= 0x4000_0C60
	//
	//				@ T4CON		= 0x4000_0C80
	//				@ T5CON		= 0x4000_0CA0
	//				@ T6CON		= 0x4000_0CC0
	//				@ T7CON		= 0x4000_0CE0
	//
	//				@ T8CON		= 0x4000_0D00
	//				@ T9CON		= 0x4000_0D20
	//
	//-----------------------------------------------------------------------------------------	
	if (enable == INTR_ENABLE)
	{
		reg_val |= (intr_mask&TnCON_INTR_MASK); 
		timer->CON = reg_val;
	}

}





/**
*********************************************************************************************************
* @ Name : _TIMER_Init
*
* @ Parameter
*		- timer_no : 0~9
*
*		- mode : TIMER_MODE_PERIODIC, TIMER_MODE_PWM, TIMER_MODE_ONE_SHOT, TIMER_MODE_CAPTURE
*
*		- p_TIMER_config
*			# clock_select			= TIMER_CLKSEL_TCLK_DIV_BY_2 ~ TIMER_CLKSEL_EXTERNAL_SOURCE
*			# capture_clear_mode	= TIMER_CAPTURE_RISING_EDGE_CLEAR, TIMER_CAPTURE_FALLING_EDGE_CLEAR
*			# GRA				= 16-bit value
*			# GRB				= 16-bit value
*
*********************************************************************************************************
*/
void _TIMER_Init (int timer_no, int mode, TIMER_CONFIG * p_TIMER_config)
{

	TIMER_Type			* timer; 
	NVIC_IntrConfig 	nvic_config; 
	

	//--------------------------------------------------------------------------------
	// check timer_no
	//--------------------------------------------------------------------------------
	if ((timer_no < 0) || (timer_no > 9)) return; 


	//--------------------------------------------------------------------------------
	// get object 
	//--------------------------------------------------------------------------------
	timer = TIMER_Get_Object(timer_no); 


	//--------------------------------------------------------------------------------
	// configure GPIO
	//--------------------------------------------------------------------------------
	TIMER_ConfigureGPIO(timer, mode); 




	//--------------------------------------------------------------------------------
	// configure GPIO - clock source selection
	//--------------------------------------------------------------------------------
	if (p_TIMER_config->clock_select == TIMER_CLKSEL_EXTERNAL_SOURCE)
	{
		TIMER_ConfigureExternalSource (timer); 
	}

	//--------------------------------------------------------------------------------
	// mode setting 
	//--------------------------------------------------------------------------------	
	TIMER_Init (timer, mode, p_TIMER_config); 


	//------------------------------------------------------------------------------
	// interrupt setting (peripheral)
	//
	//					
	//------------------------------------------------------------------------------
	TIMER_ConfigureInterrupt (timer, (TnCON_TIE0|TnCON_TIE1), INTR_ENABLE); 
//	TIMER_ConfigureInterrupt (timer, (TnCON_TOVE |TnCON_TIE0|TnCON_TIE1), INTR_ENABLE); 
//	TIMER_ConfigureInterrupt (timer, (TnCON_TOVE), INTR_ENABLE); 


	//------------------------------------------------------------------------------
	// interrupt setting (interrupt module)
	//
	//					IRQ_TIMER0=5
	//					IRQ_TIMER1=6
	//					IRQ_TIMER2=7
	//					IRQ_TIMER3=8
	//
	//					IRQ_TIMER4=9
	//					IRQ_TIMER5=10
	//					IRQ_TIMER6=11
	//					IRQ_TIMER7=12	
	//
	//					IRQ_TIMER8=13
	//					IRQ_TIMER9=14		
	//
	//------------------------------------------------------------------------------		
	nvic_config.nIRQ_Number = (IRQ_TIMER0+timer_no); 
	nvic_config.Preemption_Priority= PRIO_TIMER0_PREEMPTION; 
	nvic_config.Subpriority= PRIO_TIMER0_SUBPRIORITY; 
	nvic_config.IntrEnable = INTR_ENABLE; 
	NVIC_ConfigureInterrupt (NVIC, &nvic_config); 


}





/**
*********************************************************************************************************
* @ Name : TIMER_Get_Object
*
* @ Parameter
*		- timer_no : 0~9
*
* @ Return 
*		TIMER-register-band object 
*
*
*********************************************************************************************************
*/
TIMER_Type * TIMER_Get_Object (int timer_no)
{

	TIMER_Type 	* p_obj;


	switch (timer_no)
	{
	case 0: 
		p_obj = T0; 
		break; 

	case 1: 
		p_obj = T1; 
		break; 

	case 2: 
		p_obj = T2; 
		break; 

	case 3: 
		p_obj = T3; 
		break; 	

	case 4: 
		p_obj = T4; 
		break; 

	case 5: 
		p_obj = T5; 
		break; 

	case 6: 
		p_obj = T6; 
		break; 

	case 7: 
		p_obj = T7; 
		break; 		

	case 8: 
		p_obj = T8; 
		break; 

	case 9: 
		p_obj = T9; 
		break; 			

	default:
		p_obj = (TIMER_Type *) 0; 
		break; 

	}


	return (p_obj); 

}




/**
*********************************************************************************************************
* @ Name : TIMER_Start
*
* @ Parameter
*		- timer_no : 0~9
*
* @ Return 
*		
*
*
*********************************************************************************************************
*/
void TIMER_Start (int timer_no)
{

	TIMER_Type		* timer; 

	
	//--------------------------------------------------------------------------------
	// check timer_no
	//--------------------------------------------------------------------------------
	if ((timer_no < 0) || (timer_no > 9)) return; 



	//--------------------------------------------------------------------------------
	// get object 
	//--------------------------------------------------------------------------------
	timer = TIMER_Get_Object(timer_no); 



	//--------------------------------------------------------------------------------
	// start 
	//--------------------------------------------------------------------------------	
	timer->CMD = TnCMD_TEN;

	

}




/**
*********************************************************************************************************
* @ Name : TIMER_Stop
*
* @ Parameter
*		- timer_no : 0~9
*
* @ Return 
*		
*
*
*********************************************************************************************************
*/
void TIMER_Stop (int timer_no)
{

	TIMER_Type 	* timer; 


	//--------------------------------------------------------------------------------
	// check timer_no
	//--------------------------------------------------------------------------------
	if ((timer_no < 0) || (timer_no > 9)) return; 



	//--------------------------------------------------------------------------------
	// get object 
	//--------------------------------------------------------------------------------
	timer = TIMER_Get_Object(timer_no); 


	//--------------------------------------------------------------------------------
	// stop
	//--------------------------------------------------------------------------------	
	timer->CMD = 0;

}
