/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_timer.h
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

//==========================================================================
//
//	S T R U C T U R E S 
//
//==========================================================================

//-----------------------------------------------------------------------------------------------
// start level after counter-clear 
//-----------------------------------------------------------------------------------------------
#define TIMER_START_LEVEL_LOW					(0)
#define TIMER_START_LEVEL_HIGH				(1)


//-----------------------------------------------------------------------------------------------
// select clock
//-----------------------------------------------------------------------------------------------
#define TIMER_CLKSEL_TCLK_DIV_BY_2			(0)
#define TIMER_CLKSEL_TCLK_DIV_BY_4			(1)
#define TIMER_CLKSEL_TCLK_DIV_BY_16			(2)
#define TIMER_CLKSEL_TCLK_DIV_BY_64			(3)


#define TIMER_CLKSEL_PMUPCSR					(4)
#define TIMER_CLKSEL_EXTERNAL_SOURCE			(6)


//-----------------------------------------------------------------------------------------------
// capture clear 
//-----------------------------------------------------------------------------------------------
#define TIMER_CAPTURE_RISING_EDGE_CLEAR		(0)
#define TIMER_CAPTURE_FALLING_EDGE_CLEAR		(1)




typedef struct {

	uint8_t				start_level_after_counter_clear; 
	uint8_t				clock_select;
	uint16_t				capture_clear_mode;
	uint16_t				GRA;
	uint16_t				GRB; 
	uint16_t				PRS;

} TIMER_CONFIG; 




//==========================================================================
//
//	D E F I N A T I O N S
//
//==========================================================================

//-----------------------------------------------------------------------------------------------
// mode 
//-----------------------------------------------------------------------------------------------
#define TIMER_MODE_PERIODIC					(0)
#define TIMER_MODE_PWM						(1)
#define TIMER_MODE_ONE_SHOT					(2)
#define TIMER_MODE_CAPTURE					(3)


//-----------------------------------------------------------------------------------------------
// internal/external clock source (PMC)
//-----------------------------------------------------------------------------------------------
#define TIMER_NO_EXT_CLOCK_SRC				(0)
#define TIMER_EXT_CLOCK_SRC					(1)



//==========================================================================
// TnCON
//
//				@ T0CON		= 0x4000_0C00
//
//==========================================================================
#define TnCON_TOVF								(0x0001<<14)
#define TnCON_TIF1								(0x0001<<13)
#define TnCON_TIF0								(0x0001<<12)
#define TnCON_IFLAG_MASK						(0x0007<<12)	

#define TnCON_TOVE								(0x0001<<10)
#define TnCON_TIE1								(0x0001<<9)
#define TnCON_TIE0								(0x0001<<8)
#define TnCON_INTR_MASK						(0x0007<<8)

#define TnCON_TSTRT								(0x0001<<7)

#define TnCON_TCS_TCLK_DIV_BY_2				(0x0000<<4)
#define TnCON_TCS_TCLK_DIV_BY_4				(0x0001<<4)
#define TnCON_TCS_TCLK_DIV_BY_16				(0x0002<<4)
#define TnCON_TCS_TCLK_DIV_BY_64				(0x0003<<4)
#define TnCON_TCS_PMUPCSR						(0x0004<<4)
#define TnCON_TCS_TnC							(0x0006<<4)

#define TnCON_CAPM								(0x0001<<3)
#define TnCON_CAPM_RISING_EDGE_CLEAR			(0x0000<<3)
#define TnCON_CAPM_FALLING_EDGE_CLEAR		(0x0001<<3)

#define TnCON_TMODE_PERIODIC					(0x0000<<0)
#define TnCON_TMODE_PWM						(0x0001<<0)
#define TnCON_TMODE_ONE_SHOT					(0x0002<<0)
#define TnCON_TMODE_CAPTURE					(0x0003<<0)
#define TnCON_TMODE_MASK						(0x0003<<0)



//==========================================================================
// TnCMD
//
//				@ T0CMD		= 0x4000_0C04
//
//==========================================================================
#define TnCMD_TOUT								(0x0001<<15)

#define TnCMD_TCLR								(0x0001<<1)
#define TnCMD_TEN								(0x0001<<0)




//==========================================================================
// TnGRA
//
//				@ T0GRA		= 0x4000_0C08
//
//==========================================================================


//==========================================================================
// TnGRB
//
//				@ T0GRB		= 0x4000_0C0C
//
//==========================================================================



//==========================================================================
// TnPRS
//
//				@ T0PRS		= 0x4000_0C10
//
//==========================================================================



//==========================================================================
// TnCNT
//
//				@ T0PRS		= 0x4000_0C14
//
//==========================================================================




//==========================================================================
// 
//		F U N C T I O N    D E C L A R A T I O N S 
//
//==========================================================================
void TIMER_ConfigureGPIO (TIMER_Type * const timer, int mode); 
void TIMER_ConfigureExternalSource (TIMER_Type * const timer); 
int TIMER_Get_Object_Number (TIMER_Type * const timer); 
void TIMER_Init (TIMER_Type * const timer, int mode_extclock, TIMER_CONFIG * p_config); 
void TIMER_ConfigureInterrupt (TIMER_Type * const timer, uint32_t intr_mask, uint32_t enable); 


void _TIMER_Init (int timer_no, int mode, TIMER_CONFIG * p_TIMER_config); 
TIMER_Type * TIMER_Get_Object (int timer_no); 
void TIMER_Start (int timer_no); 
void TIMER_Stop (int timer_no); 

