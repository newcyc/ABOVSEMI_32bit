/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_pwm.h
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
#define PWM_SYNC_MODE						(1)
#define PWM_ASYNC_MODE						(0)

#define PWM_SIG_INVERT						(1)
#define PWM_SIG_NO_INVERT					(0)

#define PWM_CLKSEL_CLK_DIV_BY_2				(0)
#define PWM_CLKSEL_CLK_DIV_BY_4				(1)
#define PWM_CLKSEL_CLK_DIV_BY_8				(2)
#define PWM_CLKSEL_CLK_DIV_BY_16			(3)





typedef struct {

	uint8_t				sync_mode;
	uint8_t				invert; 
	uint16_t				clock_select; 

	uint32_t				period;
	uint32_t				compare_val; 

} PWM_CONFIG; 


//==========================================================================
// 	PWMPRSn
//		
//				@ PWMPRS0 = 0x4000_077C
//				@ PWMPRS1 = 0x4000_07FC
//
//==========================================================================
#define PWMPRSn_CLKEN							(0x0001<<15)

#define PWMPRSn_PRESCALERn_VAL(n)				(((n)&0x00FF)<<0)
#define PWMPRSn_PRESCALERn_MASK					(0x00FF<<0)




//==========================================================================
// 	PWMnCTRL
//		
//				@ PWM0CTRL = 0x4000_0700
//				@ PWM1CTRL = 0x4000_0720
//				@ PWM2CTRL = 0x4000_0740
//				@ PWM3CTRL = 0x4000_0760
//
//				@ PWM4CTRL = 0x4000_0780
//				@ PWM5CTRL = 0x4000_07A0
//				@ PWM6CTRL = 0x4000_07C0
//				@ PWM7CTRL = 0x4000_07E0
//
//==========================================================================
#define PWMnCTRL_PRF							(0x0001<<12)

#define PWMnCTRL_PRIE							(0x0001<<8)

#define PWMnCTRL_CKSEL_DIV_BY_2					(0x0000<<5)
#define PWMnCTRL_CKSEL_DIV_BY_4					(0x0001<<5)
#define PWMnCTRL_CKSEL_DIV_BY_8					(0x0002<<5)
#define PWMnCTRL_CKSEL_DIV_BY_16				(0x0003<<5)

#define PWMnCTRL_INVA							(0x0001<<3)
#define PWMnCTRL_SYNC							(0x0001<<1)
#define PWMnCTRL_STRT							(0x0001<<0)




//==========================================================================
// 	PWMnCNT
//		
//				@ PWM0CNT = 0x4000_0704
//				@ PWM1CNT = 0x4000_0724
//				@ PWM2CNT = 0x4000_0744
//				@ PWM3CNT = 0x4000_0764
//
//				@ PWM4CNT = 0x4000_0784
//				@ PWM5CNT = 0x4000_07A4
//				@ PWM6CNT = 0x4000_07C4
//				@ PWM7CNT = 0x4000_07E4
//
//==========================================================================
#define PWMnCNT_VAL(n)							(((n)&0x0000FFFF)<<0)
#define PWMnCNT_MASK							(0x0000FFFF<<0)




//==========================================================================
// 	PWMnPER
//		
//				@ PWM0PER = 0x4000_0708
//				@ PWM1PER = 0x4000_0728
//				@ PWM2PER = 0x4000_0748
//				@ PWM3PER = 0x4000_0768
//
//				@ PWM4PER = 0x4000_0788
//				@ PWM5PER = 0x4000_07A8
//				@ PWM6PER = 0x4000_07C8
//				@ PWM7PER = 0x4000_07E8
//
//==========================================================================
#define PWMnPER_VAL(n)							(((n)&0x0000FFFF)<<0)
#define PWMnPER_MASK							(0x0000FFFF<<0)




//==========================================================================
// 	PWMnCMPA
//		
//				@ PWM0CMPA = 0x4000_070C
//				@ PWM1CMPA = 0x4000_072C
//				@ PWM2CMPA = 0x4000_074C
//				@ PWM3CMPA = 0x4000_076C
//
//				@ PWM4CMPA = 0x4000_078C
//				@ PWM5CMPA = 0x4000_07AC
//				@ PWM6CMPA = 0x4000_07CC
//				@ PWM7CMPA = 0x4000_07EC
//
//==========================================================================
#define PWMnCMPA_VAL(n)							(((n)&0x0000FFFF)<<0)
#define PWMnCMPA_MASK							(0x0000FFFF<<0)




//==========================================================================
// 
//		F U N C T I O N    D E C L A R A T I O N S 
//
//==========================================================================
void PWM_ConfigureGPIO (PWM_Type * const pwm); 
int PWM_Convert_PWM_Into_PWMNo (PWM_Type * const pwm); 
void PWM_Init (PWM_Type * const pwm, PWM_CONFIG * p_config); 
void PWM_ConfigureInterrupt (PWM_Type * const pwm, uint32_t intr_mask, uint32_t enable); 


////////////////////////////////////////////////////////////////////////////////////////////
void _PWM_Init (int pwm_no, PWM_CONFIG * p_PWM_config); 
PWM_Type * PWM_Get_Object (int pwm_no); 
PWMPRS_Type * PWM_Get_Prescaler_Object (int pwm_no);
void PWM_Start(int pwm_no); 
void PWM_Stop (int pwm_no); 
