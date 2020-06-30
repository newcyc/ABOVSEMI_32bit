/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_adc.h
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


extern uint32_t	g_ADC_flag; 
extern uint32_t	g_ADC_data; 
extern uint8_t		g_ADC_ch;


//==========================================================================
//		
//		D E F I N A T I O N S
//
//==========================================================================
#define ADC_NO_TRIG								(0xFF)

#define ADC_TRIG_TIMER0						(0)
#define ADC_TRIG_TIMER1						(1)
#define ADC_TRIG_TIMER2						(2)
#define ADC_TRIG_TIMER3						(3)
#define ADC_TRIG_TIMER4						(4)
#define ADC_TRIG_TIMER5						(5)
#define ADC_TRIG_TIMER6						(6)
#define ADC_TRIG_TIMER7						(7)


#define ADC_NO_INTR								(0)
#define ADC_INTR									(1)



//==========================================================================
// 	ADCR
//		
//				@ address = 0x4000_0E00
//
//==========================================================================
#define ADCR_ADEOC								(0x0001<<8)
#define ADCR_ADST								(0x0001<<7)
#define ADCR_AFLAG								(0x0001<<6)
#define ADCR_ADIF								(0x0001<<5)

#define ADCR_ADSEL_VAL(n)						(((n)&0x0F)<<0)
#define ADCR_ADSEL_MASK						(0x000F<<0)



//==========================================================================
// 	ADMR
//		
//				@ address = 0x4000_0E04
//
//==========================================================================
#define ADMR_ADCEN								(0x0001<<15)
#define ADMR_ADSTBY							(0x0001<<14)
#define ADMR_ADIE								(0x0001<<12)
#define ADMR_EXTRG								(0x0001<<11)

#define ADMR_TSEL_VAL(n)						(((n)&0x07)<<8)
#define ADMR_TSEL_MASK							(0x07<<8)

#define ADMR_ADCS_VAL(n)						(((n)&0x00FF)<<0)
#define ADMR_ADCS_MASK						(0x00FF<<0)



//==========================================================================
// 	ADTEST
//		
//				@ address = 0x4000_0E08
//
//==========================================================================
#define ADTEST_TESTEN 							(0x0001<<4)

#define ADTEST_TESTSEL_VDC 					(0x0000<<0)
#define ADTEST_TESTSEL_BGR 					(0x0001<<0)


typedef enum {
	ADC_INPUT_0 = 0,
	ADC_INPUT_1,
	ADC_INPUT_2,
	ADC_INPUT_3,
	ADC_INPUT_4,
	ADC_INPUT_5,
	ADC_INPUT_6,
	ADC_INPUT_7,
	ADC_INPUT_8,
	ADC_INPUT_9,
#ifdef A33G527
	ADC_INPUT_10,
	ADC_INPUT_11,
	ADC_INPUT_12,
	ADC_INPUT_13,
	ADC_INPUT_14,
	ADC_INPUT_15,
#endif
	ADC_INPUT_VDC,
	ADC_INPUT_BGR
} ADC_INPUT;

//==========================================================================
// 
//		F U N C T I O N    D E C L A R A T I O N S 
//
//==========================================================================
void ADC_ConfigureGPIO (ADC_Type * const adc, int channel); 
void ADC_Set (ADC_Type * const adc, uint32_t channel, uint32_t trig); 
void ADC_ConfigureInterrupt (ADC_Type * adc, uint32_t intr_mask, uint32_t enable); 


// ADC Driver
int ADC_Init (int adc_no, int channel, int intr, int trig);
ADC_Type * ADC_Get_Object (int adc_no);
//////////////////////////////////////////////////////////////////////////////////////////////////////

