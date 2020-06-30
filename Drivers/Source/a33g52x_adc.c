/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_adc.c
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
#include "a33g52x_adc.h"
#include "a33g52x_pcu.h"
#include "a33g52x_nvic.h"
#include "a33g52x_timer.h"



//================================================================================
// ADC PORT
//
//================================================================================
uint32_t g_ADC_PORT[16][3] = {

	{PCA_BASE, PIN_0, PA0_MUX_AN0}, 				// AN 0 
	{PCA_BASE, PIN_1, PA1_MUX_AN1}, 				// AN 1 
	{PCA_BASE, PIN_2, PA2_MUX_AN2}, 				// AN 2 
	{PCA_BASE, PIN_3, PA3_MUX_AN3}, 				// AN 3 

	{PCA_BASE, PIN_4, PA4_MUX_AN4}, 				// AN 4 
	{PCA_BASE, PIN_5, PA5_MUX_AN5}, 				// AN 5 
	{PCA_BASE, PIN_6, PA6_MUX_AN6}, 				// AN 6 
	{PCA_BASE, PIN_7, PA7_MUX_AN7}, 				// AN 7 

	{PCF_BASE, PIN_0, PF0_MUX_AN8}, 				// AN 8 
	{PCF_BASE, PIN_1, PF1_MUX_AN9}, 				// AN 9 
	{PCF_BASE, PIN_2, PF2_MUX_AN10}, 				// AN 10 
	{PCF_BASE, PIN_3, PF3_MUX_AN11}, 				// AN 11 
		
	{PCF_BASE, PIN_4, PF4_MUX_AN12},				// AN 12 
	{PCF_BASE, PIN_5, PF5_MUX_AN13},				// AN 13 
	{PCA_BASE, PIN_14, PA14_MUX_AN14}, 			// AN 14 
	{PCA_BASE, PIN_15, PA15_MUX_AN15}, 			// AN 15		

}; 


/**
*********************************************************************************************************
* @ Name : ADC_ConfigureGPIO
*
* @ Parameter
*		- adc : 	ADC
*		- channel  : 0 ~ 15
*
*
*********************************************************************************************************
*/
void ADC_ConfigureGPIO (ADC_Type * const adc, int channel)
{

	PCU_Type 	*pcu_base; 
	uint32_t			pin_no; 
	uint32_t			mux_info; 



	//--------------------------------------------------------------------------------------
	// check channel 
	//
	//--------------------------------------------------------------------------------------
	if ((channel < 0) || (channel > 15)) return; 


	
	//--------------------------------------------------------------------------------------
	// port base address, pin no, mux info 
	//
	//--------------------------------------------------------------------------------------
	pcu_base = (PCU_Type *) g_ADC_PORT[channel][0]; 
	pin_no = g_ADC_PORT[channel][1]; 
	mux_info = g_ADC_PORT[channel][2]; 



	//--------------------------------------------------------------------------------------
	// setting 
	//
	//--------------------------------------------------------------------------------------
	PCU_ConfigureFunction (pcu_base, pin_no, mux_info); 
	PCU_Set_Direction_Type (pcu_base, pin_no, PnCR_INPUT_ANALOG); 
	PCU_ConfigurePullup (pcu_base, pin_no, PnPCR_PULLUP_DISABLE); 	



}



/**
*********************************************************************************************************
* @ Name : ADC_Set
*
* @ Parameter
*		- adc : ADC
*		- channel : 0~15
*		- trig : ADC_NO_TRIG, ADC_TRIG_TIMER0, ADC_TRIG_TIMER1, ..., ADC_TRIG_TIMER7
*
*
*********************************************************************************************************
*/
void ADC_Set(ADC_Type * const adc, uint32_t channel, uint32_t trig)
{

	uint32_t 		reg_val; 

	//--------------------------------------------------------------------------------------
	// enable
	//
	//				@ ADMR = 0x4000_0E04 (bit 15)
	//--------------------------------------------------------------------------------------
	reg_val = ADMR_ADCEN;
	adc->MR = reg_val;


	//--------------------------------------------------------------------------------------
	// trig 
	//
	//				@ ADMR = 0x4000_0E04 (bit 10-8)		
	//--------------------------------------------------------------------------------------	
	if (trig != ADC_NO_TRIG)
	{
		reg_val |= (ADMR_EXTRG|ADMR_TSEL_VAL(trig)); 
	}

	reg_val |= ADMR_ADCS_VAL(18); 

	adc->MR = reg_val;


	//--------------------------------------------------------------------------------------
	// channel
	//
	//				@ ADCR = 0x4000_0E00
	//--------------------------------------------------------------------------------------
	if (channel <= 15)
	{
		reg_val = adc->CR; 
		reg_val &= ~ADCR_ADSEL_MASK;
		reg_val |= ADCR_ADSEL_VAL(channel); 
		adc->CR = reg_val;

		adc->TEST = 0;	
	}
	else if (channel == 16)
	{
		adc->TEST = (ADTEST_TESTEN|ADTEST_TESTSEL_VDC);			// VDC
	}
	else if (channel == 17)
	{
		adc->TEST = (ADTEST_TESTEN|ADTEST_TESTSEL_BGR);			// BGR
	}
		

}




/**
*********************************************************************************************************
* @ Name : ADC_ConfigureInterrupt 
*
* @ Parameters
*		- adc : ADC 
*		- intr_mask: ADMR_ADIE
*		- enable : INTR_ENABLE, INTR_DISABLE 
*
*
*********************************************************************************************************
*/
void ADC_ConfigureInterrupt (ADC_Type * adc, uint32_t intr_mask, uint32_t enable)
{

	uint32_t		reg_val; 


	//-----------------------------------------------------------------------------------------
	// disable interrupt
	//
	//				@ ADMR = 0x4000_0E04
	//			
	//-----------------------------------------------------------------------------------------
	reg_val = adc->MR;
	reg_val &= ~ADMR_ADIE;
	adc->MR = reg_val;


	//-----------------------------------------------------------------------------------------
	// clear interrupt flag
	//
	//				@ ADCR = 0x4000_0E00
	//			
	//-----------------------------------------------------------------------------------------
	reg_val = adc->CR;
	reg_val |= ADCR_ADIF;
	adc->CR = reg_val;


	//-----------------------------------------------------------------------------------------
	// enable interrupt
	//
	//				@ ADMR = 0x4000_0E04
	//			
	//-----------------------------------------------------------------------------------------
	if (enable == INTR_ENABLE)
	{
		reg_val = adc->MR;
		reg_val |= (intr_mask & ADMR_ADIE);
		adc->MR = reg_val;
	}


}


/**
*********************************************************************************************************
* @ Name : ADC_Init
*
* @ Parameter
*		- adc_no : 0
*		- channel : ADC channel 
*		- intr : ADC_INTR, ADC_NO_INTR
*		- trig : ADC_NO_TRIG, ADC_TRIG_TIMER0, ADC_TRIG_TIMER1, ..., ADC_TRIG_TIMER7
*
* @ Return 
*		0 		success
*		1 		fail 
*
*********************************************************************************************************
*/
int ADC_Init (int adc_no, int channel, int intr, int trig)
{

	ADC_Type					* adc; 
	NVIC_IntrConfig			nvic_config; 

	//------------------------------------------------------------------------------------------
	// check adc_no
	//------------------------------------------------------------------------------------------
	if ((adc_no < 0) || (adc_no > 0)) return (1); 



	//------------------------------------------------------------------------------------------
	// get object 
	//------------------------------------------------------------------------------------------
	adc = ADC_Get_Object(adc_no); 



	//------------------------------------------------------------------------------------------
	// configure GPIO 
	//
	//				channel 0		PA0
	//				channel 1		PA1
	//				channel 2		PA2
	//				channel 3		PA3
	//
	//				channel 4		PA4
	//				channel 5		PA5
	//				channel 6		PA6
	//				channel 7		PA7	
	//
	//				channel 8		PF0
	//				channel 9		PF1
	//				channel 10	PF2
	//				channel 11	PF3
	//
	//				channel 12	PF4
	//				channel 13	PF5
	//				channel 14	PA14
	//				channel 15	PA15	
	//------------------------------------------------------------------------------------------
	if (channel <= 15)
	{
		ADC_ConfigureGPIO(adc, channel);
	}



	//------------------------------------------------------------------------------------------
	// ADC setting 
	//
	//				channel
	//				trig
	//				clock 
	//------------------------------------------------------------------------------------------
	ADC_Set(adc, channel, trig); 



	//------------------------------------------------------------------------------------------
	// interrupt (peripheral)
	//------------------------------------------------------------------------------------------
	if (intr == ADC_NO_INTR) return (0); 


	//------------------------------------------------------------------------------------------
	// interrupt (peripheral)
	//------------------------------------------------------------------------------------------
	if (intr == ADC_INTR)
	{
		ADC_ConfigureInterrupt (ADC, ADMR_ADIE, INTR_ENABLE); 
	}

	

	//------------------------------------------------------------------------------------------
	// interrupt (NVIC)
	//------------------------------------------------------------------------------------------
	if (intr == ADC_INTR)
	{
		nvic_config.nIRQ_Number = IRQ_ADC; 
		nvic_config.Preemption_Priority = PRIO_ADC_PREEMPTION; 
		nvic_config.Subpriority = PRIO_ADC_SUBPRIORITY; 
		nvic_config.IntrEnable = INTR_ENABLE; 
		NVIC_ConfigureInterrupt(NVIC, &nvic_config); 
	}
	

	return (0); 



}



/**
*********************************************************************************************************
* @ Name : ADC_Get_Object
*
* @ Parameter
*		- adc_no : 0
*
* @ Return 
*		- object (ADC_Type) 
*
*
*********************************************************************************************************
*/
ADC_Type * ADC_Get_Object (int adc_no)
{

	ADC_Type		* p_obj; 

	switch (adc_no)
	{
	case 0:
		p_obj = ADC; 
		break; 


	default: 
		p_obj = (ADC_Type *) 0; 
		break; 
	}


	return (p_obj); 


}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


