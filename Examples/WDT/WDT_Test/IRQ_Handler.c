/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : IRQ_Handler.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : August, 2017
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
#include "a33g52x_pmu.h"
#include "a33g52x_pcu.h"
#include "a33g52x_gpio.h"
#include "a33g52x_nvic.h"
#include "a33g52x_frt.h"
#include "a33g52x_uart.h"
#include "a33g52x_wdt.h"

//#include "a33g52x_adc.h"

uint32_t g_SYSTICK_count; 

uint32_t g_LVDCON_val; 
uint32_t	g_MXOSCFAIL_count; 


uint32_t	g_WDT_count; 
uint32_t	g_FRT_count; 

uint32_t	g_TIMER0_count; 
uint32_t	g_TIMER1_count; 
uint32_t	g_TIMER2_count; 
uint32_t	g_TIMER3_count; 			 
uint32_t	g_TIMER4_count;  
uint32_t	g_TIMER5_count; 
uint32_t	g_TIMER6_count; 
uint32_t	g_TIMER7_count; 
uint32_t	g_TIMER8_count; 
uint32_t	g_TIMER9_count; 

uint32_t	g_PWM0_mon; 

uint32_t	g_I2C0_count; 
uint32_t	g_I2C1_count; 

uint32_t	g_SPI0_count; 
uint32_t	g_SPI1_count; 

uint32_t	g_I2C0_count; 
uint32_t	g_I2C1_count;

uint32_t	g_UART0_count; 
uint32_t	g_UART1_count; 
uint32_t	g_UART2_count; 
uint32_t	g_UART3_count; 

uint32_t	g_PA_PnISR; 
uint32_t	g_PB_PnISR; 
uint32_t	g_PC_PnISR; 
uint32_t	g_PD_PnISR; 
uint32_t	g_PE_PnISR; 
uint32_t	g_PF_PnISR; 

uint32_t g_ADC_data;
uint32_t g_ADC_flag;

uint32_t g_PCU_index; 
uint32_t	g_PCU_val[8]; 


/**
****************************************************************************************************
* @ Name : SysTick_Handler 
*
*			
*
****************************************************************************************************
*/
//void SysTick_Handler (void)
//{

//	volatile uint32_t 		reg_val;

//	reg_val = SysTick->VAL;
//	SysTick->VAL = 0;

//	g_SYSTICK_count++;

////	g_on_sleep_routine_bkup = g_on_sleep_routine; 

//}



/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////




/**
****************************************************************************************************
* @ Name : WDT_Handler
*
* @ IRQ: 
*			IRQ_WDT 			(3)
*
****************************************************************************************************
*/
void WDT_Handler (void)
{

	/*
	// WDT Test : WDT Interrupt
	WDT_UpdateTimeOut(74000-1); 

	if(g_WDT_count %100 == 0)
		GPIO_OutputHigh(PD, 0);
	else
		GPIO_OutputLow(PD, 0);

	g_WDT_count++;	
	*/
	
//	WDT_UpdateTimeOut(74000-1);			// 1ms
	WDT_UpdateTimeOut(74000000-1); 	// 1s
	if(g_WDT_count == 0)
	{
		GPIO_OutputHigh(PD, 0);
		g_WDT_count = 1;
	}
	else
	{
		GPIO_OutputLow(PD, 0);
		g_WDT_count = 0;
	}
	
}



/**
****************************************************************************************************
* @ Name : UART0_Handler
*
* @ IRQ: 
*			IRQ_UART0 			(38)
*
****************************************************************************************************
*/
void UART0_Handler (void)
{
	
	UART0_Transmit_Receive_ISR(); 
	//UART0_FIFO_ISR(); 

	g_UART0_count++; 

}


