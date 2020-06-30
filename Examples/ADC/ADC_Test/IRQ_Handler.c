/**
**********************(C) Copyright 2015 ABOV Semiconductor Co., Ltd *******************
* @ File : IRQ_Handler.c
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Oct, 2017
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
#include "a33g52x_uart.h"
#include "a33g52x_frt.h"
#include "a33g52x_adc.h"

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
* @ Name : LVDFAIL_Handler
*
* @ IRQ: 
*			IRQ_LVDFAIL			(0)
*
****************************************************************************************************
*/
void LVDFAIL_Handler (void)
{
	g_LVDCON_val = PMU->LVDCON;
	PMU->LVDCON = g_LVDCON_val;

	/*
	if (g_LVDCON_val == 0x804F) 
	{
		while(1); 
	}
	*/

	
}



/**
****************************************************************************************************
* @ Name : MXOSCFAIL_Handler
*
* @ IRQ: 
*			IRQ_MXOSCFAIL 			(1)
*
****************************************************************************************************
*/
void MXOSCFAIL_Handler (void)
{
	uint32_t		reg_val;

	reg_val = PMU->CMR;
	PMU->CMR = reg_val;
	
	g_MXOSCFAIL_count++; 
	
}



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

}


/**
****************************************************************************************************
* @ Name : FRT_Handler
*
* @ IRQ: 
*			IRQ_FRT	 			(4)
*
****************************************************************************************************
*/
void FRT_Handler (void)
{
	uint32_t				reg_val; 
	volatile uint32_t		delay; 

	reg_val = FRT->CON;
	reg_val &= ~(FRTCON_FOF|FRTCON_FMF);
	FRT->CON = reg_val;
	
	//reg_val = (CSP_FRT_GET_FRTCON(FRT) & (FRTCON_FOF|FRTCON_FMF)); 
	//if (reg_val)
	//{
		//while(1);
	//}

	g_FRT_count++; 
}


/**
****************************************************************************************************
* @ Name : TIMER0_Handler
*
* @ IRQ: 
*			IRQ_TIMER0 			(5)
*
****************************************************************************************************
*/
void TIMER0_Handler(void)
{
	uint32_t		reg_val;
	
	reg_val = T0->CON;
	T0->CON = reg_val;
	
	g_TIMER0_count++; 
	
}

/**
****************************************************************************************************
* @ Name : PWM0_Handler
*
* @ IRQ: 
*			IRQ_PWM0 			(24)
*
****************************************************************************************************
*/
void PWM0_Handler (void)
{

	PWM_Type			* pwm_obj = PWM0; 
	volatile uint32_t		reg_val; 


	reg_val = pwm_obj->CTRL; 
	pwm_obj->CTRL = reg_val;
	g_PWM0_mon = pwm_obj->CTRL;
	
}



/**
****************************************************************************************************
* @ Name : MCKFAIL_Handler
*
* @ IRQ: 
*			IRQ_MCKFAIL 			(15)
*
****************************************************************************************************
*/
void MCKFAIL_Handler (void)
{

	uint32_t 		reg_val; 
	volatile int	delay; 


	reg_val = PMU->CMR;
	PMU->CMR = reg_val;

	#if 0
	reg_val = PMU->BCCR;
	reg_val &= ~PMUBCCR_MCLKSEL_MASK; 
	reg_val |= PMUBCCR_MCLKSEL_PLL; 
	PMU->BCCR = reg_val;
	#endif

	#if 0
	reg_val = PMU->RSER;
	reg_val |= PMURSER_SWRSTE; 
	PMU->RSER = reg_val;
	
	PMU->CFG = PMUCFGR_SOFTRST;
	#endif

	for (delay=0; delay<100; delay++); 

}




/**
****************************************************************************************************
* @ Name : GPIOA_Handler
*
* @ IRQ: 
*			IRQ_GPIOA 			(16)
*
****************************************************************************************************
*/
void GPIOA_Handler (void)
{

	g_PA_PnISR = PCA->ISR;
	PCA->ISR = g_PA_PnISR;
	
}

/**
****************************************************************************************************
* @ Name : GPIOB_Handler
*
* @ IRQ: 
*			IRQ_GPIOB 			(17)
*
****************************************************************************************************
*/
void GPIOB_Handler (void)
{
	g_PB_PnISR = PCB->ISR;
	PCB->ISR = g_PB_PnISR;
	GPIO_OutputHigh(PD, 0);
}

/**
****************************************************************************************************
* @ Name : GPIOC_Handler
*
* @ IRQ: 
*			IRQ_GPIOC 			(18)
*
****************************************************************************************************
*/
void GPIOC_Handler (void)
{

	g_PC_PnISR = PCC->ISR;
	PCC->ISR = g_PC_PnISR;

}

/**
****************************************************************************************************
* @ Name : GPIOD_Handler
*
* @ IRQ: 
*			IRQ_GPIOD 			(19)
*
****************************************************************************************************
*/
void GPIOD_Handler (void)
{

	g_PD_PnISR = PCD->ISR;
	PCD->ISR = g_PD_PnISR;

}

/**
****************************************************************************************************
* @ Name : GPIOE_Handler
*
* @ IRQ: 
*			IRQ_GPIOE 			(20)
*
****************************************************************************************************
*/
void GPIOE_Handler (void)
{

	g_PE_PnISR = PCE->ISR;
	PCE->ISR = g_PE_PnISR;

	if (g_PCU_index < 8)
	{
		g_PCU_val[g_PCU_index++] = g_PE_PnISR; 
	}

}

/**
****************************************************************************************************
* @ Name : GPIOF_Handler
*
* @ IRQ: 
*			IRQ_GPIOF 			(21)
*
****************************************************************************************************
*/
void GPIOF_Handler (void)
{

	g_PF_PnISR = PCF->ISR;
	PCF->ISR = g_PF_PnISR;
	
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



/**
****************************************************************************************************
* @ Name : UART1_Handler
*
* @ IRQ: 
*			IRQ_UART1 			(39)
*
****************************************************************************************************
*/
void UART1_Handler (void)
{
	
	UART1_Transmit_Receive_ISR(); 

	g_UART1_count++; 

}



/**
****************************************************************************************************
* @ Name : UART2_Handler
*
* @ IRQ: 
*			IRQ_UART2 			(40)
*
****************************************************************************************************
*/
void UART2_Handler (void)
{
	
	UART2_Transmit_Receive_ISR(); 

	g_UART2_count++; 

}



/**
****************************************************************************************************
* @ Name : UART3_Handler
*
* @ IRQ: 
*			IRQ_UART0 			(41)
*
****************************************************************************************************
*/
void UART3_Handler (void)
{
	/* This ISR is used for the communication with PC. */
	UART3_Transmit_Receive_ISR(); 

	g_UART3_count++; 
	
}


/**
****************************************************************************************************
* @ Name : ADC_Handler
*
* @ IRQ: 
*			IRQ_ADC 			(43)
*
****************************************************************************************************
*/

void ADC_Handler(void)
{
#if 1
	uint16_t reg_val;
	
	reg_val = ADC->CR;
	reg_val |= ADCR_ADIF;
	
	g_ADC_data = ( (ADC->DR) >> 4);			// Get ADC Data
	
	ADC->CR = reg_val;							// Interrupt Flag Clear

	g_ADC_flag = 1;

#endif
}
