/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_nvic.c
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
#include "a33g52x_nvic.h"




/**
************************************************************************************
* @ Name : SysTick_Init
*
* @ Parameter
*		- systick		= SYSTICK 
*		- clk_src		= ST_CORE_CLOCK, ST_EXTREF_CLOCK
*		- ext_ref		= 0 (core clock), 
*		- ext_div		= 0 (core clock), 
*		- reload 
*
*
*
************************************************************************************
*/
void SysTick_Init (SysTick_Type * const systick, uint32_t clk_src, uint32_t ext_ref, uint32_t ext_div, uint32_t reload)
{
	uint32_t	reg_val;

	if (clk_src == ST_CORE_CLOCK)
	{
		reg_val = systick->CTRL;
		reg_val &= ~(CSR_CLKSOURCE);
		systick->CTRL = reg_val;
		
		reg_val = systick->CTRL;
		reg_val |= CSR_CLKSOURCE_CORE_CLOCK;
		systick->CTRL = reg_val;	
	}
	else if (clk_src == ST_EXTREF_CLOCK)
	{
		reg_val = systick->CTRL;
		reg_val &= ~(CSR_CLKSOURCE);
		systick->CTRL = reg_val;
		
		reg_val = systick->CTRL;
		reg_val |= CSR_CLKSOURCE_EXTREF_CLOCK;
		systick->CTRL = reg_val;	
	}
	else 
		return; 



	//------------------------------------------------------------------------------
	// reload 
	//------------------------------------------------------------------------------
	
	systick->LOAD = reload;

	//------------------------------------------------------------------------------
	// clear current value 
	//------------------------------------------------------------------------------

	systick->VAL = 0;

}



/**
************************************************************************************
* @ Name : SysTick_ConfigureInterrupt
*
* @ Parameter
*		- systick		= SYSTICK 
*		- run		= 1 (run), 0 (stop) 
*
*
*
************************************************************************************
*/
void SysTick_Run (SysTick_Type * const systick, uint32_t enable)
{
	uint32_t reg_val;
	
	if (enable == 1) 
	{
		reg_val = systick->CTRL;
		reg_val |= CSR_ENABLE;
		systick->CTRL = reg_val;
	}
	else
	{
		reg_val = systick->CTRL;
		reg_val &= ~(CSR_ENABLE);
		systick->CTRL = reg_val;
	}
}






/**
************************************************************************************
* @ Name : SysTick_ConfigureInterrupt
*
* @ Parameter
*		- systick		= SYSTICK 
*		- intr_mask	= ST_INTR_TICK
*		- enable		= SYSINT_ENABLE, SYSINT_DISABLE 
*
*
*
************************************************************************************
*/
void SysTick_ConfigureInterrupt (SysTick_Type * const systick, uint32_t intr_mask, uint32_t enable)
{
	volatile uint32_t		reg_val; 

	
	//------------------------------------------------------------------------------
	// disable interrupt 
	//
	//					CSR			@ 0xE000_E010
	//------------------------------------------------------------------------------
	reg_val = systick->CTRL;
	reg_val &= ~(CSR_TICKINT);
	systick->CTRL = reg_val;

	//------------------------------------------------------------------------------
	// clear interrupt flag 
	//
	//					ICSR		@ 0xE000_ED04
	//------------------------------------------------------------------------------
	SCB->ICSR = ICSR_PENDSTCLR;
	reg_val = systick->CTRL;

	//------------------------------------------------------------------------------
	// enable interrupt 
	//
	//					CSR			@ 0xE000_E010
	//------------------------------------------------------------------------------
	if (enable == SYSINT_ENABLE)
	{
		reg_val = systick->CTRL;
		reg_val |= CSR_TICKINT;
		systick->CTRL = reg_val;
	}

}




/**
***********************************************************************************************************
* @ Name : SCB_Set_PriorityGroup
*
* @ Parameters
*		scb				SCB
*		priority_group		PRIORITY_GROUP_VALUE
*
* @ Register 
*		- AIRCR : PRIGROUP [10:8]    		(0xE000_ED0C)
*
*			bit 31-16		VECTKEY
*			bit 15		ENDIANESS
*			bit 10-8		PRIGROUP
*
*			bit 2			SYSRESETREQ
*			bit 1			VECTCLRACTIVE
*			bit 0			VECTRESET
*
*
***********************************************************************************************************
* Priority Group
*
*			0 	preemption priority field[7:1]		subpriority field[0]
*			1 	preemption priority field[7:2]		subpriority field[1:0]
*			2 	preemption priority field[7:3]		subpriority field[2:0]
*			3 	preemption priority field[7:4]		subpriority field[3:0]
*			4 	preemption priority field[7:5]		subpriority field[4:0]
*			5 	preemption priority field[7:6]		subpriority field[5:0]
*			6 	preemption priority field[7]		subpriority field[6:0]
*			7 	preemption priority field=none	subpriority field[7:0]
*
*			(referenced at page 114 in "The Definitive Guide to ARM Cortex-M3")
*
************************************************************************************************************
*/
void SCB_Set_PriorityGroup (SCB_Type * const scb, uint32_t priority_group)
{
	uint32_t		reg_val;

	reg_val = scb->AIRCR;
	
	reg_val &= ~(AIRCR_VECTKEY_MASK|AIRCR_PRIGROUP_MASK); 
	reg_val |= ((priority_group<<8) & AIRCR_PRIGROUP_MASK); 
	reg_val |= AIRCR_VECTKEY; 

	
	scb->AIRCR = reg_val; 

}


/**
***********************************************************************************************************
* @ Name : SCB_ConfigureException
*
* @ Parameters
*		scb									SCB
*		exception_config->nException_Number	= SYS_EXCEPT_MEMORY_MANAGE (-4)
*											   SYS_EXCEPT_BUS_FAULT (-5)
*											   SYS_EXCEPT_USAGE_FAULT (-6)
*											   SYS_EXCEPT_SYSTICK (-15)
*
*		exception_config->u8Preemption_Priority	= 0~1
*		exception_config->u8Subpriority			= 0~3
*		exception_config->u8ExceptionEnable		= EXCEPTION_ENABLE, EXCEPTION_DISABLE
*
* @ Registers
*
*
* @ Descriptions
*		This function is used for system exceptions. 
*
************************************************************************************************************
*/
#if 0
void SCB_ConfigureException (SCB_Type * const scb, SCB_ExceptionConfig * exception_config)
{
	int					vector_num; 
	uint32_t				group, remnant, shift; 
	
	uint32_t				priority, bit_val,bit_mask; 
	
	//int					exception_num; 
	//uint32_t				exception_group; 
	//uint32_t				exception_bitpos;

	volatile uint32_t		reg_val; 


	//--------------------------------------------------------------------------------------
	// check "interrupt number" 
	//--------------------------------------------------------------------------------------
	if ((exception_config->Exception_Number < -15) || (exception_config->Exception_Number > -4)) return; 


	//--------------------------------------------------------------------------------------
	// exception number (negative) --> vector number (positive)
	//--------------------------------------------------------------------------------------
	vector_num = -(exception_config->Exception_Number); 


	//--------------------------------------------------------------------------------------
	// disable exception
	//--------------------------------------------------------------------------------------
	switch (vector_num)
	{
	case VECT_NUM_MEMORY_MANAGE:
		reg_val = SCB->SHCSR;
		reg_val &= ~(SHCSR_MEMFAULTENA);
		SCB->SHCSR = reg_val;
		break;

	case VECT_NUM_BUS_FAULT:
		reg_val = SCB->SHCSR;
		reg_val &= ~(SHCSR_BUSFAULTENA);
		SCB->SHCSR = reg_val;	
		break; 

	case VECT_NUM_USAGE_FAULT:
		reg_val = SCB->SHCSR;
		reg_val &= ~(SHCSR_USGFAULTENA);
		SCB->SHCSR = reg_val;		
		break; 

	case VECT_NUM_SYSTICK:
		reg_val = SysTick->CTRL;
		reg_val &=  ~(CSR_TICKINT);
		SysTick->CTRL = reg_val;
		break; 

	default:
		break; 
	}
	


	//--------------------------------------------------------------------------------------
	// clear pending bit 
	//--------------------------------------------------------------------------------------
	switch (vector_num)
	{
	case VECT_NUM_MEMORY_MANAGE:
		SCB->CFSR = 0x000000FF;
		break;

	case VECT_NUM_BUS_FAULT:
		SCB->CFSR = 0x0000FF00;
		break; 

	case VECT_NUM_USAGE_FAULT:
		SCB->CFSR = 0xFFFF0000UL;
		break; 

	case VECT_NUM_SYSTICK:
		reg_val = SysTick->CTRL;
		SCB->ICSR = ICSR_PENDSTCLR;
		break; 

	default:
		break; 
	} 


	//--------------------------------------------------------------------------------------
	// filtering
	//--------------------------------------------------------------------------------------
	if (exception_config->ExceptionEnable == EXCEPTION_DISABLE) return; 
	

	//--------------------------------------------------------------------------------------
	// set priority 
	//
	//				vector_num		group 		remnant		shfit
	//				----------		-----		-------		-----
	//				4				0			0			0
	//				5				0			1			8
	//				6				0			2			16
	//				7				0			3			24	
	//
	//				8				1			0			0
	//				9				1			1			8
	//				10				1			2			16
	//				11				1			3			24		
	//
	//				12				2			0			0
	//				13				2			1			8
	//				14				2			2			16
	//				15				2			3			24	
	//
	//--------------------------------------------------------------------------------------
	// bit operations 
	group = ((vector_num - 4) >> 2);
	remnant = ((vector_num - 4) & 0x03); 
	shift = (remnant << 3); 
	
	priority = (exception_config->Subpriority << START_BITPOS_OF_PRIORITY) & SUBPRIORITY_MASK;
	priority |= (exception_config->Preemption_Priority << (PRIORITY_GROUP_VALUE+1)) & PREEMPTION_MASK; 

	bit_val = (priority << shift); 
	bit_mask = (0xFFUL << shift); 


	// action
	reg_val = SCB->SHP[group];
	reg_val &= ~bit_mask;
	reg_val |= bit_val;
	SCB->SHP[group] = reg_val;


	//--------------------------------------------------------------------------------------
	// enable interrupt 
	//--------------------------------------------------------------------------------------
	switch (vector_num)
	{
	case VECT_NUM_MEMORY_MANAGE:
		reg_val = SCB->SHCSR;
		reg_val |= SHCSR_MEMFAULTENA;
		SCB->SHCSR = reg_val;
		break;

	case VECT_NUM_BUS_FAULT:
		reg_val = SCB->SHCSR;
		reg_val |= SHCSR_BUSFAULTENA;
		SCB->SHCSR = reg_val;
		break; 

	case VECT_NUM_USAGE_FAULT:
		reg_val = SCB->SHCSR;
		reg_val |= SHCSR_USGFAULTENA;
		SCB->SHCSR = reg_val;
		break; 

	case VECT_NUM_SYSTICK:
		reg_val = SysTick->CTRL;
		reg_val |= CSR_TICKINT;
		SysTick->CTRL = reg_val;		
		break; 

	default:
		break; 
	}

}
#endif 


/**
***********************************************************************************************************
* @ Name : NVIC_ConfigureInterrupt
*
* @ Parameters
*		nvic								NVIC
*		nvic_config->nIRQ_Number			= 0~47
*		nvic_config->u8Preemption_Priority	= 0~1
*		nvic_config->u8Subpriority			= 0~3
*		nvic_config->u8IntrEnable			= INTR_ENABLE, INTR_DISABLE
*
* @ Registers
*		ICER
*		IPR
*		ISER
*
* @ Descriptions
*		This function is used for IRQ (Vector number = 16 ~ )
*
************************************************************************************************************
*/
void NVIC_ConfigureInterrupt (NVIC_Type * const nvic, NVIC_IntrConfig * nvic_config)
{
	uint8_t			priority;
	int				irq_num;
	uint8_t			irq_group; 
	uint32_t			irq_bitpos; 


	//--------------------------------------------------------------------------------------
	// check "interrupt number" 
	//--------------------------------------------------------------------------------------
	if ((nvic_config->nIRQ_Number < 0) || (nvic_config->nIRQ_Number >= A33G52x_IRQ_NUM)) return; 


	//--------------------------------------------------------------------------------------
	// bit operations 
	//--------------------------------------------------------------------------------------
	priority = (nvic_config->Subpriority << START_BITPOS_OF_PRIORITY) & SUBPRIORITY_MASK;
	priority |= (nvic_config->Preemption_Priority << (PRIORITY_GROUP_VALUE+1)) & PREEMPTION_MASK; 

	irq_num = nvic_config->nIRQ_Number;
	irq_group = (uint8_t) (nvic_config->nIRQ_Number >> 5); 
	irq_bitpos = (0x0001UL << (nvic_config->nIRQ_Number & 0x1F)); 



	//--------------------------------------------------------------------------------------
	// disable interrupt
	//--------------------------------------------------------------------------------------
	nvic->ICER[irq_group] = irq_bitpos;
		
	//--------------------------------------------------------------------------------------
	// clear pending bit 
	//--------------------------------------------------------------------------------------
	nvic->ICPR[irq_group] = irq_bitpos;

	//--------------------------------------------------------------------------------------
	// filtering
	//--------------------------------------------------------------------------------------
	if (nvic_config->IntrEnable == INTR_DISABLE) return; 
	
	//--------------------------------------------------------------------------------------
	// set priority 
	//--------------------------------------------------------------------------------------
	nvic->IP[irq_num] = priority;

	//--------------------------------------------------------------------------------------
	// enable interrupt 
	//--------------------------------------------------------------------------------------
	nvic->ISER[irq_group] = irq_bitpos;

}


