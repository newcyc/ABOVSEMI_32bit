/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_nvic.h
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



//##########################################################################
//#
//#		Data Structure 
//#
//##########################################################################
//---------------------------------------------------------------------------------------
// Vecotr Table Number : 1~15
//
//			nException_Number		Vector Name
//			----------------		-----------
//				-1					Reset
//				-2					NMI
//				-3					Hard Fault
//				-4					Memeory Manage
//				-5					Bus Fault
//				-6					Usage Fault
//
//				-11					SVCall
//				-12					Debug Monitor
//				
//				-14					PendSV
//				-15					SysTick 
//
//---------------------------------------------------------------------------------------
typedef struct {
	int		Exception_Number;
	uint8_t	Preemption_Priority;
	uint8_t	Subpriority;
	uint8_t	ExceptionEnable;
} SCB_ExceptionConfig; 


//---------------------------------------------------------------------------------------
// Vecotr Table Number : 16~239
//---------------------------------------------------------------------------------------
typedef struct {
	int		nIRQ_Number;
	uint8_t	Preemption_Priority;
	uint8_t	Subpriority;
	uint8_t	IntrEnable;
} NVIC_IntrConfig; 


//SCS_T		*		SCS 			= (SCS_T *) 0xE000E000UL;
//SYSTICK_T * 		SYSTICK	= (SYSTICK_T *) 0xE000E010UL; 
////NVIC_Type		* 		NVIC			= (NVIC_T *) 0xE000E100UL; 
//TPIU_T		* 		TPIU			= (TPIU_T *) 0xE0040000UL; 

//##########################################################################
//#
//#		Vector Number (0~15)
//#
//##########################################################################
#define VECT_NUM_RESET					(1)
#define VECT_NUM_NMI					(2)
#define VECT_NUM_HARD_FAULT				(3)

#define VECT_NUM_MEMORY_MANAGE			(4)
#define VECT_NUM_BUS_FAULT				(5)
#define VECT_NUM_USAGE_FAULT			(6)

#define VECT_NUM_SVCALL					(11)
#define VECT_NUM_DEBUG_MONITOR			(12)

#define VECT_NUM_PENDSV					(14)
#define VECT_NUM_SYSTICK				(15)
//-------------------------------------------------------------------------------------------

// System Exceptions
#define SYS_EXCEPT_RESET				(-1)
#define SYS_EXCEPT_NMI					(-2)
#define SYS_EXCEPT_HARD_FAULT			(-3)

#define SYS_EXCEPT_MEMORY_MANAGE		(-4)
#define SYS_EXCEPT_BUS_FAULT			(-5)
#define SYS_EXCEPT_USAGE_FAULT			(-6)

#define SYS_EXCEPT_SVCALL				(-11)
#define SYS_EXCEPT_DEBUG_MONITOR		(-12)

#define SYS_EXCEPT_PENDSV				(-14)
#define SYS_EXCEPT_SYSTICK				(-15)



//##########################################################################
//#
//#		IRQ number 
//#
//##########################################################################
#define IRQ_LVDFAIL						(0)
#define IRQ_MXOSCFAIL					(1)
#define IRQ_SXOSCFAIL					(2)
#define IRQ_WDT							(3)
#define IRQ_FRT							(4)
#define IRQ_TIMER0						(5)
#define IRQ_TIMER1						(6)
#define IRQ_TIMER2						(7)


#define IRQ_TIMER3						(8)
#define IRQ_TIMER4						(9)
#define IRQ_TIMER5						(10)
#define IRQ_TIMER6						(11)
#define IRQ_TIMER7						(12)
#define IRQ_TIMER8						(13)
#define IRQ_TIMER9						(14)
#define IRQ_MCKFAIL						(15)


#define IRQ_GPIOA						(16)
#define IRQ_GPIOB						(17)
#define IRQ_GPIOC						(18)
#define IRQ_GPIOD						(19)
#define IRQ_GPIOE						(20)
#define IRQ_GPIOF						(21)
// NULL									(22)
// NULL									(23)


#define IRQ_PWM0						(24)
#define IRQ_PWM1						(25)
#define IRQ_PWM2						(26)
#define IRQ_PWM3						(27)
#define IRQ_PWM4						(28)
#define IRQ_PWM5						(29)
#define IRQ_PWM6						(30)
#define IRQ_PWM7						(31)



#define IRQ_SPI0						(32)
#define IRQ_SPI1						(33)
// NULL									(34)
// NULL									(35)
#define IRQ_I2C0						(36)
#define IRQ_I2C1						(37)
#define IRQ_UART0						(38)
#define IRQ_UART1						(39)


#define IRQ_UART2						(40)
#define IRQ_UART3						(41)
// NULL									(42)
#define IRQ_ADC							(43)



#define A33G52x_IRQ_NUM				(44)



//##########################################################################
//#
//#		Priority 
//#
//##########################################################################
//-----------------------------------------------------------------------------------
// Priority Mask
//
//				AC33M8128 supports only the upper three bits, whis is the most
//				popular scheme.
//
//-----------------------------------------------------------------------------------
#define START_BITPOS_OF_PRIORITY				(5)
#define PRIORITY_MASK							(0x07<<5)


//-----------------------------------------------------------------------------------
// PRIORITY_GROUP_VALUE = 4
//
//				preemption priority					bit 7-5
//				subpriority						none 
//
//
// PRIORITY_GROUP_VALUE = 5
//
//				preemption priority					bit 7-6
//				subpriority						bit 5 
//
//
// PRIORITY_GROUP_VALUE = 6
//
//				preemption priority					bit 7
//				subpriority						bit 6-5 
//
//
// PRIORITY_GROUP_VALUE = 7
//
//				preemption priority					none
//				subpriority						bit 7-5 
//
//-----------------------------------------------------------------------------------
//#define PRIORITY_GROUP_VALUE					(4)
//#define PREEMPTION_MASK						(0x07<<5)
//#define SUBPRIORITY_MASK						(0x00<<5)


//#define PRIORITY_GROUP_VALUE					(5)
//#define PREEMPTION_MASK						(0x03<<6)
//#define SUBPRIORITY_MASK						(0x01<<5)


#define PRIORITY_GROUP_VALUE					(6)
#define PREEMPTION_MASK							(0x01<<7)
#define SUBPRIORITY_MASK						(0x03<<5)


//#define PRIORITY_GROUP_VALUE					(7)
//#define PREEMPTION_MASK						(0x00<<8)
//#define SUBPRIORITY_MASK						(0x07<<5)



//##########################################################################
//#
//#		Interrupt Enable/Disable 
//#
//##########################################################################
#define INTR_ENABLE								(1)
#define INTR_DISABLE								(0)

#define EXCEPTION_ENABLE						(1)
#define EXCEPTION_DISABLE						(0)



//==========================================================================
//#define PRIORITY_GROUP_VALUE			(6)
//#define PREEMPTION_MASK				(0x01<<7)
//#define SUBPRIORITY_MASK				(0x03<<5)
//==========================================================================
// IRQ0 - LVDFAIL 
#define PRIO_LVDFAIL_PREEMPTION			(1)
#define PRIO_LVDFAIL_SUBPRIORITY		(3)

// IRQ1 - MXOSCFAIL
#define PRIO_MXOSCFAIL_PREEMPTION		(1)
#define PRIO_MXOSCFAIL_SUBPRIORITY		(3)

// IRQ2 - SXOSCFAIL
#define PRIO_SXOSCFAIL_PREEMPTION		(1)
#define PRIO_SXOSCFAIL_SUBPRIORITY		(3)


// IRQ3 - WDT 
#define PRIO_WDT_PREEMPTION				(1)
#define PRIO_WDT_SUBPRIORITY			(3)


// IRQ4 - FRT
#define PRIO_FRT_PREEMPTION				(1)
#define PRIO_FRT_SUBPRIORITY			(3)


// IRQ5 - TIMER 0
#define PRIO_TIMER0_PREEMPTION			(1)
#define PRIO_TIMER0_SUBPRIORITY			(3)

// IRQ6 - TIMER 1
#define PRIO_TIMER1_PREEMPTION			(1)
#define PRIO_TIMER1_SUBPRIORITY			(3)

// IRQ7 - TIMER 2
#define PRIO_TIMER2_PREEMPTION			(1)
#define PRIO_TIMER2_SUBPRIORITY			(3)

// IRQ8 - TIMER 3
#define PRIO_TIMER3_PREEMPTION			(1)
#define PRIO_TIMER3_SUBPRIORITY			(3)


// IRQ9 - TIMER 4
#define PRIO_TIMER4_PREEMPTION			(1)
#define PRIO_TIMER4_SUBPRIORITY			(3)


// IRQ10 - TIMER 5 
#define PRIO_TIMER5_PREEMPTION			(1)
#define PRIO_TIMER5_SUBPRIORITY			(3)


// IRQ11 - TIMER 6
#define PRIO_TIMER6_PREEMPTION			(1)
#define PRIO_TIMER6_SUBPRIORITY			(3)


// IRQ12 - TIMER 7
#define PRIO_TIMER7_PREEMPTION			(1)
#define PRIO_TIMER7_SUBPRIORITY			(3)


// IRQ13 - TIMER 8
#define PRIO_TIMER8_PREEMPTION			(1)
#define PRIO_TIMER8_SUBPRIORITY			(3)


// IRQ14 - TIMER 9
#define PRIO_TIMER9_PREEMPTION			(1)
#define PRIO_TIMER9_SUBPRIORITY			(3)



// IRQ15 - MCKFAIL
#define PRIO_MCKFAIL_PREEMPTION			(1)
#define PRIO_MCKFAIL_SUBPRIORITY			(3)


// IRQ16 - GPIOA
#define PRIO_GPIOA_PREEMPTION			(1)
#define PRIO_GPIOA_SUBPRIORITY			(3)


// IRQ17 - GPIOB
#define PRIO_GPIOB_PREEMPTION			(1)
#define PRIO_GPIOB_SUBPRIORITY			(3)


// IRQ18 - GPIOC
#define PRIO_GPIOC_PREEMPTION			(1)
#define PRIO_GPIOC_SUBPRIORITY			(3)


// IRQ19 - GPIOD
#define PRIO_GPIOD_PREEMPTION			(1)
#define PRIO_GPIOD_SUBPRIORITY			(3)


// IRQ20 - GPIOE
#define PRIO_GPIOE_PREEMPTION			(1)
#define PRIO_GPIOE_SUBPRIORITY			(3)


// IRQ21 - GPIOF
#define PRIO_GPIOF_PREEMPTION			(1)
#define PRIO_GPIOF_SUBPRIORITY			(3)


// IRQ22 - NULL

// IRQ23 - NULL


// IRQ24 - PWM0
#define PRIO_PWM0_PREEMPTION			(1)
#define PRIO_PWM0_SUBPRIORITY			(3)


// IRQ25 - PWM1
#define PRIO_PWM1_PREEMPTION			(1)
#define PRIO_PWM1_SUBPRIORITY			(3)


// IRQ26 - PWM2
#define PRIO_PWM2_PREEMPTION			(1)
#define PRIO_PWM2_SUBPRIORITY			(3)


// IRQ27 - PWM3
#define PRIO_PWM3_PREEMPTION			(1)
#define PRIO_PWM3_SUBPRIORITY			(3)

// IRQ28 - PWM4
#define PRIO_PWM4_PREEMPTION			(1)
#define PRIO_PWM4_SUBPRIORITY			(3)


// IRQ29 - PWM5
#define PRIO_PWM5_PREEMPTION			(1)
#define PRIO_PWM5_SUBPRIORITY			(3)


// IRQ30 - PWM6
#define PRIO_PWM6_PREEMPTION			(1)
#define PRIO_PWM6_SUBPRIORITY			(3)


// IRQ31 - PWM7
#define PRIO_PWM7_PREEMPTION			(1)
#define PRIO_PWM7_SUBPRIORITY			(3)


// IRQ32 - SPI0
#define PRIO_SPI0_PREEMPTION			(1)
#define PRIO_SPI0_SUBPRIORITY			(3)


// IRQ33 - SPI1
#define PRIO_SPI1_PREEMPTION			(1)
#define PRIO_SPI1_SUBPRIORITY			(3)



// IRQ34 - NULL

// IRQ35 - NULL



// IRQ36 - I2C0
#define PRIO_I2C0_PREEMPTION			(1)
#define PRIO_I2C0_SUBPRIORITY			(3)


// IRQ37 - I2C1
#define PRIO_I2C1_PREEMPTION			(1)
#define PRIO_I2C1_SUBPRIORITY			(3)



// IRQ38 - UART0
#define PRIO_UART0_PREEMPTION			(1)
#define PRIO_UART0_SUBPRIORITY			(3)


// IRQ39 - UART1
#define PRIO_UART1_PREEMPTION			(1)
#define PRIO_UART1_SUBPRIORITY			(3)


// IRQ40 - UART2
#define PRIO_UART2_PREEMPTION			(1)
#define PRIO_UART2_SUBPRIORITY			(3)



// IRQ41 - UART3
#define PRIO_UART3_PREEMPTION			(1)
#define PRIO_UART3_SUBPRIORITY			(3)



// IRQ42 - NULL



// IRQ43 - ADC
#define PRIO_ADC_PREEMPTION				(1)
#define PRIO_ADC_SUBPRIORITY			(3)









//##########################################################################
//#
//#		SYSTEM CONTROL SPACE 
//#
//##########################################################################

//==========================================================================
// ICTR (Interrupt Control Type Register)
//
//					addr = 0xE000_E004
//
//==========================================================================
//
//					bit 4-0		INTLINESUM	
//								00000b = 0~32
//								00001b = 33~64
//								
//								00111b = 225~256
//
//==========================================================================
#define ICTR_INTLINESUM						(0x001F<<0)



//==========================================================================
// ACR (Auxiliary Control Register)
//
//					addr = 0xE000_E008
//
//==========================================================================
//
//					bit 2			DISFOLD
//								disables IT folding
//
//					bit 1			DISDEFWBUF
//								disables write buffer use during default memory map accesses
//
//					bit 0			DISMCYCINT
//								disables interruption of multi-cycle instructions. 
//
//==========================================================================
#define ACR_DISFOLD							(0x0001<<2)
#define ACR_DISDEFWBUF						(0x0001<<1)
#define ACR_DISMCYCINT						(0x0001<<0)





//##########################################################################
//#
//#		SYSTICK 
//#
//##########################################################################

//==========================================================================
// CSR (SysTick Control and Status Register)
//
//					addr = 0xE000_E010
//
//==========================================================================
//
//					bit 16		COUNTFLAG
//								1 = timer counted down to 0 since last read time
//
//					bit 2			CLKSOURCE
//								0 = external reference clock 
//								1 = core clock 
//
//					bit 1			TICKINT 
//								0 = disable interrupt 
//								1 = enable interrupt 
//
//					bit 0			ENABLE
//								0 = disable counter
//								1 = enable counter 
//
//==========================================================================
#define CSR_COUNTFLAG					(0x0001<<16)

#define CSR_CLKSOURCE 					(0x0001<<2)
#define CSR_CLKSOURCE_EXTREF_CLOCK		(0x0000<<2)
#define CSR_CLKSOURCE_CORE_CLOCK		(0x0001<<2)
#define CSR_TICKINT						(0x0001<<1)
#define CSR_ENABLE						(0x0001<<0)




//==========================================================================
// RVR (SysTick Reload Value Register)
//
//					addr = 0xE000_E014
//
//==========================================================================
//
//					bit 23-0		RELOAD 
//								
//==========================================================================
#define RVR_RELOAD_MASK					(0x00FFFFFF<<0)




//==========================================================================
// CVR (SysTick Current Value Register)
//
//					addr = 0xE000_E018
//
//==========================================================================
//
//					bit 23-0		CURRENT
//								Write to this register with any value to clear the register and
//								the COUNTFLAG to 0. (This does not cause SysTick exception.)
//
//
//==========================================================================
#define CVR_CURRENT_MASK				(0x00FFFFFF<<0)




//==========================================================================
// CALIB (SysTick Calibration Value Register)
//
//					addr = 0xE000_E01C
//
//==========================================================================
//
//					bit 31		NOREF
//								0 = external reference clock is available
//								1 = external reference clock is not available 
//
//					bit 30		SKEW
//								0 = the TENMS bit field is accurate
//								1 = the TENMS bit field is not accurate
//
//					bit 23-0		TENMS
//								Ten milisecond calibration value 
//
//==========================================================================
#define CALIB_NOREF						(0x0001UL<<31)
#define CALIB_SKEW						(0x0001UL<<30)
#define CALIB_TENMS_MASK				(0x00FFFFFFUL<<0)




//==========================================================================
// SysTick Parameters
//
//
//==========================================================================
#define ST_CORE_CLOCK						(0)
#define ST_EXTREF_CLOCK						(1)

#define ST_INTR_TICK						CSR_TICKINT




//##########################################################################
//#
//#		NVIC
//#
//##########################################################################

//==========================================================================
// ISER (Interrupt Set Enable Register)
//
//					addr = 0xE000_E100 ~ 0xE000_E11F
//
//==========================================================================



//==========================================================================
// ICER (Interrupt Clear Enable Register)
//
//					addr = 0xE000_E180 ~ 0xE000_E19F
//
//==========================================================================



//==========================================================================
// ISPR (Interrupt Set Pending Register)
//
//					addr = 0xE000_E200 ~ 0xE000_E21F
//
//==========================================================================



//==========================================================================
// ICPR (Interrupt Clear Pending Register)
//
//					addr = 0xE000_E280 ~ 0xE000_E29F
//
//==========================================================================



//==========================================================================
// IABR (Interrupt Active Bit Register)
//
//					addr = 0xE000_E300 ~ 0xE000_E31F
//
//==========================================================================



//==========================================================================
// IPR (Interrupt Priority Register)
//
//					addr = 0xE000_E400 ~ 0xE000_E4EF
//
//==========================================================================



//##########################################################################
//#
//#		SCB 
//#
//##########################################################################

//==========================================================================
// CPUID (CPUID Base Register)
//
//					addr = 0xE000_ED00
//
//==========================================================================
//
//					bit 31-24		IMPLEMENTER
//								0x41 = ARM
//
//					bit 23-20		VARIANT
//
//					bit 19-16		CONSTATNT
//								0x0F = fixed 
//
//					bit 15-4		PARTNO
//								[11:10]	11b 		= Cortex family
//								[9:8]	00b 		= version 
//								[7:6]	00b 		= reserved
//								[5:4]	10b 		= M
//								[3:0]	0011b	= Cortex-M3 family
//
//					bit 3-0		REVISION
//
//==========================================================================



//==========================================================================
// ICSR (Interrupt Control State Register)
//
//					addr = 0xE000_ED04
//
//==========================================================================
//
//					bit 31		NMIPENDSET (RW)
//								0 = do not set pending NMI
//								1 = set pending NMI
//
//
//					bit 28		PENDSVSET (RW)
//								0 = do not set pending PendSV
//								1 = set pending PendSV
//
//					bit 27		PENDSVCLR (WO)
//								0 = do not clear pending PendSV
//								1 = clear pending PendSV
//
//
//					bit 26		PENDSTSET (RW)
//								0 = do not set pending SysTick
//								1 = set pending SysTick
//
//					bit 25		PENDSTCLR (WO)
//								0 = do not clear pending SysTick
//								1 = clear pending SysTick 
//
//
//					bit 23		ISRPREEMPT (RO)
//								
//					bit 22		ISRPENDING (RO)
//								interrupt pending flags, except NMI and Faults
//								0 = interrupt not pending
//								1 = interrupt pending 
//
//
//					bit 21-12		VECTPENDING (RO)
//								the interrupt number of the highest priority pending ISR 
//
//					bit 11		RETTOBASE (RO)
//								
//
//					bit 8-0		VECTACTIVE (RO)
//								active ISR number field 
//
//==========================================================================
#define ICSR_NMIPENDSET					(0x0001UL<<31)

#define ICSR_PENDSVSET					(0x0001UL<<28)
#define ICSR_PENDSVCLR					(0x0001UL<<27)

#define ICSR_PENDSTSET					(0x0001UL<<26)
#define ICSR_PENDSTCLR					(0x0001UL<<25)

#define ICSR_ISRPREEMPT					(0x0001UL<<23)
#define ICSR_ISRPENDING					(0x0001UL<<22)

#define ICSR_VECTPENDING				(0x03FFUL<<12)

#define ICSR_RETTOBASE					(0x0001UL<<11)
#define ICSR_VECTACTIVE					(0x01FFUL<<0)




//==========================================================================
// VTOR (Vector Table Offset Register)
//
//					addr = 0xE000_ED08
//
//==========================================================================
//
//					bit 29		TBLBASE
//								0 = Code
//								1 = RAM 
//
//					bit 28-7		TBLOFF
//
//==========================================================================
#define VTOR__MASK						(0x3FFFFF80<<0)





//==========================================================================
// AIRCR (Application Interrupt and Reset Control Register)
//
//					addr = 0xE000_ED0C
//
//==========================================================================
//
//					bit 31-16		VECTKEY/VECTKEYSTAT
//								0x05FA	= VECTKEY
//								0xFA05	= VECTKEYSTAT
//
//					bit 15		ENDIANESS
//								0 = little endian
//								1 = big endian
//
//					bit 10-8		PRIGROUP
//
//					bit 2			SYSRESETREQ
//								causes a singal to be asserted to the outer system that indicates a reset is
//								requested. 
//
//					bit 1			VECTCLRACTIVE (self-clear)
//								clear active vecotor bit
//								0 = do not clear
//								1 = clear all state information for active NMI, Fault, and interrupts 
//
//					bit 0			VECTRESET (self-clear)
//								0 = do not reset system 
//								1 = reset system
//
//==========================================================================
#define AIRCR_VECTKEY					(0x05FAUL<<16)
#define AIRCR_VECTKEYSTAT				(0xFA05UL<<16)
#define AIRCR_VECTKEY_MASK				(0xFFFFUL<<16)

#define AIRCR_ENDIANESS_LITTLE_ENDIAN	(0x0000<<15)
#define AIRCR_ENDIANESS_BIG_ENDIAN		(0x0001<<15)

#define AIRCR_PRIGROUP_MASK				(0x0007<<8)

#define AIRCR_SYSRESETREQ				(0x0001<<2)
#define AIRCR_VECTCLRACTIVE				(0x0001<<1)
#define AIRCR_VECTRESET					(0x0001<<0)




//==========================================================================
// SCR (System Control Register)
//
//					addr = 0xE000_ED10
//
//==========================================================================
//
//					bit 4			SEVONPEND
//								
//					bit 2			SLEEPDEEP
//
//					bit 1			SLEEPONEXIT
//
//==========================================================================
#define SCR_SEVONPEND					(0x0001<<4)
#define SCR_SLEEPDEEP					(0x0001<<2)
#define SCR_SLEEPONEXIT					(0x0001<<1)




//==========================================================================
// CCR (Configuration Control Register)
//
//					addr = 0xE000_ED14
//
//==========================================================================
//
//					bit 9			STKALIGN
//								
//					bit 8			BFHFNMIGN
//
//					bit 4			DIV_0_TRP
//
//					bit 3			UNALIGN_TRP
//
//					bit 1			USERSETMPEND
//
//					bit 0			NONEBASETHRDENA
//
//==========================================================================
#define CCR_STKALIGN					(0x0001<<9)
#define CCR_BFHFNMIGN					(0x0001<<8)
#define CCR_DIV_0_TRP					(0x0001<<4)
#define CCR_UNALIGN_TRP					(0x0001<<3)
#define CCR_USERSETMPEND				(0x0001<<1)
#define CCR_NONEBASETHRDENA				(0x0001<<0)



//==========================================================================
// SHPR 0 (System Handler Priority Register 0)
//
//					addr = 0xE000_ED18
//
//==========================================================================
//
//					bit 31-24		PRI_7
//								
//					bit 23-16		PRI_6		Usgae Fault
//
//					bit 15-8		PRI_5		Bus Fault
//
//					bit 7-0		PRI_4		Memory Manage
//
//==========================================================================
#define PRIO_SHIFT_USAGE_FAULT		(16)
#define PRIO_SHIFT_BUS_FAULT		(8)
#define PRIO_SHIFT_MEMORY_MANAGE	(0)




//==========================================================================
// SHPR 1 (System Handler Priority Register 1)
//
//					addr = 0xE000_ED1C
//
//==========================================================================
//
//					bit 31-24		PRI_11		SVCALL
//								
//					bit 23-16		PRI_10
//
//					bit 15-8		PRI_9
//
//					bit 7-0		PRI_8
//
//==========================================================================
#define PRIO_SHIFT_SVCALL			(24)




//==========================================================================
// SHPR 2 (System Handler Priority Register 2)
//
//					addr = 0xE000_ED20
//
//==========================================================================
//
//					bit 31-24		PRI_15		SysTick
//								
//					bit 23-16		PRI_14		PendSV
//
//					bit 15-8		PRI_13
//
//					bit 7-0		PRI_12		Debug Monitor
//
//==========================================================================
#define PRIO_SHIFT_SYSTICK					(24)
#define PRIO_SHIFT_PENDSV					(16)
#define PRIO_SHIFT_DEBUG_MONITOR			(0)




//==========================================================================
// SHCSR (System Handler Control and State Register)
//
//					addr = 0xE000_ED24
//
//==========================================================================
//
//					bit 18		USGFAULTENA
//								0 = disable
//								1 = enable 
//								
//					bit 17		BUSFAULTENA
//								0 = disable
//								1 = enable 
//
//					bit 16		MEMFAULTENA
//								0 = disable
//								1 = enable 
//
//					bit 15		SVCALLPENDED
//
//					bit 14		BUSFAULTPENDED
//
//					bit 13		MEMFAULTPENDED
//
//					bit 12		USGFAULTPENDED
//
//					bit 11		SYSTICKACT
//
//					bit 10		PENDSVACT
//
//					bit 8			MONITORACT
//			
//					bit 7			SVCALLACT
//
//					bit 3			USGFAULTACT
//
//					bit 1			BUSFAULTACT
//
//					bit 0			MEMFAULTACT
//
//==========================================================================
#define SHCSR_USGFAULTENA				(0x0001<<18)
#define SHCSR_BUSFAULTENA				(0x0001<<17)
#define SHCSR_MEMFAULTENA				(0x0001<<16)

#define SHCSR_SVCALLPENDED				(0x0001<<15)
#define SHCSR_BUSFAULTPENDED			(0x0001<<14)
#define SHCSR_MEMFAULTPENDED			(0x0001<<13)
#define SHCSR_USGFAULTPENDED			(0x0001<<12)

#define SHCSR_SYSTICKACT				(0x0001<<11)
#define SHCSR_PENDSVACT					(0x0001<<10)
#define SHCSR_MONITORACT				(0x0001<<8)

#define SHCSR_USGFAULTACT				(0x0001<<3)
#define SHCSR_BUSFAULTACT				(0x0001<<1)
#define SHCSR_MEMFAULTACT				(0x0001<<0)




//==========================================================================
// CFSR (Configurable Fault Status Register)
//
//					addr = 0xE000_ED28
//					access = R/WC
//
//==========================================================================
//
//					bit 31-16		Usage Fault Status Register 
//								
//					bit 15-8		Bus Fault Status Register 
//
//					bit 7-0		Memory Manage Fault Status Register 
//
//==========================================================================
#define CFSR_UFSR_MASK				(0xFFFFUL<<16)
#define CFSR_BFSR_MASK				(0x00FFUL<<8)
#define CFSR_MMFSR_MASK				(0x00FFUL<<0)



//-----------------------------------------------------------------------------------------------
// MMFSR (Memory Manage Fault Status Register)
//
//					addr = 0xE000_ED28 
//
//-----------------------------------------------------------------------------------------------
//
//					bit 7			MMARVALID
//								Memory Manage Address Register (MMAR) address valid flag
//								0 = invalid
//							 	1 = valid
//
//	
//					bit 4			MSTKERR
//								memory stacking error
//
//					bit 3			MUNSTKERR
//								memory unstacking error
//
//					bit 1			DACCVIOL
//								data access violation flag
//
//					bit 0			IACCVIOL
//								instruction access violation flag
//
//-----------------------------------------------------------------------------------------------
#define CFSR_MMFSR_MMARVALID		(0x0001<<(0+7))
#define CFSR_MMFSR_MSTKERR			(0x0001<<(0+4))
#define CFSR_MMFSR_MUNSTKERR		(0x0001<<(0+3))
#define CFSR_MMFSR_DACCVIOL			(0x0001<<(0+1))
#define CFSR_MMFSR_IACCVIOL			(0x0001<<(0+0))




//-----------------------------------------------------------------------------------------------
// BFSR (Bus Fault Status Register)
//
//					addr = 0xE000_ED29
//
//-----------------------------------------------------------------------------------------------
//
//					bit 7			BFARVALID
//								Bus Fault Address Register (BFAR) address valid flag
//								0 = invalid
//							 	1 = valid
//
//	
//					bit 4			STKERR
//								stacking error
//
//					bit 3			UNSTKERR
//								unstacking error
//
//					bit 2			IMPRECISERR
//
//					bit 1			PRECISERR
//
//					bit 0			IBUSERR
//								0 = no instruction bus error
//								1 = instruction bus error 
//
//-----------------------------------------------------------------------------------------------
#define CFSR_BFSR_BFARVALID			(0x0001<<(8+7))
#define CFSR_BFSR_STKERR			(0x0001<<(8+4))

#define CFSR_BFSR_UNSTKERR			(0x0001<<(8+3))
#define CFSR_BFSR_IMPRECISERR		(0x0001<<(8+2))
#define CFSR_BFSR_PRECISERR			(0x0001<<(8+1))
#define CFSR_BFSR_IBUSERR			(0x0001<<(8+0))



//-----------------------------------------------------------------------------------------------
// UFSR (Usage Fault Status Register)
//
//					addr = 0xE000_ED2A
//
//-----------------------------------------------------------------------------------------------
//
//					bit 9			DIVBYZERO
//
//					bit 8 		UNALIGNED
//
//					bit 3			NOCP
//
//					bit 2			INVPC
//
//					bit 1			INVSTATE
//
//					bit 0			UNDEFINSTR
//
//-----------------------------------------------------------------------------------------------
#define CFSR_UFSR_DIVBYZERO			(0x0001UL<<(16+9))
#define CFSR_UFSR_UNALIGNED			(0x0001UL<<(16+8))

#define CFSR_UFSR_NOCP				(0x0001UL<<(16+3))
#define CFSR_UFSR_INVPC				(0x0001UL<<(16+2))
#define CFSR_UFSR_INVSTATE			(0x0001UL<<(16+1))
#define CFSR_UFSR_UNDEFINSTR		(0x0001UL<<(16+0))



//==========================================================================
// HFSR (Hard Fault Status Register)
//
//					addr = 0xE000_ED2C
//					access = R/WC
//
//==========================================================================
//
//					bit 31		DEBUGEVT
//								
//					bit 30		FORCED
//
//					bit 1			VECTTBL
//
//==========================================================================
#define HFSR_DEBUGEVT				(0x0001UL<<31)
#define HFSR_FORCED					(0x0001UL<<30)
#define HFSR_VECTTBL				(0x0001UL<<1)




//==========================================================================
// DFSR (Debug Fault Status Register)
//
//					addr = 0xE000_ED30
//					access = R/WC
//
//==========================================================================
//
//					bit 4			EXTERNAL 
//								0 = EDBGRQ signal not asserted
//								1 = EDBGRQ singal asserted
//								
//					bit 3			VCATCH
//								0 = no vector catch occurred
//								1 = vector catch occurred
//
//					bit 2			DWTTRAP
//								data watchpoint and trace flag
//								0 = no DWT match
//								1 = DWT match
//
//					bit 1			BKPT 
//								0 = no BKPT instruction execution
//								1 = BKPT instruction execution
//
//					bit 0			HALTED 
//								0 = no halt request
//								1 = halt requested by NVIC
//
//==========================================================================
#define DFSR_EXTERNAL				(0x0001<<4)

#define DFSR_VCATCH					(0x0001<<3)
#define DFSR_DWTTRAP				(0x0001<<2)
#define DFSR_BKPT 					(0x0001<<1)
#define DFSR_HALTED 				(0x0001<<0)




//==========================================================================
// MMFAR (Memory Manage Fault Address Register)
//
//					addr = 0xE000_ED34
//
//==========================================================================



//==========================================================================
// BFAR (Bus Fault Address Register)
//
//					addr = 0xE000_ED38
//
//==========================================================================



//==========================================================================
// AFAR (Auxiliary Fault Address Register)
//
//					addr = 0xE000_ED3C
//
//==========================================================================



//==========================================================================
// STIR (Software Trigger Interrupt Register)
//
//					addr = 0xE000_EF00
//
//==========================================================================
#define STIR_INTID_MASK				(0x01FF<<0)

//==========================================================================
// Interrupt Enable/Disable 
//
//					
//
//==========================================================================
#define SYSINT_ENABLE						(1)
#define SYSINT_DISABLE						(0)


//==========================================================================
//
// 	S Y S T I C K    F U N C T I O N S  
//
//==========================================================================
void SysTick_Init (SysTick_Type * const systick, uint32_t clk_src, uint32_t ext_ref, uint32_t ext_div, uint32_t reload); 
void SysTick_Run (SysTick_Type * const systick, uint32_t enable); 
void SysTick_ConfigureInterrupt (SysTick_Type * const systick, uint32_t intr_mask, uint32_t enable);  

void SCB_Set_PriorityGroup (SCB_Type * const scb, uint32_t priority_group); 
void NVIC_ConfigureInterrupt (NVIC_Type * const nvic, NVIC_IntrConfig * nvic_config); 


