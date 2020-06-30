/**************************************************************************//**
 * @file     A31G226_ARMCM0plus.S
 * @brief    CMSIS Core Device Startup File for
 *           ARMCM0plus Device Series
 * @version  V5.00
 * @date     02. March 2016
 ******************************************************************************/
/*
 * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

	.syntax	unified
	.arch	armv6-m

	.section .stack
	.align	3
#ifdef __STACK_SIZE
	.equ	Stack_Size, __STACK_SIZE
#else
	.equ	Stack_Size, 0x00000400
#endif
	.globl	__StackTop
	.globl	__StackLimit
__StackLimit:
	.space	Stack_Size
	.size	__StackLimit, . - __StackLimit
__StackTop:
	.size	__StackTop, . - __StackTop

	.section .heap
	.align	3
#ifdef __HEAP_SIZE
	.equ	Heap_Size, __HEAP_SIZE
#else
	.equ	Heap_Size, 0x00000C00
#endif
	.globl	__HeapBase
	.globl	__HeapLimit
__HeapBase:
	.if	Heap_Size
	.space	Heap_Size
	.endif
	.size	__HeapBase, . - __HeapBase
__HeapLimit:
	.size	__HeapLimit, . - __HeapLimit

	.section .vectors
	.align 2
	.globl	__Vectors
__Vectors:
	.long	__StackTop            /* Top of Stack */
	.long	Reset_Handler         /* Reset Handler */
	.long	NMI_Handler           /* NMI Handler */
	.long	HardFault_Handler     /* Hard Fault Handler */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	SVC_Handler           /* SVCall Handler */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	PendSV_Handler        /* PendSV Handler */
	.long	SysTick_Handler       /* SysTick Handler */

	/* External interrupts */
	.long	LVI_IRQHandler        /*  0:  Watchdog Timer            */
	.long	SYSCLKFAIL_IRQHandler        /*  1:  Real Time Clock           */
	.long	WDT_IRQHandler       /*  2:  Timer0 / Timer1           */
	.long	GPIOAB_IRQHandler       /*  3:  Timer2 / Timer3           */
	.long	GPIOCD_IRQHandler       /*  4:  MCIa                      */
	.long	GPIOE_IRQHandler       /*  5:  MCIb                      */
	.long	GPIOF_IRQHandler      /*  6:  UART0 - DUT FPGA          */
	.long	TIMER10_IRQHandler      /*  7:  UART1 - DUT FPGA          */
	.long	TIMER11_IRQHandler      /*  8:  UART2 - DUT FPGA          */
	.long	TIMER12_IRQHandler      /*  9:  UART4 - not connected     */
	.long	I2C0_IRQHandler       /* 10: AACI / AC97                */
	.long	USART10_IRQHandler       /* 11: CLCD Combined Interrupt    */
	.long	WT_IRQHandler       /* 12: Ethernet                   */
	.long	TIMER30_IRQHandler      /* 13: USB Device                 */
	.long	I2C1_IRQHandler      /* 14: USB Host Controller        */
	.long	TIMER20_IRQHandler      /* 15: Character LCD              */
	.long	TIMER21_IRQHandler    /* 16: Flexray                    */
	.long	USART11_IRQHandler        /* 17: CAN                        */
	.long	ADC_IRQHandler        /* 18: LIN                        */
	.long	UART0_IRQHandler        /* 19: I2C ADC/DAC                */
	.long	UART1_IRQHandler                     /* 20: Reserved                   */
	.long	TIMER13_IRQHandler                     /* 21: Reserved                   */
	.long	TIMER14_IRQHandler                     /* 22: Reserved                   */
	.long	TIMER15_IRQHandler                     /* 23: Reserved                   */
	.long	TIMER16_IRQHandler                     /* 24: Reserved                   */
	.long	I2C2_SPI20_IRQHandler                     /* 25: Reserved                   */
	.long	USART12_13_SPI21_IRQHandler                     /* 26: Reserved                   */
	.long	DAC_IRQHandler                     /* 27: Reserved                   */
	.long	TEMP_SENSOR_IRQHandler   /* 28: Reserved - CPU FPGA CLCD   */
	.long	0                     /* 29: Reserved - CPU FPGA        */
	.long	0      /* 30: UART3    - CPU FPGA        */
	.long	CMP_CRC_IRQHandler        /* 31: SPI Touchscreen - CPU FPGA */

	.size	__Vectors, . - __Vectors

	.text
	.thumb
	.thumb_func
	.align	1
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */


/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__


/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__

	movs	r0, 0


#ifndef __NO_SYSTEM_INIT
	bl	SystemInit
#endif

#ifndef __START
#define __START _start
#endif
	bl	__START

	.pool
	.size	Reset_Handler, . - Reset_Handler

	.align	1
	.thumb_func
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	def_irq_handler	NMI_Handler
	def_irq_handler	HardFault_Handler
	def_irq_handler	SVC_Handler
	def_irq_handler	PendSV_Handler
	def_irq_handler	SysTick_Handler

	def_irq_handler	LVI_IRQHandler
	def_irq_handler	SYSCLKFAIL_IRQHandler
	def_irq_handler	WDT_IRQHandler
	def_irq_handler	GPIOAB_IRQHandler
	def_irq_handler	GPIOCD_IRQHandler
	def_irq_handler	GPIOE_IRQHandler
	def_irq_handler	GPIOF_IRQHandler
	def_irq_handler	TIMER10_IRQHandler
	def_irq_handler	TIMER11_IRQHandler
	def_irq_handler	TIMER12_IRQHandler
	def_irq_handler	I2C0_IRQHandler
	def_irq_handler	USART10_IRQHandler
	def_irq_handler	WT_IRQHandler
	def_irq_handler	TIMER30_IRQHandler
	def_irq_handler	I2C1_IRQHandler
	def_irq_handler	TIMER20_IRQHandler
	def_irq_handler	TIMER21_IRQHandler
	def_irq_handler	USART11_IRQHandler
	def_irq_handler	ADC_IRQHandler
	def_irq_handler	UART0_IRQHandler
	def_irq_handler	UART1_IRQHandler
	def_irq_handler	TIMER13_IRQHandler
	def_irq_handler	TIMER14_IRQHandler
	def_irq_handler	TIMER15_IRQHandler
	def_irq_handler	TIMER16_IRQHandler
	def_irq_handler	I2C2_SPI20_IRQHandler	
	def_irq_handler	USART12_13_SPI21_IRQHandler	
	def_irq_handler	DAC_IRQHandler	
	def_irq_handler	TEMP_SENSOR_IRQHandler
	def_irq_handler	CMP_CRC_IRQHandler			

	.end
