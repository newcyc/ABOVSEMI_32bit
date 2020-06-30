;/**************************************************************************//**
; * @file     startup_A34M41x.s
; * @brief    CMSIS Cortex-M4 Core Device Startup File for
; *           Device A34M41x
; * @version  V0.10
; * @date     16. Jun 2017
; *
; * @note
; *
; ******************************************************************************/
;/* Copyright (c) 2012 ARM LIMITED
;
;   All rights reserved.
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions are met:
;   - Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;   - Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;   - Neither the name of ARM nor the names of its contributors may be used
;     to endorse or promote products derived from this software without
;     specific prior written permission.
;   *
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
;   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;   POSSIBILITY OF SUCH DAMAGE.
;   ---------------------------------------------------------------------------*/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000600

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

;Heap_Size       EQU     0x00000100
Heap_Size       EQU     0x00000600

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
				DCD		LVI_IRQHandler				;  0: LVI
				DCD		SYSCLKFAIL_IRQHandler		;  1: SYSCLKFAIL
				DCD		HSEFAIL_IRQHandler			;  2: HSEFAIL
				DCD		LSEFAIL_IRQHandler			;  3: LSEFAIL
				DCD		0							;  4: Reserved
				DCD		0							;  5: Reserved
				DCD		WDT_IRQHandler				;  6: WDT
				DCD		0							;  7: Reserved
				DCD		FRT0_IRQHandler				;  8: FRT0
				DCD		FRT1_IRQHandler				;  9: FRT1
				DCD		0							; 10: Reserved
				DCD		CFMC_IRQHandler				; 11: CFMC
				DCD		DFMC_IRQHandler				; 12: DFMC
				DCD		0							; 13: Reserved
				DCD		0							; 14: Reserved
				DCD		TIMER0_IRQHandler			; 15: TIMER0
				DCD		TIMER1_IRQHandler			; 16: TIMER1
				DCD		TIMER2_IRQHandler			; 17: TIMER2
				DCD		TIMER3_IRQHandler			; 18: TIMER3
				DCD		TIMER4_IRQHandler			; 19: TIMER4
				DCD		TIMER5_IRQHandler			; 20: TIMER5
				DCD		TIMER6_IRQHandler			; 21: TIMER6
				DCD		TIMER7_IRQHandler			; 22: TIMER7
				DCD		TIMER8_IRQHandler			; 23: TIMER8
				DCD		TIMER9_IRQHandler			; 24: TIMER9
				DCD		0							; 25: Reserved
				DCD		0							; 26: Reserved
				DCD		RNG_IRQHandler				; 27: RNG
				DCD		AES128_IRQHandler			; 28: ASE128
				DCD		0							; 29: Reserved
				DCD		0 							; 30: Reserved
				DCD		QEI0_IRQHandler				; 31: QEI0
				DCD		QEI1_IRQHandler				; 32: QEI1
				DCD		0							; 33: Reserved
				DCD		0							; 34: Reserved
				DCD		0 							; 35: Reserved
				DCD		GPIOA_IRQHandler			; 36: GPIOA
				DCD		GPIOB_IRQHandler			; 37: GPIOB
				DCD		GPIOC_IRQHandler			; 38: GPIOC
				DCD		GPIOD_IRQHandler			; 39: GPIOD
				DCD		GPIOE_IRQHandler			; 40: GPIOE
				DCD		GPIOF_IRQHandler			; 41: GPIOF
				DCD		GPIOG_IRQHandler			; 42: GPIOG
				DCD		0							; 43: Reserved
				DCD		0							; 44: Reserved
				DCD		MPWM0PROT_IRQHandler		; 45: MPWM0PROT
				DCD		MPWM0OVV_IRQHandler 		; 46: MPWM0OVV
				DCD		MPWM0U_IRQHandler			; 47: MPWM0U
				DCD		MPWM0V_IRQHandler			; 48: MPWM0V
				DCD		MPWM0W_IRQHandler			; 49: MPWM0W
				DCD		MPWM1PROT_IRQHandler		; 50: MPWM1PROT
				DCD		MPWM1OVV_IRQHandler			; 51: MPWM1OVV
				DCD		MPWM1U_IRQHandler			; 52: MPWM1U
				DCD		MPWM1V_IRQHandler			; 53: MPWM1V
				DCD		MPWM1W_IRQHandler			; 54: MPWM1W
				DCD		SPI0_IRQHandler				; 55: SPI0
				DCD		SPI1_IRQHandler				; 56: SPI1
				DCD		SPI2_IRQHandler				; 57: SPI2
				DCD		0							; 58: Reserved
				DCD		0							; 59: Reserved
				DCD		I2C0_IRQHandler				; 60: I2C0
				DCD		I2C1_IRQHandler				; 61: I2C1
				DCD		0 							; 62: Reserved
				DCD		UART0_IRQHandler 			; 63: UART0	
				DCD		UART1_IRQHandler			; 64: UART1	
				DCD		UART2_IRQHandler 			; 65: UART2	
				DCD		UART3_IRQHandler 			; 66: UART3	
				DCD		UART4_IRQHandler 			; 67: UART4	
				DCD		UART5_IRQHandler 			; 68: UART5	
				DCD		0 							; 69: Reserved	
				DCD		0							; 70: Reserved
				DCD		CAN_IRQHandler				; 71: CAN
				DCD		0							; 72: Reserved	
				DCD		0							; 73: Reserved	
				DCD		ADC0_IRQHandler				; 74: ADC0	
				DCD		ADC1_IRQHandler				; 75: ADC1	
				DCD		ADC2_IRQHandler				; 76: ADC2	
				DCD		0							; 77: Reserved	
				DCD		0							; 78: Reserved	
				DCD		COMP0_IRQHandler			; 79: COMP0	
				DCD		COMP1_IRQHandler			; 80: COMP1	
				DCD		COMP2_IRQHandler			; 81: COMP2	
				DCD		COMP3_IRQHandler			; 82: COMP3	
				DCD		0							; 83: Reserved	
				DCD		0							; 84: Reserved	
				DCD		CRC_IRQHandler				; 85: CRC	
				
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

				
;Reset_Handler   PROC
                ;EXPORT  Reset_Handler             [WEAK]
                ;IMPORT  SystemInit
                ;IMPORT  __main
				;LDR     R0, =SystemInit
                ;BLX     R0
                ;LDR     R0, =__main
                ;BX      R0
                ;ENDP

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
				IMPORT  __main
; ---------------------------- FPU Block Enable -------------------------------
				; CPACR is located at address 0xE000ED88
				LDR		R0, =0xE000ED88
				; Read CPACR
				LDR		R1, [R0]
				; Set bits 20-23 to enable CP10 and CP11 coprocessors
				ORR		R1, R1, #(0xF << 20)
				; Write back the modified value to the CPACR
				STR		R1, [R0]	; wait for store to complete
; ------------------------------------------------------------------------------
				LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP
				
NULL_IRQHandler PROC							
                EXPORT  LVI_IRQHandler		        [WEAK]
                EXPORT  SYSCLKFAIL_IRQHandler  	 	[WEAK]
                EXPORT  HSEFAIL_IRQHandler    	 	[WEAK]
                EXPORT  LSEFAIL_IRQHandler        	[WEAK]
                EXPORT  WDT_IRQHandler		        [WEAK]
                EXPORT  FRT0_IRQHandler         	[WEAK]
                EXPORT  FRT1_IRQHandler		        [WEAK]
                EXPORT  CFMC_IRQHandler		        [WEAK]
                EXPORT  DFMC_IRQHandler		        [WEAK]
                EXPORT  TIMER0_IRQHandler           [WEAK]
                EXPORT  TIMER1_IRQHandler           [WEAK]
                EXPORT  TIMER2_IRQHandler           [WEAK]
                EXPORT  TIMER3_IRQHandler           [WEAK]
                EXPORT  TIMER4_IRQHandler           [WEAK]
                EXPORT  TIMER5_IRQHandler           [WEAK]
                EXPORT  TIMER6_IRQHandler           [WEAK]
                EXPORT  TIMER7_IRQHandler           [WEAK]
                EXPORT  TIMER8_IRQHandler           [WEAK]
                EXPORT  TIMER9_IRQHandler           [WEAK]
                EXPORT  RNG_IRQHandler           	[WEAK]
                EXPORT  AES128_IRQHandler           [WEAK]
                EXPORT  QEI0_IRQHandler		        [WEAK]
                EXPORT  QEI1_IRQHandler	      	    [WEAK]
                EXPORT  GPIOA_IRQHandler	        [WEAK]
                EXPORT  GPIOB_IRQHandler	        [WEAK]
                EXPORT  GPIOC_IRQHandler            [WEAK]
                EXPORT  GPIOD_IRQHandler            [WEAK]
                EXPORT  GPIOE_IRQHandler            [WEAK]
                EXPORT  GPIOF_IRQHandler            [WEAK]
                EXPORT  GPIOG_IRQHandler            [WEAK]
                EXPORT  MPWM0PROT_IRQHandler        [WEAK]
                EXPORT  MPWM0OVV_IRQHandler         [WEAK]
                EXPORT  MPWM0U_IRQHandler	        [WEAK]
                EXPORT  MPWM0V_IRQHandler	        [WEAK]
                EXPORT  MPWM0W_IRQHandler	        [WEAK]
                EXPORT  MPWM1PROT_IRQHandler        [WEAK]
                EXPORT  MPWM1OVV_IRQHandler         [WEAK]
                EXPORT  MPWM1U_IRQHandler           [WEAK]
				EXPORT  MPWM1V_IRQHandler           [WEAK]
				EXPORT  MPWM1W_IRQHandler           [WEAK]
                EXPORT  SPI0_IRQHandler          	[WEAK]
                EXPORT  SPI1_IRQHandler           	[WEAK]
                EXPORT  SPI2_IRQHandler         	[WEAK]
                EXPORT  I2C0_IRQHandler         	[WEAK]
                EXPORT  I2C1_IRQHandler         	[WEAK]
                EXPORT  UART0_IRQHandler          	[WEAK]
                EXPORT  UART1_IRQHandler           	[WEAK]
                EXPORT  UART2_IRQHandler           	[WEAK]
                EXPORT  UART3_IRQHandler           	[WEAK]
                EXPORT  UART4_IRQHandler           	[WEAK]
                EXPORT  UART5_IRQHandler           	[WEAK]
                EXPORT  CAN_IRQHandler           	[WEAK]
                EXPORT  ADC0_IRQHandler           	[WEAK]
                EXPORT  ADC1_IRQHandler           	[WEAK]
                EXPORT  ADC2_IRQHandler           	[WEAK]
                EXPORT  COMP0_IRQHandler            [WEAK]
                EXPORT  COMP1_IRQHandler            [WEAK]
                EXPORT  COMP2_IRQHandler            [WEAK]
                EXPORT  COMP3_IRQHandler            [WEAK]
                EXPORT  CRC_IRQHandler              [WEAK]

LVI_IRQHandler
SYSCLKFAIL_IRQHandler
HSEFAIL_IRQHandler
LSEFAIL_IRQHandler
WDT_IRQHandler
FRT0_IRQHandler
FRT1_IRQHandler
CFMC_IRQHandler
DFMC_IRQHandler
TIMER0_IRQHandler
TIMER1_IRQHandler
TIMER2_IRQHandler
TIMER3_IRQHandler
TIMER4_IRQHandler
TIMER5_IRQHandler
TIMER6_IRQHandler
TIMER7_IRQHandler
TIMER8_IRQHandler
TIMER9_IRQHandler
RNG_IRQHandler
AES128_IRQHandler
QEI0_IRQHandler
QEI1_IRQHandler
GPIOA_IRQHandler
GPIOB_IRQHandler
GPIOC_IRQHandler
GPIOD_IRQHandler
GPIOE_IRQHandler
GPIOF_IRQHandler
GPIOG_IRQHandler
MPWM0PROT_IRQHandler
MPWM0OVV_IRQHandler
MPWM0U_IRQHandler
MPWM0V_IRQHandler
MPWM0W_IRQHandler
MPWM1PROT_IRQHandler
MPWM1OVV_IRQHandler
MPWM1U_IRQHandler
MPWM1V_IRQHandler
MPWM1W_IRQHandler
SPI0_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
UART4_IRQHandler
UART5_IRQHandler
CAN_IRQHandler
ADC0_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
COMP0_IRQHandler
COMP1_IRQHandler
COMP2_IRQHandler
COMP3_IRQHandler
CRC_IRQHandler

                B       .
                ENDP

                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
