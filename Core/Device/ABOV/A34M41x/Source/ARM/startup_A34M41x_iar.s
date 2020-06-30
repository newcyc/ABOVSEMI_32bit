;/**************************************************************************//**
; * @file     startup_A34M418.s
; * @brief    CMSIS Cortex-M4 Core Device Startup File for
; *           Device A34M418
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
; <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size


       DATA
__vector_table
                DCD     sfe(CSTACK)     						; Top of Stack
                DCD     Reset_Handler             	; Reset Handler
                DCD     NMI_Handler              	; NMI Handler
                DCD     HardFault_Handler         	; Hard Fault Handler
                DCD     MemManage_Handler              	; MPU Fault Handler
                DCD     BusFault_Handler                ; BUS Fault Handler
                DCD     UsageFault_Handler              ; UsageFault Handler
                DCD     0                         	; Reserved
                DCD     0                         	; Reserved
                DCD     0                         	; Reserved
                DCD     0                         	; Reserved
                DCD     SVC_Handler               	; SVCall Handler
                DCD     DebugMon_Handler                ; Debug Monitor Handler
                DCD     0                         	; Reserved
                DCD     PendSV_Handler           		; PendSV Handler
                DCD     SysTick_Handler           	; SysTick Handler

                ; External Interrupts
				DCD		LVI_IRQHandler				;  0: LVI
				DCD		SYSCLKFAIL_IRQHandler		;  1: SYSCLKFAIL
				DCD		XOSCFAIL_IRQHandler			;  2: XOSCFAIL
				DCD		SOSCFAIL_IRQHandler			;  3: SOSCFAIL
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
				DCD		CRC_IRQHandler				
__Vectors_End
__Vectors       EQU   __vector_table
__Vectors_Size  EQU     __Vectors_End - __Vectors

        THUMB
        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)		
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler     
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler     
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

       PUBWEAK LVI_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
LVI_IRQHandler
        B LVI_IRQHandler
        
        PUBWEAK SYSCLKFAIL_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SYSCLKFAIL_IRQHandler
        B SYSCLKFAIL_IRQHandler
    
        PUBWEAK XOSCFAIL_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
XOSCFAIL_IRQHandler
        B XOSCFAIL_IRQHandler
  
        PUBWEAK SOSCFAIL_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SOSCFAIL_IRQHandler
        B SOSCFAIL_IRQHandler

        PUBWEAK WDT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT_IRQHandler
        B WDT_IRQHandler
        
        PUBWEAK FRT0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
FRT0_IRQHandler
        B FRT0_IRQHandler

        PUBWEAK FRT1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
FRT1_IRQHandler
        B FRT1_IRQHandler
        
        PUBWEAK CFMC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CFMC_IRQHandler
        B CFMC_IRQHandler

        PUBWEAK DFMC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DFMC_IRQHandler
        B DFMC_IRQHandler
 
         PUBWEAK TIMER0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER0_IRQHandler
        B TIMER0_IRQHandler
 
          PUBWEAK TIMER1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER1_IRQHandler
        B TIMER1_IRQHandler
        
          PUBWEAK TIMER2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER2_IRQHandler
        B TIMER2_IRQHandler
    
          PUBWEAK TIMER3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER3_IRQHandler
        B TIMER3_IRQHandler
    
          PUBWEAK TIMER4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER4_IRQHandler
        B TIMER4_IRQHandler
        
          PUBWEAK TIMER5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER5_IRQHandler
        B TIMER5_IRQHandler  
          
          PUBWEAK TIMER6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER6_IRQHandler
        B TIMER6_IRQHandler    

          PUBWEAK TIMER7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER7_IRQHandler
        B TIMER7_IRQHandler    

          PUBWEAK TIMER8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER8_IRQHandler
        B TIMER8_IRQHandler 

          PUBWEAK TIMER9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER9_IRQHandler
        B TIMER9_IRQHandler 

          PUBWEAK RNG_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RNG_IRQHandler
        B RNG_IRQHandler 
        
          PUBWEAK AES128_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AES128_IRQHandler
        B AES128_IRQHandler 
        
          PUBWEAK QEI0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QEI0_IRQHandler
        B QEI0_IRQHandler 

         PUBWEAK QEI1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QEI1_IRQHandler
        B QEI1_IRQHandler

         PUBWEAK GPIOA_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA_IRQHandler
        B GPIOA_IRQHandler

         PUBWEAK GPIOB_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOB_IRQHandler
        B GPIOB_IRQHandler

         PUBWEAK GPIOC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOC_IRQHandler
        B GPIOC_IRQHandler
        
         PUBWEAK GPIOD_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOD_IRQHandler
        B GPIOD_IRQHandler
        
         PUBWEAK GPIOE_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOE_IRQHandler
        B GPIOE_IRQHandler

         PUBWEAK GPIOF_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOF_IRQHandler
        B GPIOF_IRQHandler

         PUBWEAK GPIOG_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOG_IRQHandler
        B GPIOG_IRQHandler
        
         PUBWEAK MPWM0PROT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM0PROT_IRQHandler
        B MPWM0PROT_IRQHandler
        
         PUBWEAK MPWM0OVV_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM0OVV_IRQHandler
        B MPWM0OVV_IRQHandler
        
         PUBWEAK MPWM0U_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM0U_IRQHandler
        B MPWM0U_IRQHandler
        
         PUBWEAK MPWM0V_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM0V_IRQHandler
        B MPWM0V_IRQHandler
        
         PUBWEAK MPWM0W_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM0W_IRQHandler
        B MPWM0W_IRQHandler

         PUBWEAK MPWM1PROT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM1PROT_IRQHandler
        B MPWM1PROT_IRQHandler
        
         PUBWEAK MPWM1OVV_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM1OVV_IRQHandler
        B MPWM1OVV_IRQHandler
        
         PUBWEAK MPWM1U_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM1U_IRQHandler
        B MPWM1U_IRQHandler
        
         PUBWEAK MPWM1V_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM1V_IRQHandler
        B MPWM1V_IRQHandler
        
         PUBWEAK MPWM1W_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPWM1W_IRQHandler
        B MPWM1W_IRQHandler
        
         PUBWEAK SPI0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI0_IRQHandler
        B SPI0_IRQHandler
        
         PUBWEAK SPI1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI1_IRQHandler
        B SPI1_IRQHandler
        
         PUBWEAK SPI2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI2_IRQHandler
        B SPI2_IRQHandler
        
         PUBWEAK I2C0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C0_IRQHandler
        B I2C0_IRQHandler

         PUBWEAK I2C1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_IRQHandler
        B I2C1_IRQHandler

         PUBWEAK UART0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART0_IRQHandler
        B UART0_IRQHandler

         PUBWEAK UART1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_IRQHandler
        B UART1_IRQHandler
        
         PUBWEAK UART2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_IRQHandler
        B UART2_IRQHandler

         PUBWEAK UART3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART3_IRQHandler
        B UART3_IRQHandler
        
         PUBWEAK UART4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART4_IRQHandler
        B UART4_IRQHandler
        
         PUBWEAK UART5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART5_IRQHandler
        B UART5_IRQHandler

         PUBWEAK CAN_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN_IRQHandler
        B CAN_IRQHandler

         PUBWEAK ADC0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC0_IRQHandler
        B ADC0_IRQHandler

         PUBWEAK ADC1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC1_IRQHandler
        B ADC1_IRQHandler

         PUBWEAK ADC2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC2_IRQHandler
        B ADC2_IRQHandler

         PUBWEAK COMP0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP0_IRQHandler
        B COMP0_IRQHandler
        
         PUBWEAK COMP1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP1_IRQHandler
        B COMP1_IRQHandler
        
         PUBWEAK COMP2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP2_IRQHandler
        B COMP2_IRQHandler
        
         PUBWEAK COMP3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP3_IRQHandler
        B COMP3_IRQHandler
        
        PUBWEAK CRC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CRC_IRQHandler
        B CRC_IRQHandler
        END