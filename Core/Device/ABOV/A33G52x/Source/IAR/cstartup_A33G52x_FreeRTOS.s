;/**************************************************************************//**
; * @file     startup_A33G52x.s
; * @brief    CMSIS Core Device Startup File for
; *           ARMCM3 Device Series
; * @version  V1.08
; * @date     7. September 2017
; *
; * @note
; *
; ******************************************************************************/
;/* Copyright (c) 2011 - 2012 ARM LIMITED
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
        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
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
        DCD     vPortSVCHandler           ; SVCall Handler
        DCD     DebugMon_Handler          ; Debug Monitor Handler
        DCD     0                         ; Reserved
        DCD     xPortPendSVHandler        ; PendSV Handler
        DCD     xPortSysTickHandler       ; SysTick Handler

        ; External Interrupts (A33G52x)
        DCD     LVDFAIL_Handler                   ; #0 Low Voltage Detect Fail
        DCD     MXOSCFAIL_Handler                 ; #1 Main XTAL Fail
        DCD     NULL_Handler                      ; #2 Reserved
        DCD     WDT_Handler                       ; #3 Watch dog timer
        DCD     FRT_Handler                       ; #4 Free run timer
        DCD     TIMER0_Handler                    ; #5 16 bit timer 0
        DCD     TIMER1_Handler                    ; #6 16 bit timer 1
        DCD     TIMER2_Handler                    ; #7 16 bit timer 2
        DCD     TIMER3_Handler                    ; #8 16 bit timer 3
        DCD     TIMER4_Handler                    ; #9 16 bit timer 4
        DCD     TIMER5_Handler                    ; #10 16 bit timer 5
        DCD     TIMER6_Handler                    ; #11 16 bit timer 6
        DCD     TIMER7_Handler                    ; #12 16 bit timer 7
        DCD     TIMER8_Handler                    ; #13 16 bit timer 8
        DCD     TIMER9_Handler                    ; #14 16 bit timer 9
        DCD     MCKFAIL_Handler                   ; #15 Reserved
        DCD     GPIOA_Handler                     ; #16 PORT A
        DCD     GPIOB_Handler                     ; #17 PORT B
        DCD     GPIOC_Handler                     ; #18 PORT C
        DCD     GPIOD_Handler                     ; #19 PORT D
        DCD     GPIOE_Handler                     ; #20 PORT E
        DCD     GPIOF_Handler                     ; #21 PORT F
        DCD     NULL_Handler                      ; #22 Reserved
        DCD     NULL_Handler                      ; #23 Reserved
        DCD     PWM0_Handler                      ; #24 PWM 0
        DCD     PWM1_Handler                      ; #25 PWM 1
        DCD     PWM2_Handler                      ; #26 PWM 2
        DCD     PWM3_Handler                      ; #27 PWM 3
        DCD     PWM4_Handler                      ; #28 PWM 4
        DCD     PWM5_Handler                      ; #29 PWM 5
        DCD     PWM6_Handler                      ; #30 PWM 6
        DCD     PWM7_Handler                      ; #31 PWM 7
        DCD     SPI0_Handler                      ; #32 SPI 0
        DCD     SPI1_Handler                      ; #33 SPI 1
        DCD     NULL_Handler                      ; #34 OLD SFC (Serial Flash Controller) --> Spec out
        DCD     NULL_Handler                      ; #35 Reserved
        DCD     I2C0_Handler                      ; #36 I2C 0
        DCD     I2C1_Handler                      ; #37 I2C 1
        DCD     UART0_Handler                     ; #38 UART 0
        DCD     UART1_Handler                     ; #39 UART 1
        DCD     UART2_Handler                     ; #40 UART 2
        DCD     UART3_Handler                     ; #41 UART 3
        DCD     NULL_Handler                      ; #42 Reserved
        DCD     ADC_Handler                       ; #43 ADC
        DCD     NULL_Handler                      ; #44 Reserved
        DCD     NULL_Handler                      ; #45 Reserved
        DCD     NULL_Handler                      ; #46 Reserved
        DCD     NULL_Handler                      ; #47 Reserved
        DCD     NULL_Handler                      ; #48 Reserved
        DCD     NULL_Handler                      ; #49 Reserved
        DCD     NULL_Handler                      ; #50 Reserved
        DCD     NULL_Handler                      ; #51 Reserved
        DCD     NULL_Handler                      ; #52 Reserved
        DCD     NULL_Handler                      ; #53 Reserved
        DCD     NULL_Handler                      ; #54 Reserved
        DCD     NULL_Handler                      ; #55 Reserved
        DCD     NULL_Handler                      ; #56 Reserved
        DCD     NULL_Handler                      ; #57 Reserved
        DCD     NULL_Handler                      ; #58 Reserved
        DCD     NULL_Handler                      ; #59 Reserved
        DCD     NULL_Handler                      ; #60 Reserved
        DCD     NULL_Handler                      ; #61 Reserved
        DCD     NULL_Handler                      ; #62 Reserved
        DCD     NULL_Handler                      ; #63 Reserved


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
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

        PUBWEAK vPortSVCHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
vPortSVCHandler
        B vPortSVCHandler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK xPortPendSVHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
xPortPendSVHandler
        B xPortPendSVHandler

        PUBWEAK xPortSysTickHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
xPortSysTickHandler
        B xPortSysTickHandler

		
; A33G52x Interrupt Handler				
		
        PUBWEAK LVDFAIL_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
LVDFAIL_Handler  
        B LVDFAIL_Handler

        PUBWEAK MXOSCFAIL_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MXOSCFAIL_Handler  
        B MXOSCFAIL_Handler

        PUBWEAK WDT_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)  
WDT_Handler  
        B WDT_Handler

        PUBWEAK FRT_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
FRT_Handler  
        B FRT_Handler

        PUBWEAK TIMER0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER0_Handler  
        B TIMER0_Handler

        PUBWEAK TIMER1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER1_Handler  
        B TIMER1_Handler

        PUBWEAK TIMER2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER2_Handler  
        B TIMER2_Handler

        PUBWEAK TIMER3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER3_Handler  
        B TIMER3_Handler

        PUBWEAK TIMER4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER4_Handler
        B TIMER4_Handler

        PUBWEAK TIMER5_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
TIMER5_Handler  
        B TIMER5_Handler

        PUBWEAK TIMER6_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
TIMER6_Handler  
        B TIMER6_Handler

        PUBWEAK TIMER7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
TIMER7_Handler  
        B TIMER7_Handler

        PUBWEAK TIMER8_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
TIMER8_Handler  
        B TIMER8_Handler

        PUBWEAK TIMER9_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
TIMER9_Handler  
        B TIMER9_Handler

        PUBWEAK MCKFAIL_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
MCKFAIL_Handler  
        B MCKFAIL_Handler

        PUBWEAK GPIOA_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
GPIOA_Handler  
        B GPIOA_Handler

        PUBWEAK GPIOB_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
GPIOB_Handler  
        B GPIOB_Handler

        PUBWEAK GPIOC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
GPIOC_Handler  
        B GPIOC_Handler

        PUBWEAK GPIOD_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOD_Handler  
        B GPIOD_Handler

        PUBWEAK GPIOE_Handler
        SECTION .text:CODE:REORDER:NOROOT(1) 
GPIOE_Handler  
        B GPIOE_Handler

        PUBWEAK GPIOF_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)  
GPIOF_Handler  
        B GPIOF_Handler

        PUBWEAK PWM0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)  
PWM0_Handler  
        B PWM0_Handler

        PUBWEAK PWM1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)  
PWM1_Handler  
        B PWM1_Handler

        PUBWEAK PWM2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1) 
PWM2_Handler  
        B PWM2_Handler

        PUBWEAK PWM3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
PWM3_Handler  
        B PWM3_Handler

        PUBWEAK PWM4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
PWM4_Handler  
        B PWM4_Handler

        PUBWEAK PWM5_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
PWM5_Handler  
        B PWM5_Handler
        
        PUBWEAK PWM6_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)    
PWM6_Handler  
        B PWM6_Handler

        PUBWEAK PWM7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM7_Handler  
        B PWM7_Handler

        PUBWEAK SPI0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI0_Handler  
        B SPI0_Handler

        PUBWEAK SPI1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI1_Handler  
        B SPI1_Handler

        PUBWEAK I2C0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1) 
I2C0_Handler  
        B I2C0_Handler

        PUBWEAK I2C1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1) 
I2C1_Handler  
        B I2C1_Handler

        PUBWEAK UART0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1) 
UART0_Handler  
        B UART0_Handler

        PUBWEAK UART1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1) 
UART1_Handler  
        B UART1_Handler

        PUBWEAK UART2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_Handler  
        B UART2_Handler

        PUBWEAK UART3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART3_Handler  
        B UART3_Handler

        PUBWEAK ADC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC_Handler  
        B ADC_Handler

        PUBWEAK NULL_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NULL_Handler  
        B NULL_Handler

        END