;;/**
;;**********************(C) Copyright 2018 ABOV Semiconductor Co., Ltd *******************
;;* @ File : startup_A33G52x.s
;;* 
;;* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
;;*
;;* @ Version : V1.0
;;*
;;* @ Date : July, 2018
;;*
;;* @ Description 
;;*   ABOV Semiconductor is supplying this software for use with A33G52x
;;*   processor. This software contains the confidential and proprietary information
;;*   of ABOV Semiconductor Co., Ltd ("Confidential Information").
;;*
;;*
;;**************************************************************************************
;;* DISCLAIMER 
;;*
;;* 	THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS  
;;* 	WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE  
;;* 	TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
;;* 	DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING  
;;* 	FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE  
;;* 	CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.  
;;*
;;**************************************************************************************
;;*/


EXCEPTION_NUMBER_IRQ_BASE			EQU			16


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000400

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size


__Vectors       DCD     __initial_sp              	; Top of Stack
                DCD     Reset_Handler             		; Reset Handle
                DCD     NMI_Handler               		; NMI Handle
                DCD     HardFault_Handler         	; Hard Fault Handle
                DCD     MemManage_Handler         	; MPU Fault Handle
                DCD     BusFault_Handler          		; Bus Fault Handle
                DCD     UsageFault_Handler        	; Usage Fault Handle
                DCD     0                         			; Reserved
                DCD     0                         			; Reserved
                DCD     0                         			; Reserved
                DCD     0                         			; Reserved
                DCD     SVC_Handler               		; SVCall Handle
                DCD     DebugMon_Handler         	; Debug Monitor Handle
                DCD     0                         			; Reserved
                DCD     PendSV_Handler            	; PendSV Handle
                DCD     SysTick_Handler           		; SysTick Handle

		;;==============================================================        
       	;; external interrupts
		;;
		;;					consult "2.2 Interrupt Controller" in the user's guide
		;;==============================================================
		;---<16~23>-----------------------------------------------------
		DCD		LVDFAIL_Handler							; IRQ 0
		DCD		MXOSCFAIL_Handler						; IRQ 1
		DCD		SXOSCFAIL_Handler						; IRQ 2
		DCD 		WDT_Handler								; IRQ 3
		DCD		FRT_Handler								; IRQ 4
		DCD		TIMER0_Handler							; IRQ 5
		DCD		TIMER1_Handler							; IRQ 6
		DCD		TIMER2_Handler							; IRQ 7
		;-----------<24~31>------------------------------------------------------------------------
		DCD		TIMER3_Handler 							; IRQ 8
		DCD		TIMER4_Handler							; IRQ 9
		DCD		TIMER5_Handler							; IRQ 10
		DCD		TIMER6_Handler							; IRQ 11
		DCD		TIMER7_Handler 							; IRQ 12
		DCD		TIMER8_Handler							; IRQ 13
		DCD		TIMER9_Handler							; IRQ 14
		DCD		MCKFAIL_Handler 						; IRQ 15
		;-----------<32~39>------------------------------------------------------------------------
		DCD		GPIOA_Handler							; IRQ 16
		DCD		GPIOB_Handler							; IRQ 17
		DCD		GPIOC_Handler 							; IRQ 18
		DCD		GPIOD_Handler 							; IRQ 19
		DCD		GPIOE_Handler 							; IRQ 20
		DCD		GPIOF_Handler 							; IRQ 21
		DCD		NULL_Handler 								; IRQ 22
		DCD		NULL_Handler 								; IRQ 23 
		;-----------<40~47>------------------------------------------------------------------------
		DCD		PWM0_Handler							; IRQ 24
		DCD		PWM1_Handler 							; IRQ 25
		DCD		PWM2_Handler 							; IRQ 26
		DCD		PWM3_Handler 							; IRQ 27
		DCD		PWM4_Handler 							; IRQ 28
		DCD		PWM5_Handler 							; IRQ 29
		DCD		PWM6_Handler 							; IRQ 30
		DCD		PWM7_Handler 							; IRQ 31 
		;-----------<48~55>------------------------------------------------------------------------
		DCD		SPI0_Handler 								; IRQ 32
		DCD		SPI1_Handler 								; IRQ 33
		DCD		NULL_Handler								; IRQ 34 
		DCD		NULL_Handler 								; IRQ 35
		DCD		I2C0_Handler								; IRQ 36
		DCD		I2C1_Handler 								; IRQ 37
		DCD		UART0_Handler 							; IRQ 38
		DCD		UART1_Handler 							; IRQ 39
		;-----------<56~63>------------------------------------------------------------------------
		DCD		UART2_Handler 							; IRQ 40
		DCD		UART3_Handler 							; IRQ 41
		DCD		NULL_Handler 								; IRQ 42
		DCD		ADC_Handler 								; IRQ 43
		DCD		NULL_Handler 								; IRQ 44
		DCD		NULL_Handler 								; IRQ 45
		DCD		NULL_Handler 								; IRQ 46
		DCD		NULL_Handler 								; IRQ 47
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors




                AREA    |.text|, CODE, READONLY
					
; Reset Handler					

Reset_Handler   PROC
						EXPORT  Reset_Handler             [WEAK]  
						;IMPORT  SystemInit
						IMPORT  __main
						
						
						;LDR     R0, =SystemInit
						;BLX     R0					
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

PendSV_Handler  PROC
		EXPORT PendSV_Handler		[WEAK]
		B		.
 		ENDP

SysTick_Handler  PROC
		EXPORT SysTick_Handler		[WEAK]
		B		.
 		ENDP


NULL_Handler PROC
                EXPORT  LVDFAIL_Handler         [WEAK]
                EXPORT  MXOSCFAIL_Handler	[WEAK]
                EXPORT  SXOSCFAIL_Handler 	[WEAK]
                EXPORT  WDT_Handler           	[WEAK]
                EXPORT  FRT_Handler           	[WEAK]
                EXPORT  TIMER0_Handler          [WEAK]
                EXPORT  TIMER1_Handler          [WEAK]              
                EXPORT  TIMER2_Handler          [WEAK]
                EXPORT  TIMER3_Handler          [WEAK]
                EXPORT  TIMER4_Handler          [WEAK]
                EXPORT  TIMER5_Handler          [WEAK]              
                EXPORT  TIMER6_Handler          [WEAK]
                EXPORT  TIMER7_Handler          [WEAK]     
                EXPORT  TIMER8_Handler          [WEAK]
                EXPORT  TIMER9_Handler          [WEAK]                     
                EXPORT  MCKFAIL_Handler		[WEAK]
                EXPORT  GPIOA_Handler           [WEAK]
                EXPORT  GPIOB_Handler           [WEAK]     
                EXPORT  GPIOC_Handler           [WEAK]
                EXPORT  GPIOD_Handler           [WEAK]     
                EXPORT  GPIOE_Handler 		[WEAK]
                EXPORT  GPIOF_Handler  		[WEAK]            
                EXPORT  PWM0_Handler           	[WEAK]
                EXPORT  PWM1_Handler           	[WEAK]       
                EXPORT  PWM2_Handler           	[WEAK]
                EXPORT  PWM3_Handler           	[WEAK]       
                EXPORT  PWM4_Handler           	[WEAK]
                EXPORT  PWM5_Handler           	[WEAK]       
                EXPORT  PWM6_Handler           	[WEAK]
                EXPORT  PWM7_Handler           	[WEAK]    
                EXPORT  SPI0_Handler           	[WEAK]
                EXPORT  SPI1_Handler           	[WEAK]       
                EXPORT  I2C0_Handler           	[WEAK]   
                EXPORT  I2C1_Handler           	[WEAK]                     
                EXPORT  UART0_Handler          	[WEAK]
                EXPORT  UART1_Handler          	[WEAK]
                EXPORT  UART2_Handler          	[WEAK]
                EXPORT  UART3_Handler          	[WEAK]
                EXPORT  ADC_Handler           	[WEAK]


LVDFAIL_Handler
MXOSCFAIL_Handler
SXOSCFAIL_Handler
WDT_Handler
FRT_Handler
TIMER0_Handler
TIMER1_Handler
TIMER2_Handler
TIMER3_Handler
TIMER4_Handler
TIMER5_Handler
TIMER6_Handler
TIMER7_Handler
TIMER8_Handler
TIMER9_Handler
MCKFAIL_Handler 
GPIOA_Handler
GPIOB_Handler
GPIOC_Handler
GPIOD_Handler
GPIOE_Handler
GPIOF_Handler
PWM0_Handler
PWM1_Handler
PWM2_Handler
PWM3_Handler
PWM4_Handler
PWM5_Handler
PWM6_Handler
PWM7_Handler
SPI0_Handler
SPI1_Handler
I2C0_Handler
I2C1_Handler
UART0_Handler
UART1_Handler
UART2_Handler
UART3_Handler
ADC_Handler

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
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


				END
