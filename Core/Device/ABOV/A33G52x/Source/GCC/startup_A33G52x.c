/**
 ******************************************************************************
 * @file      startup_a33g52x.c
 * @author    Coocox
 * @version   V1.0
 * @date      03/28/2016
 * @brief     A33G52x Devices Startup code.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the vector table entries with the exceptions ISR address
 *                - Initialize data and bss
 *                - Setup the microcontroller system.
 *                - Call the application's entry point.
 *            After Reset the Cortex-M3 processor is in Thread mode,
 *            priority is Privileged, and the Stack is set to Main.
 *******************************************************************************
 */
 

/*----------Stack Configuration-----------------------------------------------*/  
#define STACK_SIZE       0x00000400      /*!< The Stack size suggest using even number   */
__attribute__ ((section(".co_stack")))
unsigned long pulStack[STACK_SIZE];      


/*----------Macro definition--------------------------------------------------*/  
#define WEAK __attribute__ ((weak))           


/*----------Declaration of the default fault handlers-------------------------*/  
/* System exception vector handler */
__attribute__ ((used))
void WEAK  Reset_Handler(void);   
void WEAK  NMI_Handler(void);       
void WEAK  HardFault_Handler(void); 
void WEAK  MemManage_Handler(void); 
void WEAK  BusFault_Handler(void);  
void WEAK  UsageFault_Handler(void);
void WEAK  SVC_Handler(void);       
void WEAK  DebugMon_Handler(void);  
void WEAK  PendSV_Handler(void);    
void WEAK  SysTick_Handler(void);   

void WEAK  LVDFAIL_Handler(void);
void WEAK  MXOSCFAIL_Handler(void);
void WEAK  SXOSCFAIL_Handler(void);
void WEAK  WDT_Handler(void);
void WEAK  FRT_Handler(void);
void WEAK  TIMER0_Handler(void);
void WEAK  TIMER1_Handler(void);
void WEAK  TIMER2_Handler(void);
void WEAK  TIMER3_Handler(void);
void WEAK  TIMER4_Handler(void);
void WEAK  TIMER5_Handler(void);
void WEAK  TIMER6_Handler(void);
void WEAK  TIMER7_Handler(void);
void WEAK  TIMER8_Handler(void);
void WEAK  TIMER9_Handler(void);
void WEAK  MCKFAIL_Handler(void);
void WEAK  GPIOA_Handler(void);
void WEAK  GPIOB_Handler(void);
void WEAK  GPIOC_Handler(void);
void WEAK  GPIOD_Handler(void);
void WEAK  GPIOE_Handler(void);
void WEAK  GPIOF_Handler(void);
void WEAK  NULL_Handler(void);
void WEAK  NULL_Handler(void);
void WEAK  PWM0_Handler(void);
void WEAK  PWM1_Handler(void);
void WEAK  PWM2_Handler(void);
void WEAK  PWM3_Handler(void);
void WEAK  PWM4_Handler(void);
void WEAK  PWM5_Handler(void);
void WEAK  PWM6_Handler(void);
void WEAK  PWM7_Handler(void);
void WEAK  SPI0_Handler(void);
void WEAK  SPI1_Handler(void);
void WEAK  NULL_Handler(void);
void WEAK  NULL_Handler(void);
void WEAK  I2C0_Handler(void);
void WEAK  I2C1_Handler(void);
void WEAK  UART0_Handler(void);
void WEAK  UART1_Handler(void);
void WEAK  UART2_Handler(void);
void WEAK  UART3_Handler(void);
void WEAK  NULL_Handler(void);
void WEAK  ADC_Handler(void);

/*----------Symbols defined in linker script----------------------------------*/  
extern unsigned long _sidata;    /*!< Start address for the initialization 
                                      values of the .data section.            */
extern unsigned long _sdata;     /*!< Start address for the .data section     */    
extern unsigned long _edata;     /*!< End address for the .data section       */    
extern unsigned long _sbss;      /*!< Start address for the .bss section      */
extern unsigned long _ebss;      /*!< End address for the .bss section        */      
extern void _eram;               /*!< End address for ram                     */


/*----------Function prototypes-----------------------------------------------*/  
extern int main(void);           /*!< The entry point for the application.    */
extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */
void Default_Reset_Handler(void);   /*!< Default reset handler                */
static void Default_Handler(void);  /*!< Default exception handler            */


/**
  *@brief The minimal vector table for a Cortex M3.  Note that the proper constructs
  *       must be placed on this to ensure that it ends up at physical address
  *       0x00000000.  
  */
__attribute__ ((used,section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{	
  /*----------Core Exceptions------------------------------------------------ */
  (void *)&pulStack[STACK_SIZE],     /*!< The initial stack pointer                            */
  Reset_Handler,                                /*!< Reset Handler                            */
  NMI_Handler,                                   /*!< NMI Handler                            */
  HardFault_Handler,                          /*!< Hard Fault Handler                            */
  MemManage_Handler,                    /*!< MPU Fault Handler                            */
  BusFault_Handler,                           /*!< Bus Fault Handler                            */
  UsageFault_Handler,                       /*!< Usage Fault Handler                            */
  0,0,0,0,                                            /*!< Reserved                            */
  SVC_Handler,                                 /*!< SVCall Handler                            */
  DebugMon_Handler,                      /*!< Debug Monitor Handler                            */
  0,                                                   /*!< Reserved                            */
  PendSV_Handler,                          /*!< PendSV Handler                            */
  SysTick_Handler,                          /*!< SysTick Handler                            */
  
  /*----------External Exceptions---------------------------------------------*/
  LVDFAIL_Handler,              /*!<  0: LVDDETECT                            */
  MXOSCFAIL_Handler,            /*!<  1: SYSCLKFAIL                            */
  SXOSCFAIL_Handler,            /*!<  2: XOSCFAIL                            */
  WDT_Handler,                  /*!<  3: WDT                            */
  FRT_Handler,                  /*!<  4: Default                            */
  TIMER0_Handler,               /*!<  5: TIMER0                            */
  TIMER1_Handler,               /*!<  6: TIMER1                            */
  TIMER2_Handler,               /*!<  7: TIMER2                            */
  TIMER3_Handler,               /*!<  8: TIMER3                            */
  TIMER4_Handler,               /*!<  9: TIMER4                            */
  TIMER5_Handler,               /*!< 10: TIMER5                            */
  TIMER6_Handler,               /*!< 11: TIMER6                            */
  TIMER7_Handler,               /*!< 12: TIMER7                            */
  TIMER8_Handler,               /*!< 13: TIMER8                            */
  TIMER9_Handler,               /*!< 14: TIMER9                            */
  MCKFAIL_Handler,              /*!< 15: MCKFAIL                            */
  GPIOA_Handler,                /*!< 16: GPIOA                            */
  GPIOB_Handler,                /*!< 17: GPIOB                            */
  GPIOC_Handler,                /*!< 18: GPIOC                            */
  GPIOD_Handler,                /*!< 19: GPIOD                            */
  GPIOE_Handler,                /*!< 20: GPIOE                            */
  GPIOF_Handler,                /*!< 21: GPIOF                            */
  Default_Handler,                 /*!< 22: Default                            */
  Default_Handler,                 /*!< 23: Default                            */
  PWM0_Handler,                 /*!< 24: PWM0                            */
  PWM1_Handler,                 /*!< 25: PWM1                            */
  PWM2_Handler,                 /*!< 26: PWM2                            */
  PWM3_Handler,                 /*!< 27: PWM3                            */
  PWM4_Handler,                 /*!< 28: PWM4                            */
  PWM5_Handler,                 /*!< 29: PWM5                            */
  PWM6_Handler,                 /*!< 30: PWM6                            */
  PWM7_Handler,                 /*!< 31: PWM7                            */
  SPI0_Handler,                 /*!< 32: SPI0                            */
  SPI1_Handler,                 /*!< 33: SPI1                            */
  Default_Handler,                 /*!< 34: Default                            */
  Default_Handler,                 /*!< 35: Default                            */
  I2C0_Handler,                 /*!< 36: I2C0                            */
  I2C1_Handler,                 /*!< 37: I2C1                            */
  UART0_Handler,                /*!< 38: UART0                            */
  UART1_Handler,                /*!< 39: UART1                            */
  UART2_Handler,                /*!< 40: UART2                            */
  UART3_Handler,                /*!< 41: UART3                            */
  Default_Handler,                 /*!< 42: Default                            */
  ADC_Handler,                  /*!< 43: ADC                            */
  Default_Handler,                 /*!< 44: Default                            */
  Default_Handler,                 /*!< 45: Default                            */
  Default_Handler,                 /*!< 46: Default                            */
  Default_Handler,                 /*!< 47: Default                            */
  Default_Handler,                 /*!< 48: Default                            */
  Default_Handler,                 /*!< 49: Default                            */
  Default_Handler,                 /*!< 50: Default                            */
  Default_Handler,                 /*!< 51: Default                            */
  Default_Handler,                 /*!< 52: Default                            */
  Default_Handler,                       /*!< 53: Default                            */    
  Default_Handler,                       /*!< 54: Default                            */    
  Default_Handler,                       /*!< 55: Default                            */    
  Default_Handler,                       /*!< 56: Default                            */    
  Default_Handler,                       /*!< 57: Default                            */    
  Default_Handler,                       /*!< 58: Default                            */    
  Default_Handler,                       /*!< 59: Default                            */    
  Default_Handler,                       /*!< 60: Default                            */    
  Default_Handler,                       /*!< 61: Default                            */    
  Default_Handler,                       /*!< 62: Default                            */    
  Default_Handler,                       /*!< 63: Default                            */                              
};


/**
  * @brief  This is the code that gets called when the processor first
  *         starts execution following a reset event. Only the absolutely
  *         necessary set is performed, after which the application
  *         supplied main() routine is called. 
  * @param  None
  * @retval None
  */
void Default_Reset_Handler(void)
{
  /* Initialize data and bss */
  unsigned long *pulSrc, *pulDest;

  /* Copy the data segment initializers from flash to SRAM */
  pulSrc = &_sidata;

  for(pulDest = &_sdata; pulDest < &_edata; )
  {
    *(pulDest++) = *(pulSrc++);
  }
  
  /* Zero fill the bss segment.  This is done with inline assembly since this
     will clear the value of pulDest if it is not kept in a register. */
  __asm("  ldr     r0, =_sbss\n"
        "  ldr     r1, =_ebss\n"
        "  mov     r2, #0\n"
        "  .thumb_func\n"
        "zero_loop:\n"
        "    cmp     r0, r1\n"
        "    it      lt\n"
        "    strlt   r2, [r0], #4\n"
        "    blt     zero_loop");
	
  /* Call the application's entry point.*/
  main();
}


/**
  *@brief Provide weak aliases for each Exception handler to the Default_Handler. 
  *       As they are weak aliases, any function with the same name will override 
  *       this definition.
  */
#pragma weak Reset_Handler = Default_Reset_Handler
#pragma weak NMI_Handler = Default_Handler     
#pragma weak HardFault_Handler = Default_Handler     
#pragma weak MemManage_Handler = Default_Handler     
#pragma weak BusFault_Handler = Default_Handler      
#pragma weak UsageFault_Handler = Default_Handler    
#pragma weak SVC_Handler = Default_Handler           
#pragma weak DebugMon_Handler = Default_Handler      
#pragma weak PendSV_Handler = Default_Handler        
#pragma weak SysTick_Handler = Default_Handler       

#pragma weak LVDFAIL_Handler = Default_Handler
#pragma weak MXOSCFAIL_Handler = Default_Handler
#pragma weak SXOSCFAIL_Handler = Default_Handler
#pragma weak WDT_Handler = Default_Handler
#pragma weak FRT_Handler = Default_Handler
#pragma weak TIMER0_Handler = Default_Handler
#pragma weak TIMER1_Handler = Default_Handler
#pragma weak TIMER2_Handler = Default_Handler
#pragma weak TIMER3_Handler = Default_Handler
#pragma weak TIMER4_Handler = Default_Handler
#pragma weak TIMER5_Handler = Default_Handler
#pragma weak TIMER6_Handler = Default_Handler
#pragma weak TIMER7_Handler = Default_Handler
#pragma weak TIMER8_Handler = Default_Handler
#pragma weak TIMER9_Handler = Default_Handler
#pragma weak GPIOA_Handler = Default_Handler
#pragma weak GPIOB_Handler = Default_Handler
#pragma weak GPIOC_Handler = Default_Handler
#pragma weak GPIOD_Handler = Default_Handler
#pragma weak GPIOE_Handler = Default_Handler
#pragma weak GPIOF_Handler = Default_Handler
#pragma weak PWM0_Handler = Default_Handler
#pragma weak PWM1_Handler = Default_Handler
#pragma weak PWM2_Handler = Default_Handler
#pragma weak PWM3_Handler = Default_Handler
#pragma weak PWM4_Handler = Default_Handler
#pragma weak PWM5_Handler = Default_Handler
#pragma weak PWM6_Handler = Default_Handler
#pragma weak PWM7_Handler = Default_Handler
#pragma weak SPI0_Handler = Default_Handler
#pragma weak SPI1_Handler = Default_Handler
#pragma weak I2C0_Handler = Default_Handler
#pragma weak I2C1_Handler = Default_Handler
#pragma weak UART0_Handler = Default_Handler
#pragma weak UART1_Handler = Default_Handler
#pragma weak UART2_Handler = Default_Handler
#pragma weak UART3_Handler = Default_Handler
#pragma weak ADC_Handler = Default_Handler

/**
  * @brief  This is the code that gets called when the processor receives an 
  *         unexpected interrupt.  This simply enters an infinite loop, 
  *         preserving the system state for examination by a debugger.
  * @param  None
  * @retval None  
  */
static void Default_Handler(void) 
{
	/* Go into an infinite loop. */
	while (1) 
	{
	}
}

/*********************** (C) COPYRIGHT 2009 Coocox ************END OF FILE*****/
