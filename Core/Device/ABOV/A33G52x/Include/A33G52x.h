
/****************************************************************************************************//**
 * @file     A33G52x.h
 *
 * @brief    CMSIS Cortex-M3 Peripheral Access Layer Header File for
 *           A33G52x from ABOV Semiconductor Co., Ltd..
 *
 * @version  V0.1
 * @date     11. July 2017
 *
 * @note     Generated with SVDConv V2.85b 
 *           from CMSIS SVD File 'A33G52x.svd' Version 0.1,
 *
 * @par      THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *           OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *           MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *           ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *           CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER. 
 *
 *******************************************************************************************************/



/** @addtogroup ABOV Semiconductor Co., Ltd.
  * @{
  */

/** @addtogroup A33G52x
  * @{
  */

#ifndef A33G52x_H
#define A33G52x_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M3 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* ---------------------  A33G52x Specific Interrupt Numbers  --------------------- */
  LVDDETECT_IRQn                =   0,              /*!<   0  LVDDETECT                                                        */
  SCLKFAIL_IRQn                 =   1,              /*!<   1  SCLKFAIL                                                         */
  MXOSCFAIL_IRQn                 =   2,              /*!<   2  XOSCFAIL                                                         */
  WDT_IRQn                      =   3,              /*!<   3  WDT                                                              */
  FRT_IRQn                      =   4,              /*!<   4  FRT                                                              */
  TIMER0_IRQn                   =   5,              /*!<   5  TIMER0                                                           */
  TIMER1_IRQn                   =   6,              /*!<   6  TIMER1                                                           */
  TIMER2_IRQn                   =   7,              /*!<   7  TIMER2                                                           */
  TIMER3_IRQn                   =   8,              /*!<   8  TIMER3                                                           */
  TIMER4_IRQn                   =   9,              /*!<   9  TIMER4                                                           */
  TIMER5_IRQn                   =  10,              /*!<  10  TIMER5                                                           */
  TIMER6_IRQn                   =  11,              /*!<  11  TIMER6                                                           */
  TIMER7_IRQn                   =  12,              /*!<  12  TIMER7                                                           */
  TIMER8_IRQn                   =  13,              /*!<  13  TIMER8                                                           */
  TIMER9_IRQn                   =  14,              /*!<  14  TIMER9                                                           */
  MCKFAIL_IRQn				   =  15,				 /*!<	15	 MCKFAIL														  */
  GPIOA_IRQn                    =  16,              /*!<  16  GPIOA                                                            */
  GPIOB_IRQn                    =  17,              /*!<  17  GPIOB                                                            */
  GPIOC_IRQn                    =  18,              /*!<  18  GPIOC                                                            */
  GPIOD_IRQn                    =  19,              /*!<  19  GPIOD                                                            */
  GPIOE_IRQn                    =  20,              /*!<  20  GPIOE                                                            */
#ifdef A33G527
  GPIOF_IRQn                    =  21,              /*!<  21  GPIOF                                                            */
#endif  
  PWM0_IRQn                     =  24,              /*!<  24  PWM0                                                             */
  PWM1_IRQn                     =  25,              /*!<  25  PWM1                                                             */
  PWM2_IRQn                     =  26,              /*!<  26  PWM2                                                             */
  PWM3_IRQn                     =  27,              /*!<  27  PWM3                                                             */
  PWM4_IRQn                     =  28,              /*!<  28  PWM4                                                             */
  PWM5_IRQn                     =  29,              /*!<  29  PWM5                                                             */
  PWM6_IRQn                     =  30,              /*!<  30  PWM6                                                             */
  PWM7_IRQn                     =  31,              /*!<  31  PWM7                                                             */
  SPI0_IRQn                     =  32,              /*!<  32  SPI0                                                             */
  SPI1_IRQn                     =  33,              /*!<  33  SPI1                                                             */
  I2C0_IRQn                     =  36,              /*!<  36  I2C0                                                             */
  I2C1_IRQn                     =  37,              /*!<  37  I2C1                                                             */
  UART0_IRQn                    =  38,              /*!<  38  UART0                                                            */
  UART1_IRQn                    =  39,              /*!<  39  UART1                                                            */
  UART2_IRQn                    =  40,              /*!<  40  UART2                                                            */
  UART3_IRQn                    =  41,              /*!<  41  UART3                                                            */
  ADC_IRQn                      =  43               /*!<  43  ADC                                                              */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M3 Processor and Core Peripherals---------------- */
#define __CM3_REV                 0x0100            /*!< Cortex-M3 Core Revision                                               */
#define __MPU_PRESENT                  0            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               3            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm3.h"                               /*!< Cortex-M3 processor and core peripherals                              */
#include "system_A33G52x.h"                         /*!< A33G52x System                                                        */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                       PMU                      ================ */
/* ================================================================================ */


/**
  * @brief POWER MANAGEMENT UNIT (PMU)
  */

typedef struct {                                    /*!< (@ 0x40000000) PMU Structure                                          */
  __I  uint32_t  IDR;                               /*!< (@ 0x40000000) Chip ID Register                                       */
  __IO uint32_t  MR;                                /*!< (@ 0x40000004) PMU Mode Register                                      */
  __IO uint32_t  CFG;                               /*!< (@ 0x40000008) PMU CONFIGURATION REGISTER                             */
  __I  uint32_t  RESERVED;
  __IO uint32_t  WSER;                              /*!< (@ 0x40000010) PMU Wake-up Source Enable Register                     */
  __I  uint32_t  WSSR;                              /*!< (@ 0x40000014) PMU Wak-up Source Status Register                      */
  __IO uint32_t  RSER;                              /*!< (@ 0x40000018) Reset Source Enable Register                           */
  __IO uint32_t  RSSR;                              /*!< (@ 0x4000001C) Reset Source Status Register                           */
  __IO uint32_t  PRER;                              /*!< (@ 0x40000020) PMU Peripheral Event Reset Register                    */
  __IO uint32_t  PER;                               /*!< (@ 0x40000024) PMU Perpheral Enable Register                          */
  __IO uint32_t  PCCR;                              /*!< (@ 0x40000028) PMU Peripheral Clock Control Register                  */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CCR;                               /*!< (@ 0x40000030) PMU Clock Control Register                             */
  __IO uint32_t  CMR;                               /*!< (@ 0x40000034) PMU Clock Monitoring Register                          */
  __IO uint32_t  MCMR;                              /*!< (@ 0x40000038) PMU Main Clock Monitoring Register                     */
  __IO uint32_t  BCCR;                              /*!< (@ 0x4000003C) PMU BUS Clock Control Register                         */
  __IO uint32_t  PCSR;                              /*!< (@ 0x40000040) PMU Peripheral Clock Select Register                   */
  __IO uint32_t  COR;                               /*!< (@ 0x40000044) PMU Clock Output Register                              */
  __I  uint32_t  RESERVED2[2];
  __IO uint32_t  PLLCON;                            /*!< (@ 0x40000050) PLL Control Register                                   */
  __IO uint32_t  LVDCON;                            /*!< (@ 0x40000054) LVD Control Register                                   */
  __IO uint32_t  VDCCON;                            /*!< (@ 0x40000058) VDC/LVD Trimming Register                              */
  __IO uint32_t  IOSC16TRIM;                        /*!< (@ 0x4000005C) IOSC16 Trimming Register                               */
  __IO uint32_t  EOSCCON;                           /*!< (@ 0x40000060) External Oscillator Control Register                   */
  __I  uint32_t  RESERVED3[3];
  __IO uint32_t  EXTMODER;                          /*!< (@ 0x40000070) External Mode Read Register                            */
} PMU_Type;



/* ================================================================================ */
/* ================                       PCA                      ================ */
/* ================================================================================ */


/**
  * @brief PORT MAP CONTROLLER (PCA)
  */

typedef struct {                                    /*!< PCA Structure                                                         */
  __IO uint32_t  MR;                              /*!< PORT x Pin MUX Register                                               */
  __IO uint32_t  CR;                              /*!< PORT n Pin Control Register                                           */
  __IO uint32_t  PCR;                             /*!< PORT n Pull-up/Pull-down Resistor Control Register                    */
  __IO uint32_t  DER;                             /*!< PORT n Debounce Enable Register                                       */
  __IO uint32_t  IER;                             /*!< PORT n Interrupt Enable Register                                      */
  __IO uint32_t  ISR;                             /*!< PORT n Interrupt Status Register                                      */
  __IO uint32_t  ICR;                             /*!< PORT n Interrupt Control Register                                     */
  __IO uint32_t  DPR;                             /*!< PORT n Debounce Prescaler Register                                    */
} PCU_Type;


/* ================================================================================ */
/* ================                       PA                       ================ */
/* ================================================================================ */


/**
  * @brief GENERAL PURPOSE I/O (PA)
  */

typedef struct {                                    /*!< PA Structure                                                          */
  __IO uint32_t  ODR;                             /*!< PORT n Output Data Register                                           */
  __I  uint32_t  IDR;                             /*!< PORT n Input Data Register                                            */
  __IO uint32_t  SRR;                             /*!< Port n Set/Reset Register                                             */
} GPIO_Type;


/* ================================================================================ */
/* ================                       WDT                      ================ */
/* ================================================================================ */


/**
  * @brief WATCH-DOG TIMER (WDT)
  */

typedef struct {                                    /*!< WDT Structure                                                         */
  __IO uint32_t  LR;                             /*!< Watchdog Timer Load Register                                          */
  __I  uint32_t  CVR;                            /*!< WDT Counter Value Register                                            */
  __IO uint32_t  CON;                            /*!< WDT Control Register                                                  */
} WDT_Type;


/* ================================================================================ */
/* ================                       FRT                      ================ */
/* ================================================================================ */


/**
  * @brief Free-Run Timer (FRT)
  */

typedef struct {                                    /*!< FRT Structure                                                         */
  __IO uint32_t  PRD;                            /*!< FRT Period Register                                                   */
  __IO uint32_t  CNT;                            /*!< FRT Count Register                                                    */
  __IO uint32_t  CON;                            /*!< FRT Control Register                                                  */
} FRT_Type;


/* ================================================================================ */
/* ================                       T0                       ================ */
/* ================================================================================ */


/**
  * @brief 16-BIT TIMER (T0)
  */

typedef struct {                                    /*!< T0 Structure                                                          */
  __IO uint32_t  CON;                             /*!< Timer n Control Register                                              */
  __IO uint32_t  CMD;                             /*!< Timer n Command Register                                              */
  __IO uint32_t  GRA;                             /*!< Timer n General Purpose Register A                                    */
  __IO uint32_t  GRB;                             /*!< Timer n General Register B                                            */
  __IO uint32_t  PRS;                             /*!< Timer n Prescaler Register                                            */
  __IO uint32_t  CNT;                             /*!< Timer n Count Register                                                */
} TIMER_Type;


/* ================================================================================ */
/* ================                      PWM0                      ================ */
/* ================================================================================ */


/**
  * @brief PWM Generator (PWM0)
  */

typedef struct {                                    /*!< PWM0 Structure                                                        */
  __IO uint32_t  CTRL;                          /*!< PWM n Control Register                                                */
  __IO uint32_t  CNT;                           /*!< PWM n Counter Register                                                */
  __IO uint32_t  PER;                           /*!< PWM n Counter Period Register                                         */
  __IO uint32_t  CMP;                           /*!< PWM n Compare Register                                                */
} PWM_Type;


/* ================================================================================ */
/* ================                     PWMPRS0                    ================ */
/* ================================================================================ */


/**
  * @brief PWMPRS0 (PWMPRS0)
  */

typedef struct {                                    /*!< PWMPRS0 Structure                                                     */
  __IO uint32_t  PRSn;                           /*!< PWM Prescaler Register n                                              */
} PWMPRS_Type;


/* ================================================================================ */
/* ================                      I2C0                      ================ */
/* ================================================================================ */


/**
  * @brief I2C Interface (I2C0)
  */

typedef struct {                                    /*!< I2C0 Structure                                                        */
  __IO uint32_t  DR;                             /*!< I2C Data Register                                                     */
  __I  uint32_t  RESERVED;
  __IO uint32_t  SR;                             /*!< Status register                                                       */
  __IO uint32_t  SAR;                            /*!< I2C Slave Address Register                                            */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CR;                             /*!< I2C Control Register                                                  */
  __IO uint32_t  SCLL;                           /*!< I2C SCL LOW duration Register                                         */
  __IO uint32_t  SCLH;                           /*!< I2C SCL HIGH duration Register                                        */
  __IO uint32_t  SDH;                            /*!< I2C SDA Hold Register                                                 */
} I2C_Type;


/* ================================================================================ */
/* ================                      UART0                     ================ */
/* ================================================================================ */


/**
  * @brief UNIVERSAL ASYNCHRONOUS RECEIVER/TRANSMITTER (UART0)
  */

typedef struct {                                    /*!< UART0 Structure                                                       */
  __IO uint32_t  RBR_THR_DLL;                       /*!< Rx/Tx Buffer REGISTER                                                 */
  __IO uint32_t  IER_DLM;                           /*!< UART Interrupt Enable Register                                        */
  
  union {
    __O  uint32_t  FCR;                           /*!< FIFO Control Register                                                 */
    __I  uint32_t  IIR;                           /*!< UART Interrupt ID Register                                            */
  };
  __IO uint32_t  LCR;                             /*!< UART Line Control Register                                            */
  __I  uint32_t  RESERVED;
  __IO uint32_t  LSR;                             /*!< UART Line Status Register                                             */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  SCR;                             /*!< UART Scratch Register                                                 */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  BFR;                             /*!< UART BaudRate Fraction Counter Register                               */
  __IO uint32_t  DTR;                             /*!< UART Inter-frame Delay Time Register                                  */
} UART_Type;


/* ================================================================================ */
/* ================                      SPI0                      ================ */
/* ================================================================================ */


/**
  * @brief SERIAL PERIPHERAL INTERFACE (SPI0)
  */

typedef struct {                                    /*!< SPI0 Structure                                                        */
  __IO uint32_t  RDR_TDR;                           /*!< SPI n Receive Data Register                                           */
  __IO uint32_t  CR;                             /*!< SPI n Control Register                                                */
  __IO uint32_t  SR;                             /*!< SPI n Status Register                                                 */
  __IO uint32_t  BR;                             /*!< SPI n Baud Rate Register                                              */
  __IO uint32_t  EN;                             /*!< SPI n Enable register                                                 */
  __IO uint32_t  LR;                             /*!< SPI n Timing register                                                 */
} SPI_Type;


/* ================================================================================ */
/* ================                       ADC                      ================ */
/* ================================================================================ */


/**
  * @brief 12-BIT A/D CONVERTER (ADC)
  */

typedef struct {                                    /*!< ADC Structure                                                         */
  __IO uint32_t  CR;                              /*!< ADC Control Register                                                  */
  __IO uint32_t  MR;                              /*!< ADC Mode Register                                                     */
  __I  uint32_t   DR;                              /*!< A/D Data Register                                                     */
 __IO uint32_t  TEST;                              /*!< ADC TEST Mode Register                                                     */
	
} ADC_Type;


/* ================================================================================ */
/* ================                       FMC                      ================ */
/* ================================================================================ */


/**
  * @brief INTERNAL FLASH MEMORY (FMC)
  */

typedef struct {                                    /*!< FMC Structure                                                         */
  __IO uint32_t  CFG;                            /*!< Flash Memory Configuration Register                                   */
  __IO uint32_t  CON;                             /*!< Flash Memory Control Register                                         */
  __IO uint32_t  ODR;                             /*!< Flash Memory Output Data Register                                     */
  __I  uint32_t   IDR;                             /*!< Flash Memory Input Data Register                                      */
	
  __IO uint32_t  AR;                              /*!< Flash Memory Address Register                                         */
  __IO uint32_t  TEST;                            /*!< Flash Memory Extended Mode Control Register                           */
  __IO uint32_t  CRC;                             /*!< Flash Memory CRC Register                                             */
  __IO uint32_t  PROTECT;                        /*!< Flash Memory Protection Register                                      */
	
  __IO uint32_t  RPROT;                           /*!< Flash Memory Read Protection Register                                 */
  __I  uint32_t  HWID;                            /*!< Flash Hardware ID Register                                            */
  __I  uint32_t  SIZE;                            /*!< Flash Size Register                                                   */
  __IO uint32_t  BOOT;                           /*!< Flash Boot Register                                */	
	
  __IO uint32_t  DCT0;                           /*!< Flash Memory DCT0                                 */
  __IO uint32_t  DCT1;                           /*!< Flash Memory DCT1                                 */
  __IO uint32_t  DCT2;                           /*!< Flash Memory DCT2                                 */
  __IO uint32_t  DCT3;                           /*!< Flash Memory DCT3                                */		
} FMC_Type;


/* ================================================================================ */
/* ================                      MEMCR                     ================ */
/* ================================================================================ */


/**
  * @brief Memory Control Register (MEMCR)
  */

typedef struct {                                  /*!< MEMCR Structure                                                       */
  __IO uint32_t  CR;                             /*!< Memory Control Register                                               */
} MEMCR_Type;


/* ================================================================================ */
/* ================                    PMULEGACY                   ================ */
/* ================================================================================ */


/**
  * @brief PMULEGACY A33G52x Extended Mode Register (PMULEGACY)
  */

typedef struct {                                    /*!< PMULEGACY Structure                                                   */
  __IO uint32_t  LEGACY;                         /*!< PMULEGACY A33G52x Extended Mode Register                            */
} PMULEGACY_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define PMU_BASE                       0x40000000UL
#define PCA_BASE                       0x40000200UL
#define PCB_BASE                       0x40000220UL
#define PCC_BASE                       0x40000240UL
#define PCD_BASE                       0x40000260UL
#define PCE_BASE                       0x40000280UL
#define PCF_BASE                       0x400002A0UL
#define PA_BASE                         0x40000300UL
#define PB_BASE                         0x40000310UL
#define PC_BASE                         0x40000320UL
#define PD_BASE                         0x40000330UL
#define PE_BASE                         0x40000340UL
#define PF_BASE                         0x40000350UL
#define WDT_BASE                      0x40000400UL
#define FRT_BASE                       0x40000500UL
#define T0_BASE                         0x40000C00UL
#define T1_BASE                         0x40000C20UL
#define T2_BASE                         0x40000C40UL
#define T3_BASE                         0x40000C60UL
#define T4_BASE                         0x40000C80UL
#define T5_BASE                         0x40000CA0UL
#define T6_BASE                         0x40000CC0UL
#define T7_BASE                         0x40000CE0UL
#define T8_BASE                         0x40000D00UL
#define T9_BASE                         0x40000D20UL
#define PWM0_BASE                       0x40000700UL
#define PWM1_BASE                       0x40000720UL
#define PWM2_BASE                       0x40000740UL
#define PWM3_BASE                       0x40000760UL
#define PWM4_BASE                       0x40000780UL
#define PWM5_BASE                       0x400007A0UL
#define PWM6_BASE                       0x400007C0UL
#define PWM7_BASE                       0x400007E0UL
#define PWMPRS0_BASE                    0x4000077CUL
#define PWMPRS1_BASE                    0x400007FCUL
#define I2C0_BASE                       0x40000A00UL
#define I2C1_BASE                       0x40000A80UL
#define UART0_BASE                      0x40000B00UL
#define UART1_BASE                      0x40000B40UL
#define UART2_BASE                      0x40000B80UL
#define UART3_BASE                      0x40000BC0UL
#define SPI0_BASE                       0x40000800UL
#define SPI1_BASE                       0x40000820UL
#define ADC_BASE                        0x40000E00UL
#define FMC_BASE                        0x40000100UL
#define MEMCR_BASE                      0x400000E0UL
#define PMULEGACY_BASE                  0x400000F8UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define PMU                             ((PMU_Type                *) PMU_BASE)
#define PCA                             ((PCU_Type                *) PCA_BASE)
#define PCB                             ((PCU_Type                *) PCB_BASE)
#define PCC                             ((PCU_Type                *) PCC_BASE)
#define PCD                             ((PCU_Type                *) PCD_BASE)
#define PCE                             ((PCU_Type                *) PCE_BASE)
#define PCF                             ((PCU_Type                *) PCF_BASE)
#define PA                              ((GPIO_Type                 *) PA_BASE)
#define PB                              ((GPIO_Type                 *) PB_BASE)
#define PC                              ((GPIO_Type                 *) PC_BASE)
#define PD                              ((GPIO_Type                 *) PD_BASE)
#define PE                              ((GPIO_Type                 *) PE_BASE)
#define PF                              ((GPIO_Type                 *) PF_BASE)
#define WDT                             ((WDT_Type                *) WDT_BASE)
#define FRT                             ((FRT_Type                *) FRT_BASE)
#define T0                              ((TIMER_Type                 *) T0_BASE)
#define T1                              ((TIMER_Type                 *) T1_BASE)
#define T2                              ((TIMER_Type                 *) T2_BASE)
#define T3                              ((TIMER_Type                 *) T3_BASE)
#define T4                              ((TIMER_Type                 *) T4_BASE)
#define T5                              ((TIMER_Type                 *) T5_BASE)
#define T6                              ((TIMER_Type                 *) T6_BASE)
#define T7                              ((TIMER_Type                 *) T7_BASE)
#define T8                              ((TIMER_Type                 *) T8_BASE)
#define T9                              ((TIMER_Type                 *) T9_BASE)
#define PWM0                            ((PWM_Type               *) PWM0_BASE)
#define PWM1                            ((PWM_Type               *) PWM1_BASE)
#define PWM2                            ((PWM_Type               *) PWM2_BASE)
#define PWM3                            ((PWM_Type               *) PWM3_BASE)
#define PWM4                            ((PWM_Type               *) PWM4_BASE)
#define PWM5                            ((PWM_Type               *) PWM5_BASE)
#define PWM6                            ((PWM_Type               *) PWM6_BASE)
#define PWM7                            ((PWM_Type               *) PWM7_BASE)
#define PWMPRS0                         ((PWMPRS_Type            *) PWMPRS0_BASE)
#define PWMPRS1                         ((PWMPRS_Type            *) PWMPRS1_BASE)
#define I2C0                            ((I2C_Type               *) I2C0_BASE)
#define I2C1                            ((I2C_Type               *) I2C1_BASE)
#define UART0                           ((UART_Type              *) UART0_BASE)
#define UART1                           ((UART_Type              *) UART1_BASE)
#define UART2                           ((UART_Type              *) UART2_BASE)
#define UART3                           ((UART_Type              *) UART3_BASE)
#define SPI0                            ((SPI_Type               *) SPI0_BASE)
#define SPI1                            ((SPI_Type               *) SPI1_BASE)
#define ADC                             ((ADC_Type                *) ADC_BASE)
#define FMC                             ((FMC_Type                *) FMC_BASE)
#define MEMCR                           ((MEMCR_Type              *) MEMCR_BASE)
#define PMULEGACY                       ((PMULEGACY_Type          *) PMULEGACY_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group A33G52x */
/** @} */ /* End of group ABOV Semiconductor Co., Ltd. */

#ifdef __cplusplus
}
#endif


#endif  /* A33G52x_H */

