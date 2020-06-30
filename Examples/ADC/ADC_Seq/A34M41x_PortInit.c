/**********************************************************************
* @file		A34M41x_PortInit.c
* @brief	Contains all macro definitions and function prototypes
* 			support for PCU firmware library
* @version	1.0
* @date		
* @author	ABOV M team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/
#include "main_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Port_Init(void);
/* Private variables ---------------------------------------------------------*/




/**********************************************************************
 * @brief		This function handles NMI exception.
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void Port_Init(void)
{
 SYST_ACCESS_EN();
	
	// Peripheral Enable Register1	0 : Disable,	1 : Enable
	SCU->PER1 = SCU->PER1
	| (1<<14)						// GPIOG
	| (1<<13)						// GPIOF
	| (1<<12)						// GPIOE
	| (1<<11)						// GPIOD
	| (1<<10)						// GPIOC
	| (1<<9)						// GPIOB
	| (1<<8)						// GPIOA
	;
	
	// Peripheral Clock Enable Register1	0 : Disable,	1 : Enable
	SCU->PCER1 = SCU->PCER1
	| (1<<14)						// GPIOG
	| (1<<13)						// GPIOF
	| (1<<12)						// GPIOE
	| (1<<11)						// GPIOD
	| (1<<10)						// GPIOC
	| (1<<9)						// GPIOB
	| (1<<8)						// GPIOA
	;
	
	SYST_ACCESS_DIS();
	
	// Enable Writing Permittion of ALL PCU Register
	PORT_ACCESS_EN();
	
	//------------------------------------------------------------
	//	PORT - A
	//------------------------------------------------------------
	
	// PinMux Setting
	PA->MR2 = 0
	| (0<<28)				// 0:PA15,		1:MISO0,	2:QEI0_IDX,		3:OVIN0U,		7:AN15
	| (0<<24)				// 0:PA14,		1:MOSI0,	2:QEI0_B,		3:PRTIN0U,		7:AN14
	| (0<<20)				// 0:PA13,		1:SCK0,		2:QEI0_A,		3:				7:AN13
	| (0<<16)				// 0:PA12,		1:SS0,		2:QEI0_UPDN,	3:SCAP0W,		7:AN12
	| (0<<12)				// 0:PA11,		1:TXD1,		2:				3:SCAP0V,		7:AN11/CREF1
	| (0<<8)				// 0:PA10,		1:RXD1,		2:				3:SCAP0U,		7:AN10/CP1C
	| (0<<4)				// 0:PA9,		1:CAN_H,	2:				3:				7:AN9/CP1B
	| (0<<0)				// 0:PA8,		1:CAN_L,	2:				3:				7:AN8/CP1A
	;
	
	PA->MR1 = 0
	| (0<<28)				// 0:PA7,		1:			2:T3IO,			3:CAP0W,		7:AN7/CREF0
	| (0<<24)				// 0:PA6,		1:			2:T2IO,			3:CAP0V,		7:AN6/CP0C
	| (0<<20)				// 0:PA5,		1:			2:T1IO,			3:CAP0U,		7:AN5/CP0B
	| (0<<16)				// 0:PA4,		1:			2:T0IO,			3:				7:AN4/CP0A
	| (0<<12)				// 0:PA3,		1:			2:				3:				7:AN3
	| (0<<8)				// 0:PA2,		1:			2:				3:				7:AN2
	| (0<<4)				// 0:PA1,		1:			2:				3:				7:AN1
	| (0<<0)				// 0:PA0,		1:			2:				3:				7:AN0
	;
	
	//Input, Output, Open Drain (only used when PinMux Setting as GPIO)
	PA->CR = 0
	| (0<<30)   // P15 		0:Push-pull output, 1:Open-drain output, 2,3:Input
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	// Pullup/down Setting
	PA->PRCR = 0
	| (0<<30)   // P15 		0,1:Disable pull-up/down resistor, 2:Enable pull-up resistor, 3:Enable Pull-down resistor
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	

	//------------------------------------------------------------
	//	PORT - B
	//------------------------------------------------------------
	
	// PinMux Setting
	PB->MR2 = 0
	| (0<<28)				// 0:PB15,		1:			2:				3:MP1WL,		7:
	| (0<<24)				// 0:PB14,		1:			2:				3:MP1WH,		7:
	| (0<<20)				// 0:PB13,		1:			2:				3:MP1VL,		7:
	| (0<<16)				// 0:PB12,		1:			2:				3:MP1VH,		7:
	| (0<<12)				// 0:PB11,		1:			2:				3:MP1UL,		7:
	| (0<<8)				// 0:PB10,		1:			2:				3:MP1UH,		7:
	| (0<<4)				// 0:PB9,		1:TXD3,		2:				3:OVIN1,		7:
	| (0<<0)				// 0:PB8,		1:RXD3,		2:				3:PRTIN1,		7:
	;
	
	PB->MR1 = 0
	| (0<<28)				// 0:PB7,		1:			2:STBYO,		3:OVIN0W,		7:
	| (0<<24)				// 0:PB6,		1:			2:WDTO,			3:PRTIN0W,		7:
	| (0<<20)				// 0:PB5,		1:			2:T9IO,			3:MP0WL,		7:
	| (0<<16)				// 0:PB4,		1:			2:T8IO,			3:MP0WH,		7:
	| (0<<12)				// 0:PB3,		1:			2:				3:MP0VL,		7:
	| (0<<8)				// 0:PB2,		1:			2:				3:MP0VH,		7:
	| (0<<4)				// 0:PB1,		1:			2:				3:MP0UL,		7:
	| (0<<0)				// 0:PB0,		1:			2:				3:MP0UH,		7:
	;
	
	//Input, Output, Open Drain (only used when PinMux Setting as GPIO)
	PB->CR = 0
	| (0<<30)   // P15 		0:Push-pull output, 1:Open-drain output, 2,3:Input
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	// Pullup/down Setting
	PB->PRCR = 0
	| (0<<30)   // P15 		0,1:Disable pull-up/down resistor, 2:Enable pull-up resistor, 3:Enable Pull-down resistor
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	
	//------------------------------------------------------------
	//	PORT - C
	//------------------------------------------------------------
	
	// PinMux Setting
	PC->MR2 = 0
	| (3<<28)				// 0:PC15,		1:MISO0,	2:				3:TXD0,			7:
	| (3<<24)				// 0:PC14,		1:MOSI0,	2:				3:RXD0,			7:
	| (7<<20)				// 0:PC13,		1:			2:				3:				7:XOUT
	| (7<<16)				// 0:PC12,		1:			2:				3:				7:XIN
	| (3<<12)				// 0:PC11,		1:			2:T9IO,			3:BOOT,			7:
	| (3<<8)				// 0:PC10,		1:			2:				3:nRESET,		7:
	| (3<<4)				// 0:PC9,		1:			2:T8IO,			3:CLKO,			7:
	| (0<<0)				// 0:PC8,		1:SDA0,		2:T4IO,			3:				7:
	;
	
	PC->MR1 = 0
	| (0<<28)				// 0:PC7,		1:SCL0,		2:T3IO,			3:				7:
	| (0<<24)				// 0:PC6,		1:TXD1,		2:T2IO,			3:				7:
	| (0<<20)				// 0:PC5,		1:RXD1,		2:T1IO,			3:				7:
	| (3<<16)				// 0:PC4,		1:			2:T0IO,			3:nTRST,		7:
	| (3<<12)				// 0:PC3,		1:			2:				3:TDI,			7:
	| (3<<8)				// 0:PC2,		1:			2:				3:TDO/SWO,		7:
	| (3<<4)				// 0:PC1,		1:TXD0,		2:				3:TMS/SWDIO,	7:
	| (3<<0)				// 0:PC0,		1:RXD0,		2:				3:TCK/SWCLK,	7:
	;
	
	//Input, Output, Open Drain (only used when PinMux Setting as GPIO) 
	PC->CR = 0
	| (0<<30)   // P15 		0:Push-pull output, 1:Open-drain output, 2,3:Input
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	// Pullup/down Setting
	PC->PRCR = 0
	| (0<<30)   // P15 		0,1:Disable pull-up/down resistor, 2:Enable pull-up resistor, 3:Enable Pull-down resistor
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	
	//------------------------------------------------------------
	//	PORT - D
	//------------------------------------------------------------
	
	// PinMux Setting
	PD->MR2 = 0
	| (0<<28)				// 0:PD15,		1:SCK0,		2:				3:AD2E,			7:
	| (0<<24)				// 0:PD14,		1:SS0,		2:				3:AD2S,			7:
	| (0<<20)				// 0:PD13,		1:			2:T3IO,			3:AD1E,			7:
	| (0<<16)				// 0:PD12,		1:			2:T2IO,			3:AD1S,			7:
	| (0<<12)				// 0:PD11,		1:			2:T1IO,			3:AD0E,			7:
	| (0<<8)				// 0:PD10,		1:			2:T0IO,			3:AD0S,			7:
	| (0<<4)				// 0:PD9,		1:			2:T7IO,			3:STBYO,		7:
	| (0<<0)				// 0:PD8,		1:			2:T6IO,			3:WDTO,			7:
	;
	
	PD->MR1 = 0
	| (0<<28)				// 0:PD7,		1:RXD2,		2:				3:				7:AN19
	| (0<<24)				// 0:PD6,		1:TXD2,		2:				3:				7:AN18
	| (0<<20)				// 0:PD5,		1:SDA1,		2:				3:				7:AN17
	| (0<<16)				// 0:PD4,		1:SCL1,		2:				3:				7:AN16
	| (0<<12)				// 0:PD3,		1:MISO1,	2:				3:				7:
	| (0<<8)				// 0:PD2,		1:MOSI1,	2:				3:				7:
	| (7<<4)				// 0:PD1,		1:SCK1,		2:				3:				7:SXOUT
	| (7<<0)				// 0:PD0,		1:SS1,		2:				3:				7:SXIN
	;
	
	//Input, Output, Open Drain (only used when PinMux Setting as GPIO)
	PD->CR = 0
	| (0<<30)   // P15 		0:Push-pull output, 1:Open-drain output, 2,3:Input
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	// Pullup/down Setting
	PD->PRCR = 0
	| (0<<30)   // P15 		0,1:Disable pull-up/down resistor, 2:Enable pull-up resistor, 3:Enable Pull-down resistor
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	
	//------------------------------------------------------------
	//	PORT - E
	//------------------------------------------------------------
	
	// PinMux Setting
	PE->MR2 = 0
	| (0<<28)				// 0:PE15,		1:			2:				3:				7:
	| (0<<24)				// 0:PE14,		1:RXD4,		2:T3IO,			3:				7:
	| (0<<20)				// 0:PE13,		1:TXD4,		2:T2IO,			3:				7:
	| (0<<16)				// 0:PE12,		1:SDA1,		2:T1IO,			3:				7:
	| (0<<12)				// 0:PE11,		1:SCL1,		2:T0IO,			3:				7:
	| (0<<8)				// 0:PE10,		1:			2:T9IO,			3:				7:
	| (0<<4)				// 0:PE9,		1:			2:T8IO,			3:QEI1_IDX,		7:
	| (0<<0)				// 0:PE8,		1:			2:T7IO,			3:QEI1_B,		7:
	;
	
	PE->MR1 = 0
	| (0<<28)				// 0:PE7,		1:			2:T6IO,			3:QEI1_A		7:
	| (0<<24)				// 0:PE6,		1:			2:T5IO,			3:QEI1_UPDN		7:
	| (0<<20)				// 0:PE5,		1:			2:T5IO,			3:				7:
	| (0<<16)				// 0:PE4,		1:SDA0,		2:				3:				7:
	| (0<<12)				// 0:PE3,		1:SCL0,		2:				3:				7:
	| (0<<8)				// 0:PE2,		1:			2:				3:				7:AN21/CREF2
	| (0<<4)				// 0:PE1,		1:			2:				3:				7:AN20/CP2
	| (0<<0)				// 0:PE0,		1:			2:				3:				7:
	;
	
	//Input, Output, Open Drain (only used when PinMux Setting as GPIO)
	PE->CR = 0
	| (0<<30)   // P15 		0:Push-pull output, 1:Open-drain output, 2,3:Input
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	// Pullup/down Setting
	PE->PRCR = 0
	| (0<<30)   // P15 		0,1:Disable pull-up/down resistor, 2:Enable pull-up resistor, 3:Enable Pull-down resistor
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	
	//------------------------------------------------------------
	//	PORT - F
	//------------------------------------------------------------
	
	// PinMux Setting
	PF->MR2 = 0
	| (0<<28)				// 0:PF15,		1:			2:				3:				7:
	| (0<<24)				// 0:PF14,		1:			2:				3:				7:
	| (0<<20)				// 0:PF13,		1:			2:				3:				7:
	| (0<<16)				// 0:PF12,		1:			2:				3:				7:
	| (0<<12)				// 0:PF11,		1:			2:				3:				7:
	| (0<<8)				// 0:PF10,		1:			2:				3:				7:
	| (0<<4)				// 0:PF9,		1:			2:				3:				7:
	| (0<<0)				// 0:PF8,		1:			2:				3:				7:
	;
	
	PF->MR1 = 0
	| (0<<28)				// 0:PF7,		1:CAN_H,	2:				3:OVIN0V,		7:
	| (0<<24)				// 0:PF6,		1:CAN_L,	2:				3:PRTIN0V,		7:
	| (0<<20)				// 0:PF5,		1:RXD5,		2:				3:				7:
	| (0<<16)				// 0:PF4,		1:TXD5,		2:				3:				7:
	| (0<<12)				// 0:PF3,		1:			2:				3:				7:AN23/CREF3
	| (0<<8)				// 0:PF2,		1:			2:				3:				7:AN22/CP3
	| (0<<4)				// 0:PF1,		1:			2:				3:				7:
	| (0<<0)				// 0:PF0,		1:			2:				3:				7:
	;
	
	//Input, Output, Open Drain (only used when PinMux Setting as GPIO)
	PF->CR = 0
	| (0<<30)   // P15 		0:Push-pull output, 1:Open-drain output, 2,3:Input
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	// Pullup/down Setting
	PF->PRCR = 0
	| (0<<30)   // P15 		0,1:Disable pull-up/down resistor, 2:Enable pull-up resistor, 3:Enable Pull-down resistor
	| (0<<28)   // P14
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	
	//------------------------------------------------------------
	//	PORT - G
	//------------------------------------------------------------
	
	// PinMux Setting
	PG->MR2 = 0
	| (0<<8)				// 0:PG10,		1:			2:				3:				7:
	| (0<<4)				// 0:PG9,		1:TXD3,		2:				3:				7:
	| (0<<0)				// 0:PG8,		1:RXD3,		2:				3:				7:
	;
	
	PG->MR1 = 0
	| (0<<28)				// 0:PG7,		1:			2:				3:				7:
	| (0<<24)				// 0:PG6,		1:			2:				3:				7:
	| (0<<20)				// 0:PG5,		1:			2:				3:				7:
	| (0<<16)				// 0:PG4,		1:			2:				3:				7:
	| (0<<12)				// 0:PG3,		1:MISO2		2:				3:				7:
	| (0<<8)				// 0:PG2,		1:MOSI2		2:				3:				7:
	| (0<<4)				// 0:PG1,		1:SCK2		2:				3:				7:
	| (0<<0)				// 0:PG0,		1:SS2		2:				3:				7:
	;
	
	//Input, Output, Open Drain (only used when PinMux Setting as GPIO)
	PG->CR = 0
	| (0<<20)   // P10 		0:Push-pull output, 1:Open-drain output, 2,3:Input
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	// Pullup/down Setting
	PG->PRCR = 0
	| (0<<20)   // P10 		0,1:Disable pull-up/down resistor, 2:Enable pull-up resistor, 3:Enable Pull-down resistor
	| (0<<18)   // P9
	| (0<<16)   // P8
	| (0<<14)   // P7
	| (0<<12)   // P6
	| (0<<10)   // P5
	| (0<<8)   // P4
	| (0<<6)   // P3
	| (0<<4)   // P2
	| (0<<2)   // P1
	| (0<<0)   // P0	
	;
	
	
	// Disable Writing Permittion of ALL PCU register.
	PORT_ACCESS_DIS();
	
}
