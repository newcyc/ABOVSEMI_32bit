
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "A33G52x.h"
#include "a33g52x_gpio.h"
#include "a33g52x_pcu.h"
#include "a33g52x_pmu.h"
#include "a33g52x_nvic.h"
#include "a33g52x_frt.h"
#include "a33g52x_timer.h"
#include "console.h"
#include "slib.h"



void PCU_Init(void);
void mainloop(void);
void Disp_MainMenu(void);

void timerOneshot(void);
void timerPeriodic(void);
void timerCapture(void);
void timerPwm(void);

/* main function */
int main(void)
{
	SystemInit();						// RINGOSC (1MHz)
	
	PCU_Init();							// Initialization of A33G52x's Pin 
	
	SystemCoreClockUpdate();	// Clock Update to PLL Frequency
	
	csetport(0);						// Set UART Channel

	cinit();								// Initialization of UART
	
	mainloop();							// User Code

	return 0;
}

/* user code */
void mainloop(void)
{
	char 	ch;
	int	disp_menu = 1;
	
	for (;;)
	{
		if (disp_menu)
		{
			Disp_MainMenu(); 
			disp_menu = 0; 
		}

		cputs("[G527]#"); 

		InData[0] = 0; 

		for (;;)
		{
			ch = getstring(); 
			if (ch == ASCII_CARRIAGE_RETURN) break; 
		}

		cputs("\r\n\r\n"); 

		if (InFlag)
		{
			if (!strncmp(InData, "1", 1))
			{
				cputs(">>> TIMER OneShot\r\n");
				timerOneshot();				
			}
			else if (!strncmp(InData, "2", 1))
			{
				cputs(">>> Timer Periodic\r\n");
				timerPeriodic();				
			}				
			else if (!strncmp(InData, "3", 1))
			{
				cputs(">>> Timer Capture\r\n");
				timerCapture();				
			}	
			else if (!strncmp(InData, "4", 1))
			{
				cputs(">>> Timer PWM\r\n");
				timerPwm();				
			}	
			
			
			InFlag = InCount = 0; 
		}
	}

}

extern uint32_t	g_TIMER0_count; 
extern uint32_t	g_TIMER1_count; 
extern uint32_t	g_TIMER2_count; 
extern uint32_t	g_TIMER3_count; 			 
extern uint32_t	g_TIMER4_count;  
extern uint32_t	g_TIMER5_count; 
extern uint32_t	g_TIMER6_count; 
extern uint32_t	g_TIMER7_count; 
extern uint32_t	g_TIMER8_count; 
extern uint32_t	g_TIMER9_count; 


void timerOneshot(void)
{
	TIMER_CONFIG		timer_config;

	timer_config.start_level_after_counter_clear = TIMER_START_LEVEL_LOW; 
	timer_config.clock_select = TnCON_TCS_TCLK_DIV_BY_2;								// TIMERCLKIN = PCLK/2 = 74MHz/2
	timer_config.capture_clear_mode = TIMER_CAPTURE_RISING_EDGE_CLEAR;	
	timer_config.GRA = (1*1000); 														// 1ms Oneshot Match
	timer_config.GRB = (2*1000);														// 2ms Period
	timer_config.PRS = 36;																// TCLK = TIMERCLKIN/(PRS+1) = 37MHz/(36+1) = 1MHz		
	_TIMER_Init(0, TIMER_MODE_ONE_SHOT, &timer_config); 
	TIMER_Start(0);
	
}
void timerPeriodic(void)
{
	uint32_t					reg_val;
	TIMER_CONFIG		timer_config;
	
	// SET Timer0/1 Clock Source
	reg_val = PMU->PCSR;
	reg_val |= PMUPCSR_T01CS_MAIN_XTAL;
	PMU->PCSR = reg_val;	
	
	timer_config.start_level_after_counter_clear = TIMER_START_LEVEL_LOW;
	timer_config.clock_select = TIMER_CLKSEL_PMUPCSR; 									//  T0_TIMERCLKIN = XTAL8MHz
	timer_config.capture_clear_mode = TIMER_CAPTURE_RISING_EDGE_CLEAR;
	timer_config.GRA = 8000;
	timer_config.GRB = 0;

	_TIMER_Init(0, TIMER_MODE_PERIODIC, &timer_config); 

	timer_config.start_level_after_counter_clear = TIMER_START_LEVEL_LOW;
	timer_config.clock_select = TIMER_CLKSEL_PMUPCSR; 									// T1_TIMERCLKIN = XTAL8MHz
	timer_config.capture_clear_mode = TIMER_CAPTURE_RISING_EDGE_CLEAR;
	timer_config.GRA = 4000;
	timer_config.GRB = 0;
	
	
	_TIMER_Init(1, TIMER_MODE_PERIODIC, &timer_config); 
	
	TIMER_Start(0);
	TIMER_Start(1);
	
}
void timerCapture(void)
{
	uint32_t					reg_val;
	int k;

	TIMER_CONFIG		T0_Config, T2_Config;
		
	
	// T0O (Timer Output : PB0)
	reg_val = PMU->PCSR;
	reg_val |= PMUPCSR_T01CS_MAIN_XTAL;			// SET Timer0/1 Clock Source (8MHz XTAL)
	PMU->PCSR = reg_val;	
	
	T0_Config.start_level_after_counter_clear = TIMER_START_LEVEL_LOW;
	T0_Config.clock_select = TIMER_CLKSEL_PMUPCSR; 
	T0_Config.capture_clear_mode = TIMER_CAPTURE_RISING_EDGE_CLEAR;
	T0_Config.GRA =  8000;
	T0_Config.GRB = 0;

	_TIMER_Init(0, TIMER_MODE_PERIODIC, &T0_Config); 
	

	// T2C (Capture Timer : PA8)	
	T2_Config.start_level_after_counter_clear = TIMER_START_LEVEL_LOW; 
	T2_Config.clock_select = TIMER_CLKSEL_EXTERNAL_SOURCE; 
	T2_Config.capture_clear_mode = TIMER_CAPTURE_RISING_EDGE_CLEAR;
	T2_Config.GRA = 0; 
	T2_Config.GRB = 0;

	_TIMER_Init(2, TIMER_MODE_CAPTURE, &T2_Config); 	
	
	g_TIMER2_count	= 0;
	//-----------------------------------------------------------------------------------
	// timer start
	//-----------------------------------------------------------------------------------	
	TIMER_Start(0);	
	TIMER_Start(2);
	
	
	for(k = 0; k < 200000; k++);		// delay

	//-----------------------------------------------------------------------------------
	// timer stop 
	//-----------------------------------------------------------------------------------
	TIMER_Stop(0);
	TIMER_Stop(2);

	//-----------------------------------------------------------------------------------
	// disp buffer
	//-----------------------------------------------------------------------------------
	cprintf("g_TIMER2_count : %d\r\n", g_TIMER2_count);
	
}
void timerPwm(void)
{
	TIMER_CONFIG		timer_config;
	
	timer_config.start_level_after_counter_clear = TIMER_START_LEVEL_HIGH; 
	timer_config.clock_select = TIMER_CLKSEL_TCLK_DIV_BY_2; 
	timer_config.PRS = 36;
	timer_config.capture_clear_mode = 0;
	// PWM duty = 25%, PWM period(Freq) = 1ms(1kHz) 
	timer_config.GRA =  250; 				
	timer_config.GRB = 1000; 				
	_TIMER_Init(0, TIMER_MODE_PWM, &timer_config);

	TIMER_Start(0);
	
}


void Disp_MainMenu(void)
{
	cprintf("\r\n========================================\r\n");
	cprintf("Timer Demo\r\n");  
	cprintf("\t - MCU : A33G52x\r\n");
	cprintf("\t - Core: ARM Cortex-M3 \n\r");
	cprintf("\t - Communicate via: UART0 - 38400 bps \n\r");
	cprintf("1. Timer OneShot\r\n");
	cprintf("2. Timer Periodic\r\n");
	cprintf("3. Timer Capture\r\n");
	cprintf("4. Timer PWM\r\n");	
	cprintf("========================================\r\n"); 
}


void PCU_Init(void)
{
	//Peripheral Enable Register 1  0:Disable, 1:Enable	
	PMU->PER=PMU->PER
		| (1<<13)   // GPIOF
		| (1<<12)   // GPIOE
		| (1<<11)   // GPIOD
		| (1<<10)   // GPIOC
		| (1<<9)    // GPIOB
		| (1<<8)    // GPIOA
		; 		
//Peripheral Clock Enable Register 1 0:Disable, 1:Enable	
	PMU->PCCR=PMU->PCCR
		| (1<<8)    // GPIO
		; 	
	
	//--------------------------------------------------------------
	//	PORT INIT
	//		PA, PB, PC, PD, PE, PF
	//--------------------------------------------------------------
	// PORT - A
	PCA->MR = 0
		| (0x0UL<<30)              // 0:PA15		1:T9C			2:				3:AN15
		| (0x0UL<<28)              // 0:PA14		1:T8C			2:				3:AN14
		| (0x0UL<<26)              // 0:PA13		1:T7C			2:				3:
		| (0x0UL<<24)              // 0:PA12		1:T6C			2:				3:
		| (0x0UL<<22)              // 0:PA11		1:T5C			2:				3:
		| (0x0UL<<20)              // 0:PA10		1:T4C			2:				3:
		| (0x0UL<<18)              // 0:PA9			1:T3C			2:				3:
		| (0x0UL<<16)              // 0:PA8			1:T2C			2:				3:
		| (0x0UL<<14)              // 0:PA7			1:T1C			2:				3:AN7
		| (0x0UL<<12)              // 0:PA6			1:T0C			2:				3:AN6
		| (0x0UL<<10)              // 0:PA5			1:				2:				3:AN5
		| (0x0UL<<8)               // 0:PA4			1:				2:				3:AN4
		| (0x0UL<<6)               // 0:PA3			1:				2:				3:AN3
		| (0x0UL<<4)               // 0:PA2			1:				2:				3:AN2
		| (0x0UL<<2)               // 0:PA1			1:				2:				3:AN1
		| (0x0UL<<0)               // 0:PA0			1:				2:				3:AN0
		;
	
	PCA->CR = 0              // 0 : Output(Push-pull) Mode,	1 : Output(Open-drain) Mode,	2 : Input,	3: Analog
		| (0x3UL<<30)              // P15
		| (0x3UL<<28)              // P14
		| (0x3UL<<26)              // P13
		| (0x3UL<<24)              // P12
		| (0x3UL<<22)              // P11
		| (0x3UL<<20)              // P10
		| (0x3UL<<18)              // P9
		| (0x3UL<<16)              // P8
		| (0x3UL<<14)              // P7
		| (0x3UL<<12)              // P6
		| (0x3UL<<10)              // P5
		| (0x3UL<<8)              // P4
		| (0x3UL<<6)              // P3
		| (0x3UL<<4)              // P2
		| (0x3UL<<2)              // P1
		| (0x3UL<<0)              // P0
		;
	
	PCA->PCR = 0
		| (0x0UL<<31)             // P15		0: Pull up,			1: Pull down
		| (0x0UL<<30)             // P14		0: Pull up,			1: Pull down
		| (0x0UL<<29)             // P13		0: Pull up,			1: Pull down
		| (0x0UL<<28)             // P12		0: Pull up,			1: Pull down
		| (0x0UL<<27)             // P11		0: Pull up,			1: Pull down
		| (0x0UL<<26)             // P10		0: Pull up,			1: Pull down
		| (0x0UL<<25)             // P9			0: Pull up,			1: Pull down
		| (0x0UL<<24)             // P8			0: Pull up,			1: Pull down
		| (0x0UL<<23)             // P7			0: Pull up,			1: Pull down
		| (0x0UL<<22)             // P6			0: Pull up,			1: Pull down
		| (0x0UL<<21)             // P5			0: Pull up,			1: Pull down
		| (0x0UL<<20)             // P4			0: Pull up,			1: Pull down
		| (0x0UL<<19)             // P3			0: Pull up,			1: Pull down
		| (0x0UL<<18)             // P2			0: Pull up,			1: Pull down
		| (0x0UL<<17)             // P1			0: Pull up,			1: Pull down
		| (0x0UL<<16)             // P0			0: Pull up,			1: Pull down
		| (0x0UL<<15)             // P15		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<14)             // P14		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<13)             // P13		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<12)             // P12		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<11)             // P11		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<10)             // P10		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<9)              // P9			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<8)              // P8			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<7)              // P7			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<6)              // P6			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<5)              // P5			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<4)              // P4			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<3)              // P3			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<2)              // P2			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<1)              // P1			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<0)              // P0			0: Disable PU/PD 	1: Enable PU/PD
		;
	
//////////////////////////////////////////////////////////////////////////////////////////////////////	
	// PORT - B
	PCB->MR = 0
		| (0x0UL<<30)              // 0:PB15		1:SDA0			2:				3:
		| (0x0UL<<28)              // 0:PB14		1:SCL0			2:				3:
		| (0x0UL<<26)              // 0:PB13		1:MISO0			2:				3:
		| (0x0UL<<24)              // 0:PB12		1:MOSI0			2:				3:
		| (0x0UL<<22)              // 0:PB11		1:SCK0			2:				3:
		| (0x0UL<<20)              // 0:PB10		1:SS0			2:				3:
		| (0x0UL<<18)              // 0:PB9			1:T9O			2:				3:
		| (0x0UL<<16)              // 0:PB8			1:T8O			2:				3:
		| (0x0UL<<14)              // 0:PB7			1:T7O			2:				3:
		| (0x0UL<<12)              // 0:PB6			1:T6O			2:				3:
		| (0x0UL<<10)              // 0:PB5			1:T5O			2:				3:
		| (0x0UL<<8)               // 0:PB4			1:T4O			2:				3:
		| (0x0UL<<6)               // 0:PB3			1:T3O			2:				3:
		| (0x0UL<<4)               // 0:PB2			1:T2O			2:				3:
		| (0x0UL<<2)               // 0:PB1			1:T1O			2:				3:
		| (0x0UL<<0)               // 0:PB0			1:T0O			2:				3:
		;
	
	PCB->CR = 0              // 0 : Output(Push-pull) Mode,	1 : Output(Open-drain) Mode,	2 : Input,	3: Analog
		| (0x3UL<<30)              // P15
		| (0x3UL<<28)              // P14
		| (0x3UL<<26)              // P13
		| (0x3UL<<24)              // P12
		| (0x3UL<<22)              // P11
		| (0x3UL<<20)              // P10
		| (0x3UL<<18)              // P9
		| (0x3UL<<16)              // P8
		| (0x3UL<<14)              // P7
		| (0x3UL<<12)              // P6
		| (0x3UL<<10)              // P5
		| (0x3UL<<8)              // P4
		| (0x3UL<<6)              // P3
		| (0x3UL<<4)              // P2
		| (0x3UL<<2)              // P1
		| (0x3UL<<0)              // P0
		;
	
	PCB->PCR = 0
		| (0x0UL<<31)             // P15		0: Pull up,			1: Pull down
		| (0x0UL<<30)             // P14		0: Pull up,			1: Pull down
		| (0x0UL<<29)             // P13		0: Pull up,			1: Pull down
		| (0x0UL<<28)             // P12		0: Pull up,			1: Pull down
		| (0x0UL<<27)             // P11		0: Pull up,			1: Pull down
		| (0x0UL<<26)             // P10		0: Pull up,			1: Pull down
		| (0x0UL<<25)             // P9			0: Pull up,			1: Pull down
		| (0x0UL<<24)             // P8			0: Pull up,			1: Pull down
		| (0x0UL<<23)             // P7			0: Pull up,			1: Pull down
		| (0x0UL<<22)             // P6			0: Pull up,			1: Pull down
		| (0x0UL<<21)             // P5			0: Pull up,			1: Pull down
		| (0x0UL<<20)             // P4			0: Pull up,			1: Pull down
		| (0x0UL<<19)             // P3			0: Pull up,			1: Pull down
		| (0x0UL<<18)             // P2			0: Pull up,			1: Pull down
		| (0x0UL<<17)             // P1			0: Pull up,			1: Pull down
		| (0x0UL<<16)             // P0			0: Pull up,			1: Pull down
		| (0x0UL<<15)             // P15		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<14)             // P14		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<13)             // P13		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<12)             // P12		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<11)             // P11		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<10)             // P10		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<9)              // P9			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<8)              // P8			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<7)              // P7			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<6)              // P6			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<5)              // P5			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<4)              // P4			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<3)              // P3			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<2)              // P2			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<1)              // P1			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<0)              // P0			0: Disable PU/PD 	1: Enable PU/PD
		;
	
//////////////////////////////////////////////////////////////////////////////////////////////////////	
	// PORT - C
	PCC->MR = 0
		| (0x0UL<<30)              // 0:PC15		1:XTALI			2:CLKIN			3:
		| (0x0UL<<28)              // 0:PC14		1:XTALO			2:				3:
		| (0x0UL<<26)              // 0:PC13		1:CLKO			2:				3:
		| (0x0UL<<24)              // 0:PC12		1:STBYO			2:				3:
		| (0x0UL<<22)              // 0:PC11		1:TXD2			2:				3:
		| (0x0UL<<20)              // 0:PC10		1:RXD2			2:				3:
		| (0x0UL<<18)              // 0:PC9			1:TXD0			2:				3:
		| (0x0UL<<16)              // 0:PC8			1:RXD0			2:				3:
		| (0x0UL<<14)              // 0:PC7			1:				2:				3:
		| (0x1UL<<12)              // 0:PC6			1:nRESET		2:				3:
		| (0x0UL<<10)              // 0:PC5			1:				2:				3:
		| (0x1UL<<8)               // 0:PC4			1:TDO			2:				3:
		| (0x1UL<<6)               // 0:PC3			1:TCK			2:				3:
		| (0x1UL<<4)               // 0:PC2			1:TMD			2:				3:
		| (0x1UL<<2)               // 0:PC1			1:TDI			2:				3:
		| (0x1UL<<0)               // 0:PC0			1:nTRST			2:				3:
		;
	
	PCC->CR = 0              // 0 : Output(Push-pull) Mode,	1 : Output(Open-drain) Mode,	2 : Input,	3: Analog
		| (0x3UL<<30)              // P15
		| (0x3UL<<28)              // P14
		| (0x3UL<<26)              // P13
		| (0x3UL<<24)              // P12
		| (0x3UL<<22)              // P11
		| (0x3UL<<20)              // P10
		| (0x3UL<<18)              // P9
		| (0x3UL<<16)              // P8
		| (0x3UL<<14)              // P7
		| (0x2UL<<12)              // P6
		| (0x3UL<<10)              // P5
		| (0x0UL<<8)              // P4
		| (0x2UL<<6)              // P3
		| (0x2UL<<4)              // P2
		| (0x2UL<<2)              // P1
		| (0x2UL<<0)              // P0
		;
	
	PCC->PCR = 0
		| (0x0UL<<31)             // P15		0: Pull up,			1: Pull down
		| (0x0UL<<30)             // P14		0: Pull up,			1: Pull down
		| (0x0UL<<29)             // P13		0: Pull up,			1: Pull down
		| (0x0UL<<28)             // P12		0: Pull up,			1: Pull down
		| (0x0UL<<27)             // P11		0: Pull up,			1: Pull down
		| (0x0UL<<26)             // P10		0: Pull up,			1: Pull down
		| (0x0UL<<25)             // P9			0: Pull up,			1: Pull down
		| (0x0UL<<24)             // P8			0: Pull up,			1: Pull down
		| (0x0UL<<23)             // P7			0: Pull up,			1: Pull down
		| (0x0UL<<22)             // P6			0: Pull up,			1: Pull down
		| (0x0UL<<21)             // P5			0: Pull up,			1: Pull down
		| (0x0UL<<20)             // P4			0: Pull up,			1: Pull down
		| (0x0UL<<19)             // P3			0: Pull up,			1: Pull down
		| (0x0UL<<18)             // P2			0: Pull up,			1: Pull down
		| (0x0UL<<17)             // P1			0: Pull up,			1: Pull down
		| (0x0UL<<16)             // P0			0: Pull up,			1: Pull down
		| (0x0UL<<15)             // P15		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<14)             // P14		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<13)             // P13		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<12)             // P12		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<11)             // P11		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<10)             // P10		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<9)              // P9			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<8)              // P8			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<7)              // P7			0: Disable PU/PD 	1: Enable PU/PD
		| (0x1UL<<6)              // P6			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<5)              // P5			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<4)              // P4			0: Disable PU/PD 	1: Enable PU/PD
		| (0x1UL<<3)              // P3			0: Disable PU/PD 	1: Enable PU/PD
		| (0x1UL<<2)              // P2			0: Disable PU/PD 	1: Enable PU/PD
		| (0x1UL<<1)              // P1			0: Disable PU/PD 	1: Enable PU/PD
		| (0x1UL<<0)              // P0			0: Disable PU/PD 	1: Enable PU/PD
		;
	
//////////////////////////////////////////////////////////////////////////////////////////////////////	
	// PORT - D
	PCD->MR = 0
		| (0x0UL<<30)              // 0:PD15		1:SDA1			2:				3:
		| (0x0UL<<28)              // 0:PD14		1:SCL1			2:				3:
		| (0x0UL<<26)              // 0:PD13		1:TXD1			2:				3:
		| (0x0UL<<24)              // 0:PD12		1:RXD1			2:				3:
		| (0x0UL<<22)              // 0:PD11		1:MISO1			2:				3:
		| (0x0UL<<20)              // 0:PD10		1:MOSI1			2:				3:
		| (0x0UL<<18)              // 0:PD9			1:SCK1			2:				3:
		| (0x0UL<<16)              // 0:PD8			1:SS1			2:				3:
		| (0x0UL<<14)              // 0:PD7			1:MPWMA7		2:				3:
		| (0x0UL<<12)              // 0:PD6			1:MPWMA6		2:				3:
		| (0x0UL<<10)              // 0:PD5			1:MPWMA5		2:				3:
		| (0x0UL<<8)               // 0:PD4			1:MPWMA4		2:				3:
		| (0x0UL<<6)               // 0:PD3			1:MPWMA3		2:				3:
		| (0x0UL<<4)               // 0:PD2			1:MPWMA2		2:				3:
		| (0x0UL<<2)               // 0:PD1			1:MPWMA1		2:				3:
		| (0x0UL<<0)               // 0:PD0			1:MPWMA0		2:				3:
		;
	
	PCD->CR = 0              // 0 : Output(Push-pull) Mode,	1 : Output(Open-drain) Mode,	2 : Input,	3: Analog
		| (0x3UL<<30)              // P15
		| (0x3UL<<28)              // P14
		| (0x3UL<<26)              // P13
		| (0x3UL<<24)              // P12
		| (0x3UL<<22)              // P11
		| (0x3UL<<20)              // P10
		| (0x3UL<<18)              // P9
		| (0x3UL<<16)              // P8
		| (0x3UL<<14)              // P7
		| (0x3UL<<12)              // P6
		| (0x3UL<<10)              // P5
		| (0x3UL<<8)              // P4
		| (0x3UL<<6)              // P3
		| (0x3UL<<4)              // P2
		| (0x3UL<<2)              // P1
		| (0x3UL<<0)              // P0
		;
	
	PCD->PCR = 0
		| (0x0UL<<31)             // P15		0: Pull up,			1: Pull down
		| (0x0UL<<30)             // P14		0: Pull up,			1: Pull down
		| (0x0UL<<29)             // P13		0: Pull up,			1: Pull down
		| (0x0UL<<28)             // P12		0: Pull up,			1: Pull down
		| (0x0UL<<27)             // P11		0: Pull up,			1: Pull down
		| (0x0UL<<26)             // P10		0: Pull up,			1: Pull down
		| (0x0UL<<25)             // P9			0: Pull up,			1: Pull down
		| (0x0UL<<24)             // P8			0: Pull up,			1: Pull down
		| (0x0UL<<23)             // P7			0: Pull up,			1: Pull down
		| (0x0UL<<22)             // P6			0: Pull up,			1: Pull down
		| (0x0UL<<21)             // P5			0: Pull up,			1: Pull down
		| (0x0UL<<20)             // P4			0: Pull up,			1: Pull down
		| (0x0UL<<19)             // P3			0: Pull up,			1: Pull down
		| (0x0UL<<18)             // P2			0: Pull up,			1: Pull down
		| (0x0UL<<17)             // P1			0: Pull up,			1: Pull down
		| (0x0UL<<16)             // P0			0: Pull up,			1: Pull down
		| (0x0UL<<15)             // P15		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<14)             // P14		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<13)             // P13		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<12)             // P12		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<11)             // P11		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<10)             // P10		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<9)              // P9			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<8)              // P8			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<7)              // P7			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<6)              // P6			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<5)              // P5			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<4)              // P4			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<3)              // P3			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<2)              // P2			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<1)              // P1			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<0)              // P0			0: Disable PU/PD 	1: Enable PU/PD
		;
	
//////////////////////////////////////////////////////////////////////////////////////////////////////	
	// PORT - E
	PCE->MR = 0
		| (0x0UL<<30)              // 0:PE15		1:TraceCLK		2:				3:
		| (0x0UL<<28)              // 0:PE14		1:TraceD0		2:				3:
		| (0x0UL<<26)              // 0:PE13		1:TraceD1		2:				3:
		| (0x0UL<<24)              // 0:PE12		1:TraceD2		2:				3:
		| (0x0UL<<22)              // 0:PE11		1:TraceD3		2:				3:
		| (0x1UL<<18)              // 0:PE9			1:SXOUT			2:				3:
		| (0x1UL<<16)              // 0:PE8			1:SXIN			2:				3:
		| (0x0UL<<14)              // 0:PE7			1:MPWMB7		2:TXD3			3:
		| (0x0UL<<12)              // 0:PE6			1:MPWMB6		2:RXD3			3:
		| (0x0UL<<10)              // 0:PE5			1:MPWMB5		2:				3:
		| (0x0UL<<8)               // 0:PE4			1:MPWMB4		2:				3:
		| (0x0UL<<6)               // 0:PE3			1:MPWMB3		2:				3:
		| (0x0UL<<4)               // 0:PE2			1:MPWMB2		2:				3:
		| (0x0UL<<2)               // 0:PE1			1:MPWMB1		2:				3:
		| (0x0UL<<0)               // 0:PE0			1:MPWMB0		2:				3:
		;
	
	PCE->CR = 0              // 0 : Output(Push-pull) Mode,	1 : Output(Open-drain) Mode,	2 : Input,	3: Analog
		| (0x3UL<<30)              // P15
		| (0x3UL<<28)              // P14
		| (0x3UL<<26)              // P13
		| (0x3UL<<24)              // P12
		| (0x3UL<<22)              // P11
		| (0x3UL<<18)              // P9
		| (0x3UL<<16)              // P8
		| (0x3UL<<14)              // P7
		| (0x3UL<<12)              // P6
		| (0x3UL<<10)              // P5
		| (0x3UL<<8)              // P4
		| (0x3UL<<6)              // P3
		| (0x3UL<<4)              // P2
		| (0x3UL<<2)              // P1
		| (0x3UL<<0)              // P0
		;
	
	PCE->PCR = 0
		| (0x0UL<<31)             // P15		0: Pull up,			1: Pull down
		| (0x0UL<<30)             // P14		0: Pull up,			1: Pull down
		| (0x0UL<<29)             // P13		0: Pull up,			1: Pull down
		| (0x0UL<<28)             // P12		0: Pull up,			1: Pull down
		| (0x0UL<<27)             // P11		0: Pull up,			1: Pull down
		| (0x0UL<<25)             // P9			0: Pull up,			1: Pull down
		| (0x0UL<<24)             // P8			0: Pull up,			1: Pull down
		| (0x0UL<<23)             // P7			0: Pull up,			1: Pull down
		| (0x0UL<<22)             // P6			0: Pull up,			1: Pull down
		| (0x0UL<<21)             // P5			0: Pull up,			1: Pull down
		| (0x0UL<<20)             // P4			0: Pull up,			1: Pull down
		| (0x0UL<<19)             // P3			0: Pull up,			1: Pull down
		| (0x0UL<<18)             // P2			0: Pull up,			1: Pull down
		| (0x0UL<<17)             // P1			0: Pull up,			1: Pull down
		| (0x0UL<<16)             // P0			0: Pull up,			1: Pull down
		| (0x0UL<<15)             // P15		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<14)             // P14		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<13)             // P13		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<12)             // P12		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<11)             // P11		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<9)              // P9			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<8)              // P8			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<7)              // P7			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<6)              // P6			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<5)              // P5			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<4)              // P4			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<3)              // P3			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<2)              // P2			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<1)              // P1			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<0)              // P0			0: Disable PU/PD 	1: Enable PU/PD
		;
	
//////////////////////////////////////////////////////////////////////////////////////////////////////	
	// PORT - F
	PCF->MR = 0
		| (0x0UL<<22)              // 0:PF11		1:				2:FRTO_FMF		3:
		| (0x0UL<<20)              // 0:PF10		1:				2:				3:
		| (0x0UL<<18)              // 0:PF9		1:				2:				3:
		| (0x0UL<<16)              // 0:PF8		1:				2:				3:
		| (0x0UL<<14)              // 0:PF7		1:				2:				3:
		| (0x0UL<<10)              // 0:PF5		1:				2:				3:AN13
		| (0x0UL<<8)               // 0:PF4		1:				2:				3:AN12
		| (0x0UL<<6)               // 0:PF3		1:				2:				3:AN11
		| (0x0UL<<4)               // 0:PF2		1:				2:				3:AN10
		| (0x0UL<<2)               // 0:PF1		1:				2:				3:AN9
		| (0x0UL<<0)               // 0:PF0		1:				2:				3:AN8
		;
	
	PCF->CR = 0              // 0 : Output(Push-pull) Mode,	1 : Output(Open-drain) Mode,	2 : Input,	3: Analog
		| (0x3UL<<22)              // P11
		| (0x3UL<<20)              // P10
		| (0x3UL<<18)              // P9
		| (0x3UL<<16)              // P8
		| (0x3UL<<14)              // P7
		| (0x3UL<<10)              // P5
		| (0x3UL<<8)              // P4
		| (0x3UL<<6)              // P3
		| (0x3UL<<4)              // P2
		| (0x3UL<<2)              // P1
		| (0x3UL<<0)              // P0
		;
	
	PCF->PCR = 0
		| (0x0UL<<27)             // P11		0: Pull up,			1: Pull down
		| (0x0UL<<26)             // P10		0: Pull up,			1: Pull down
		| (0x0UL<<25)             // P9			0: Pull up,			1: Pull down
		| (0x0UL<<24)             // P8			0: Pull up,			1: Pull down
		| (0x0UL<<23)             // P7			0: Pull up,			1: Pull down
		| (0x0UL<<21)             // P5			0: Pull up,			1: Pull down
		| (0x0UL<<20)             // P4			0: Pull up,			1: Pull down
		| (0x0UL<<19)             // P3			0: Pull up,			1: Pull down
		| (0x0UL<<18)             // P2			0: Pull up,			1: Pull down
		| (0x0UL<<17)             // P1			0: Pull up,			1: Pull down
		| (0x0UL<<16)             // P0			0: Pull up,			1: Pull down
		| (0x0UL<<11)             // P11		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<10)             // P10		0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<9)              // P9			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<8)              // P8			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<7)              // P7			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<5)              // P5			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<4)              // P4			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<3)              // P3			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<2)              // P2			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<1)              // P1			0: Disable PU/PD 	1: Enable PU/PD
		| (0x0UL<<0)              // P0			0: Disable PU/PD 	1: Enable PU/PD
		;
}