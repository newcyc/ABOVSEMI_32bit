#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "A33G52x.h"
#include "a33g52x_nvic.h"
#include "a33g52x_pmu.h"
#include "a33g52x_gpio.h"
#include "a33g52x_pcu.h"
#include "a33g52x_uart.h"
#include "a33g52x_timer.h"
#include "a33g52x_pwm.h"
#include "a33g52x_wdt.h"
#include "a33g52x_spi.h"
#include "a33g52x_i2c.h"
#include "a33g52x_adc.h"
#include "a33g52x_frt.h"
#include "console.h"
#include "slib.h"


void PCU_Init(void);
void mainloop(void);
void Disp_MainMenu(void);

void mode_run(void);
void mode_sleep(void);
void mode_deepsleep(void);

void setModulesRun(void);
void setModulesSleep(void);
void setModulesDeepsleep(void);



int main(void)
{
	SystemInit();								// RINGOSC (1MHz)
	
 	SystemCoreClockUpdate();			// Clock Update to PLL Frequency
	
	csetport(0);								// Set UART Channel

	cinit();										// Initialization of UART
		
	mainloop();									// User Code

	return 0;
}

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
				cputs(">>> Normal (Run) Mode\r\n");
				mode_run();				
			}
			else if (!strncmp(InData, "2", 1))
			{
				cputs(">>> Idle (Sleep) Mode\r\n");
				mode_sleep();				
			}
			else if (!strncmp(InData, "3", 1))
			{
				cputs(">>> Powerdown (Deep-Sleep) Mode\r\n");
				mode_deepsleep();				
			}			
			
			InFlag = InCount = 0; 
		}
	}
}

void Disp_MainMenu(void)
{
	cprintf("\r\n============================================\r\n");
	cprintf("Operating Mode Demo\r\n");  
	cprintf("\t - MCU : A33G52x\r\n");
	cprintf("\t - Core: ARM Cortex-M3 \n\r");
	cprintf("\t - Communicate via: UART0 - 38400 bps \n\r");
	cprintf("1. Normal (Run) Mode\r\n");
	cprintf("2. Idle (Sleep) Mode\r\n");	
	cprintf("3. Powerdown (Deep-Sleep) Mode\r\n");	
	cprintf("==================================================\r\n"); 
}



void mode_run(void)
{
	uint32_t reg_val;
	

	// Set PA0 for Dhrystone
	PCU_ConfigureFunction (PCA, PIN_0, PA0_MUX_PA0); 
	PCU_Set_Direction_Type(PCA, PIN_0, PnCR_OUTPUT_PUSH_PULL); 
	PCU_ConfigurePullup_Pulldown (PCA, PIN_0, 0, PnPCR_PULLUPDOWN_DISABLE);	
	

	cprintf("Run...\r\n");
	
	setModulesRun();		// Core and All Peripheral operate
	
//	__disable_irq();
//	__enable_irq();
	
	// Go to Run mode
	reg_val = PMU->MR;
	reg_val &= ~PMUMR_PMUMODE_MASK; 
	reg_val |= PMUMR_PMUMODE_RUN; 
	PMU->MR = reg_val;		// Run Mode	
	
	while(1);
}

void mode_sleep(void)
{
	cprintf("Sleep Mode...\r\n");	
	
	setModulesRun();			// Core is stopped.
	
//	__disable_irq();
//	__enable_irq();

	// Go to Sleep Mode
	SCB->SCR = 0;
	
#ifdef  __CC_ARM
		 __wfi();		// Use only keil arm compiler
#elif	__ICCARM__
		 asm("WFI");	// Use only iar arm compiler
#elif	__GNUC__
		 asm("wfi");	// Use only gnu arm gcc compiler
#endif


	while(1);
}

void mode_deepsleep(void)
{
	
	cprintf("Deep-Sleep Mode...\r\n");	

	setModulesDeepsleep();

	// Go to Deep-sleep
	#if 0	// VDCLP=1, Cortex-M3 Command : PASS
	reg_val = PMU->MR;
	reg_val |= PMUMR_VDCLP; 
	PMU->MR = VDCLP;
	CSB->SCR = (0x0001<<2);
#ifdef  __CC_ARM
		 __wfi();		// Use only keil arm compiler
#elif	__ICCARM__
		 asm("WFI");	// Use only iar arm compiler
#elif	__GNUC__
		 asm("WFI");	// use only gcc arm compiler
#endif	
	#else 	// VDCLP=1, PWDN Command : PASS
	PMU->MR = (PMUMR_VDCLP|PMUMR_PMUMODE_PWDN);
	#endif 
	
	while(1);

}
 
void setModulesRun(void)
{

	SPI_CONFIG 			spi_config; 
	PWM_CONFIG			pwm_config;
	I2C_CONFIG			i2c_config;
	WDT_CFG_Type		wdt_config;
	NVIC_IntrConfig 		nvic_config; 
	TIMER_CONFIG		timer_config;
	

	// PMU Setting
	PMU->CCR = 0xAA;							// ROSC, IOSC, SXOSC, MXOSC
	PMU->PER = 0xEFFFFFFF;				// Enable all peripherals event reset except ADC
	PMU->PRER = 0xEFFFFFFF;				// Enable all peripherals except ADC
	PMU->PCCR = 0xEFFFFFFF;				// Enable all peripherals clock except ADC
		

	// LVD & nReset
	PMU->LVDCON |= (LVDCON_LVDEN|LVDCON_LVDRF|LVDCON_LVDREN|LVDCON_LVDRL_2_60V);
	PCC->DPR = 8;						// 
	PCC->DER = (0x0001<<6);			// PC6 (nReset)

	PMU->RSER |= PMURSER_RSTRSTE|PMURSER_LVDRSTE;

	
	// UART Init (UART0~UART3)
	_UART_Init(0);
	_UART_Init(1);
	_UART_Init(2);
	_UART_Init(3);


	
	// SPI Init (SPI0~SPI1)
	spi_config.master_slave = SPI_MASTER; 
	spi_config.endian = SPI_LSB_FIRST; 
	spi_config.SS_polarity = SPI_SS_ACTIVE_LOW; 
	spi_config.CPHA = 1; 
	spi_config.CPOL = 1; 
	spi_config.bitsize = SPI_BITSIZE_8_BITS; 
	spi_config.baudrate = 0xFF; 
	_SPI_Init(0, &spi_config); 					// Init SPI : Master
	spi_config.master_slave = SPI_SLAVE;
	_SPI_Init(1, &spi_config);					// Init SPI : Slave
	

	
//	// ADC Init (AN0~AN15)
//	ADC_Init(0, 0, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 1, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 3, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 4, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 5, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 6, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 7, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 8, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 9, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 10, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 12, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 13, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source	
//	ADC_Init(0, 14, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC_Init(0, 15, ADC_NO_INTR, ADC_NO_TRIG);		// Polling and No Trigger Source
//	ADC->MR |= ADMR_ADCS_VAL(18); 

	// PWM Init (PWM0~PWM7)
	pwm_config.sync_mode = PWM_ASYNC_MODE; 
	pwm_config.invert = PWM_SIG_NO_INVERT;
	pwm_config.clock_select = PWMnCTRL_CKSEL_DIV_BY_2; 
	pwm_config.period = 24000-1;
	pwm_config.compare_val = (24000/2)-1;

	_PWM_Init(0, &pwm_config);
	_PWM_Init(1, &pwm_config);
	_PWM_Init(2, &pwm_config);
	_PWM_Init(3, &pwm_config);
	_PWM_Init(4, &pwm_config);
	_PWM_Init(5, &pwm_config);
	_PWM_Init(6, &pwm_config);
	_PWM_Init(7, &pwm_config);
	
 	PWM_Start(0); 
	PWM_Start(1); 
	PWM_Start(2); 
	PWM_Start(3); 
	PWM_Start(4); 
	PWM_Start(5); 
	PWM_Start(6); 
	PWM_Start(7); 


	// I2C Init (I2C0~I2C1)
	i2c_config.slave_addr = 0; 
	i2c_config.general_call = I2C_GENERAL_CALL_DISABLE; 
	i2c_config.scl_low_duration = 50; 
	i2c_config.scl_high_duration = 50; 
	i2c_config.sda_hold_duration = 12; 
	i2c_config.interval = 1;
	
	_I2C_Init (0, I2C_MASTER, &i2c_config); 
 	i2c_config.slave_addr =  0x20; 
	_I2C_Init (1, I2C_SLAVE, &i2c_config);


	// WDT
	WDT_Init();
	PMU->PCSR |= PMUPCSR_WDTCS_RINGOSC;		// 	Set WDT Clock Source to RINGOSC
	wdt_config.wdtDebugEn = WDT_DBG_ENABLE;
	wdt_config.wdtInterruptEn = WDT_INTR_DISABLE;
	wdt_config.wdtResetEn = WDT_RST_DISABLE;
	wdt_config.wdtClkSel = WDT_CLK_PMUPCSR;			// Select clock source from PMUPCSR
	WDT_Configure(wdt_config);	
	WDT_UpdateTimeOut(500000 - 1);

	WDT_Start(WDT_START);										// WDT Start

	
	
//	// FRT
	FRT_ConfigureInterrupt(FRT, FRTCON_FOIE|FRTCON_FMIE, INTR_ENABLE); 		

	nvic_config.nIRQ_Number = IRQ_FRT; 
	nvic_config.Preemption_Priority= PRIO_FRT_PREEMPTION; 
	nvic_config.Subpriority= PRIO_FRT_SUBPRIORITY; 
	nvic_config.IntrEnable = INTR_ENABLE; 
	NVIC_ConfigureInterrupt (NVIC, &nvic_config); 	

	FRT->CON |= FRTCON_FEC_PCLK|FRTCON_FPRS_FRTCLKIN_DIV_BY_8|FRTCON_FAC_FREERUN;
	FRT->CNT = 0;
	FRT->PRD = (48-1);
	FRT->CON |= FRTCON_FEN;
	
	
	
	// TIMER
	PMU->PCSR |= PMUPCSR_T01CS_MAIN_XTAL;	
	
	timer_config.start_level_after_counter_clear = TIMER_START_LEVEL_LOW;
	timer_config.clock_select = TIMER_CLKSEL_PMUPCSR; 
	timer_config.capture_clear_mode = TIMER_CAPTURE_RISING_EDGE_CLEAR;
	timer_config.GRA = 8000;
	timer_config.GRB = 0;

	_TIMER_Init(0, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(1, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(2, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(3, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(4, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(5, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(6, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(7, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(8, TIMER_MODE_PERIODIC, &timer_config);
	_TIMER_Init(9, TIMER_MODE_PERIODIC, &timer_config);
			
	TIMER_Start(0);
	TIMER_Start(1);
	TIMER_Start(2);
	TIMER_Start(3);
	TIMER_Start(4);
	TIMER_Start(5);
	TIMER_Start(6);
	TIMER_Start(7);
	TIMER_Start(8);
	TIMER_Start(9);

}

void setModulesSleep(void)
{
	;
}

void setModulesDeepsleep(void)
{
//	UART_CONFIG cfg;
	
	SystemInit();

//	cfg.Baudrate = 1200;
//	UART_Init(UART0, &cfg);
		
	// PMU Setting
	PMU->CCR = 0x88;						// Enable RINGOSC, RTC only

//	PMU->PER = 0x00100000;
//	PMU->PCCR = 0x00100000;	
//	PMU->PRER = 0x00100000;

	PMU->PER = 0x00000000;
	PMU->PCCR = 0x00000000;	
	PMU->PRER = 0x00000000;
	
	
	PMU->WSER = 0;

	PMU->RSER = 0;
	PMU->RSSR = 0;	
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
