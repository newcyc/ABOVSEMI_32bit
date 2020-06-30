
#include <stdio.h>
#include "A33G52x.h"
#include <stdint.h>
#include "console.h"
#include "slib.h"



void PCU_Init(void);
void mainloop(void);

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

void mainloop(void)
{
	cprintf("Hello, World!\r\n");
	
	while(1);

}

void PCU_Init(void)
{
	// Port A
	PCA->MR = 0
	| (0<<30)				// 0 : PA15				1 : T9C				3 : AN15
	| (0<<28)				// 0 : PA14				1 : T8C				3 : AN14
	| (0<<26)				// 0 : PA13				1 : T7C				
	| (0<<24)				// 0 : PA12				1 : T6C
	| (0<<22)				// 0 : PA11				1 : T5C
	| (0<<20)				// 0 : PA10				1 : T4C
	| (0<<18)				// 0 : PA9				1 : T3C
	| (0<<16)				// 0 : PA8				1 : T2C
	| (0<<14)				// 0 : PA7				1 : T1C				3 : AN7
	| (0<<12)				// 0 : PA6				1 : T0C				3 : AN6
	| (0<<10)				// 0 : PA5										3 : AN5
	| (0<<8)   				// 0 : PA4,              				        3 : AN4
	| (0<<6)  			 	// 0 : PA3,          						   	3 : AN3
	| (0<<4)   				// 0 : PA2,                          			3 : AN2	
	| (0<<2) 				    // 0 : PA1,               			            3 : AN1
	| (0<<0)  			 	// 0 : PA0,                 			        	3 : AN0	
	;
	
	PCA->CR = 0
	| (0UL<<30)  			// P15 0:Push-pull output, 1:Open-drain output, 2:Input, 3:Analog
	| (0<<28)   				// P14
	| (0<<26)   				// P13
	| (0<<24)   				// P12
	| (0<<22)  				// P11
	| (0<<20)   				// P10
	| (0<<18)				// P9
	| (0<<16)   				// P8
	| (0<<14)   				// P7
	| (0<<12)   				// P6
	| (0<<10)   				// P5
	| (0<<8)   				// P4
	| (0<<6)  				// P3
	| (0<<4)   				// P2
	| (0<<2)   				// P1
	| (0<<0)   				// P0
	;
	
	PCA->PCR = 0
	| (0<<15)   				// P15 0:Disable pull-up resistor, 1:Enable pull-up resister
	| (0<<14)   				// P14
	| (0<<13)   				// P13
	| (0<<12)   				// P12
	| (0<<11)   				// P11
	| (0<<10)   				// P10
	| (0<<9)   				// P9
	| (0<<8)   				// P8
	| (0<<7)   				// P7
	| (0<<6)   				// P6
	| (0<<5)  				// P5
	| (0<<4)   				// P4
	| (0<<3)  				// P3
	| (0<<2)   				// P2
	| (0<<1)   				// P1
	| (0<<0)   				// P0	
	;
		
	PA->SRR = 0xFFFF0000;
	
	
	// Port B
	PCB->MR = 0
	| (0<<30)				// 0 : PB15				1 : SDA0				
	| (0<<28)				// 0 : PB14				1 : SCL0				
	| (0<<26)				// 0 : PB13				1 : MISO0				
	| (0<<24)				// 0 : PB12				1 : MOSI0
	| (0<<22)				// 0 : PB11				1 : SCK0
	| (0<<20)				// 0 : PB10				1 : SS0
	| (0<<18)				// 0 : PB9				1 : T9O
	| (0<<16)				// 0 : PB8				1 : T8O
	| (0<<14)				// 0 : PB7				1 : T7O
	| (0<<12)				// 0 : PB6				1 : T6O				
	| (0<<10)				// 0 : PB5				1 : T5O				// chagne PB5->T5O
	| (0<<8)   				// 0 : PB4				1 : T4O
	| (0<<6)  			 	// 0 : PB3				1 : T3O
	| (0<<4)   				// 0 : PB2				1 : T2O
	| (0<<2) 				  	// 0 : PB1				1 : T1O
	| (0<<0)  			 	// 0 : PB0				1 : T0O
	;
	
	PCB->CR = 0
	| (0<<30)   				// P15 0:Push-pull output, 1:Open-drain output, 2:Input, 3:Analog
	| (0<<28)   				// P14
	| (0<<26)   				// P13
	| (0<<24)   				// P12
	| (0<<22)   				// P11
	| (0<<20)  				// P10
	| (0<<18)  				// P9
	| (0<<16)   				// P8
	| (0<<14)   				// P7
	| (0<<12)   				// P6
	| (0<<10)   				// P5
	| (0<<8)   				// P4
	| (0<<6)  				// P3
	| (0<<4)   				// P2
	| (0<<2)   				// P1
	| (0<<0)   				// P0
	;
	
	PCB->PCR = 0
	| (0<<15)   				// P15 0:Disable pull-up resistor, 1:Enable pull-up resister
	| (0<<14)   				// P14
	| (0<<13)   				// P13
	| (0<<12)   				// P12
	| (0<<11)   				// P11
	| (0<<10)   				// P10
	| (0<<9)   				// P9
	| (0<<8)   				// P8
	| (0<<7)   				// P7
	| (0<<6)   				// P6
	| (0<<5)   				// P5
	| (0<<4)   				// P4
	| (0<<3)   				// P3
	| (0<<2)   				// P2
	| (0<<1)   				// P1
	| (0<<0)   				// P0	
	;

	PB->SRR = 0xFFFF0000;


	// Port C
	PCC->MR = 0
	| (1<<30)				// 0 : PC15					1 : XTALI					2 : CLKIN		    //!! don't change value
	| (1<<28)				// 0 : PC14					1 : XTALO											//!! don't change value
	| (1<<26)				// 0 : PC13					1 : CLKO											// changed PC13 --> CLKO
	| (0<<24)				// 0 : PC12					1 : STBYO
	| (0<<22)				// 0 : PC11					1 : TXD2
	| (0<<20)				// 0 : PC10			 		1 : RXD2
	| (1<<18)				// 0 : PC9					1 : TXD0
	| (1<<16)				// 0 : PC8					1 : RXD0
	| (0<<14)				// 0 : PC7/BOOT																	//!! don't chagne value				
	| (1<<12)				// 0 : PC6					1 : nRESET											//!! don't change value
	| (0<<10)				// 0 : PC5					
	| (1<<8)   				// 0 : PC4					1 : TDO												//!! don't change value
	| (1<<6)  			 	// 0 : PC3					1 : TCK												//!! don't change value
	| (1<<4)   				// 0 : PC2					1 : TMS												//!! don't change value
	| (1<<2) 				    // 0 : PC1					1 : TDI												//!! don't change value
	| (1<<0)  			 	// 0 : PC0					1 : nTRST											//!! don't change value
	;
	
	PCC->CR = 0
	| (3UL<<30)   // P15 0:Push-pull output, 1:Open-drain output, 2:Input, 3:Analog
	| (3<<28)   // P14						// P15, P14 don't change value
	| (0<<26)   // P13
	| (0<<24)   // P12
	| (0<<22)   // P11
	| (0<<20)   // P10
	| (0<<18)   // P9
	| (2<<16)   // P8	
	| (2<<14)   // P7									//!! don't change value
	| (2<<12)   // P6									//!! don't change value
	| (0<<10)   // P5
	| (0<<8)   // P4									//!! don't change value
	| (2<<6)   // P3									//!! don't change value
	| (2<<4)   // P2									//!! don't change value
	| (2<<2)   // P1									//!! don't change value
	| (2<<0)   // P0									//!! don't change value
	;
	
	PCC->PCR = 0
	| (0<<15)   // P15 0:Disable pull-up resistor, 1:Enable pull-up resister
	| (0<<14)   // P14						
	| (0<<13)   // P13
	| (0<<12)   // P12
	| (0<<11)   // P11
	| (0<<10)   // P10
	| (0<<9)   // P9
	| (1<<8)   // P8
	| (0<<7)   // P7
	| (1<<6)   // P6									//!! don't change value
	| (0<<5)   // P5
	| (0<<4)   // P4
	| (1<<3)   // P3									//!! don't change value
	| (1<<2)   // P2									//!! don't change value
	| (1<<1)   // P1									//!! don't change value
	| (1<<0)   // P0									//!! don't change value
	;
		
	PC->SRR = 0
//	| (1<<(15+16))
	| (1<<(14+16))
	| (1<<(13+16))
	| (1<<12)		// P12 Output High
	| (1<<(11+16))
	| (1<<(10+16))
	| (1<<(9+16))
	| (1<<(7+16))
	| (1<<(6+16))
	| (1<<5)
	| (1<<(4+16))
	| (1<<(3+16))
	| (1<<(2+16))
	| (1<<(1+16))
	| (1<<(0+16))
	;

		
	// Port D
	PCD->MR = 0
	| (0<<30)				// 0 : PD15				1 : SDA1			
	| (0<<28)				// 0 : PD14				1 : SCL1				
	| (0<<26)				// 0 : PD13				1 : TXD1				
	| (0<<24)				// 0 : PD12				1 : RXD1
	| (0<<22)				// 0 : PD11				1 : MISO1
	| (0<<20)				// 0 : PD10				1 : MOSI1
	| (0<<18)				// 0 : PD9				1 : SCK1
	| (0<<16)				// 0 : PD8				1 : SS1
	| (0<<14)				// 0 : PD7				1 : PWMA7
	| (0<<12)				// 0 : PD6				1 : PWMA6	
	| (0<<10)				// 0 : PD5				1 : PWMA5
	| (0<<8)   				// 0 : PD4				1 : PWMA4
	| (0<<6)  			 	// 0 : PD3				1 : PWMA3
	| (0<<4)   				// 0 : PD2				1 : PWMA2
	| (0<<2) 				  	// 0 : PD1				1 : PWMA1
	| (0<<0)  			 	// 0 : PD0				1 : PWMA0
	;
	
	PCD->CR = 0
	| (0<<30)   // P15 0:Push-pull output, 1:Open-drain output, 2:Input, 3:Analog
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
	
	PCD->PCR = 0
	| (0<<15)   // P15 0:Disable pull-up resistor, 1:Enable pull-up resister
	| (0<<14)   // P14
	| (0<<13)   // P13
	| (0<<12)   // P12
	| (0<<11)   // P11
	| (0<<10)   // P10
	| (0<<9)   // P9
	| (0<<8)   // P8
	| (0<<7)   // P7
	| (0<<6)   // P6
	| (0<<5)   // P5
	| (0<<4)   // P4
	| (0<<3)   // P3
	| (0<<2)   // P2
	| (0<<1)   // P1
	| (0<<0)   // P0	
		;

	PD->SRR = 0
//	| (1<<(15+16))
	| (1<<(14+16))
	| (1<<(13+16))
	| (1<<(12+16))
	| (1<<(11+16))
	| (1<<(10+16))
	| (1<<(9+16))
	| (1<<(8+16))
	| (1<<7)
	| (1<<6)
	| (1<<5)				// P7 Output high
	| (1<<4)
	| (1<<3)
	| (1<<2)
	| (1<<1)
	| (1<<0)
	;

	
	// Port E
	PCE->MR = 0
	| (0<<30)				// 0 : PE15					1 : TraceCLK				
	| (0<<28)				// 0 : PE14					1 : TraceD0
	| (0<<26)				// 0 : PE13					1 : TraceD1		
	| (0<<24)				// 0 : PE12					1 : TraceD2
	| (0<<22)				// 0 : PE11					1 : TraceD3
	| (0<<20)				// 0 : PE10				
	| (0<<18)				// 0 : PE9					1 : SXOUT
	| (0<<16)				// 0 : PE8					1 : SXIN
	| (0<<14)				// 0 : PE7					1 : PWMB7				2 : TXD3
	| (0<<12)				// 0 : PE6					1 : PWMB6				2 : RXD3
	| (0<<10)				// 0 : PE5					1 : PWMB5
	| (0<<8)   				// 0 : PE4					1 : PWMB4
	| (0<<6)  			 	// 0 : PE3					1 : PWMB3
	| (0<<4)   				// 0 : PE2					1 : PWMB2
	| (0<<2) 				  	// 0 : PE1					1 : PWMB1
	| (0<<0)  			 	// 0 : PE0					1 : PWMB0
	;
	
	PCE->CR = 0
	| (0<<30)   // P15 		0:Push-pull output, 1:Open-drain output, 2:Input, 3:Analog
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
	
	PCE->PCR = 0
	| (0<<15)   // P15 		0:Disable pull-up resistor, 1:Enable pull-up resister
	| (0<<14)   // P14
	| (0<<13)   // P13
	| (0<<12)   // P12
	| (0<<11)   // P11
	| (0<<10)   // P10
	| (0<<9)   // P9
	| (0<<8)   // P8
	| (0<<7)   // P7
	| (0<<6)   // P6
	| (0<<5)   // P5
	| (0<<4)   // P4
	| (0<<3)   // P3
	| (0<<2)   // P2
	| (0<<1)   // P1
	| (0<<0)   // P0	
	;

	PE->SRR = 0xFFFF0000;
		

	// Port F
	PCF->MR = 0	
	| (0<<22)				// 0 : PF11					2 : FRTO_FMF
	| (0<<20)				// 0 : PF10
	| (0<<18)				// 0 : PF9
	| (0<<16)				// 0 : PF8
	| (0<<14)				// 0 : PF7
	| (0<<10)				// 0 : PF5										3 : AN13
	| (0<<8)   				// 0 : PF4										3 : AN12
	| (0<<6)  			 	// 0 : PF3										3 : AN11
	| (0<<4)   				// 0 : PF2										3 : AN10
	| (0<<2) 					// 0 : PF1										3 : AN9
	| (0<<0)  			 	// 0 : PF0										3 : AN8
	;
	
	PCF->CR = 0
	| (0<<22)				// P11		0:Push-pull output, 1:Open-drain output, 2:Input, 3:Analog
	| (0<<20)				// P10
	| (0<<18)				// P9
	| (0<<16)				// P8
	| (0<<14)				// P7
	| (0<<10)  				// P5			
	| (0<<8)   				// P4
	| (0<<6)   				// P3
	| (0<<4)   				// P2
	| (0<<2)   				// P1
	| (0<<0)   				// P0
	;
	
	PCF->PCR = 0
	| (0<<11)				// P11		0:Disable pull-up resistor, 1:Enable pull-up resister
	| (0<<10)				// P10
	| (0<<9)					// P9
	| (0<<8)					// P8
	| (0<<7)					// P7
	| (0<<5)  				// P5			
	| (0<<4)   				// P4
	| (0<<3)   				// P3
	| (0<<2)   				// P2
	| (0<<1)   				// P1
	| (0<<0)   				// P0	
	;
	
	PF->SRR = 0xFFFF0000;
}



