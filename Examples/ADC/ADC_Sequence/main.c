/***************************************************************************//**
* @file     main.c
* @brief    An example of ADC using interrupt mode on A31G22x
* @author   AE Team, ABOV Semiconductor Co., Ltd.
* @version  V0.0.1
* @date     30. Jul. 2018
*
* Copyright(C) 2018, ABOV Semiconductor
* All rights reserved.
*
*
********************************************************************************
* DISCLAIMER 
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

/*******************************************************************************
* Included File
*******************************************************************************/
#include "A31G22x_adc.h"
#include "A31G22x_pcu.h"
#include "A31g22x_scu.h"
#include "A31g22x_timer1n.h"

#include "debug_frmwrk.h"


/*******************************************************************************
* Pre-processor Definition & Macro
*******************************************************************************/
//-------------------------
#define USED_CLKO
//-------------------------
#define USED_HSI
//#define USED_LSI
//#define USED_MOSC
//#define USED_SOSC
//#define USED_MOSCPLL
//#define USED_HSIPLL


/*******************************************************************************
* Private Typedef
*******************************************************************************/


/*******************************************************************************
* Private Variable
*******************************************************************************/
static const uint8_t TEST_MENU[] =
"************************************************\r\n"
" ADC demo\r\n"
"\t - MCU: A31G22x\r\n"
"\t - Core: ARM Cortex-M0+\r\n"
"\t - Communicate via: USART10 - 38400 bps\r\n"
"\t - Using ADC Sequence(interrupt) mode\r\n"
"************************************************\r\n";

static Bool DoneFlag;

static uint32_t MilliSec;

static TIMER1n_CFG_Type TIMER1n_Config;
static ADC_CFG_Type ADC_Config;


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/
static void Main_InitializeTimer1n(void);
static void Main_PrintMenu(void);
static void Main_MainLoop(void);
static void Main_InitializePCU(void);
static void Main_InitializeClock(void);


/*******************************************************************************
* Public Function
*******************************************************************************/
/*******************************************************************************
* @brief      SysTick interrupt handler @ 1000Hz(1ms)
* @param      None
* @return     None
*******************************************************************************/
void SysTick_Handler(void)
{
	if (MilliSec) {
		MilliSec--;
	}
}

/*******************************************************************************
* @brief      Timer10 interrupt handler
* @param      None
* @return     None
*******************************************************************************/
void TIMER10_IRQHandler(void)
{
	if (TIMER1n_GetStatus(TIMER10) & TIMER1n_STATUS_MATCH_INT) {
		TIMER1n_ClearStatus(TIMER10, TIMER1n_STATUS_MATCH_INT);
	}
}

/*******************************************************************************
* @brief      ADC interrupt handler
* @param      None
* @return     None
*******************************************************************************/
void ADC_IRQHandler(void)
{
	ADC_STATUS_Type status;

	status = ADC_GetStatus(ADC);

	if (status & ADC_STATUS_SEQUENCE_INT) {
		DoneFlag = TRUE;
		ADC_ClearStatus(ADC, ADC_STATUS_SEQUENCE_INT);
	}
}

/*******************************************************************************
* @brief      Main function
* @param      None
* @return     None
*******************************************************************************/
int main(void)
{
	SystemInit();
	Main_InitializePCU();
	Main_InitializeClock();

	DEBUG_Init(DEBUG_INTERFACE_PERI);

	Main_MainLoop();

	return (0);
}


/*******************************************************************************
* Private Function
*******************************************************************************/
/*******************************************************************************
* @brief      Initialize timer1n
* @param      None
* @return     None
*******************************************************************************/
static void Main_InitializeTimer1n(void)
{
	// Initialize and configure timer1n for ADC trigger
	SCU_SetMCCRx(1, TIMER10_TYPE, CLKSRC_MCLK, 2);
	SCU_SetTimer1nClk(T1NCLK_MCCR1); 

	TIMER1n_Config.StartSync = DISABLE;
	TIMER1n_Config.ClearSync = DISABLE;
	TIMER1n_Config.ExtClock = DISABLE; // MCCR1, 24MHz
	TIMER1n_Config.Mode = TIMER1n_MODE_PERIODIC;
	TIMER1n_Config.MatchInterrupt = ENABLE;
	TIMER1n_Config.CaptureInterrupt = DISABLE;

	TIMER1n_Config.AData = 10000 - 1; // 1us * 10000 = 10ms

	TIMER1n_Config.PrescalerData = 24 - 1; // 24MHz / 24 = 1MHz -> 1us

	TIMER1n_Init(TIMER10, &TIMER1n_Config);

	NVIC_SetPriority(TIMER10_IRQn, 3);
	NVIC_EnableIRQ(TIMER10_IRQn);
}

/*******************************************************************************
* @brief      Print menu
* @param      None
* @return     None
*******************************************************************************/
static void Main_PrintMenu(void)
{
	_DBG(TEST_MENU);
}

/*******************************************************************************
* @brief      Main loop
* @param      None
* @return     None
*******************************************************************************/
static void Main_MainLoop(void)
{
	uint32_t Data[4];

	Main_PrintMenu();

	// Initialize Timer1n
	Main_InitializeTimer1n();

	// Set ADC input pin
	PCU_ConfigureDirection(PA, 0, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(PA, 0, PCU_ALT_FUNCTION_3);
	PCU_ConfigurePullupdown(PA, 0, PCU_PUPD_DISABLE);

	PCU_ConfigureDirection(PA, 1, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(PA, 1, PCU_ALT_FUNCTION_3);
	PCU_ConfigurePullupdown(PA, 1, PCU_PUPD_DISABLE);

	PCU_ConfigureDirection(PA, 2, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(PA, 2, PCU_ALT_FUNCTION_3);
	PCU_ConfigurePullupdown(PA, 2, PCU_PUPD_DISABLE);

	PCU_ConfigureDirection(PA, 3, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(PA, 3, PCU_ALT_FUNCTION_3);
	PCU_ConfigurePullupdown(PA, 3, PCU_PUPD_DISABLE);

	// Set ADC external clock
	SCU_SetMCCRx(5, ADC_TYPE, CLKSRC_HSI, 8);

	// Initialize and configure ADC
	ADC_Config.TriggerInfo = DISABLE;
	ADC_Config.ChannelInfo = ENABLE;
	ADC_Config.Restart = DISABLE;

	ADC_Config.Mode = ADC_MODE_SEQUENTIAL;
	ADC_Config.TriggerSelection = ADC_TRIGGER_SEL_TIMER;

	ADC_Config.SamplingTime = 0x1F;
	ADC_Config.SequenceCounter = 4;

	ADC_Config.ExternalClock = ENABLE;
	ADC_Config.InternalClockDivider = 1;

	ADC_Init(ADC, &ADC_Config);
	ADC_ConfigureInterrupt(ADC, ADC_INTERRUPT_SEQUENCE, ENABLE);
	Data[0] = ADC_TRIGGER_SOURCE_0;
	Data[1] = ADC_TRIGGER_SOURCE_0;
	Data[2] = ADC_TRIGGER_SOURCE_0;
	Data[3] = ADC_TRIGGER_SOURCE_0;
	ADC_ConfigureAllTrigger(ADC, 4, (ADC_TRIGGER_SOURCE_Type *)(&Data[0]));
	Data[0] = ADC_CHANNEL_0;
	Data[1] = ADC_CHANNEL_1;
	Data[2] = ADC_CHANNEL_2;
	Data[3] = ADC_CHANNEL_3;
	ADC_ConfigureAllSequence(ADC, 4, (ADC_CHANNEL_Type *)(&Data[0]));

	// Enable SysTick and ADC interrupt
	SysTick_Config(SystemCoreClock / 1000);

	NVIC_SetPriority(ADC_IRQn, 3);
	NVIC_EnableIRQ(ADC_IRQn);

	__enable_irq();

	// Start timer
	DoneFlag = FALSE;
	TIMER1n_Start(TIMER10);
	while (1) {
		ADC_Start(ADC);

		while (DoneFlag == FALSE);
		DoneFlag = TRUE;

		Data[0] = ADC_GetConversionData(ADC, ADC_DATA_SEL_SEQ0);
		Data[1] = ADC_GetConversionData(ADC, ADC_DATA_SEL_SEQ1);
		Data[2] = ADC_GetConversionData(ADC, ADC_DATA_SEL_SEQ2);
		Data[3] = ADC_GetConversionData(ADC, ADC_DATA_SEL_SEQ3);
		
		_DBG("AN0 = "); _DBH16(Data[0]); _DBG(" ");
		_DBG("AN1 = "); _DBH16(Data[1]); _DBG(" ");
		_DBG("AN2 = "); _DBH16(Data[2]); _DBG(" ");
		_DBG("AN3 = "); _DBH16(Data[3]); _DBG_(" ");

		// 1 second delay
		MilliSec = 1000;
		while (MilliSec);
	}
}

/*******************************************************************************
* @brief      Initialize default clock for A31G22x
* @param      None
* @return     None
*******************************************************************************/
static void Main_InitializeClock(void)
{
	uint32_t i;

//	CLKO function setting. check PORT setting (PF4).

#ifdef USED_CLKO
	SCU_SetCOR(4,ENABLE); //    /10
#else
	SCU_SetCOR(4,DISABLE);
#endif

	SCU->CMR&=~(1<<7); //mclk monitoring disable
	
#ifdef USED_LSI			//500khz
	SCU_SetLSI(LSI_EN); //LSI_EN_DIV2, LSI_EN_DIV4
	SystemCoreClock=500000; //500khz
	SystemPeriClock=500000; //500khz	
	
	for (i=0;i<10;i++);	
	
	SCU_ChangeSysClk(SCCR_LSI);
#endif 
	
#ifdef USED_SOSC  //32.768khz
	SCU_SetLSE(LSE_EN);
	SystemCoreClock=32768; //32.768khz
	SystemPeriClock=32768; //32.768khz	

	for (i=0;i<100;i++);	

// wait for SOSC stable	
	SCU_WaitForLSEStartUp();
	SCU_ChangeSysClk(SCCR_LSE);
#endif 	

#ifdef USED_HSI		//48MHz
	SCU_SetHSI(HSI_EN);
	SystemCoreClock=48000000; //48MHz
	SystemPeriClock=48000000; //48MHz	
	
	for (i=0;i<10;i++);	

	SCU_ChangeSysClk(SCCR_HSI);
#endif 	

#ifdef USED_MOSC	//xMHz
	SCU_SetHSE(HSE_EN);
	SystemCoreClock=8000000; //xMHz
	SystemPeriClock=8000000; //xMHz	

	for (i=0;i<100;i++);	

	SCU_WaitForHSEStartUp();
	SCU_ChangeSysClk(SCCR_HSE);
#endif


#ifdef USED_MOSCPLL
// PLL setting 
//    FIN=PLLINCLK/(R+1)                                             ; R: Pre Divider   
//    FOUT=(FIN*(N1+1)*(D+1))  / ((N2+1)*(P+1))          ; N1: Post Divider1, N2:Post Divider2, P:Output Divider,      
//             = FVCO *(D+1)                                             ; D:Frequency Doubler
//
//ex)    FIN=PLLINCLK/(R+1) = 8M/(3+1) = 2M                               ; R:3, PLLINCLK:8MHz(MOSC)
//         FOUT=(2M*(47+1)*(0+1)) / ((1+1)*(0+1) = 48MHz              ; N1:47, D:0, N2:1, P:0
//
	if (SCU_SetPLLandWaitForPLLStartUp(ENABLE,  
		PLLCON_BYPASS_PLL,    //PLLCON_BYPASS_FIN:0, PLLCON_BYPASS_PLL:1
		0,                                    //0:FOUT==VCO, 1:FOUT==2xVCO,  D=0
//		3,                                    //PREDIV, R=3
		1,                                    //PREDIV, R=1
		47,                                  //POSTDIV1, N1=47  
		1,                                    //POSTDIV2, N2=1
		0)==ERROR)                    //OUTDIV P=0
	{
		while(1);
	}

//	EOSC -->  EOSCPLL
	SCU_ChangeSysClk(SCCR_HSE_PLL);
	
	SystemCoreClock=48000000; 
	SystemPeriClock=48000000; 
#endif

	
#ifdef USED_HSIPLL
// PLL setting 
//    FIN=PLLINCLK/(R+1)                                             ; R: Pre Divider   
//    FOUT=(FIN*(N1+1)*(D+1))  / ((N2+1)*(P+1))          ; N1: Post Divider1, N2:Post Divider2, P:Output Divider,      
//             = FVCO *(D+1)                                             ; D:Frequency Doubler
//
//ex)    FIN=PLLINCLK/(R+1) = 8M/(3+1) = 2M                               ; R:3, PLLINCLK:8MHz(MOSC)
//         FOUT=(2M*(47+1)*(0+1)) / ((1+1)*(0+1) = 48MHz              ; N1:47, D:0, N2:1, P:0
//
	if (SCU_SetPLLandWaitForPLLStartUp(ENABLE,  
		PLLCON_BYPASS_PLL,    //PLLCON_BYPASS_FIN:0, PLLCON_BYPASS_PLL:1
		0,                                    //0:FOUT==VCO, 1:FOUT==2xVCO,  D=0
//		3,                                    //PREDIV, R=3
		1,                                    //PREDIV, R=1
		47,                                  //POSTDIV1, N1=47  
		1,                                    //POSTDIV2, N2=1
		0)==ERROR)                    //OUTDIV P=0
	{
		while(1);
	}

//	EOSC -->  EOSCPLL
	SCU_ChangeSysClk(SCCR_HSI_PLL);
	
	SystemCoreClock=48000000; 
	SystemPeriClock=48000000; 
#endif

	SCU->CMR|=(1<<7); //mclk monitoring enable

// wait setting order 1. default wait setting -> 2. clock change -> 3. adjust wait setting
//// flash memory controller
	CFMC->MR = 0x81;       // after changing 0x81 -> 0x28 in MR reg, flash access timing will be able to be set.
	CFMC->MR = 0x28;       // enter flash access timing changing mode
//	CFMC->CFG = (0x7858<<16) | (0<<8);  //flash access cycles 	20
//	CFMC->CFG = (0x7858<<16) | (1<<8);  //flash access cycles 	40
	CFMC->CFG = (0x7858<<16) | (2<<8);  //flash access cycles 	60		
//	CFMC->CFG = (0x7858<<16) | (3<<8);  //flash access cycles 		
	                              // flash access time cannot overflow 20MHz.
	                              // ex) if MCLK=48MHz, 
	                              //       48/1 = 48 (can't set no wait)
	                              //       48/2 = 24 (1 wait is not ok)
	                              //       48/3 = 16 (2 wait is ok)								  
	                              // so, 2 wait is possible.
	CFMC->MR = 0;	      // exit flash access timing --> normal mode				
}

/*******************************************************************************
* @brief      Initialize PCU(IO Port)
* @param      None
* @return     None
*******************************************************************************/
static void Main_InitializePCU(void)
{
//Peripheral Enable Register 1  0:Disable, 1:Enable	
	SCU->PER1=SCU->PER1  
	| (1<<13)   // GPIOF
	| (1<<12)   // GPIOE
	| (1<<11)   // GPIOD
	| (1<<10)   // GPIOC
	| (1<<9)   // GPIOB
	| (1<<8)   // GPIOA
		; 		
//Peripheral Clock Enable Register 1 0:Disable, 1:Enable	
	SCU->PCER1=SCU->PCER1
	| (1<<13)   // GPIOF
	| (1<<12)   // GPIOE
	| (1<<11)   // GPIOD
	| (1<<10)   // GPIOC
	| (1<<9)   // GPIOB
	| (1<<8)   // GPIOA
		; 	

 	PORT_ACCESS_EN();  // enable writing permittion of ALL PCU register

	//--------------------------------------------------------------
	//	PORT INIT
	//		PA, PB, PC, PD, PE, PF
	//--------------------------------------------------------------
	// PORT - A
	PA->MOD = 0					// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9

	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PA->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9

	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PA->AFSR1 = 0
	| (0x00<<28)						// P7		0 : SEG 42,	1 : 			, 2 :				, 3 : AN7/CREF0/DAVREF	, 4 : ISEG18	, 5 : 
	| (0x00<<24)						// P6		0 : SEG 43,	1 : 			, 2 :				, 3 : AN6/CREF1/DAO		, 4 : ISEG19	, 5 : 
	| (0x00<<20)						// P5		0 :			,	1 : T12O	, 2 : T12CA	, 3 : AN5/CP1A/DAO			, 4 : ISEG20	, 5 : 
	| (0x00<<16)						// P4		0 :			,	1 : 			, 2 :				, 3 : AN4/CP1B					, 4 : ISEG21	, 5 : CS7
	| (0x00<<12)						// P3		0 : 			,	1 : 			, 2 :				, 3 : AN3/CP1C					, 4 : ISEG22	, 5 : CS6
	| (0x00<<8)						// P2		0 : 			,	1 : EC12	, 2 :				, 3 : AN2/AVREF/CP0		, 4 : ISEG23	, 5 : CS5
	| (0x00<<4)						// P1		0 : 			,	1 : SCL1	, 2 :				, 3 : AN1/USBDP				, 4 : ISEG24	, 5 : CS4
	| (0x00<<0)						// P0		0 : 			,	1 : SDA1	, 2 :				, 3 : AN0/USBDM				, 4 : ISEG25	, 5 : CS3
	;

	PA->AFSR2 = 0
	| (0x00<<12)						// P11		0 : 			,	1 : 			, 2 :				, 3 : AN14							, 4 : 				, 5 : CS2
	| (0x00<<8)						// P10		0 : 			,	1 : 			, 2 :				, 3 : AN13							, 4 : 				, 5 : CS1
	| (0x00<<4)						// P9		0 : 			,	1 : 			, 2 :				, 3 : AN12							, 4 : 				, 5 : CS0
	;
	
	PA->PUPD = 0				// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9

	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PA->OUTDR = 0				// 0 : Output Low,	1 : Output High
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9

	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;

//////////////////////////////////////////////////////////////////////////////////////////////////////
	// PORT - B
	PB->MOD = 0					// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<30)						// P15
	| (0x01<<28)						// P14
	| (0x01<<26)						// P13
	| (0x01<<24)						// P12
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9
	| (0x01<<16)						// P8
	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x02<<10)						// P5
	| (0x02<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PB->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<15)						// P15
	| (0x00<<14)						// P14
	| (0x00<<13)						// P13
	| (0x00<<12)						// P12
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x02<<5)						// P5
	| (0x02<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PB->AFSR1 = 0
	| (0x01<<28)					// P7		0 : SEG 34,	1 : RXD1	, 2 : 				, 3 :			, 4 : ISEG10	, 5 : 
	| (0x01<<24)					// P6		0 : SEG 35,	1 : TXD1	, 2 :				, 3 :			, 4 : ISEG11	, 5 : 
	| (0x02<<20)					// P5		0 : SEG 36,	1 : RXD0	, 2 : SWDIO	, 3 :			, 4 : ISEG12	, 5 : 
	| (0x02<<16)					// P4		0 : SEG 37,	1 : TXD0	, 2 : SWCLK	, 3 :			, 4 : ISEG13	, 5 : 
	| (0x01<<12)					// P3		0 : SEG 38,	1 : BOOT	, 2 : SS10		, 3 :			, 4 : ISEG14	, 5 : 
	| (0x00<<8)					// P2		0 : SEG 39,	1 : 			, 2 : SCK10	, 3 : AN10	, 4 : ISEG15	, 5 : CS10
	| (0x00<<4)					// P1		0 : SEG 40,	1 : RXD10	, 2 : MISO10	, 3 : AN9	, 4 : ISEG16	, 5 : CS9
	| (0x00<<0)					// P0		0 : SEG 41,	1 : TXD10	, 2 : MOSI10	, 3 : AN8	, 4 : ISEG17	, 5 : CS8
	;

	PB->AFSR2 = 0
	| (0x0<<28)					// P15		0 : SEG 26,	1 :			, 2 :				, 3 :			, 4 :			, 5 : CS18
	| (0x0<<24)					// P14		0 : SEG 27,	1 : 			, 2 :				, 3 :			, 4 :			, 5 : CS17
	| (0x0<<20)					// P13		0 : SEG 28,	1 :			, 2 :				, 3 :			, 4 :			, 5 : CS16
	| (0x0<<16)					// P12		0 : SEG 29,	1 :			, 2 :				, 3 :			, 4 :			, 5 : CS15
	| (0x0<<12)					// P11		0 : SEG 30,	1 : T15C	, 2 : EC16		, 3 :			, 4 : T15O	, 5 : CS14
	| (0x0<<8)					// P10		0 : SEG 31,	1 : T16C	, 2 : EC15		, 3 : AN10	, 4 : T16O	, 5 : CS13
	| (0x0<<4)					// P9		0 : SEG 32,	1 : T16O	, 2 : T16C		, 3 : AN9	, 4 : EC15	, 5 : CS12
	| (0x0<<0)					// P8		0 : SEG 33,	1 : T15O	, 2 : T15C		, 3 : AN8	, 4 : EC16	, 5 : CS11
	;
	
	PB->PUPD = 0			// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<30)						// P15
	| (0x0<<28)						// P14
	| (0x0<<26)						// P13
	| (0x0<<24)						// P12
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9
	| (0x0<<16)						// P8
	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PB->OUTDR = 0			// 0 : Output Low,	1 : Output High
	| (0x00<<15)						// P15
	| (0x00<<14)						// P14
	| (0x00<<13)						// P13
	| (0x00<<12)						// P12
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8	
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;

//////////////////////////////////////////////////////////////////////////////////////////////////////
	// PORT - C
	PC->MOD = 0				// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<24)						// P12
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9
	| (0x01<<16)						// P8
	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PC->TYP = 0				// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<12)						// P12
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PC->AFSR1 = 0
	| (0x0<<28)					// P7		0 : SEG 18	, 1 :				, 2 :				, 3 : 		, 4 :				, 5 :	
	| (0x0<<24)					// P6		0 : SEG 19	, 1 : SCL2		, 2 :				, 3 : 		, 4 :				, 5 :	
	| (0x0<<20)					// P5		0 : SEG 20	, 1 : SDA2		, 2 :				, 3 : 		, 4 :				, 5 :	
	| (0x0<<16)					// P4		0 : SEG 21	, 1 :				, 2 :				, 3 : 		, 4 : ISEG5	, 5 : CS23 
	| (0x0<<12)					// P3		0 : SEG 22	, 1 : EC21		, 2 :				, 3 : 		, 4 : ISEG6	, 5 : CS22 
	| (0x0<<8)					// P2		0 : SEG 23	, 1 : EC20		, 2 :				, 3 : 		, 4 : ISEG7	, 5 : CS21 
	| (0x0<<4)					// P1		0 : SEG 24	, 1 : T21O		, 2 : T21C		, 3 : 		, 4 : ISEG8	, 5 : CS20 
	| (0x0<<0)					// P0		0 : SEG 25	, 1 : T20O		, 2 : T20C		, 3 : 		, 4 : ISEG9	, 5 : CS19 
	;
	PC->AFSR2 = 0
	| (0x0<<16)					// P12		0 : SEG 13	, 1 : EC11		, 2 :				, 3 : 		, 4 :				, 5 :	
	| (0x0<<12)					// P11		0 : SEG 14	, 1 : EC10		, 2 :				, 3 : 		, 4 :				, 5 :	
	| (0x0<<8)					// P10		0 : SEG 15	, 1 :				, 2 :				, 3 : 		, 4 :				, 5 :	
	| (0x0<<4)					// P9		0 : SEG 16	, 1 :				, 2 :	T21CAP, 3 : 		, 4 :				, 5 :	
	| (0x0<<0)					// P8		0 : SEG 17	, 1 :				, 2 :	T20CAP, 3 : 		, 4 :				, 5 :	
	;
	
	PC->PUPD = 0				// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<24)						// P12
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9
	| (0x0<<16)						// P8
	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PC->OUTDR = 0			// 0 : Output Low,	1 : Output High
	| (0x0<<12)						// P12
	| (0x0<<11)						// P11
	| (0x0<<10)						// P10
	| (0x0<<9)						// P9
	| (0x0<<8)						// P8	
	| (0x0<<7)						// P7
	| (0x0<<6)						// P6
	| (0x0<<5)						// P5
	| (0x0<<4)						// P4
	| (0x0<<3)						// P3
	| (0x0<<2)						// P2
	| (0x0<<1)						// P1
	| (0x0<<0)						// P0
	;

//////////////////////////////////////////////////////////////////////////////////////////////////////
	// PORT - D
	PD->MOD = 0			// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PD->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PD->AFSR1 = 0
	| (0x00<<20)					// P5		0 : SEG 7	,	1 : 			, 2 : SS11			, 3 : 		, 4 : ICOM8				, 5 : SS11
	| (0x00<<16)					// P4		0 : SEG 8	,	1 :			, 2 : SCK11		, 3 : 		, 4 : ICOM9/ISEG0	, 5 : SCK11
	| (0x00<<12)					// P3		0 : SEG 9	,	1 : RXD11	, 2 : MISO11		, 3 : 		, 4 : ICOM10/ISEG1	, 5 : MISO11
	| (0x00<<8)					// P2		0 : SEG 10,	1 : TXD11	, 2 : MOSI11		, 3 : 		, 4 : ISEG2				, 5 : MOSI11
	| (0x00<<4)					// P1		0 : SEG 11,	1 : SDA0	, 2 :					, 3 : 		, 4 : ISEG3				, 5 : 
	| (0x00<<0)					// P0		0 : SEG 12,	1 : SCL0	, 2 :					, 3 : 		, 4 : ISEG4				, 5 : 
	;
	
	PD->PUPD = 0			// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PD->OUTDR = 0			// 0 : Output Low,	1 : Output High
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;

//////////////////////////////////////////////////////////////////////////////////////////////////////
	// PORT - E
	PE->MOD = 0			// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<30)						// P15
	| (0x01<<28)						// P14
	| (0x01<<26)						// P13
	| (0x01<<24)						// P12
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9
	| (0x01<<16)						// P8
	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PE->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<15)						// P15
	| (0x00<<14)						// P14
	| (0x00<<13)						// P13
	| (0x00<<12)						// P12
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PE->AFSR1 = 0
	| (0x00<<28)					// P7		0 : COM7/SEG4	, 1 : T11O			, 2 : T11C	, 3 :		, 4 : ICOM7	, 5 :
	| (0x00<<24)					// P6		0 : COM6/SEG3	, 1 : T10O			, 2 : T10C	, 3 :		, 4 : ICOM6	, 5 :
	| (0x00<<20)					// P5		0 : COM5/SEG2	, 1 : PWM30CB	, 2 :			, 3 :		, 4 : ICOM5	, 5 :
	| (0x00<<16)					// P4		0 : COM4/SEG1	, 1 : PWM30CA	, 2 :			, 3 :		, 4 : ICOM4	, 5 :
	| (0x00<<12)					// P3		0 : COM3/SEG0	, 1 : PWM30BB	, 2 :			, 3 :		, 4 : ICOM3	, 5 :
	| (0x00<<8)					// P2		0 : COM2				, 1 : PWM30BA	, 2 :			, 3 :		, 4 : ICOM2	, 5 :
	| (0x00<<4)					// P1		0 : COM1				, 1 : PWM30AB	, 2 :			, 3 :		, 4 : ICOM1	, 5 :
	| (0x00<<0)					// P0		0 : COM0				, 1 : PWM30AA	, 2 :			, 3 :		, 4 : ICOM0	, 5 :
	;
	PE->AFSR2 = 0
	| (0x00<<28)					// P15		0 : SEG6	, 1 :					, 2 : SS12			, 3 :				, 4 :		, 5 :
	| (0x00<<24)					// P14		0 : SEG5	, 1 :					, 2 : SCK12		, 3 :				, 4 :		, 5 :
	| (0x00<<20)					// P13		0 :			, 1 : RXD13		, 2 : MISO12		, 3 :				, 4 :		, 5 :
	| (0x00<<16)					// P12		0 :			, 1 : TXD12		, 2 : MOSI12		, 3 :				, 4 :		, 5 :
	| (0x00<<12)					// P11		0 :			, 1 :					, 2 : SS13			, 3 : VLC3		, 4 :		, 5 :
	| (0x00<<8)					// P10		0 :			, 1 :					, 2 : SCK13		, 3 : VLC2		, 4 :		, 5 :
	| (0x00<<4)					// P9		0 :			, 1 : RXD13		, 2 : MISO13		, 3 : VLC1		, 4 :		, 5 :
	| (0x00<<0)					// P8		0 :			, 1 : TXD13		, 2 : MOSI13		, 3 : VLC0		, 4 :		, 5 :
	;
	
	PE->PUPD = 0			// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<30)						// P15
	| (0x0<<28)						// P14
	| (0x0<<26)						// P13
	| (0x0<<24)						// P12
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9
	| (0x0<<16)						// P8
	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PE->OUTDR = 0				// 0 : Output Low,	1 : Output High
	| (0x0<<15)						// P15
	| (0x0<<14)						// P14
	| (0x0<<13)						// P13
	| (0x0<<12)						// P12
	| (0x0<<11)						// P11
	| (0x0<<10)						// P10
	| (0x0<<9)						// P9
	| (0x0<<8)						// P8
	| (0x0<<7)						// P7
	| (0x0<<6)						// P6
	| (0x0<<5)						// P5
	| (0x0<<4)						// P4
	| (0x0<<3)						// P3
	| (0x0<<2)						// P2
	| (0x0<<1)						// P1
	| (0x0<<0)						// P0
	;
	
//////////////////////////////////////////////////////////////////////////////////////////////////////	
	// PORT - F
	PF->MOD = 0					// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9
	| (0x01<<16)						// P8
	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PF->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x0<<11)						// P11
	| (0x0<<10)						// P10
	| (0x0<<9)						// P9
	| (0x0<<8)						// P8
	| (0x0<<7)						// P7
	| (0x0<<6)						// P6
	| (0x0<<5)						// P5
	| (0x0<<4)						// P4
	| (0x0<<3)						// P3
	| (0x0<<2)						// P2
	| (0x0<<1)						// P1
	| (0x0<<0)						// P0
	;
	
	PF->AFSR1 = 0
	| (0x0<<28)					// P7		0 :		, 1 : T30C		, 2 : SDA0		, 3 :				, 4 :				, 5 : 
	| (0x0<<24)					// P6		0 :		, 1 : EC30		, 2 : SCL0		, 3 :				, 4 :				, 5 : 
	| (0x0<<20)					// P5		0 :		, 1 : BLNK		, 2 : INT		, 3 :				, 4 :				, 5 : 
	| (0x0<<16)					// P4		0 :		, 1 : CLKO		, 2 :				, 3 : R-SET	, 4 :				, 5 : 
	| (0x0<<12)					// P3		0 :		, 1 : RXD1		, 2 :				, 3 : SXOUT	, 4 :				, 5 : 
	| (0x0<<8)					// P2		0 :		, 1 : TXD1		, 2 :				, 3 : SXIN		, 4 :				, 5 : 
	| (0x0<<4)					// P1		0 :		, 1 : SDA1		, 2 :				, 3 : XIN		, 4 :				, 5 : 
	| (0x0<<0)					// P0		0 :		, 1 : SCL1		, 2 :				, 3 : XOUT	, 4 : ISEG26	, 5 : 
	;
	PF->AFSR2 = 0
	| (0x0<<12)					// P11		0 : SXOUT, 1 : T14O		, 2 : T14C		, 3 : 			, 4 :				, 5 : 
	| (0x0<<8)					// P10		0 : SXIN	, 1 : T13O		, 2 : T13C		, 3 : 			, 4 :				, 5 : 
	| (0x0<<4)					// P9		0 : XIN		, 1 : EC14		, 2 :				, 3 : 			, 4 :				, 5 : 
	| (0x0<<0)					// P8		0 : XOUT	, 1 : EC13		, 2 :				, 3 : 			, 4 :				, 5 : 
	;
	
	PF->PUPD = 0			// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9
	| (0x0<<16)						// P8
	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PF->OUTDR = 0			// 0 : Output Low,	1 : Output High
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8	
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PORT_ACCESS_DIS(); // disable writing permittion of ALL PCU register 	
}

/* --------------------------------- End Of File ------------------------------ */
