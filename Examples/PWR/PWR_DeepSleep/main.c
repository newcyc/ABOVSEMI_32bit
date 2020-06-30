/***************************************************************************//**
* @file     main.c
* @brief    An example of deep sleep on A31G22x
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
#include "A31G22x_pcu.h"
#include "A31G22x_pwr.h"
#include "A31G22x_scu.h"

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

#define DEEPSLEEP_TEST_DELAY_LOOP		(10)
#define DEEPSLEEP_TEST_DELAY_TIME		(1000) // 1000ms


/*******************************************************************************
* Private Typedef
*******************************************************************************/


/*******************************************************************************
* Private Variable
*******************************************************************************/
const uint8_t TestMenu[] =
"************************************************\r\n"
"Deep sleep demo\r\n"
"\t - MCU: A31G22x\r\n"
"\t - Core: ARM Cortex-M0+\r\n"
"\t - Communicate via: USART10 - 38400 bps\r\n"
"************************************************\r\n";

static uint32_t MilliSec;

static LVR_CFG_Type LVR_Config;


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/
static void Main_TestDeepSleep(void);
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
* @brief      GPIO F interrupt handler
* @param      None
* @return     None
*******************************************************************************/
void GPIOF_IRQHandler(void)
{
	uint32_t Status;

	// Check F6(USER_SW) interrupt
	Status = GPIO_GetStatus((PORT_Type *)PF) & PORT_ISR_P6_Msk;
	if (Status) {
		GPIO_ClearStatus((PORT_Type *)PF, Status);
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
* @brief      Test Deep Sleep mode
* @param      None
* @return     None
*******************************************************************************/
static void Main_TestDeepSleep(void)
{
	volatile uint32_t i;

	// Disable SysTick
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

	// SCU Status Read Sequence
	_DBG_(" ");
	_DBG("SCU->RSSR = 0x"); _DBH32(SCU->RSSR); _DBG_(" ");
	SCU->RSSR = SCU->RSSR;
	_DBG("Clear SCU->RSSR = 0x"); _DBH32(SCU->RSSR); _DBG_(" ");
	_DBG("SCU->SMR = 0x"); _DBH32(SCU->SMR); _DBG_(" ");
	_DBG("SCU->VDCCON = 0x"); _DBH32(SCU->VDCCON); _DBG_(" ");
	_DBG("Low Voltage Reset"); _DBG_(" ");
	_DBG("\t SCU->LVRCR = 0x"); _DBH32(SCULV->LVRCR); _DBG_(" ");
	_DBG("\t SCU->LVRCNFIG = 0x"); _DBH32(SCULV->LVRCNFIG); _DBG_(" ");

	_DBG_("Press USER_SW(PF6) to Enter Deep Sleep Mode !"); _DBG_(" ");
	while(GPIO_ReadValue((PORT_Type *)PF) & PORT_INDR_INDR6_Msk);

	_DBG_("Enter Deep Sleep... ");
	_DBG_("Once again press USER_SW(PF6) to Exit Deep Sleep Mode !"); _DBG_(" ");
	for (i = 0; i < 1000; i++) {
		__NOP();
	}

	GPIO_ConfigureInterrupt((PORT_Type *)PF, 6, PCU_INTERRUPT_MODE_EDGE, PCU_INTERRUPT_CTRL_EDGE_FALLING);

	// Change Clock to LSI(40KHz)
	SCU_ChangeSysClk(SCCR_LSI);
	SCU_SetLSE(DISABLE);
	SCU_SetHSI(DISABLE);
	SCU_SetHSE(DISABLE);

	PWR_EnterDeepSleep();

	GPIO_ConfigureInterrupt((PORT_Type *)PF, 6, PCU_INTERRUPT_MODE_DISABLE, PCU_INTERRUPT_CTRL_PROHIBIT);

	SCU_ClearWakeUpSRC(WAKEUP_GPIOE);
	NVIC_ClearPendingIRQ(GPIOE_IRQn);
	Main_InitializeClock();

	_DBG_("Waked up from Deep Sleep !"); _DBG_(" ");

	// Enable SysTick
	SysTick->CTRL |= (0x01 << SysTick_CTRL_ENABLE_Pos);

	_DBG_("Toggle LED(PE0)"); _DBG_(" ");
}

/*******************************************************************************
* @brief      Print test menu
* @param      None
* @return     None
*******************************************************************************/
static void Main_PrintMenu(void)
{
	_DBG(TestMenu);
}

/*******************************************************************************
* @brief      Main loop
* @param      None
* @return     None
*******************************************************************************/
static void Main_MainLoop(void)
{
	uint32_t Loop;

	Main_PrintMenu();

	// Test pin setting : PE0
	PCU_ConfigureDirection(PE, 0, PCU_MODE_PUSH_PULL);
	PCU_ConfigurePullupdown(PE, 0, PCU_PUPD_DISABLE);	
	GPIO_ClearValue(PE, _BIT(0));

	// External interrupt pin : F6(USER_SW)
	PCU_ConfigureDirection((PORT_Type *)PF, 6, PCU_MODE_INPUT);
	PCU_ConfigurePullupdown((PORT_Type *)PF, 6, PCU_PUPD_PULL_UP);
	GPIO_ClearValue((PORT_Type *)PF, _BIT(6));
	GPIO_ConfigureInterrupt((PORT_Type *)PF, 6, PCU_INTERRUPT_MODE_DISABLE, PCU_INTERRUPT_CTRL_PROHIBIT);

	// Disable low voltage reset
	LVR_Config.EnableReset = FALSE;
	LVR_Config.LVR_Voltage = LVR_VOLTAGE_160;
	LVR_Init(&LVR_Config);

	// Wakeup setting
	SCU_WakeUpSRCCmd(WAKEUP_GPIOF, ENABLE);

	SCU->SMR = 0
	| (SCU_SMR_LSIAON_Msk)
	| (SCU_SMR_BGRAON_Msk)
	| (SCU_SMR_VDCAON_Msk)
	;

	SysTick_Config(SystemCoreClock / 1000); // 1ms

	NVIC_SetPriority(GPIOF_IRQn, 7);
	NVIC_EnableIRQ(GPIOF_IRQn);

	__enable_irq();

	Loop = DEEPSLEEP_TEST_DELAY_LOOP;
	while (1) {
		Loop++;
		if (Loop > DEEPSLEEP_TEST_DELAY_LOOP) {
			Loop = 0;
			Main_TestDeepSleep();
		}

		MilliSec = DEEPSLEEP_TEST_DELAY_TIME;
		while (MilliSec);
		GPIO_SetValue(PE, _BIT(0));
		
		MilliSec = DEEPSLEEP_TEST_DELAY_TIME;
		while (MilliSec);
		GPIO_ClearValue(PE, _BIT(0));
	}
}

/*******************************************************************************
* @brief      Initialize default clock
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
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
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
	
	PORT_ACCESS_DIS();  // disable writing permittion of ALL PCU register 	
}

/* --------------------------------- End Of File ------------------------------ */
