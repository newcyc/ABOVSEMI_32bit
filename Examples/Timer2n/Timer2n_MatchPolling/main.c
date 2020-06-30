/***************************************************************************//**
* @file     main.c
* @brief    An example of 32-bit timer2n using match(polling) mode on A31G22x
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
#include "A31G22x_scu.h"
#include "A31G22x_timer2n.h"

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

#define USED_CLOCK_MCCR				(0)
#define USED_CLOCK_PCLK				(1)
#define USED_CLOCK_EC				(2)

#define TIMER2n_CLOCK				(USED_CLOCK_MCCR)

#define TARGET_TIMER				(TIMER2n_IP_INDEX_TIMER20) // TIMER2n_INDEX_TIMER20 ~ TIMER2n_INDEX_TIMER21


/*******************************************************************************
* Private Typedef
*******************************************************************************/
// Timer2n match demo configuration structure definition
typedef struct {
	// base address
	TIMER2n_Type *pTIMER2x;
	// IRQ number
	IRQn_Type TIMER2x_IRQ;
	// Output port
	PORT_Type *pOutputPort_Group;
	uint32_t OutputPort_Number;
	PCU_ALT_FUNCTION_Type OutputPort_AltFunc;
	// External clock port
	PORT_Type *pExtClockPort_Group;
	uint32_t ExtClockPort_Number;
	PCU_ALT_FUNCTION_Type ExtClockPort_AltFunc;
} TIMER2n_MATCH_Type;


/*******************************************************************************
* Private Variable
*******************************************************************************/
static const uint8_t TEST_MENU[] =
"************************************************\r\n"
"Timer2n demo\r\n"
"\t - MCU: A31G22x\r\n"
"\t - Core: ARM Cortex-M0+\r\n"
"\t - Communicate via: USART10 - 38400 bps\r\n"
"\t - Using timer2n match(polling) mode\r\n"
"************************************************\r\n";

static const TIMER2n_MATCH_Type TIMER2n_MATCH_CFG[TIMER2n_IP_INDEX_MAX] = {
	{TIMER20, TIMER20_IRQn, PC, 0, PCU_ALT_FUNCTION_1, PC, 2, PCU_ALT_FUNCTION_1},
	{TIMER21, TIMER21_IRQn, PC, 1, PCU_ALT_FUNCTION_1, PC, 3, PCU_ALT_FUNCTION_1}
};

static TIMER2n_CFG_Type TIMER2n_Config;


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/
static void Main_PrintMenu(void);
static void Main_MainLoop(void);
static void Main_InitializeClock(void);
static void Main_InitializePCU(void);


/*******************************************************************************
* Public Function
*******************************************************************************/

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
* @brief      Print test menu
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
	Bool Flag;
	uint32_t Count;
	const TIMER2n_MATCH_Type * pConfig;

	pConfig = &(TIMER2n_MATCH_CFG[TARGET_TIMER]);

	Main_PrintMenu();

	// Test port(PE0) setting
	PCU_ConfigureDirection(PE, 0, PCU_MODE_PUSH_PULL);
	PCU_ConfigurePullupdown(PE, 0, PCU_PUPD_DISABLE);	
	GPIO_ClearValue(PE, _BIT(0));

	// Timer2n output port setting
	PCU_ConfigureDirection(pConfig->pOutputPort_Group, pConfig->OutputPort_Number, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(pConfig->pOutputPort_Group, pConfig->OutputPort_Number, pConfig->OutputPort_AltFunc);

	// Timer2n and clock source setting (MCCR1 / PCLK / ECn)
#if (TIMER2n_CLOCK == USED_CLOCK_MCCR)
	SCU_SetMCCRx(2, TIMER20_TYPE, CLKSRC_MCLK, 2);
	SCU_SetTimer20Clk(T20CLK_MCCR2); 

	TIMER2n_Config.ExtClock = DISABLE; // MCCR 16MHz
	TIMER2n_Config.PrescalerData = 16 - 1; // 16MHz / 16 = 1MHz -> 1us
#elif (TIMER2n_CLOCK == USED_CLOCK_PCLK)
	SCU_SetTimer20Clk(T20CLK_PCLK);

	TIMER2n_Config.ExtClock = DISABLE; // PCLK 32MHz
	TIMER2n_Config.PrescalerData = 32 - 1; // 32MHz / 32 = 1MHz -> 1us
#elif (TIMER2n_CLOCK == USED_CLOCK_EC)
	PCU_ConfigureDirection(pConfig->pExtClockPort_Group, pConfig->ExtClockPort_Number, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(pConfig->pExtClockPort_Group, pConfig->ExtClockPort_Number, pConfig->ExtClockPort_AltFunc);

	TIMER2n_Config.ExtClock = ENABLE; // EC
	TIMER2n_Config.ExtClockEdge = TIMER2n_EXT_CLOCK_EDGE_FALLING;
	TIMER2n_Config.PrescalerData = 0; // Bypass
#endif

	TIMER2n_Config.Mode = TIMER2n_MODE_PERIODIC;
	TIMER2n_Config.OutputPolarity = TIMER2n_OUTPUT_POLARITY_LOW;
	TIMER2n_Config.MatchInterrupt = DISABLE;
	TIMER2n_Config.CaptureInterrupt = DISABLE;

	TIMER2n_Config.AData = 1000 - 1; // 1ms

	TIMER2n_Init(pConfig->pTIMER2x, &TIMER2n_Config);

	TIMER2n_Start(pConfig->pTIMER2x);

	Flag = FALSE;
	Count = 0;

	while (1) {
		while ((TIMER2n_GetStatus(pConfig->pTIMER2x) & TIMER2n_STATUS_MATCH_INT) == 0);
		TIMER2n_ClearStatus(pConfig->pTIMER2x, TIMER2n_STATUS_MATCH_INT);

		Count++;
		if (Count >= 200) {
			Count = 0;

			if (Flag == TRUE) {
				Flag = FALSE;
				GPIO_ClearValue(PE, _BIT(0));
			} else {
				Flag = TRUE;
				GPIO_SetValue(PE, _BIT(0));
			}
		}
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

#ifdef USED_HSI // 32MHz
	SCU_SetHSI(HSI_EN);
	SystemCoreClock = 32000000; // 32MHz
	SystemPeriClock = 32000000; // 32MHz

	for (i = 0; i < 10; i++);

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
	// Peripheral Enable Register 1  0:Disable, 1:Enable	
	SCU->PER1 |= 0x00UL 
			| (0x01UL << SCU_PER1_GPIOF_Pos) // GPIO F
			| (0x01UL << SCU_PER1_GPIOE_Pos) // GPIO E
			| (0x01UL << SCU_PER1_GPIOD_Pos) // GPIO D
			| (0x01UL << SCU_PER1_GPIOC_Pos) // GPIO C
			| (0x01UL << SCU_PER1_GPIOB_Pos) // GPIO B
			| (0x01UL << SCU_PER1_GPIOA_Pos) // GPIO A
			;
	// Peripheral Clock Enable Register 1 0:Disable, 1:Enable	
	SCU->PCER1 |= 0x00UL
			| (0x01UL << SCU_PCER1_GPIOF_Pos) // GPIO F
			| (0x01UL << SCU_PCER1_GPIOE_Pos) // GPIO E
			| (0x01UL << SCU_PCER1_GPIOD_Pos) // GPIO D
			| (0x01UL << SCU_PCER1_GPIOC_Pos) // GPIO C
			| (0x01UL << SCU_PCER1_GPIOB_Pos) // GPIO B
			| (0x01UL << SCU_PCER1_GPIOA_Pos) // GPIO A
			;

	PORT_ACCESS_EN();  // enable writing permittion of ALL PCU register

	//--------------------------------------------------------------
	//	PORT INIT
	//		PA, PB, PC, PD, PE, PF
	//--------------------------------------------------------------
	// PORT - A
	PA->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PA->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PA->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 :               , 2 :               , 3 : AN7/CREF0     , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : T11O          , 2 : T11C          , 3 : AN6/CREF1/DAO , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : T12O          , 2 : T12C          , 3 : AN5/CP1A      , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 :               , 2 :               , 3 : AN4/CP1B      , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 :               , 2 :               , 3 : AN3/CP1C      , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : EC12          , 2 :               , 3 : AN2/CP0       , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : SCL1          , 2 :               , 3 : AN1           , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : SDA1          , 2 :               , 3 : AN0           , 4 :               
			;

	PA->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 :               , 2 :               , 3 : AN14          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 :               , 2 :               , 3 : AN13          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 :               , 2 :               , 3 : AN12          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 :               , 2 :               , 3 : AN11          , 4 :               
			;

	PA->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PA->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - B
	PB->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE15_Pos) // P15
			| (0x01UL << PORT_MOD_MODE14_Pos) // P14
			| (0x01UL << PORT_MOD_MODE13_Pos) // P13
			| (0x01UL << PORT_MOD_MODE12_Pos) // P12
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x02UL << PORT_MOD_MODE5_Pos)  // P5  - Alternative function mode (SWDIO)
			| (0x02UL << PORT_MOD_MODE4_Pos)  // P4  - Alternative function mode (SWCLK)
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PB->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP15_Pos) // P15
			| (0x00UL << PORT_TYP_TYP14_Pos) // P14
			| (0x00UL << PORT_TYP_TYP13_Pos) // P13
			| (0x00UL << PORT_TYP_TYP12_Pos) // P12
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PB->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 : RXD1          , 2 :               , 3 : AN16          , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : TXD1          , 2 : EC11          , 3 : AN15          , 4 :               
			| (0x02UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : RXD0          , 2 : SWDIO         , 3 :               , 4 :               
			| (0x02UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 : TXD0          , 2 : SWCLK         , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : BOOT          , 2 : SS10/SS20     , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 :               , 2 : SCK10/SCK20   , 3 : AN10          , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : RXD10         , 2 : MISO10/MISO20 , 3 : AN9           , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : TXD10         , 2 : MOSI10/MOSI20 , 3 : AN8           , 4 :               
			;

	PB->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR15_Pos) // P15 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR14_Pos) // P14 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR13_Pos) // P13 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR12_Pos) // P12 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 : T15C          , 2 : EC16          , 3 : T15O          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 : T16C          , 2 : EC15          , 3 : T16O          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 : T16O          , 2 : T16C          , 3 : EC15          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 : T15O          , 2 : T15C          , 3 : EC16          , 4 :               
			;

	PB->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD15_Pos) // P15
			| (0x00UL << PORT_PUPD_PUPD14_Pos) // P14
			| (0x00UL << PORT_PUPD_PUPD13_Pos) // P13
			| (0x00UL << PORT_PUPD_PUPD12_Pos) // P12
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PB->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR15_Pos) // P15
			| (0x00UL << PORT_OUTDR_OUTDR14_Pos) // P14
			| (0x00UL << PORT_OUTDR_OUTDR13_Pos) // P13
			| (0x00UL << PORT_OUTDR_OUTDR12_Pos) // P12
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - C
	PC->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE12_Pos) // P12
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PC->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP12_Pos) // P12
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PC->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : SCL2          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : SDA2          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 :               , 2 : SCK20         , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : EC21          , 2 : MISO20        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : EC20          , 2 : MOSI20        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : T21O          , 2 : T21C          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : T20O          , 2 : T20C          , 3 : AN17          , 4 :               
			;

	PC->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR12_Pos) // P12 - 0 :               , 1 : EC11          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 : EC10          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			;

	PC->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD12_Pos) // P12
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PC->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR12_Pos) // P12
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - D
	PD->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PD->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PD->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 :               , 2 : SS11/SS21     , 3 :               , 4 : ICOM6         
			| (0x00UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 : BLNK          , 2 : SCK11/SCK21   , 3 :               , 4 : ICOM7         
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : RXD11         , 2 : MISO11/MISO21 , 3 :               , 4 : ICOM8         
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : TXD11         , 2 : MOSI11/MOSI21 , 3 :               , 4 : ICOM9         
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : SDA0          , 2 : EC10          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : SCL0          , 2 : SS20          , 3 :               , 4 :               
			;

	PD->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PD->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - E
	PE->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE15_Pos) // P15
			| (0x01UL << PORT_MOD_MODE14_Pos) // P14
			| (0x01UL << PORT_MOD_MODE13_Pos) // P13
			| (0x01UL << PORT_MOD_MODE12_Pos) // P12
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PE->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP15_Pos) // P15
			| (0x00UL << PORT_TYP_TYP14_Pos) // P14
			| (0x00UL << PORT_TYP_TYP13_Pos) // P13
			| (0x00UL << PORT_TYP_TYP12_Pos) // P12
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PE->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 : T11O          , 2 : T11C          , 3 :               , 4 : ICOM5         
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : T10O          , 2 : T10C          , 3 :               , 4 : ICOM4         
			| (0x02UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : PWM30CB       , 2 : MOSI21        , 3 :               , 4 : ICOM3         
			| (0x02UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 : PWM30CA       , 2 : MISO21        , 3 :               , 4 : ICOM2         
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : PWM30BB       , 2 : SCK21         , 3 :               , 4 : ICOM1         
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : PWM30BA       , 2 : SS21          , 3 :               , 4 : ICOM0         
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : PWM30AB       , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : PWM30AA       , 2 : SS11          , 3 :               , 4 :               
			;

	PE->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR15_Pos) // P15 - 0 :               , 1 :               , 2 : SS12          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR14_Pos) // P14 - 0 :               , 1 :               , 2 : SCK12         , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR13_Pos) // P13 - 0 :               , 1 : RXD12         , 2 : MISO12        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR12_Pos) // P12 - 0 :               , 1 : TXD12         , 2 : MOSI12        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 :               , 2 : SS13          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 :               , 2 : SCK13         , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 : RXD13         , 2 : MISO13        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 : TXD13         , 2 : MOSI13        , 3 :               , 4 :               
			;

	PE->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD15_Pos) // P15
			| (0x00UL << PORT_PUPD_PUPD14_Pos) // P14
			| (0x00UL << PORT_PUPD_PUPD13_Pos) // P13
			| (0x00UL << PORT_PUPD_PUPD12_Pos) // P12
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PE->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR15_Pos) // P15
			| (0x00UL << PORT_OUTDR_OUTDR14_Pos) // P14
			| (0x00UL << PORT_OUTDR_OUTDR13_Pos) // P13
			| (0x00UL << PORT_OUTDR_OUTDR12_Pos) // P12
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - F
	PF->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PF->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PF->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 : T30C          , 2 : SDA0          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : EC30          , 2 : SCL0          , 3 :               , 4 :               
			| (0x02UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : BLNK          , 2 :               , 3 :               , 4 :               
			| (0x02UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 : CLKO          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : RXD1          , 2 :               , 3 : SXOUT         , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : TXD1          , 2 :               , 3 : SXIN          , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : SDA1          , 2 :               , 3 : XIN           , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : SCL1          , 2 :               , 3 : XOUT          , 4 :               
			;

	PF->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 : T14O          , 2 : T14C          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 : T13O          , 2 : T13C          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 : EC14          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 : EC13          , 2 :               , 3 :               , 4 :               
			;

	PF->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PF->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	PORT_ACCESS_DIS(); // disable writing permittion of ALL PCU register
}

/* --------------------------------- End Of File ------------------------------ */
