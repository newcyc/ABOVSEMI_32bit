/***************************************************************************//**
* @file     A31G22x_scu.h
* @brief    Contains all macro definitions and function prototypes support
*           for SCU(System Control Unit) driver on A31G22x
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

#ifndef _A31G22x_SCU_H_
#define _A31G22x_SCU_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------- */
#include "A31G22x.h"
#include "aa_types.h"


/* Private macros ------------------------------------------------------------- */
/* Low speed external oscillator control */
#define LSE_STOP					(0x00UL)
#define LSE_EN						(0x08UL)
#define LSE_EN_DIV2					(0x09UL)
#define LSE_EN_DIV4					(0x0AUL)

/* Low speed external oscillator stable timeout */
#define LSE_STARTUP_TIMEOUT			(0xFFFFFFFFUL)

/* Low speed interanl oscillator control */
#define LSI_STOP					(0x00UL)
#define LSI_EN						(0x08UL)
#define LSI_EN_DIV2					(0x09UL)
#define LSI_EN_DIV4					(0x0AUL)

/* High speed internal oscillator control */
#define HSI_STOP					(0x00UL)
#define HSI_EN						(0x08UL)
#define HSI_EN_DIV2					(0x09UL)
#define HSI_EN_DIV4					(0x0AUL)
#define HSI_EN_DIV8					(0x0BUL)
#define HSI_EN_DIV16				(0x0CUL)
#define HSI_EN_DIV32				(0x0DUL)

/* High speed external oscillator control */
#define HSE_STOP					(0x00UL)
#define HSE_EN						(0x08UL)
#define HSE_EN_DIV2					(0x09UL)
#define HSE_EN_DIV4					(0x0AUL)

/* High speed external oscillator stable timeout */
#define HSE_STARTUP_TIMEOUT			(0xFFFFFFFFUL)

/* PLL stable timeout */
#define PLL_STARTUP_TIMEOUT			(0xFFFFFFFFUL)

/* PLLCON[BYPASS] */
#define PLLCON_BYPASS_FIN			(0x00UL)
#define PLLCON_BYPASS_PLL			(0x01UL)

// PLL freq = (FIN or FIN/2) * M / N
/* PLLCON[PREDIV] */
#define PLLCON_FIN_DIV_1			(0x00UL)
#define PLLCON_FIN_DIV_2			(0x01UL)

/* PLLCON[FBCTRL] */
#define PLLCON_M_6					(0x00UL)
#define PLLCON_M_8					(0x01UL)
#define PLLCON_M_10					(0x02UL)
#define PLLCON_M_12					(0x03UL)
#define PLLCON_M_16					(0x04UL)
#define PLLCON_M_18					(0x05UL)
#define PLLCON_M_20					(0x06UL)
#define PLLCON_M_26					(0x07UL)
#define PLLCON_M_32					(0x08UL)
#define PLLCON_M_36					(0x09UL)
#define PLLCON_M_40					(0x0AUL)
#define PLLCON_M_64					(0x0BUL)

/* PLLCON[POSTDIV] */
#define PLLCON_N_1					(0x00UL)
#define PLLCON_N_2					(0x01UL)
#define PLLCON_N_3					(0x02UL)
#define PLLCON_N_4					(0x03UL)
#define PLLCON_N_6					(0x04UL)
#define PLLCON_N_8					(0x05UL)
#define PLLCON_N_16					(0x07UL)

/* system clock control setting */
#define SCCR_LSI					(0x00UL)
#define SCCR_LSE					(0x01UL)
#define SCCR_HSI					(0x02UL)
#define SCCR_HSI_PLL				(0x03UL)
#define SCCR_HSE					(0x06UL)
#define SCCR_HSE_PLL				(0x07UL)

/* System Control Register - Write Identification Key */
#define	SCU_SCR_WTIDKY			(0x9EB3UL)
#define SCB_AIRCR_VECTKEY			(0x05FAUL)

/* SCUCC CHIP ID */
#define	CHIPID_G226	(0x4D31A00AUL)
#define	CHIPID_G224	(0x4D31A00BUL)

enum {
	SYSTICK_TYPE = 0,
	TIMER1n_TYPE,
	TIMER2n_TYPE,
	TIMER3n_TYPE,
	WDT_TYPE,
	WT_TYPE,
	PD0_TYPE,
	PD1_TYPE,
	ADC_TYPE,
	LCD_TYPE,
	TS_TYPE,
};

typedef enum {
	CLKSRC_LSI = 0,
	CLKSRC_LSE = 3,
	CLKSRC_MCLK = 4,
	CLKSRC_HSI = 5,
	CLKSRC_HSE = 6,
	CLKSRC_PLL = 7
} CLKSRC_Type;

typedef enum {
	LVD_LEVEL_1_65 = 0,
	LVD_LEVEL_2_5,
	LVD_LEVEL_3_6,
	LVD_LEVEL_4_2
} LVD_LEVEL_Type;

typedef enum {
	LSE_ISEL_2_04 = 0,
	LSE_ISEL_1_93,
	LSE_ISEL_1_79,
	LSE_ISEL_1_57
} LSE_ISEL_Type;

typedef enum {
	HSE_ISEL_6_72 = 0,
	HSE_ISEL_6_41,
	HSE_ISEL_5_78,
	HSE_ISEL_2_71
} HSE_ISEL_Type;

typedef enum {
	HSE_NCOPT_23 = 0,
	HSE_NCOPT_18,
	HSE_NCOPT_13,
	HSE_NCOPT_8,
} HSE_NCOPT_Type;

typedef enum {
	PREMODE_RUN = 0,
	PREMODE_SLEEP = 1,
	PREMODE_DEEPSLEEP = 2,
	PREMODE_INIT = 3
} PREMODE_Type;

typedef enum {
	LSIAON_DISABLE = 0,
	LSIAON_ENABLE = 1
} LSIAON_Type;

typedef enum {
	BGRAON_DISABLE = 0,
	BGRAON_ENABLE = 1
} BGRAON_Type;

typedef enum {
	VDCAON_DISABLE = 0,
	VDCAON_ENABLE = 1
} VDCAON_Type;

/**
 * @brief  TS(Temperature Sensor) reference clock selection enumerated definition
 */
typedef enum {
	SCU_TS_REFERENCE_HSI = 0x00UL,  /*!< HSI */
	SCU_TS_REFERENCE_MCLK = 0x01UL, /*!< MCLK */
	SCU_TS_REFERENCE_HSE = 0x02UL,  /*!< HSE */
	SCU_TS_REFERENCE_LSE = 0x03UL   /*!< LSE */
} SCU_TS_REFERENCE_Type;

/**
 * @brief  TS(Temperature Sensor) sensing clock selection enumerated definition
 */
typedef enum {
	SCU_TS_SESNING_LSI_TS = 0x00UL, /*!< LSI TS, Temperature sensing */
	SCU_TS_SESNING_LSI = 0x01UL,    /*!< LSI, timer function */
	SCU_TS_SESNING_HSI = 0x03UL,    /*!< HSI, timer function */
} SCU_TS_SENSING_Type;

//----------------- SCU Reset Source definition -----------------
#define SCU_CSCR_WTIDKY				(0xA507UL)
#define SCU_RSTDBCR_WTIDKY			(0x0514UL)

#define RST_PINRST					(0x01UL << 6)
#define RST_CPURST					(0x01UL << 5)
#define RST_SWRST					(0x01UL << 4)
#define RST_WDTRST					(0x01UL << 3)
#define RST_MCKFRST					(0x01UL << 2)
#define RST_HSEFRST					(0x01UL << 1)
#define RST_LVDRST					(0x01UL << 0)


#define WAKEUP_SYSTICK				(0x01UL << 16)
#define WAKEUP_GPIOF				(0x01UL << 13)
#define WAKEUP_GPIOE				(0x01UL << 12)
#define WAKEUP_GPIOD				(0x01UL << 11)
#define WAKEUP_GPIOC				(0x01UL << 10)
#define WAKEUP_GPIOB				(0x01UL << 9)
#define WAKEUP_GPIOA				(0x01UL << 8)
#define WAKEUP_WT					(0x01UL << 2)
#define WAKEUP_WDT					(0x01UL << 1)
#define WAKEUP_LVD					(0x01UL << 0)


//----------------- T1N SCU Clock Constant Definition -----------------
#define T1NCLK_MCCR1				(0x00UL)
#define T1NCLK_PCLK					(0x01UL)

//----------------- T20 SCU Clock Constant Definition -----------------
#define T20CLK_MCCR2				(0x00UL)
#define T20CLK_PCLK					(0x01UL)

//----------------- T30 SCU Clock Constant Definition -----------------
#define T30CLK_MCCR2				(0x00UL)
#define T30CLK_PCLK					(0x01UL)

//--------------- Watch Timer Clock Constant Definition ---------------
#define WTCLK_MCCR3					(0x00UL)
#define WTCLK_LSE					(0x01UL)
#define WTCLK_WDTRC					(0x02UL)

//------------- Watch-Dog Timer Clock Constant Definition -------------
#define WDTCLK_WDTRC				(0x00UL)
#define WDTCLK_MCCR3				(0x01UL)

//------------- SCU_NMI ENABLE / STATUS definition -------------------------------
#define SCU_NMI_ACCESSCODE			(0xA32C << SCU_NMIR_ACCESSCODE_Pos)
#define SCU_NMI_LVD_ENABLE			(1 << SCU_NMIR_LVDEN_Pos)
#define SCU_NMI_LVD_DISABLE			(0 << SCU_NMIR_LVDEN_Pos)
#define SCU_NMI_MCLKFAIL_ENABLE		(1 << SCU_NMIR_MCLKFAILEN_Pos)
#define SCU_NMI_MCLKFAIL_DIABLE		(0 << SCU_NMIR_MCLKFAILEN_Pos)
#define SCU_NMI_WDTINT_ENABLE		(1 << SCU_NMIR_WDTINTEN_Pos)
#define SCU_NMI_WDTINT_DISABLE		(0 << SCU_NMIR_WDTINTEN_Pos)
#define SCU_NMI_T30INT_ENABLE		(1 << SCU_NMIR_T30IEN_Pos)
#define SCU_NMI_T30INT_DISABLE		(0 << SCU_NMIR_T30IEN_Pos)
#define SCU_NMI_CMPINT_ENABLE		(1 << SCU_NMIR_CMPINTEN_Pos)
#define SCU_NMI_CMPINT_DIABLE		(0 << SCU_NMIR_CMPINTEN_Pos)
#define SCU_NMI_SWAPERR_ENABLE		(1 << SCU_NMIR_SWAPERRIEN_Pos)
#define SCU_NMI_SWAPERR_DISABLE		(0 << SCU_NMIR_SWAPERRIEN_Pos)

#define SCU_NMI_ACCESSCODE			(0xA32C << SCU_NMIR_ACCESSCODE_Pos)
#define SCU_NMI_LVD_STATUS			(1 << SCU_NMIR_LVDSTS_Pos)
#define SCU_NMI_MCLKFAIL_STATUS		(1 << SCU_NMIR_MCLKFAILSTS_Pos)
#define SCU_NMI_WDTINT_STATUS		(1 << SCU_NMIR_WDTINTSTS_Pos)
#define SCU_NMI_T30INT_STATUS		(1 << SCU_NMIR_T30INTSTS_Pos)
#define SCU_NMI_CMPINT_STSTUS		(1 << SCU_NMIR_CMPINTSTS_Pos)
#define SCU_NMI_SWAPERR_STATUS		(1 << SCU_NMIR_SWAPERRSTS_Pos)

//------------ SCU_SMR Constant Definition -------------
#define SCU_SMR_LSIAON_ENABLE	 	(1 << SCU_SMR_LSIAON_Pos)
#define SCU_SMR_LSIAON_DISABLE		(0 << SCU_SMR_LSIAON_Pos)
#define SCU_SMR_BGRAON_ENABLE		(1 << SCU_SMR_BGRAON_Pos)
#define SCU_SMR_BGRAON_DISABLE		(0 << SCU_SMR_BGRAON_Pos)
#define SCU_SMR_VDCAON_ENABLE		(1 << SCU_SMR_VDCAON_Pos)
#define SCU_SMR_VDCAON_DISABLE		(0 << SCU_SMR_VDCAON_Pos)

#define	SCU_SMR_PREVMODE_RUN		(0 << SCU_SMR_PREVMODE_Pos)
#define	SCU_SMR_PREVMODE_SLEEP		(1 << SCU_SMR_PREVMODE_Pos)
#define	SCU_SMR_PREVMODE_DEEPSLEEP	(2 << SCU_SMR_PREVMODE_Pos)
#define	SCU_SMR_PREVMODE_INIT		(3 << SCU_SMR_PREVMODE_Pos)

/* Public Functions ----------------------------------------------------------- */
extern void SCU_SetLSE(uint32_t LSE_Control);
extern void SCU_WaitForLSEStartUp(void);
extern void SCU_SetLSI(uint32_t LSI_Control);
extern void SCU_SetHSI(uint32_t HSI_Control);
extern void SCU_SetHSE(uint32_t HSE_Control);
extern Status SCU_WaitForHSEStartUp(void);


void SCU_ChangeSysClk(uint32_t SystemClock);
void SCU_SetCOR(uint8_t divval, FunctionalState endis);
Status SCU_SetPLLandWaitForPLLStartUp(FunctionalState pllsetstate, uint8_t selbypass, uint8_t selfin,  uint8_t PREDIV, uint8_t POSTDIV1, uint8_t POSTDIV2, uint8_t OUTDIV);

void SCU_SetTimer1nClk(uint32_t t1nclk);
void SCU_SetTimer20Clk(uint32_t t20clk);
void SCU_SetTimer30Clk(uint32_t t30clk);
void SCU_SetWtClk(uint32_t wtclk);
void SCU_SetWdtClk(uint32_t wdtclk);

void SCU_SetMCCRx(uint8_t mccrnum, uint8_t type, uint8_t clksrc, uint8_t clkdiv);

void SCU_WakeUpSRCCmd(uint32_t WakeUpSrc, FunctionalState NewState);
uint32_t SCU_GetWakeUpSRCStatus(void);


uint32_t SCU_GetNMIStatus(void);
void SCU_SetNMI(uint32_t nmisrc);

void SCU_SetResetSrc(uint32_t RstSrc, FunctionalState NewState);
uint32_t SCU_GetResetSrc(void);
void SCU_ClearResetStatus(uint32_t rststatus);

Bool SCU_GetExtModeStatus(void);
void SCU_SetResetPinDebounce(FunctionalState En, uint32_t CntVal);

uint32_t SCU_GetPreviousModeBeforeResetEvent(void);

void SCU_SetTSClock(FunctionalState LSI_TS, SCU_TS_REFERENCE_Type ReferenceClock, SCU_TS_SENSING_Type SensingClock);

#ifdef __cplusplus
}
#endif

#endif /* _A31G22x_SCU_H_ */
/* --------------------------------- End Of File ------------------------------ */
