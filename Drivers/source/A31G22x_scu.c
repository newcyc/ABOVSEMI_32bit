/***************************************************************************//**
* @file     A31G22x_scu.c
* @brief    Contains all functions support for SCU(System Control Unit) dirver
*           on A31G22x
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

/* Includes ------------------------------------------------------------------- */
#include "A31G22x_scu.h"

#define CSCR_WTIDKY				(0xA507UL << SCU_CSCR_WTIDKY_Pos)
#define SCCR_WTIDKY				(0x570AUL << SCU_SCCR_WTIDKY_Pos)


/***************************************************************************//**
* @brief      Control LSE(low Speed External oscillator)
*             Before LSE enable setting,
*             you have to set clock ports as SXIN,SXOUT in MOD,AFSRx reg
* @param      LSE_Control, should be:
*              - LSE_STOP    : 0x00
*              - LSE_EN      : 0x08
*              - LSE_EN_DIV2 : 0x09
*              - LSE_EN_DIV4 : 0x0A
* @return     None
*******************************************************************************/
void SCU_SetLSE(uint32_t LSE_Control)
{
	volatile uint32_t reg_val32;

	reg_val32 = SCU->CSCR;
	reg_val32 &= ~SCU_CSCR_LSECON_Msk;
	reg_val32 |= (LSE_Control << SCU_CSCR_LSECON_Pos);
	SCU->CSCR = CSCR_WTIDKY | reg_val32;
}

/***************************************************************************//**
* @brief      Waiting LSE starting up
* @param      None
* @return     None
*******************************************************************************/
void SCU_WaitForLSEStartUp(void)
{
	volatile uint32_t i;

	/* need more Wait for SOSC stable, check SXOUT pin oscillation  */
	for (i = 0; i < 0x100; i++)
	{
		__NOP();
	}
}

/***************************************************************************//**
* @brief      Control LSI(Low Speed Internal oscillator)
* @param      LSI_Control, should be:
*              - LSI_STOP    : 0
*              - LSI_EN      : 8
*              - LSI_EN_DIV2 : 9
*              - LSI_EN_DIV4 : 10
* @return     None
*******************************************************************************/
void SCU_SetLSI(uint32_t LSI_Control)
{
	volatile uint32_t reg_val32;

	reg_val32 = SCU->CSCR;
	reg_val32 &= ~SCU_CSCR_LSICON_Msk;
	reg_val32|=(LSI_Control << SCU_CSCR_LSICON_Pos);
	SCU->CSCR = CSCR_WTIDKY | reg_val32;
}

/***************************************************************************//**
* @brief      Control HSI(How Speed Internal oscillator)
* @param      HSI_Control, should be:
*              - HSI_STOP     : 0
*              - HSI_EN       : 8
*              - HSI_EN_DIV2  : 9
*              - HSI_EN_DIV4  : 10
*              - HSI_EN_DIV8  : 11
*              - HSI_EN_DIV16 : 12
*              - HSI_EN_DIV32 : 13
* @return     None
*******************************************************************************/
void SCU_SetHSI(uint32_t HSI_Control)
{
	volatile uint32_t reg_val32;

	reg_val32 = SCU->CSCR;
	reg_val32 &= ~SCU_CSCR_HSICON_Msk;
	reg_val32 |= (HSI_Control << SCU_CSCR_HSICON_Pos);
	SCU->CSCR = CSCR_WTIDKY | reg_val32;
}

/***************************************************************************//**
* @brief      Control HSE(How Speed External oscillator)
*             Before exteranl enable setting,
*             you have to set clock ports as XIN,XOUT in MR.
* @param      HSE_Control, should be:
*              - HSE_STOP    : 0
*              - HSE_EN      : 8
*              - HSE_EN_DIV2 : 9
*              - HSE_EN_DIV4 : 10
* @return     None
*******************************************************************************/
void SCU_SetHSE(uint32_t HSE_Control)
{
	volatile uint32_t reg_val32;

	reg_val32 = SCU->CSCR;
	reg_val32 &= ~SCU_CSCR_HSECON_Msk;
	reg_val32 |= (HSE_Control << SCU_CSCR_HSECON_Pos);
	SCU->CSCR = CSCR_WTIDKY | reg_val32;
}

/***************************************************************************//**
* @brief      Waiting HSE starting up
* @param      None
* @return     Status enumeration value:
*              - ERROR   : 0
*              - SUCCESS : 1
*******************************************************************************/
Status SCU_WaitForHSEStartUp(void)
{
	Status Result;
	volatile uint32_t i;
	volatile uint16_t Status;
	volatile uint32_t StartUpCounter;

	// Monitoring HSE
	SCU->CMR |= (0x01UL << SCU_CMR_HSEMNT_Pos);

	// Wait till HSE is ready and if timeout is reached exit
	StartUpCounter = 0;
	do
	{
		Status = (SCU->CMR & SCU_CMR_HSESTS_Msk);
		StartUpCounter++;
	} while((Status == 0) && (StartUpCounter < HSE_STARTUP_TIMEOUT));

	// Need more Wait for HSE stable, check XOUT pin oscillation
	for (i = 0; i < 0x800; i++)
	{
		__NOP();
	}

	// Check HSE status finally
	if (SCU->CMR & SCU_CMR_HSESTS_Msk) {
		Result = SUCCESS;
	} else {
		Result = ERROR;
	}

	return Result;
}


/***************************************************************************//**
* @brief      Change system control
* @param      SystemClock, should be:
*              - SCCR_LSI     : 0
*              - SCCR_LSE     : 1
*              - SCCR_HSI     : 2
*              - SCCR_HSI_PLL : 3
*              - SCCR_HSE     : 6
*              - SCCR_HSE_PLL : 7
* @return     None
*******************************************************************************/
void SCU_ChangeSysClk(uint32_t SystemClock)
{
	volatile uint32_t reg_val32;

	reg_val32 = SCU->SCCR;
	reg_val32 &= ~(SCU_SCCR_MCLKSEL_Msk | SCU_SCCR_FINSEL_Msk);
	reg_val32 |= SystemClock;
	SCU->SCCR = SCCR_WTIDKY | reg_val32;
}

/**********************************************************************
 * @brief 		Configure clock out  on PF4
 *       Before this function setting, you have to set PF4 as CLKO in PF.MODand PF.AFSR0.
 * @param[in]	divval(=CLKODIV) : 0~15  // CLKO=MCLK/(2*(CLKODIV+1))
 *                   ex) MCLK=20Mh, CLKODIV=4,    20MHz/ (2*(4+1)) = 2MHz
 * @param[in]	FunctionalState, should be:
 * 					- DISABLE		   : 0
 * 					- ENABLE    	   : 1
 * @return		None
 **********************************************************************/
void SCU_SetCOR(uint8_t divval, FunctionalState endis)
{
	SCU->COR=(endis<<4)|(divval&0xf);
}

/**********************************************************************
 * @brief 	  monitoring External oscillator
 * @param  none
 * @return	status enumeration value:
 *              - ERROR
 *              - SUCCESS
 **********************************************************************/
Status SCU_SetPLLandWaitForPLLStartUp(FunctionalState pllsetstate, uint8_t selbypass, uint8_t selfin,  uint8_t PREDIV, uint8_t POSTDIV1, uint8_t POSTDIV2, uint8_t OUTDIV)
{
	__IO uint32_t StartUpCounter = 0;
	__IO uint32_t status;
//	uint32_t i;

	if (pllsetstate == DISABLE){
		SCU->PLLCON=0;
		return SUCCESS;
	}
	else {
// PLLenable    PLL freq = (FIN or FIN/2) * M / N
		SCU->PLLCON=0
		|(1<<23)   // PLL reset is negated
		|(1<<22)  // PLLEn
		|((selbypass&1)<<21)     //BYPASS 0:FOUT is bypassed as FIN, 1:FOUT is PLL output
		|((selfin&1)<<20)             //PLL VCO mode  0:VCO is the same with FOUT, 1:VCO frequency is x2 of FOUT   D
		|((PREDIV&0x7)<<16)      //PREDIV        R
		|((POSTDIV1&0xff)<<8)    //POSTDIV1   N1
		|((POSTDIV2&0xf)<<4)     //POSTDIV2   N2
		|((OUTDIV&0xf)<<0)         //OUTDIV	     P
			;

//		SCU->PLLCON&=~(1<<23);   // PLL reset is asserted
//		for (i=0;i<1000;i++);
//		SCU->PLLCON|=(1<<23);     // PLL reset is negated

	  /* Wait till PLL LOCK is ready and if timeout is reached exit */
		do
		{
			status = (SCU->PLLCON & (1UL<<31));
			StartUpCounter++;
		} while((StartUpCounter != PLL_STARTUP_TIMEOUT) && (status != (1UL<<31)));

		status = (SCU->PLLCON & (1UL<<31));
		if (status != (1UL<<31)){
			return ERROR;
		}
		else {
			return SUCCESS;
		}
	}
}

/*----------------------------------------------------------------------------
   @brief        Set Timer1n Clock
   @param[in]    t1nclk                           T1NCLK_MCCR1, T1NCLK_PCLK
   @explain      This macro sets timer1n clock
 *----------------------------------------------------------------------------*/
void SCU_SetTimer1nClk(uint32_t t1nclk)
{
	uint32_t temp;

	temp = SCU->PPCLKSR;
	temp &= ~SCU_PPCLKSR_T1xCLK_Msk;
	temp |= (t1nclk << SCU_PPCLKSR_T1xCLK_Pos);
	SCU->PPCLKSR = temp;
}

/*----------------------------------------------------------------------------
   @brief        Set Timer20 Clock
   @param[in]    t20clk                           T20CLK_MCCR2, T20CLK_PCLK
   @explain      This macro sets timer20 clock
 *----------------------------------------------------------------------------*/
void SCU_SetTimer20Clk(uint32_t t20clk)
{
	uint32_t temp;

	temp = SCU->PPCLKSR;
	temp &= ~SCU_PPCLKSR_T20CLK_Msk;
	temp |= (t20clk << SCU_PPCLKSR_T20CLK_Pos);
	SCU->PPCLKSR = temp;
}

/*----------------------------------------------------------------------------
   @brief        Set Timer30 Clock
   @param[in]    t30clk                           T30CLK_MCCR2, T30CLK_PCLK
   @explain      This macro sets timer30 clock
 *----------------------------------------------------------------------------*/
void SCU_SetTimer30Clk(uint32_t t30clk)
{
	uint32_t temp;

	temp = SCU->PPCLKSR;
	temp &= ~SCU_PPCLKSR_T30CLK_Msk;
	temp |= (t30clk << SCU_PPCLKSR_T30CLK_Pos);
	SCU->PPCLKSR = temp;
}

/*----------------------------------------------------------------------------
   @brief        Set Watch Timer Clock
   @param[in]    wtclk                           WTCLK_MCCR3, WTCLK_SOSC, WTCLK_WDTRC
   @explain      This macro sets watch timer clock
 *----------------------------------------------------------------------------*/
void SCU_SetWtClk(uint32_t wtclk)
{
	uint32_t temp;

	temp = SCU->PPCLKSR;
	temp &= ~(3<<3);
	temp |= (wtclk<<3);
	SCU->PPCLKSR = temp;
}

/*----------------------------------------------------------------------------
   @brief        Set WatchDog Timer Clock
   @param[in]    wdtclk                           WDTCLK_WDTRC, WDTCLK_MCCR3
   @explain      This macro sets watchdog timer clock
 *----------------------------------------------------------------------------*/
void SCU_SetWdtClk(uint32_t wdtclk)
{
	uint32_t temp;

	temp = SCU->PPCLKSR;
	temp &= ~(1<<0);
	temp |= (wdtclk<<0);
	SCU->PPCLKSR = temp;
}

/**********************************************************************
 * @brief 		Configure peri clock setting
 * @param[in]	mccrnum: select REG number
 *                     - MCCR1 : 1, * - MCCR2 : 2
 *                     - MCCR3 : 3, * - MCCR4 : 4
 *                     - MCCR5 : 5, * - MCCR6 : 6
 *                     - MCCR7 : 7, * - MCCR6 : 6
 * @param[in]	type :
 *	                     SYSTICK_TYPE, TIMER10_TYPE,
 *	                     TIMER20_TYPE, TIMER30_TYPE,
 *						 WDT_TYPE, WT_TYPE,
 *						 PD0_TYPE, PD1_TYPE,
 *						 LED_TYPE, LCD_TYPE,
 *						 USB_TYPE
 * @param[in]	clksrc :
 *						- CLKSRC_LSI=0,
 *						- CLKSRC_LSE=3,
 *						- CLKSRC_MCLK=4,
 *						- CLKSRC_HSI=5,
 *						- CLKSRC_HSE=6,
 *                     - CLKSRC_PLL=7
 * @param[in]	clkdiv :
 * @return		None
 **********************************************************************/
void SCU_SetMCCRx(uint8_t mccrnum, uint8_t type, uint8_t clksrc, uint8_t clkdiv)
{
	volatile uint32_t tmp;

	switch(mccrnum){
		case 1:
			tmp=SCU->MCCR1;
			if (type==SYSTICK_TYPE){
				tmp&=0xFFFFF800;
				tmp|= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			else if (type==TIMER1n_TYPE){
				tmp&=0xF800FFFF;
				tmp|= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR1=tmp;
			break;

		case 2:
			tmp=SCU->MCCR2;
			if (type==TIMER2n_TYPE){
				tmp&=0xFFFFF800;
				tmp|= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			else if (type==TIMER3n_TYPE){
				tmp&=0xF800FFFF;
				tmp|= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR2=tmp;
			break;

		case 3:
			tmp=SCU->MCCR3;
			if (type==WDT_TYPE){
				tmp&=0xFFFFF800;
				tmp|= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			else if (type==WT_TYPE){
				tmp&=0xF800FFFF;
				tmp|= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR3=tmp;
			break;

		case 4:
			tmp=SCU->MCCR4;
			if (type==PD0_TYPE){
				tmp&=0xFFFFF800;
				tmp|= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			else if (type==PD1_TYPE){
				tmp&=0xF800FFFF;
				tmp|= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR4=tmp;
			break;

		case 5:
			// This code will be modified
			tmp = SCU->MCCR5;
			if (type == LCD_TYPE){
//				tmp &= ~(SCU_MCCR5_LCDCDIV_Msk | SCU_MCCR5_LCDCSEL_Msk);
//				tmp |= (((clkdiv & 0xFF) << SCU_MCCR5_LCDCDIV_Pos) | ((clksrc & 0x07)  << SCU_MCCR5_LCDCSEL_Pos));
			}
			else if (type == TS_TYPE) {
				
			}
			SCU->MCCR5 = tmp;
			break;
		case 6:
			break;
		case 7:
			tmp = SCU->MCCR7;
			if (type == ADC_TYPE){
				tmp &= ~(SCU_MCCR7_ADCCDIV_Msk | SCU_MCCR7_ADCCSEL_Msk);
				tmp |= (((clkdiv & 0xFF) << SCU_MCCR7_ADCCDIV_Pos) | ((clksrc & 0x07)  << SCU_MCCR7_ADCCSEL_Pos));
			}
			SCU->MCCR7 = tmp;
			break;
	}
}

/**********************************************************************
 * @brief	 	enable/disable WakeUp Source
 * @param[in]	WakeUpSrc, oring is possible:
 * 					- WAKEUP_GPIOD :    (1UL<<11)
 * 					- WAKEUP_GPIOC :    (1UL<<10)
 * 					- WAKEUP_GPIOB :    (1UL<<9)
 * 					- WAKEUP_GPIOA :   (1UL<<8)
*					- WAKEUP_FRT     :   (1UL<<2)
 * 					- WAKEUP_WDT    :    (1UL<<1)
 * 					- WAKEUP_LVD    :    (1UL<<0)
 * @param[in]	NewState
 * 					- ENABLE  	:Set WakeUp Source enable
 * 					- DISABLE 	:Disable WakeUp Source
 * @return 		None
 **********************************************************************/
void SCU_WakeUpSRCCmd(uint32_t WakeUpSrc, FunctionalState NewState)
{
	if (NewState == ENABLE)
	{
		SCU->WUER |=  (WakeUpSrc);
	}
	else
	{
		SCU->WUER &= ~(WakeUpSrc);
	}
}

/*********************************************************************
 * @brief 		Get current value of WakeUpSRC Status.
 * @param[in]	None.
 * @return		Current value of WakeUpSRC Status register.
 *********************************************************************/
uint32_t SCU_GetWakeUpSRCStatus(void)
{
	return SCU->WUSR;
}

/*********************************************************************
 * @brief 		Get current value of NMI Status.
 * @param[in]	None.
 * @return		Current value of NMI Status register.
 *********************************************************************/
uint32_t SCU_GetNMIStatus(void)
{
	return (SCU->NMIR);
}

/**********************************************************************
 * @brief	 	enable/disable NMI Source
 * @param[in]	nmisrc
 * @return 		None
 **********************************************************************/
void SCU_SetNMI(uint32_t nmisrc)
{
	SCU->NMIR = (0xA32CUL<<16) | (nmisrc & 0x3F);
}

/**********************************************************************
 * @brief	 	enable/disable Reset Source
 * @param[in]	RstSrc, oring is possible:
 * 					- RST_PINRST :    (1UL<<11)
 * 					- RST_CPURST :    (1UL<<10)
 * 					- RST_SWRST :    (1UL<<9)
 * 					- RST_WDTRST :   (1UL<<8)
 *					- RST_MCKFRST     :   (1UL<<2)
 * 					- RST_MOFRST    :    (1UL<<1)
 * 					- RST_LVDRST    :    (1UL<<0)
 * @param[in]	NewState
 * 					- ENABLE  	:Set Reset Source enable
 * 					- DISABLE 	:Disable Reset Source
 * @return 		None
 **********************************************************************/
void SCU_SetResetSrc(uint32_t RstSrc, FunctionalState NewState)
{
	if (NewState == ENABLE)
	{
		SCU->RSER |=  (RstSrc);
	}
	else
	{
		SCU->RSER &= ~(RstSrc);
	}
}

/***************************************************************************//**
 * @brief 		Get current value of RSSR Status.
 * @param[in]	None.
 * @return		Current value of RSSR Status register.
*******************************************************************************/
uint32_t SCU_GetResetSrc(void)
{
	return (SCU->RSSR);
}

/***************************************************************************//**
 * @brief	 	clear Reset Status
 * @param[in]	rststatus
 * @return 		None
*******************************************************************************/
void SCU_ClearResetStatus(uint32_t rststatus)
{
	SCU->RSSR = rststatus;
}

/***************************************************************************//**
 * @brief 		Get current value of EMODR Status.
 * @param[in]	None.
 * @return		Current value of EMODR Status register.
*******************************************************************************/
Bool SCU_GetExtModeStatus(void)
{
	return (Bool) SCU->EMODR;
}

/***************************************************************************//**
 * @brief 		Control Reset Pin Debounce. This function must be operated with 500kHz LSI Clock.
 * @param[in]	En
 * 					- ENABLE
 * 					- DISABLE
 * @param[in]	CntVal
 * 					- Noise cancel delay option of Reset Pin
 * 					- Count Value range (0 ~ 31)
 * @return		None
*******************************************************************************/
void SCU_SetResetPinDebounce(FunctionalState En, uint32_t CntVal)
{
	uint32_t Reg32;

	if(En == ENABLE)
	{

		Reg32 = SCU->RSTDBCR = (SCU_RSTDBCR_WTIDKY << SCU_RSTDBCR_WTIDKY_Pos) \
					| (1 << SCU_RSTDBCR_EN_Pos) \
					| (CntVal << SCU_RSTDBCR_CLKCNT_Pos) \
					;
	}
	else
	{
		Reg32 = (SCU_RSTDBCR_WTIDKY << SCU_RSTDBCR_WTIDKY_Pos) \
						| (0 << SCU_RSTDBCR_EN_Pos) \
						| (0 << SCU_RSTDBCR_CLKCNT_Pos) \
						;
	}

	SCU->RSTDBCR = Reg32;
}

/***************************************************************************//**
 * @brief 		Get previous mode before current reset event
 * @param[in]	None
 * @return		Previous Mode
 *					- PREVMODE_RUN
 *					- PREVMODE_SLEEP
 *					- PREVMODE_DEEPSLEEP
 *					- PREVMODE_INIT
 *					- ERROR
*******************************************************************************/
uint32_t SCU_GetPreviousModeBeforeResetEvent(void)
{
	uint32_t Reg32;

	Reg32 = SCU->SMR&SCU_SMR_PREVMODE_Msk;

	if ( (Reg32 & SCU_SMR_PREVMODE_Msk) == SCU_SMR_PREVMODE_RUN)				return 	PREMODE_RUN;
	else if ( (Reg32 & SCU_SMR_PREVMODE_Msk) == SCU_SMR_PREVMODE_SLEEP)			return  PREMODE_SLEEP;
	else if ( (Reg32 & SCU_SMR_PREVMODE_Msk) == SCU_SMR_PREVMODE_DEEPSLEEP)		return  PREMODE_DEEPSLEEP;
	else if ( (Reg32 & SCU_SMR_PREVMODE_Msk) == SCU_SMR_PREVMODE_INIT)			return  PREMODE_INIT;
	else																		return ERROR;
}

/***************************************************************************//**
 * @brief 		Set TS(Temparature Sensor) clock
 * @param[in]	LSI_TS : LSI TS clock
 * 					- ENABLE
 * 					- DISABLE
 * @param[in]	ReferenceClock
 * 					- SCU_TS_REFERENCE_HSI
 * 					- SCU_TS_REFERENCE_MCLK
 * 					- SCU_TS_REFERENCE_HSE
 * 					- SCU_TS_REFERENCE_LSE
 * @param[in]	SensingClock
 * 					- SCU_TS_SESNING_LSI_TS
 * 					- SCU_TS_SESNING_LSI
 * 					- SCU_TS_SESNING_HSI
 * @return		None
*******************************************************************************/
void SCU_SetTSClock(FunctionalState LSI_TS, SCU_TS_REFERENCE_Type ReferenceClock, SCU_TS_SENSING_Type SensingClock)
{
	volatile uint32_t Reg32;

	Reg32 = SCU->MCCR5;
	Reg32 &= ~(SCU_MCCR5_TSREFCLKSEL_Msk | SCU_MCCR5_TSSENSECLKSEL_Msk | SCU_MCCR5_TSSLSITSEN_Msk);
	Reg32 |= 0x00UL
		| ((ReferenceClock << SCU_MCCR5_TSREFCLKSEL_Pos) & SCU_MCCR5_TSREFCLKSEL_Msk)
		| ((SensingClock << SCU_MCCR5_TSSENSECLKSEL_Pos) & SCU_MCCR5_TSSENSECLKSEL_Msk)
		| ((LSI_TS << SCU_MCCR5_TSSLSITSEN_Pos) & SCU_MCCR5_TSSLSITSEN_Msk)
		;
	SCU->MCCR5 = Reg32;
}

/* --------------------------------- End Of File ------------------------------ */
