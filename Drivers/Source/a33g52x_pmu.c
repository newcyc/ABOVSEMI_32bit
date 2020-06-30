/**********************************************************************
* @file		a33g52x_pmu.c
* @brief	Contains all functions support for SCU firmware library on A33G52x
* @version	1.0
* @date
* @author	ABOV M team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include <stdint.h>
#include "A33G52x.h"
#include "a33g52x_pmu.h"
#include "a33g52x_flash.h"
#include "console.h"


extern uint32_t		SystemCoreClock;
extern uint32_t		SystemPeriClock;

/**
************************************************************************************
* @ Name : PMU_SetPLLFreq
*
* @ Parameters
*
*		- pmu				PMU
*		- pllInClk 			XTAL4MHz, XTAL8MHz, XTAL10MHz, IOSC16MHz
*		- setPLLFreq		PLL75MHz, PLL74MHz, ..., PLL1MHz
*
* @ Description : this function helps users easily set the value of PLL operating frequency.
************************************************************************************
*/
int32_t PMU_SetPLLFreq(PMU_Type *pmu, uint32_t pllInClk, uint32_t setPllFreq) {
	uint32_t	reg_val;

	uint32_t r       = 0;							// PLL input clock divider (R)
	uint32_t n1 	= 0;							// PLL multiplyer (N1)
	uint32_t n2 	= 0;							// PLL divider (N2)
	uint32_t p 		= 0;							// PLL post-divider (P)
	uint32_t d		= 0;							// PLL VCO doubler bit value (D)

	uint32_t f_in   = 0;							// PLL input clock freq.
	uint32_t f_vco = 0;							// PLL VCO freq.


	PMU->PLLCON = 0x80750000;			// Key value for setting PLL Frequency (Extended Mode)
	FMC->CFG = 0x0202;					// Set Flash Access Timing (Max 75MHz/(2+2) = 18.75MHz)


	if(pllInClk == XTAL4MHz || pllInClk == XTAL8MHz ||  pllInClk == XTAL10MHz) {
		PMU->CCR |= PMUCCR_MXOSCEN_DIV_BY_1;		// Activate XTAL oscillation

		PMU->CMR |= PMUCMR_MXOSCMNT;					// Activate monitoring function of MXOSC oscillation
		while(!(PMU->CMR & PMUCMR_MXOSCSTS));		// Check whether MXOSC oscillation working properly

		reg_val = 0;
		reg_val |= PMUBCCR_HCLKDIV_DIV_BY_1 \
					| PMUBCCR_PCLKDIV_DIV_BY_1 \
					| PMUBCCR_PLLCLKDIV_DIV_BY_1 \
					| PMUBCCR_PLLCLKSEL_XTAL \
					| PMUBCCR_MCLKSEL_PLL;					// Select PLL Input Freq. = XTAL / 1 = 4/8/10MHz
		PMU->BCCR = reg_val;									// Select the XTAL as a PLL Input

	}
	else if (pllInClk == IOSC16MHz) {
		PMU->CCR |= PMUCCR_IOSC16EN_DIV_BY_1;
		PMU->BCCR |= PMUBCCR_PLLCLKSEL_IOSC16M;	// Select the IOSC16 as a PLL Input

		reg_val = 0;
		reg_val |= PMUBCCR_HCLKDIV_DIV_BY_1 \
					| PMUBCCR_PCLKDIV_DIV_BY_1 \
					| PMUBCCR_PLLCLKDIV_DIV_BY_1 \
					| PMUBCCR_PLLCLKSEL_IOSC16M \
					| PMUBCCR_MCLKSEL_PLL;					// Select PLL Input Freq. = IOSC16 / 2 = 8MHz
		PMU->BCCR = reg_val;									// Select the IOSC16 as a PLL Input

	}
	else {
		return PLL_WRONG;
	}

	// Set Deviders
	if(pllInClk == XTAL4MHz) 				r = 1;							// FIN = 4/(1+1) == 2
	else if(pllInClk == XTAL8MHz)		r = 3;							// FIN = 8/(1+3) == 2
	else if(pllInClk == XTAL10MHz)		r = 4;							// FIN = 10/(1+4) == 2
	else if(pllInClk == IOSC16MHz)		r = 7;							// FIN = 16/(1+7) == 2
	else											return PLL_WRONG;
	f_in = pllInClk/(r+1);														// f_in is fixed to 2MHz.

	if(f_in == 2) {
		if( (setPllFreq <= 75) && (setPllFreq >= 26)) {
			n1 = setPllFreq-1;
			n2 = 0;
			p = 1;
			d = 0;
		}
		else if( (setPllFreq <= 25) && (setPllFreq >= 11)) {
			n1 = (setPllFreq*3)-1;
			n2 = 0;
			p = 5;
			d = 0;
		}
		else if( (setPllFreq <= 10) && (setPllFreq >= 4)) {
			n1 = (setPllFreq*8)-1;
			n2 = 0;
			p = 15;
			d = 0;
		}
		else if( (setPllFreq <= 3) && (setPllFreq >= 2)) {
			n1 = (setPllFreq*16)-1;
			n2 = 1;
			p = 15;
			d = 0;
		}
		else if(setPllFreq == 1) {
			n1 = (setPllFreq*32);
			n2 = 3;
			p = 15;
			d = 0;
		}
		else
		{
			return PLL_WRONG;
		}

		f_vco = f_in * n1;
		if( f_vco <= 200) {
			PMU_PLLEnable (PMU, r, n1, n2, p, d);
		}
	}
	else {
		return	PLL_WRONG;
	}

	SystemCoreClock = (setPllFreq*1000000);
	SystemPeriClock =  (setPllFreq*1000000);

	return PLL_OK;
}

/**
************************************************************************************
* @ Name : PMU_PLLEnable
*
* @ Parameters
*
*		- pmu			PMU
*		- prediv 			0~7
*		- mul			(0~255
*		- div			0~15
*		- postdiv			0~15
*		- vco_mode 		PLLCON_VCOMODE_NORMAL, PLLCON_VCOMODE_DOUBLE
*
************************************************************************************
*/

void PMU_PLLEnable (PMU_Type *pmu, uint32_t prediv, uint32_t mul, uint32_t div, uint32_t postdiv, uint32_t vco_mode)
{
	uint32_t			reg_val = 0;

	// PLL Setting
	reg_val = (PLLCON_PLLnRESB|PLLCON_PLLEN);
	reg_val |= (PLLCON_PREDIV_VAL(prediv)|PLLCON_MULTI_VAL(mul)|PLLCON_DIV_VAL(div)|\
		          PLLCON_POSTDIV_VAL(postdiv)|vco_mode);

	PMU->PLLCON = reg_val;

	while(!(PMU->PLLCON & PLLCON_LOCKSTS)); 			//  Is the PLL stable(lock-state)?

	PMU->PLLCON |= PLLCON_BYPASS_DISABLE;				// PLL-bypass (0 = bypass the input clock on PLL to the out-clock , 1 = multiply the input clock on PLL to the out-clock)
}


/**
************************************************************************************
* @ Name : PMU_ConfigureInterrupt
*
* @ Parameter
*		pmu			= PMU
*
*		intr_mask	= PMUCMR_SXOSCIE, PMUCMR_MXOSCIE, (PMUCMR_SXOSCIE|PMUCMR_MXOSCIE)
*
*		enable		= INTR_ENABLE, INTR_DISABLE
*
*
************************************************************************************
*/
void PMU_ConfigureInterrupt (PMU_Type *pmu, uint32_t intr_mask, uint32_t enable)
{
	uint32_t		reg_val;

	reg_val = PMU->CMR;
	reg_val &= ~(PMUCMR_SXOSCIE|PMUCMR_MXOSCIE); 							//disable interrupt

	PMU->CMR = (reg_val|(PMUCMR_SXOSCIF|PMUCMR_MXOSCIF)); 			// clear interrupt flag

	if (enable)
	{
		reg_val |= (intr_mask & (PMUCMR_SXOSCIE|PMUCMR_MXOSCIE)); 	// enable interrupt
		PMU->CMR = reg_val;
	}
}

/**
************************************************************************************
* @ Name : PMU_CheckResetEvent
*
* @ Parameter
*		pmu			= PMU
*
* @ Function
*		Print Reset Event
************************************************************************************
*/
void PMU_CheckResetEvent (PMU_Type *pmu)
{
	uint32_t		reg_val;

	reg_val = (pmu->RSSR);
	cprintf("\r\n\r\n[0x%08X] Reset Event by ", reg_val);

	if((reg_val&PMURSSR_MCKFAILRST) == PMURSSR_MCKFAILRST)
		cprintf("/Main Clock Failure ");
	if((reg_val&PMURSSR_RSTRST) == PMURSSR_RSTRST)
		cprintf("/nReset");
	if((reg_val&PMURSSR_SYSRST) == PMURSSR_SYSRST)
		cprintf("/Core Reset Request ");
	if((reg_val&PMURSSR_SWRST) == PMURSSR_SWRST)
		cprintf("/Software Request ");
	if((reg_val&PMURSSR_WDTRST) == PMURSSR_WDTRST)
		cprintf("/Watchdog Reset ");
	if((reg_val&PMURSSR_MXFAILRST) == PMURSSR_MXFAILRST)
		cprintf("/Main X-tal OSC Failure ");
	if ((reg_val&PMURSSR_LVDRST) == PMURSSR_LVDRST)
		cprintf("/LVD Reset ");
	if(reg_val == 0x0)
		cputs("/Power On Reset\r\n");

	pmu->RSSR = 0;
	cputs("\r\n");

}


/* --------------------------------- End Of File ------------------------------ */
