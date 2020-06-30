/**************************************************************************//**
 * @file     system_A33G52x.c
 * @brief    CMSIS Device System Source File for
 *           ARMCM3 Device Series
 * @version  V1.08
 * @date     26. July 2017
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2011 - 2017 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#include <stdint.h>
#include "A33G52x.h"
#include "system_A33G52x.h"
#include "a33g52x_pmu.h"
#include "a33g52x_pcu.h"



/*----------------------------------------------------------------------------
  Clock variables
 *----------------------------------------------------------------------------*/
uint32_t		SystemCoreClock;
uint32_t		SystemPeriClock;

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
	uint32_t		reg_val;
	
 	///////////////////////////////////////////////////////////////////////////////////
 	PMULEGACY->LEGACY = PMULEGACY_NORMAL_STEP1;
 	PMULEGACY->LEGACY = PMULEGACY_NORMAL_STEP2;
	__enable_irq();			// Enable Global interrupt in core
 	do {
 		reg_val = PMULEGACY->LEGACY;
 	} while ((reg_val & PMULEGACY_MODE) != PMULEGACY_MODE_NORMAL);
 	///////////////////////////////////////////////////////////////////////////////////	
	
	WDT->CON = 0;				// Disable Watchdog Timer
	FMC->CFG = 0x0303;		// Set Flash Wait Time
	
	// JTAG(SWO) Port Setting
	PCU_ConfigureFunction (PCC, PIN_0, PC0_MUX_nTRST);
	PCU_ConfigureFunction (PCC, PIN_1, PC1_MUX_TDI); 
	PCU_ConfigureFunction (PCC, PIN_2, PC2_MUX_TMS); 
	PCU_ConfigureFunction (PCC, PIN_3, PC3_MUX_TCK); 
	PCU_ConfigureFunction (PCC, PIN_4, PC4_MUX_TDO); 	
	
	PCU_Set_Direction_Type(PCC, PIN_0, PnCR_INPUT_LOGIC); 
	PCU_Set_Direction_Type(PCC, PIN_1, PnCR_INPUT_LOGIC); 
	PCU_Set_Direction_Type(PCC, PIN_2, PnCR_INPUT_LOGIC);
	PCU_Set_Direction_Type(PCC, PIN_3, PnCR_INPUT_LOGIC); 
	PCU_Set_Direction_Type(PCC, PIN_4, PnCR_OUTPUT_PUSH_PULL); 	

	PCC->PCR = 0x4F;

	reg_val = PMU->CCR;
	reg_val &= ~PMUCCR_ROSCEN_MASK; 
	reg_val |= PMUCCR_ROSCEN_DIV_BY_1; 				// RingOSC

	PMU->CCR = reg_val;
	PMU->BCCR = 0x00;
	PMU->PLLCON = 0;
	
	SystemCoreClock 	= 1000000;
	SystemPeriClock 	= 1000000;
}
 
 
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
	uint32_t		reg_val;
	
//	uint32_t		hclk_info, pclk_info; 

	
	reg_val = PMU->BCCR;
	reg_val &= ~(PMUBCCR_MCLKSEL_MASK|PMUBCCR_PCLKDIV|PMUBCCR_HCLKDIV_MASK); 
	reg_val |= PMUBCCR_MCLKSEL_RING_OSC; 			// RingOSC to main clock
	reg_val |= PMUBCCR_HCLKDIV_DIV_BY_1; 				// Main Clock ->  HCLK = HCLK/1 (AHB Bus Clock)
	reg_val |= PMUBCCR_PCLKDIV_DIV_BY_1; 				// Main Clock -> PCLK = PCLK/1 (APB Bus Clock)
	PMU->BCCR = reg_val;

	PCU_ConfigureFunction (PCC, PIN_14, PC14_MUX_XTALO); 						// XTAL_OUT
	PCU_ConfigureFunction (PCC, PIN_15, PC15_MUX_XTALI); 						// XTAL_INPUT
	PCU_Set_Direction_Type(PCC, PIN_14, PnCR_INPUT_ANALOG); 
	PCU_Set_Direction_Type(PCC, PIN_15, PnCR_INPUT_ANALOG); 
	PCU_ConfigurePullup_Pulldown (PCC, PIN_14, 0, PnPCR_PULLUPDOWN_DISABLE); 
	PCU_ConfigurePullup_Pulldown (PCC, PIN_15, 0, PnPCR_PULLUPDOWN_DISABLE); 
			
	PCU_ConfigureFunction (PCE, PIN_8, PE8_MUX_SXIN); 							// SXTAL_INPUT
	PCU_ConfigureFunction (PCE, PIN_9, PE9_MUX_SXOUT); 							// SXTAL_OUTPUT
	PCU_Set_Direction_Type(PCE, PIN_8, PnCR_INPUT_ANALOG); 
	PCU_Set_Direction_Type(PCE, PIN_9, PnCR_INPUT_ANALOG); 
	PCU_ConfigurePullup_Pulldown (PCE, PIN_8, 0, PnPCR_PULLUPDOWN_DISABLE); 
	PCU_ConfigurePullup_Pulldown (PCE, PIN_9, 0, PnPCR_PULLUPDOWN_DISABLE); 
	
	reg_val = PMU->CCR;
	reg_val &= ~(PMUCCR_IOSC16EN_MASK|PMUCCR_SXOSCEN_MASK|PMUCCR_MXOSCEN_MASK); 
	reg_val |= (PMUCCR_IOSC16EN_DIV_BY_1); 						// IOSC16 Enable
	reg_val |= PMUCCR_MXOSCEN_DIV_BY_1; 							// MXOSC Enable
	reg_val |= PMUCCR_SXOSCEN_DIV_BY_1; 							// SXOSC Enable
	PMU->CCR = reg_val;

	reg_val = PMUCMR_MXOSCSTS; 
	reg_val |= PMUCMR_MXOSCMNT; 										// MXOSC -> Monitoring of MXOSC Oscillation
	PMU->CMR = reg_val;
	
	if (PMU->CCR&PMUCCR_MXOSCEN_DIV_BY_1)					// Check Oscillation of MXOSC
	{
		while(1) {
			reg_val = PMU->CMR;
			if( (reg_val & PMUCMR_MXOSCSTS) == PMUCMR_MXOSCSTS)			// Does MXOSC oscillate normally? 
				break;
		}
	}

	
	if (PMU_SetPLLFreq(PMU, XTAL8MHz, PLL74MHz))			// PLL Setting
		;

	
	PMU->PCCR = 0xFFFFFFFFUL;					// Peipheral Clock Control Enable
	PMU->PER = 0xFFFFFFFFUL;					// Peripheral Enable
	PMU->PRER = 0xFFFFFFFFUL;					// Peripheral Reset Enable
	
	PMU->LVDCON = (LVDCON_LVDEN|LVDCON_LVDRF|LVDCON_LVDREN|LVDCON_LVDRL_2_60V);			// LVD Reset 2.6V

//	PMU->RSSR = 0;																		// clear Reset Source Status

	PCC->DPR	 	= 8;																	// PC6 debounce 
	PCC->DER 	=  (0x0001<<6);													// PC6 (nReset)
	PCU_ConfigureFunction (PCC, PIN_6, PC6_MUX_nRESET); 
	PCU_Set_Direction_Type(PCC, PIN_6, PnCR_INPUT_LOGIC); 
	PCU_ConfigurePullup_Pulldown (PCD, PIN_6, 0, PnPCR_PULLUPDOWN_DISABLE); 

	PMU->RSER = (PMU->RSER) | (PMURSER_RSTRSTE|PMURSER_LVDRSTE);			// set Reset Source (LVD, nRESET)
}
