/***************************************************************************//**
 * @file     system_A34M41x.c
 * @brief    CMSIS Cortex-M# Device Peripheral Access Layer Source File for
 *           Device A34M41x
 * @version  V3.10
 * @date     30. August 2016
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED

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
#include "A34M41x.h"
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_cfmc.h"


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
 
/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  HLCK */
uint32_t SystemPeriClock;     /*!< System Clock Frequency (Peri Clock)  PCLK */


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/

void SystemInit (void)
{
	
  __disable_irq();
	// WDT Disable
	WDT->AEN = 0xA55A;
	WDT->CON = 0;
	WDT->AEN = 0;
	
	SystemCoreClock = 500000;				// 500KHz
	SystemPeriClock = 500000;				// 500KHz
	
	//=========================================================================
	// Flash Memory Control
	//=========================================================================
	
	// Data/Instruction Cache Disable
	HAL_CFMC_CacheCmd((CFMC_CONF_INST_CACHE_ON | CFMC_CONF_DATA_CACHE_ON), DISABLE);
	
	// Data/Instruction Cache Reset
	HAL_CFMC_CacheCmd((CFMC_CONF_INST_CACHE_RST|CFMC_CONF_DATA_CACHE_RST), ENABLE);
	
	// Data/Instruction Cache Enable
	HAL_CFMC_CacheCmd((CFMC_CONF_INST_CACHE_ON | CFMC_CONF_DATA_CACHE_ON), ENABLE);
	
	HAL_CFMC_WaitCmd(7);		// Flash Wait 7		
}

