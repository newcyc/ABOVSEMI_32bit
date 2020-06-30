/***************************************************************************//**
* @file     A31G22x_pwr.c
* @brief    Contains all functions support for power control dirver on A31G22x
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
#include "A31G22x_scu.h"
#include "A31G22x_pwr.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/



/*******************************************************************************
* Private Typedef
*******************************************************************************/


/*******************************************************************************
* Private Variable
*******************************************************************************/


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/


/*******************************************************************************
* Public Function
*******************************************************************************/

/***************************************************************************//**
* @brief      Initialize LVI(Low Voltage Indicator)
* @param      pConfig : Pointer contains configuration of LVI
* @return     None
*******************************************************************************/
void LVI_Init(LVI_CFG_Type *pConfig)
{
	SCULV->LVICR = 0
	| ((pConfig->EnableIndicator << SCULV_LVICR_LVIEN_Pos) & SCULV_LVICR_LVIEN_Msk)
	| ((pConfig->EnableInterrupt << SCULV_LVICR_LVINTEN_Pos) & SCULV_LVICR_LVINTEN_Msk)
	| ((pConfig->LVI_Voltage << SCULV_LVICR_LVIVS_Pos) & SCULV_LVICR_LVIVS_Msk)
	;
}

/***************************************************************************//**
* @brief      Get interrupt status of LVI
* @param      None
* @return     SET : Occurred, RESET : Not occurred
*******************************************************************************/
FlagStatus LVI_GetInterruptStatus(void)
{
	return (SCULV->LVICR & SCULV_LVICR_LVIFLAG) ? SET : RESET;
}

/***************************************************************************//**
* @brief      Clear interrupt status of LVI
* @param      None
* @return     None
*******************************************************************************/
void LVI_ClearInterruptStatus(void)
{
	SCULV->LVICR |= SCULV_LVICR_LVIFLAG;
}

/***************************************************************************//**
* @brief      Initialize LVR(Low Voltage Reset)
* @param      pConfig : Pointer contains configuration of LVR
* @return     None
*******************************************************************************/
void LVR_Init(LVR_CFG_Type *pConfig)
{
	volatile uint32_t Reg32;

	// LVR Enable/Disable
	SCULV->LVRCR = pConfig->EnableReset;			// Get LVR Enable/Disable

	// Setting LVR operation mode and LVR Level
	Reg32 = SCULV_LVRCNFIG_WTIDKY \
				| ((pConfig->LVR_Voltage << SCULV_LVRCNFIG_LVRVS_Pos) & SCULV_LVRCNFIG_LVRVS_Msk) \
				| ((pConfig->EnableOperation << SCULV_LVRCNFIG_LVRENM_Pos) & SCULV_LVRCNFIG_LVRENM_Msk);
	
	SCULV->LVRCNFIG = Reg32;
}

/***************************************************************************//**
* @brief      Enter Sleep mode with co-operated instruction by the Cortex-M0+
* @param      None
* @return     None
*******************************************************************************/
void PWR_EnterSleep(void)
{
	SCB->SCR = 0x00UL
		| (0x00UL << SCB_SCR_SEVONPEND_Pos) // Only enabled interrupts or events can wake up the processor
		| (0x00UL << SCB_SCR_SLEEPDEEP_Pos) // Sleep
		| (0x00UL << SCB_SCR_SLEEPONEXIT_Pos) // Do not sleep when returning to Thread mode
		;

	__DSB();
	__WFI();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

/***************************************************************************//**
* @brief      Enter Power Down mode with co-operated instruction by the Cortex-M0+
* @param      None
* @return     None
*******************************************************************************/
void PWR_EnterDeepSleep(void)
{
	SCB->SCR = 0x00UL
		| (0x00UL << SCB_SCR_SEVONPEND_Pos) // Only enabled interrupts or events can wake up the processor
		| (0x01UL << SCB_SCR_SLEEPDEEP_Pos) // Deep sleep
		| (0x00UL << SCB_SCR_SLEEPONEXIT_Pos) // do not sleep when returning to Thread mode
		;

	__WFI();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

/***************************************************************************//**
 * @brief      Ready LSI, BGR, VDC AlwaysOn Mode before entering Power Down mode 
 * @param[in]	LSI
 * 					- LSIAON_DISABLE		   : 0
 * 					- LSIAON_ENABLE    	   : 1
 * @param[in]	BGR
 * 					- BGRAON_DISABLE        : 0
 * 					- BGRAON_ENABLE    	   : 1
 * @param[in]	VDC
 * 					- VDCAON_DISABLE       : 0
 * 					- VDCAON_ENABLE    	   : 1
 * @return     None
*******************************************************************************/
void PWR_AlwaysOnLSIForDeepSleep(uint8_t LSI, uint8_t BGR, uint8_t VDC)
{
#if 0
	uint32_t Reg32;
	LVR_CFG_Type LvrCfg;
	
	// Trim mode
	CFMC->MR = 0xA5;
	CFMC->MR = 0x5A;
	
	// VDCCON : STOP2 (Power Down Setting)
	Reg32 = SCU->VDCCON;
	Reg32 = (Reg32) | (0x5 << SCU_VDCCON_VDC_WTIDKY_Pos) | (1 << SCU_VDCCON_VDC_PDBGR_Pos) | (1 << SCU_VDCCON_VDC_STOP_Pos);
	SCU->VDCCON = Reg32;
	
	// LVR Disable
	LvrCfg.EnableReset = LVR_DISABLE;
	LVR_Init(&LvrCfg);
#endif
	// Set System Mode
	SCU->SMR = (LSI << SCU_SMR_LSIAON_Pos) | (BGR << SCU_SMR_BGRAON_Pos) | (VDC << SCU_SMR_VDCAON_Pos);
}

/* --------------------------------- End Of File ------------------------------ */
