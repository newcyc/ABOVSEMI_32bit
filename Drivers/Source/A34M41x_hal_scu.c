/**********************************************************************
* @file		A34M41x_scu.c
* @brief	Contains all functions support for SCU firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_cfmc.h"

/**********************************************************************
 * @brief 		System Mode Register  
 * @param[in]	operating clk
 * 					- SCU_SMR_LSEAON : (1<<13)
 * 					- SCU_SMR_HSEAON : (1<<12)
 * 					- SCU_SMR_PLLAON  : (1<<11)
 * 					- SCU_SMR_HSIAON : (1<<10)
 *					- SCU_SMR_LSIAON : (1<<9)
 *					- SCU_SMR_VDCAON  : (1<<8)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 **********************************************************************/
void HAL_SCU_SMRCmd(uint16_t operating_clk, FunctionalState NewState)
{
	uint16_t	reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->SMR;
	reg_val &= ~(operating_clk);

	if (NewState == ENABLE)
	{
		reg_val |= (operating_clk);
	}
	
	SCU->SMR = reg_val;
	
	SYST_ACCESS_DIS();
}



/**********************************************************************
 * @brief 		Prev Mode Register  
 * @param[in]	
 * 
 * 
 * @return		Pervious mode bit
 * 					- 0 : Run Mode
 * 					- 1 : Sleep Mode
 * 					- 2 : Deep-Sleep Mode
* 					- 3 : INIT Mode
 **********************************************************************/
uint16_t HAL_SCU_GetPREVStatus (void)
{
	uint16_t	reg_val;
	
	reg_val = (((SCU->SMR)>>4) & 0x03);
	
	return reg_val;
}



/**********************************************************************
 * @brief 		Wake Up Source Enable/Disable  
 * @param[in]	
 * 				WakeUpSrc
 * 					- GPIOGWUE : (1<<14)
 * 					- GPIOFWUE : (1<<13)
 * 					- GPIOEWUE : (1<<12)
 * 					- GPIODWUE : (1<<11)
 *					- GPIOCWUE : (1<<10)
 * 					- GPIOBWUE : (1<<9)
 * 					- GPIOAWUE : (1<<8)
 * 					- FRT1WUE  : (1<<3)
 * 					- FRT0WUE  : (1<<2)
 *					- WDTWUE   : (1<<1)
 * 					- LVIWUE   : (1<<0)
 *
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * 
 * @return		None
 **********************************************************************/
void HAL_SCU_WakeUpSRCCmd(uint16_t WakeUpSrc, FunctionalState NewState)
{
	uint16_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->WUER;
	reg_val &= ~(WakeUpSrc);
	
	if (NewState == ENABLE)
	{
		reg_val |= (WakeUpSrc);
	}
	
	SCU->WUER = reg_val;
	
	SYST_ACCESS_DIS();
}

/*********************************************************************
 * @brief 		Get current value of WakeUpSRC Status.
 * @param[in]	None.
 * @return		Current value of WakeUpSRC Status register.
 *********************************************************************/
uint16_t HAL_SCU_GetWakeUpSRCStatus(void)
{
	return (SCU->WUSR);
}



/**********************************************************************
 * @brief 		Reset Source Enable/Disable Register  
 * @param[in]	
 * 				ResetSRC
 * 					- RST_LOCKUPRST : (1<<9)
 * 					- RST_PORST    : (1<<8) // POR cannot set by user. (20191210)
 * 					- RST_PINRST    : (1<<7)
 * 					- RST_CPURST     : (1<<6)
 *					- RST_SWRST    : (1<<5)
 * 					- RST_WDTRST   : (1<<4)
 * 					- RST_MCLKFRST    : (1<<3)
 * 					- RST_LSEFRST    : (1<<2)
 * 					- RST_HSEFRST    : (1<<1)
 *					- RST_LVDRST  : (1<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * 
 * @return		None
 **********************************************************************/
void HAL_SCU_ResetSRCCmd(uint16_t ResetSRC, FunctionalState NewState)
{
	uint16_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->RSER;
	reg_val &= ~(ResetSRC);
	
	if (NewState == ENABLE)
	{
		reg_val |= (ResetSRC);
	}
	
	SCU->RSER = reg_val;
	
	SYST_ACCESS_DIS();
}




/*********************************************************************
 * @brief 		Get current value of Reset Source Status.
 * @param[in]	None.
 * @return		Current value of ResetSRC Status register.
 *********************************************************************/
uint16_t HAL_SCU_GetResetSRCStatus(void)
{
	return (SCU->RSSR);
}


/**********************************************************************
 * @brief 		Cleare Reset Event Flag
 * @param[in]	
 * 				ResetSRC
 * 					- RST_LOCKUPRST : (1<<9)
 * 					- RST_PORST    : (1<<8)
 * 					- RST_PINRST    : (1<<7)
 * 					- RST_CPURST     : (1<<6)
 *					- RST_SWRST    : (1<<5)
 * 					- RST_WDTRST   : (1<<4)
 * 					- RST_MCLKFRST    : (1<<3)
 * 					- RST_LSEFRST    : (1<<2)
 * 					- RST_HSEFRST    : (1<<1)
 *					- RST_LVDRST  : (1<<0)
 * @return		None
 **********************************************************************/
void
HAL_SCU_ClearResetFlag(uint16_t ResetEvent)
{
	SYST_ACCESS_EN();
	
	SCU->RSSR = SCU->RSSR | ResetEvent;
	
	SYST_ACCESS_DIS();
}

/**********************************************************************
 * @brief 		Peripheral Enable/Disable Register  
 * @param[in]	
 * 				PeriSRC
 * 					- PRRI_QEI1    : (1<<29)
 * 					- PERI_QEI0    : (1<<28)
 * 					- PERI_TIMER9  : (1<<25)
 * 					- PERI_TIMER8  : (1<<24)
 * 					- PERI_TIMER7  : (1<<23)
 * 					- PERI_TIMER6  : (1<<22)
 * 					- PERI_TIMER5  : (1<<21)
 * 					- PERI_TIMER4  : (1<<20)
 * 					- PERI_TIMER3  : (1<<19)
 * 					- PERI_TIMER2  : (1<<18)
 * 					- PERI_TIMER1  : (1<<17)
 * 					- PERI_TIMER0  : (1<<16)
 *					- PERI_GPIOG   : (1<<14)
 *					- PERI_GPIOF   : (1<<13)
 *					- PERI_GPIOE   : (1<<12)
 *					- PERI_GPIOD   : (1<<11)
 *					- PERI_GPIOC   : (1<<10)
 *					- PERI_GPIOB   : (1<<9)
 *					- PERI_GPIOA   : (1<<8)
 * 					- PERI_FRT1    : (1<<7)
 * 					- PERI_FRT0    : (1<<6)
 * 					- PERI_DMA     : (1<<4)
 * 					- PERI_DFMC    : (1<<3)	Read Only 
 * 					- PERI_WDT     : (1<<2) Read Only 
 * 					- PERI_CFMC    : (1<<1)	Read Only 
 * 					- PERI_SCU     : (1<<0) Read Only 
 *
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * 
 * @return		None
 **********************************************************************/
void HAL_SCU_PeriSRCCmd_1(uint32_t PeriSRC, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->PER1;
	reg_val &= ~(PeriSRC);
	
	if (NewState == ENABLE)
	{
		reg_val |= (PeriSRC);
	}
	
	SCU->PER1 = reg_val;
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief 		Peripheral Enable/Disable Register  
 * @param[in]	
 * 				PeriSRC
 * 					- PERI_RNG        : (1<<31)
 * 					- PERI_AES        : (1<<30)
 * 					- PERI_CRC : (1<<29)
 * 					- PERI_COMPARATOR : (1<<28)
 * 					- PERI_CAN : (1<<26)
 * 					- PERI_PGA : (1<<24)
 * 					- PERI_ADC2       : (1<<22)
 * 					- PERI_ADC1       : (1<<21)
 * 					- PERI_ADC0       : (1<<20)
 * 					- PERI_MPWM1      : (1<<17)
 * 					- PERI_MPWM0      : (1<<16)
 * 					- PERI_UART5      : (1<<13)
 * 					- PERI_UART4      : (1<<12)
 * 					- PERI_UART3      : (1<<11)
 *					- PERI_UART2      : (1<<10)
 *					- PERI_UART1      : (1<<9)
 *					- PERI_UART0      : (1<<8)
 *					- PERI_I2C1       : (1<<5)
 *					- PERI_I2C0       : (1<<4)
 *					- PERI_SPI2       : (1<<2)
 * 					- PERI_SPI1       : (1<<1)
 * 					- PERI_SPI0       : (1<<0)
 *
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * 
 * @return		None
 **********************************************************************/
void HAL_SCU_PeriSRCCmd_2(uint32_t PeriSRC, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->PER2;
	reg_val &= ~(PeriSRC);
	
	if (NewState == ENABLE)
	{
		reg_val |= (PeriSRC);
	}
	
	SCU->PER2 = reg_val;
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief 		Peripheral Clock Enable/Disable Register  
 * @param[in]	
 * 				PeriSRC
 * 					- PERI_QEI1    : (1<<29)
 * 					- PERI_QEI0    : (1<<28)
 * 					- PERI_TIMER9  : (1<<25)
 * 					- PERI_TIMER8  : (1<<24)
 * 					- PERI_TIMER7  : (1<<23)
 * 					- PERI_TIMER6  : (1<<22)
 * 					- PERI_TIMER5  : (1<<21)
 * 					- PERI_TIMER4  : (1<<20)
 * 					- PERI_TIMER3  : (1<<19)
 * 					- PERI_TIMER2  : (1<<18)
 * 					- PERI_TIMER1  : (1<<17)
 * 					- PERI_TIMER0  : (1<<16)
 *					- PERI_GPIOG   : (1<<14)
 *					- PERI_GPIOF   : (1<<13)
 *					- PERI_GPIOE   : (1<<12)
 *					- PERI_GPIOD   : (1<<11)
 *					- PERI_GPIOC   : (1<<10)
 *					- PERI_GPIOB   : (1<<9)
 *					- PERI_GPIOA   : (1<<8)
 * 					- PERI_FRT1    : (1<<7)
 * 					- PERI_FRT0    : (1<<6)
 * 					- PERI_DMA     : (1<<4)
 *
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * 
 * @return		None
 **********************************************************************/
void HAL_SCU_PeriCLKCmd_1(uint32_t PeriSRC, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->PCER1;
	reg_val &= ~(PeriSRC);
	
	if (NewState == ENABLE)
	{
		reg_val |= (PeriSRC);
	}
	
	SCU->PCER1 = reg_val;
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief 		Peripheral Clock Enable/Disable Register  
 * @param[in]	
 * 				PeriSRC
 * 					- PERI_RNG        : (1<<31)
 * 					- PERI_AES        : (1<<30)
 * 					- PERI_COMPARATOR : (1<<28)
 * 					- PERI_OPAMP      : (1<<24)
 * 					- PERI_ADC2       : (1<<22)
 * 					- PERI_ADC1       : (1<<21)
 * 					- PERI_ADC0       : (1<<20)
 * 					- PERI_MPWM1      : (1<<17)
 * 					- PERI_MPWM0      : (1<<16)
 * 					- PERI_UART5      : (1<<13)
 * 					- PERI_UART4      : (1<<12)
 * 					- PERI_UART3      : (1<<11)
 *					- PERI_UART2      : (1<<10)
 *					- PERI_UART1      : (1<<9)
 *					- PERI_UART0      : (1<<8)
 *					- PERI_CAN        : (1<<6)
 *					- PERI_I2C1       : (1<<5)
 *					- PERI_I2C0       : (1<<4)
 *					- PERI_SPI2       : (1<<2)
 * 					- PERI_SPI1       : (1<<1)
 * 					- PERI_SPI0       : (1<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * 
 * @return		None
 **********************************************************************/
void HAL_SCU_PeriCLKCmd_2(uint32_t PeriSRC, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->PER2;
	reg_val &= ~(PeriSRC);
	
	if (NewState == ENABLE)
	{
		reg_val |= (PeriSRC);
	}
	
	SCU->PER2 = reg_val;
	
	SYST_ACCESS_DIS();
}



/**********************************************************************
 * @brief 		Clock Source Control Register  
 * @param[in]	clock Function mode, should be:
 * 					- LSE		   : (1<<5)
 * 					- HSE   	   : (1<<4)
 * 					- HSI    	   : (1<<1)
 * 					- LSI 		   : (1<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 **********************************************************************/
void HAL_SCU_ClockSRCCmd (uint8_t ClockSRC, FunctionalState NewState)
{
	uint32_t		reg_val, i;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->CSCR;
	
	reg_val &= ~ ClockSRC;

	if (NewState == ENABLE)
	{
		reg_val |= (ClockSRC);
	}
	
	SCU->CSCR = reg_val;
	
	// Stable Time
	for(i=0; i<100; i++);
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief 		PCLK Input Clock Divider Register  
 * @param[in]	clock Function mode, should be:
 * 					- PCLKDIV_HCLK_1  : (0)
 * 					- PCLKDIV_HCLK_2  : (1)
 * 					- PCLKDIV_HCLK_4  : (2)
 * 					- PCLKDIV_HCLK_8  : (3)
 * 					- PCLKDIV_HCLK_16 : (4)
 *
 * @return		None
 **********************************************************************/
void HAL_SCU_PCLKDIVCmd (uint8_t PCLKDIV)
{
	SYST_ACCESS_EN();
	
	SCU->SCCR = (SCU->SCCR & ~SCU_SCCR_PCLKDIV_MSK) | ((PCLKDIV & 0x7) << SCU_SCCR_PCLKDIV_POS);
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief 		PLL Input Clock Selection  
 * @param[in]	clock Function mode, should be:
 * 					- PLLINCLK=HSI  : (0)
 * 					- PLLINCLK=HSE  : (1)
 *
 * @return		None
 **********************************************************************/
void HAL_SCU_PLLINCLKCmd (uint32_t PLLINSRC)
{
	SYST_ACCESS_EN();
	
	SCU->SCCR = (SCU->SCCR & ~SCU_SCCR_PLLCLKSEL_MSK) | ((PLLINSRC & 0x1) << SCU_SCCR_PLLCLKSEL_POS);
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief 		PLL Input Clock Divider  
 * @param[in]	clock Function mode, should be:
 * 					- PLLPREDIV_PLLINCLK_1	   : (0)
 * 					- PLLPREDIV_PLLINCLK_2	   : (1)
 * 					- PLLPREDIV_PLLINCLK_4	   : (2)
 * 					- PLLPREDIV_PLLINCLK_8	   : (3)
 *
 * @return		None
 **********************************************************************/
void HAL_SCU_PLLPREDIVCmd (uint8_t PLLPREDIV)
{
	SYST_ACCESS_EN();
	
	SCU->SCCR = (SCU->SCCR & ~SCU_SCCR_PLLPREDIV_MSK) | ((PLLPREDIV & 0x3) << SCU_SCCR_PLLPREDIV_POS);
	
	SYST_ACCESS_DIS();
}

/**********************************************************************
 * @brief 		HCLK Divider Selection  
 * @param[in]	clock Function mode, should be:
 * 					- HCLKDIV_MCLK_1	   : (0)
 * 					- HCLKDIV_MCLK_2	   : (1)
 * 					- HCLKDIV_MCLK_4	   : (2)
 * 					- HCLKDIV_MCLK_8	   : (3)
 * 					- HCLKDIV_MCLK_16	   : (4)
 * 					- HCLKDIV_MCLK_32	   : (5)
 * 					- HCLKDIV_MCLK_64	   : (6)
 * 					- HCLKDIV_MCLK_128	   : (7)
 * 					- HCLKDIV_MCLK_256	   : (8)
 * 					- HCLKDIV_MCLK_512	   : (9)
 *
 * @return		None
 **********************************************************************/
void HAL_SCU_HCLKDIVCmd (uint8_t HCLKDIV)
{
	SYST_ACCESS_EN();
	
	SCU->SCCR = (SCU->SCCR & ~SCU_SCCR_HCLKDIV_MSK) | ((HCLKDIV & 0xf) << SCU_SCCR_HCLKDIV_POS);
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief 		Main(System) Clock Selection  
 * @param[in]	clock Function mode, should be:
 * 					- SCU_SCCR_MCLKSEL_LSI		   	: (0)
 * 					- SCU_SCCR_MCLKSEL_LSE			: (1)
 * 					- SCU_SCCR_MCLKSEL_HSI			: (2)
 * 					- SCU_SCCR_MCLKSEL_HSE		   	: (6)
 * 					- SCU_SCCR_MCLKSEL_PLL			: (7)
 *
 * @return		None
 **********************************************************************/
void HAL_SCU_MainCLKCmd (uint8_t MCLKSRC)
{
	SYST_ACCESS_EN();
	
	SCU->SCCR = (SCU->SCCR & ~SCU_SCCR_MCLKSEL_MSK) | ((MCLKSRC & 0x7) << SCU_SCCR_MCLKSEL_POS);
	
	SYST_ACCESS_DIS();
}



/**********************************************************************
 * @brief 		Clock Monitoring Register  
 * @param[in]	clock Function mode, should be:
 * 					- SCU_CMR_MCLKREC		: (1<<15)
 * 					- SCU_CMR_LSEMNT		: (1<<11)
 * 					- SCU_CMR_LSEIE		: (1<<10)
 * 					- SCU_CMR_MCLKMNT		: (1<<7)
 * 					- SCU_CMR_MCLKIE		: (1<<6)
 * 					- SCU_CMR_HSEMNT		: (1<<3)
 * 					- SCU_CMR_HSEIE		: (1<<2)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 **********************************************************************/
void HAL_SCU_CLKMNTCmd (uint16_t MonioringSRC, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->CMR;
	
	reg_val &= ~(MonioringSRC&0x8CCC);

	if (NewState == ENABLE)
	{
		reg_val |= (MonioringSRC&0x8CCC);
	}
	
	SCU->CMR = reg_val;
	
	SYST_ACCESS_DIS();
}


/**********************************************************************
 * @brief 		Clock Monitoring Flag Clear  
 * @param[in]	clock Function mode, should be:
 * 					- SCU_CMR_LSEFAIL		: (1<<9)
 * 					- SCU_CMR_MCLKFAIL		: (1<<5)
 * 					- SCU_CMR_HSEFAIL		: (1<<1)
 *
 * @return		None
 **********************************************************************/
void HAL_SCU_ClearCLKMNTFLG (uint16_t ClearMNTSRC)
{
	SYST_ACCESS_EN();
	
	SCU->CMR |= (ClearMNTSRC&0x222);

	SYST_ACCESS_DIS();
}


/*********************************************************************
 * @brief 		Get current value of Clock Monitoring Status.
 * @param[in]	None.
 * @return		Current value of Clock Monitoring Status register.
 *********************************************************************/
uint16_t HAL_SCU_GetCLKMNTStatus(void)
{
	return (SCU->CMR);
}


/**********************************************************************
 * @brief 		NMI Control Register  
 * @param[in]	NMI Source selection, should be:
 * 					- NMISRC : 0x00~0xFF
 * @param[in]	NMI Interrupt selection, should be:
 * 					- SCU_NMICR_NMIINEN		: (1<<15)
 * 					- SCU_NMICR_PROT1EN		: (1<<6)
 * 					- SCU_NMICR_OVP1EN		: (1<<5)
 * 					- SCU_NMICR_PROT0EN		: (1<<4)
 * 					- SCU_NMICR_OVP0EN		: (1<<3)
 * 					- SCU_NMICR_WDTINTEN	: (1<<2)
 * 					- SCU_NMICR_MCLKFAILEN	: (1<<1)
 * 					- SCU_NMICR_LVDEN		: (1<<0)
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 **********************************************************************/
void HAL_SCU_NMICmd (uint8_t NMISRC, uint16_t NMIINTB, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->NMICR;
	
	reg_val &= ~((0xFF<<16) | (NMIINTB));
	
	reg_val |= ((NMISRC&0xFF)<<16);
	
	if (NewState == ENABLE)
	{
		reg_val |= (NMIINTB);
	}
	
	SCU->NMICR = reg_val;
	
	SYST_ACCESS_DIS();
}


/*********************************************************************
 * @brief 		Get current value of NMI Status.
 * @param[in]	None.
 * @return		Current value of NMI Status register.
 *********************************************************************/
uint32_t HAL_SCU_GetNMIStatus(void)
{
	return (SCU->NMISR);
}


/**********************************************************************
 * @brief 		NMI Flag Clear  
 * @param[in]	clock Function mode, should be:
 * 					- SCU_NMISR_NMIINTSTS	: (1<<15)
 * 					- SCU_NMISR_PROT1		: (1<<6)
 * 					- SCU_NMISR_OVP1		: (1<<5)
 * 					- SCU_NMISR_PROT0		: (1<<4)
 * 					- SCU_NMISR_OVP0		: (1<<3)
 * 					- SCU_NMISR_WDTINT		: (1<<2)
 * 					- SCU_NMISR_MCLKFAIL		: (1<<1)
 * 					- SCU_NMISR_LVDSTS		: (1<<0)
 * @return		None
 **********************************************************************/
void HAL_SCU_ClearNMIFLG (uint16_t ClearNMISRC)
{
	SCU->NMISR |= (SCU_NMISR_WTIDKY | (ClearNMISRC&0x7F));
}


/**********************************************************************
 * @brief 		Clock Output Register  
* @param[in]	Clock source selection, should be:
 * 					- SCU_COR_CLKOINSEL_LSI : (0<<5)
 * 					- SCU_COR_CLKOINSEL_LSE : (1<<5)
 * 					- SCU_COR_CLKOINSEL_MCLK : (4<<5)
 * 					- SCU_COR_CLKOINSEL_HSI : (5<<5)
 * 					- SCU_COR_CLKOINSEL_HSE : (6<<5)
 * 					- SCU_COR_CLKOINSEL_PLL : (7<<5)
* @param[in]	Clock divider, should be:
* 					- 0x0 ~ 0xF
 * @param[in]	NewState
 *					- Enable : Bit Enable
 * 					- Disable : Bit Disable
 * @return		None
 **********************************************************************/
void HAL_SCU_CORCmd (uint8_t CLKOSRC, uint8_t CLKODIV, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->COR;
	
	reg_val = 0;
	
	reg_val |= ((CLKOSRC&(0x07<<5)) | (CLKODIV&0x0F));
	
	if (NewState == ENABLE)
	{
		reg_val |= (SCU_COR_CLKOEN_ENABLE);
	}
	
	SCU->COR = reg_val;
	
	SYST_ACCESS_DIS();
}



/**********************************************************************
 * @brief 		PLL Control Register  
 * @param[in]	clock Function mode, should be:
 * 					
 *								Fin*(N1+1)
 *					Fout = --------------------- x D
 *							(R+1)*(N2+1)*(P+1)
 *
 *					-> List
 * 					  D  = PLLMODE  [20:20], 0 to 1
 * 					  R  = PREDIV   [18:16], 0 to 7
 *			 		  N1 = POSTDIV1 [15:08], 0 to 255
 *					  N2 = POSTDIV2 [07:04], 0 to 15
 *					  P  = OUTDIV   [03:00], 0 to 15
 * @return		None
 **********************************************************************/
void HAL_SCU_PLLCmd (uint32_t PLLMODE, uint8_t PREDIV, uint8_t POSTDIV1, uint8_t POSTDIV2, uint8_t OUTDIV, FunctionalState NewState)
{
	uint32_t		reg_val;
	
	SYST_ACCESS_EN();
	
	reg_val = SCU->PLLCON;
	
	reg_val &= ~(0x001FFFFF);
	
	reg_val |= (SCU_PLLCON_PLLRSTB) |(SCU_PLLCON_BYPASSB) ;

	reg_val |= (PLLMODE | ((PREDIV&0x7)<<16) | ((POSTDIV1&0xFF)<<8) | ((POSTDIV2&0xF)<<4) | ((OUTDIV&0xF)<<0));
	
	if (NewState == ENABLE)
	{
		reg_val |= (SCU_PLLCON_PLLEN);
	}
	
	SCU->PLLCON = reg_val;
	
	SYST_ACCESS_DIS();
}

uint32_t
HAL_SCU_GetCMRStatus(void)
{
	return (SCU->CMR);
}

/*********************************************************************
 * @brief 		Get current value of PLL Status.
 * @param[in]	None.
 * @return		Current value of PLL Status register.
 *********************************************************************/
uint32_t HAL_SCU_GetPLLStatus(void)
{
	return (SCU->PLLCON);
}



/**********************************************************************
 * @brief 		Configure peri clock setting
 * @param[in]	mccrnum: select REG number
 *                     - MCCR1 : 1, * - MCCR2 : 2
 *                     - MCCR3 : 3, * - MCCR4 : 4
 *                     - MCCR5 : 5, * - MCCR6 : 6
 * 					   - MCCR7 : 7
 * @param[in]	type : 
 *	                    WDT_TYPE, SYSTICK_TYPE, WDT_TYPE, MPWM0_TYPE, MPWM1_TYPE, 
 *						TIMER59_TYPE, TIMER04_TYPE, PGA_TYPE, ADC_TYPE,
 *						PGC_TYPE, PGB_TYPE, FRT1_TYPE, FRT0_TYPE,
 * 						CAN_TYPE, UART_TYPE
 * @param[in]	clksrc : 
 *						- SCU_MCCR_CSEL_LSI=0,
 *						- SCU_MCCR_CSEL_LSE=1,
 *						- SCU_MCCR_CSEL_MCLK=4,	
 *						- SCU_MCCR_CSEL_HSI=5,
 *						- SCU_MCCR_CSEL_HSE=6,
 * 						- SCU_MCCR_CSEL_PLL =7
 * @param[in]	clkdiv : 
 * @return		None
 **********************************************************************/
void HAL_SCU_SetMCCRx(uint8_t mccrnum, uint8_t type, uint8_t clksrc, uint8_t clkdiv)
{
	uint32_t reg_val;
	
	SYST_ACCESS_EN();
	
	switch(mccrnum){
		case 1:
			reg_val=SCU->MCCR1;
			if (type==SYSTICK_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK1 | SCU_MCCR_CSEL_MSK1);
				
				reg_val |= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			if (type==WDT_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK2 | SCU_MCCR_CSEL_MSK2);
				
				reg_val |= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR1=reg_val;
			break;
			
		case 2:
			reg_val=SCU->MCCR2;
			if (type==MPWM0_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK1 | SCU_MCCR_CSEL_MSK1);
				
				reg_val |= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			if (type==MPWM1_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK2 | SCU_MCCR_CSEL_MSK2);
				
				reg_val |= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR2=reg_val;
			break;
			
		case 3:
			reg_val=SCU->MCCR3;
			if (type==TIMER04_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK1 | SCU_MCCR_CSEL_MSK1);
				
				reg_val |= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			if (type==TIMER59_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK2 | SCU_MCCR_CSEL_MSK2);
				
				reg_val |= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR3=reg_val;
			break;
			
		case 4:
			reg_val=SCU->MCCR4;
			if (type==ADC_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK1 | SCU_MCCR_CSEL_MSK1);
				
				reg_val |= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			if (type==PGA_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK2 | SCU_MCCR_CSEL_MSK2);
				
				reg_val |= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR4=reg_val;
			break;
			
		case 5:
			reg_val=SCU->MCCR5;
			if (type==PGB_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK1 | SCU_MCCR_CSEL_MSK1);
				
				reg_val |= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			if (type==PGC_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK2 | SCU_MCCR_CSEL_MSK2);
				
				reg_val |= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR5=reg_val;
			break;

		case 6:
			reg_val=SCU->MCCR6;
			if (type==FRT0_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK1 | SCU_MCCR_CSEL_MSK1);
				
				reg_val |= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			if (type==FRT1_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK2 | SCU_MCCR_CSEL_MSK2);
				
				reg_val |= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR6=reg_val;		
			break;	

		case 7:
			reg_val=SCU->MCCR7;
			if (type==CAN_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK1 | SCU_MCCR_CSEL_MSK1);
				
				reg_val |= (((clksrc&7)<<8)|((clkdiv&0xFF)<<0));
			}
			if (type==UART_TYPE){
				reg_val &= ~(SCU_MCCR_CDIV_MSK2 | SCU_MCCR_CSEL_MSK2);
				
				reg_val |= (((clksrc&7)<<24)|((clkdiv&0xFF)<<16));
			}
			SCU->MCCR7=reg_val;	
			break;
	}
	
	SYST_ACCESS_DIS();
}


//-------------------------
#define USED_CLKO
//-------------------------

//--------------------------------------
//#define USED_LSI		
//#define USED_HSI		
#define USED_HSE		
//#define USED_LSE		
//#define USED_PLL120		
//--------------------------------------
/**************************************************************************
 * @brief			Initialize default clock for A34M41x Board
 * @param[in]		None
 * @return			None
 **************************************************************************/
void HAL_SCU_ClockInit(void)
{
	// Flash Wait Config
	HAL_CFMC_WaitCmd(7);
	
	// CLKO Function Setting, Check PORT Setting (PC9)
#ifdef USED_CLKO
	HAL_SCU_CORCmd(SCU_COR_CLKOINSEL_MCLK, 0x5, ENABLE);			// CLKO=PCLK, Divider=5
#else
	HAL_SCU_CORCmd(SCU_COR_CLKOINSEL_MCLK, 0x5, DISABLE);			
#endif
	
	//!! For Clock Change, Enable All Clock Source !!
	HAL_SCU_ClockSRCCmd((SCU_CSCR_LSE_ON|SCU_CSCR_HSE_ON|SCU_CSCR_HSI_ON|SCU_CSCR_LSI_ON), ENABLE);
	
	// Main Clock Selection
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_LSI);
	
	// MCLK Monitoring Disable
	HAL_SCU_CLKMNTCmd(SCU_CMR_MCLKMNT, DISABLE);
	
#ifdef USED_LSI		//500khz
	
	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_LSI);

	HAL_SystemCoreClock = 500000;
	HAL_SystemPeriClock = 500000;

	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_LSI, 2);
#endif

#ifdef USED_HSI		//32Mhz

	SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_HSI);
	
	SystemCoreClock = 32000000;
	SystemPeriClock = 32000000;	

	SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_HSI, 2);
#endif

#ifdef USED_HSE		//8MHz

	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_HSE);
	
	SystemCoreClock = 8000000;
	SystemPeriClock = 8000000;
	
	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_HSE, 2);
#endif

#ifdef USED_LSE		//32.768khz

	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_LSE);
	
	SystemCoreClock = 32768;
	SystemPeriClock = 32768;
	
	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_LSE, 2);
#endif

#ifdef USED_PLL120		// 120MHz, Source HSE

	HAL_CFMC_WaitCmd(7);
	
	HAL_SCU_HCLKDIVCmd(0);			// HCLK=MCLK
	HAL_SCU_PCLKDIVCmd(0);			// PCLK=HCLK
	
	HAL_SCU_PLLINCLKCmd(SCU_SCCR_PLLCLKSEL_HSE);	// PLL Input Clock = HSE
	
	HAL_SCU_PLLPREDIVCmd(0);		// PLLINCLK = 8MHz
	
	HAL_SCU_PLLCmd(SCU_PLLCON_PLLMODE_FOUT, 3, 59, 0, 0, ENABLE);

	// Lock Status Check
	while(1)
	{
		if ((HAL_SCU_GetPLLStatus()&(SCU_PLLCON_LOCKSTS))==SCU_PLLCON_LOCKSTS)
		{
			break;
		}
	}
	
	HAL_SCU_MainCLKCmd(SCU_SCCR_MCLKSEL_PLL);
	
	SystemCoreClock = 120000000;
	SystemPeriClock = 120000000;
	
	HAL_SCU_SetMCCRx(7, UART_TYPE, SCU_MCCR_CSEL_PLL, 2);
#endif

	
}


/* --------------------------------- End Of File ------------------------------ */
