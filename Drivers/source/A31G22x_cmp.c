/***************************************************************************//**
* @file     A31G22x_cmp.c
* @brief    Contains all functions support for comparator dirver on A31G22x
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
#include "A31G22x_cmp.h"


/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/
#define SCU_PER2_CMP_ENABLE_PERI			(0x01UL << SCU_PER2_CMP_Pos)
#define SCU_PCER2_CMP_ENABLE_CLOCK			(0x01UL << SCU_PCER2_CMP_Pos)

#define CMP_CMP0CR_C0EN_ENABLE_CMP0			(0x01UL << CMP_CMP0CR_C0EN_Pos)
#define CMP_CMP1CR_C1EN_ENABLE_CMP1			(0x01UL << CMP_CMP1CR_C1EN_Pos)

#define CMP_IST_C1IRQ_INTERRUPT_STATUS		(0x01UL << CMP_IST_C1IRQ_Pos)
#define CMP_IST_C0IRQ_INTERRUPT_STATUS		(0x01UL << CMP_IST_C0IRQ_Pos)

#define CMP_ICLR_C1ICLR_CLEAR_INTERRUPT		(0x01UL << CMP_ICLR_C1ICLR_Pos)
#define CMP_ICLR_C0ICLR_CLEAR_INTERRUPT		(0x01UL << CMP_ICLR_C0ICLR_Pos)


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
* @brief      Initialize comparator peripheral
* @param      None
* @return     None
*******************************************************************************/
void CMP_Init(void)
{
	// Disable Comparator peripheral & clock
	SCU->PER2 &= ~SCU_PER2_CMP_Msk;
	SCU->PCER2 &= ~SCU_PCER2_CMP_Msk;

	// Enable Comparator peripheral & clock
	SCU->PER2 |= SCU_PER2_CMP_ENABLE_PERI;
	SCU->PCER2 |= SCU_PCER2_CMP_ENABLE_CLOCK;

	// Disable comparator and comparator interrupt
	CMP->CMP0CR &= ~(CMP_CMP0CR_C0EN_Msk);
	CMP->CMP1CR &= ~(CMP_CMP1CR_C1EN_Msk);
	CMP->IEN &= ~(CMP_IEN_C1IEN_Msk | CMP_IEN_C0IEN_Msk);

	// Clear comparator interrupt
	CMP_ClearInterruptStatus(CMP_0);
	CMP_ClearInterruptStatus(CMP_1);
}

/***************************************************************************//**
* @brief      De-Initialize comparator peripheral
* @param      None
* @return     None
*******************************************************************************/
void CMP_DeInit(void)
{
	CMP_Stop(CMP_0);
	CMP_Stop(CMP_1);

	// Disable Comparator peripheral & clock
	SCU->PER2 &= ~SCU_PER2_CMP_ENABLE_PERI;
	SCU->PCER2 &= ~SCU_PCER2_CMP_ENABLE_CLOCK;
}

/***************************************************************************//**
* @brief      Get comparator n interrupt status
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @return     SET : Occurred, RESET : Not occurred
*******************************************************************************/
FlagStatus CMP_GetInterruptStatus(CMP_NUMBER_Type Number)
{
	uint32_t TestBit;

	if (Number == CMP_0) {
		TestBit = CMP_IST_C0IRQ_INTERRUPT_STATUS;
	} else {
		TestBit = CMP_IST_C1IRQ_INTERRUPT_STATUS;
	}

	return (CMP->IST & TestBit) ? SET : RESET;
}

/***************************************************************************//**
* @brief      Clear comparator n interrupt status
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @return     None
*******************************************************************************/
void CMP_ClearInterruptStatus(CMP_NUMBER_Type Number)
{
	uint32_t ClearBit;

	if (Number == CMP_0) {
		ClearBit = CMP_ICLR_C0ICLR_CLEAR_INTERRUPT;
	} else {
		ClearBit = CMP_ICLR_C1ICLR_CLEAR_INTERRUPT;
	}

	CMP->ICLR |= ClearBit;
}

/***************************************************************************//**
* @brief      Configure comparator hysterisis
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @param      HysterisisLevel : Comparator hysterisis level selected, should be
*              - CMP_HYSSEL_5_MILLIVOLTS : 5mV,
*              - CMP_HYSSEL_20_MILLIVOLTS : 20mV
* @param      HysterisisEnable : Hs
*              - ENABLE : Enable hysterisis
*              - DISABLE : Disable hysterisis
* @return     None
*******************************************************************************/
void CMP_ConfigureHysterisis(CMP_NUMBER_Type Number, CMP_HYSSEL_Type HysterisisLevel, FunctionalState HysterisisEnable)
{
	volatile uint32_t Reg32;

	if (Number == CMP_0) {
		Reg32 = CMP->CMP0CR;
		Reg32 &= ~(CMP_CMP0CR_C0HYSEN_Msk | CMP_CMP0CR_C0HYSSEL_Msk);
		Reg32 |= (HysterisisEnable << CMP_CMP0CR_C0HYSEN_Pos | (HysterisisLevel << CMP_CMP0CR_C0HYSSEL_Pos));
		CMP->CMP0CR = Reg32;
	} else {
		Reg32 = CMP->CMP1CR;
		Reg32 &= ~(CMP_CMP1CR_C1HYSEN_Msk | CMP_CMP1CR_C1HYSSEL_Msk);
		Reg32 |= (HysterisisEnable << CMP_CMP1CR_C1HYSEN_Pos | (HysterisisLevel << CMP_CMP1CR_C1HYSSEL_Pos));
		CMP->CMP1CR = Reg32;
	}
}

/***************************************************************************//**
* @brief      Configure comparator source/reference input
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @param      Source : Source (inptut +) selected, should be
*              | enum         | CMP_0    | CMP_1     |
*              |:------------:|:--------:|:---------:|
*              | CMP_SOURCE_0 | CP0(PA2) | CP1A(PA5) |
*              | CMP_SOURCE_1 | GND      | CP1B(PA4) |
*              | CMP_SOURCE_2 | GND      | CP1C(PA3) |
* @param      Reference : Reference (inptut -) selected, should be
*              | enum            | CMP_0      | CMP_1      |
*              |:---------------:|:----------:|:----------:|
*              | CMP_REFERENCE_0 | CREF0(PA7) | CREF1(PA6) |
*              | CMP_REFERENCE_1 | BGR                    ||
*              | CMP_REFERENCE_2 | GND                    ||
* @return     None
*******************************************************************************/
void CMP_ConfigureInput(CMP_NUMBER_Type Number, CMP_SOURCE_Type Source, CMP_REFERENCE_Type Reference)
{
	volatile uint32_t Reg32;

	if (Number == CMP_0) {
		Reg32 = CMP->CMP0CR;
		Reg32 &= ~(CMP_CMP0CR_C0INNSEL_Msk | CMP_CMP0CR_C0INPSEL_Msk);
		Reg32 |= (Reference << CMP_CMP0CR_C0INNSEL_Pos) | (Source << CMP_CMP0CR_C0INPSEL_Pos);
		CMP->CMP0CR = Reg32;
	} else {
		Reg32 = CMP->CMP1CR;
		Reg32 &= ~(CMP_CMP1CR_C1INNSEL_Msk | CMP_CMP1CR_C1INPSEL_Msk);
		Reg32 |= (Reference << CMP_CMP1CR_C1INNSEL_Pos) | (Source << CMP_CMP1CR_C1INPSEL_Pos);
		CMP->CMP1CR = Reg32;
	}
}

/***************************************************************************//**
* @brief      Configure comparator debounce clock
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @param      Counter : Shift clock counter of debounce logic selected, should be 
*              | Counter         | Clock                        |
*              |:---------------:|:----------------------------:|
*              | 0               | System Clock                 |
*              | 0x0001 ~ 0xFFFF | System Clock / (Counter * 2) |
* @return     None
*******************************************************************************/
void CMP_ConfigureDBNCCounter(uint32_t Counter)
{
	volatile uint32_t Reg32;

	Reg32 = CMP->DBNC;
	Reg32 &= ~CMP_DBNC_DBNCTB_Msk;
	Reg32 |= ((Counter << CMP_DBNC_DBNCTB_Pos) & CMP_DBNC_DBNCTB_Msk);
	CMP->DBNC = Reg32;
}

/***************************************************************************//**
* @brief      Configure comparator debounce shift selection
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @param      ShiftNumber, Shift number of debounce logic selected, should be in range from 0 to 0xF
* @return     None
*******************************************************************************/
void CMP_ConfigureDBNCNumber(CMP_NUMBER_Type Number, uint32_t ShiftNumber)
{
	volatile uint32_t Reg32;

	Reg32 = CMP->DBNC;
	if (Number == CMP_0) {
		Reg32 &= ~CMP_DBNC_C0DBNC_Msk;
		Reg32 |= ((ShiftNumber << CMP_DBNC_C0DBNC_Pos) & CMP_DBNC_C0DBNC_Msk);
	} else {
		Reg32 &= ~CMP_DBNC_C1DBNC_Msk;
		Reg32 |= ((ShiftNumber << CMP_DBNC_C1DBNC_Pos) & CMP_DBNC_C1DBNC_Msk);
	}
	CMP->DBNC = Reg32;
}

/***************************************************************************//**
* @brief      Configure comparator interrupt
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @param      TriggerPolarity : Trigger polarity selected, should be
*              - CMP_TRIGGER_POL_NORMAL : Output normal (comparator out high activates trigger)
*              - CMP_TRIGGER_POL_INVERTED : Output inverted (XOR)
* @param      Polarity : Interrupt polarity (level mode) selected, should be
*              - CMP_INTERRUPT_POL_HIGH : Interrupt at comparator out high
*              - CMP_INTERRUPT_POL_LOW : Interrupt at comparator out low
* @param      Interrupt : Interrupt mode selected, should be
*              - CMP_INTERRUPT_MODE_LEVEL : Level interrupt
*              - CMP_INTERRUPT_MODE_RISING_EDGE : Rising edge interrupt
*              - CMP_INTERRUPT_MODE_FALLING_EDGE : Falling edge interrupt
*              - CMP_INTERRUPT_MODE_BOTH_EDGE : Both edge interrupt
* @return     None
*******************************************************************************/
void CMP_ConfigureInterrupt(CMP_NUMBER_Type Number, CMP_TRIGGER_POL_Type TriggerPolarity, CMP_INTERRUPT_POL_Type Polarity, CMP_INTERRUPT_MODE_Type Mode)
{
	volatile uint32_t Reg32;

	Reg32 = CMP->ICON;
	if (Number == CMP_0) {
		Reg32 &= ~(CMP_ICON_C0TPOL_Msk | CMP_ICON_C0IPOL_Msk | CMP_ICON_C0IMODE_Msk);
		Reg32 |= ((TriggerPolarity << CMP_ICON_C0TPOL_Pos) | (Polarity << CMP_ICON_C0IPOL_Pos) | (Mode << CMP_ICON_C0IMODE_Pos));
	} else {
		Reg32 &= ~(CMP_ICON_C1TPOL_Msk | CMP_ICON_C1IPOL_Msk | CMP_ICON_C1IMODE_Msk);
		Reg32 |= ((TriggerPolarity << CMP_ICON_C1TPOL_Pos) | (Polarity << CMP_ICON_C1IPOL_Pos) | (Mode << CMP_ICON_C1IMODE_Pos));
	}
	CMP->ICON = Reg32;
}

/***************************************************************************//**
* @brief      Configure comparator interrupt
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @param      Interrupt :
*              - ENABLE : Enable interrupt
*              - DISABLE : Disable interrupt
* @return     None
*******************************************************************************/
void CMP_EnableInterrupt(CMP_NUMBER_Type Number, FunctionalState Interrupt)
{
	volatile uint32_t Reg32;

	if (Number == CMP_0) {
		Reg32 = CMP->IEN;
		Reg32 &= ~CMP_IEN_C0IEN_Msk;
		Reg32 |= (Interrupt << CMP_IEN_C0IEN_Pos);
		CMP->IEN = Reg32;
	} else {
		Reg32 = CMP->IEN;
		Reg32 &= ~CMP_IEN_C1IEN_Msk;
		Reg32 |= (Interrupt << CMP_IEN_C1IEN_Pos);
		CMP->IEN = Reg32;
	}
}

/***************************************************************************//**
* @brief      Start comparator
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @return     None
*******************************************************************************/
void CMP_Start(CMP_NUMBER_Type Number)
{
	volatile uint32_t Reg32;

	if (Number == CMP_0) {
		Reg32 = CMP->CMP0CR;
		Reg32 &= ~CMP_CMP0CR_C0EN_Msk;
		Reg32 |= CMP_CMP0CR_C0EN_ENABLE_CMP0;
		CMP->CMP0CR = Reg32;
	} else {
		Reg32 = CMP->CMP1CR;
		Reg32 &= ~CMP_CMP1CR_C1EN_Msk;
		Reg32 |= CMP_CMP1CR_C1EN_ENABLE_CMP1;
		CMP->CMP1CR = Reg32;
	}
}

/***************************************************************************//**
* @brief      Stop comparator
* @param      Number : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @return     None
*******************************************************************************/
void CMP_Stop(CMP_NUMBER_Type Number)
{
	volatile uint32_t Reg32;

	if (Number == CMP_0) {
		CMP->CMP0CR &= ~CMP_CMP0CR_C0EN_Msk;
	} else {
		CMP->CMP1CR &= ~CMP_CMP1CR_C1EN_Msk;
	}
}

/***************************************************************************//**
* @brief      Comparator Output Monitoring
* @param      OutMonMode : Comparator block number selected, should be
*              - CMP_0 : Comarator 0 block
*              - CMP_1 : Comarator 1 block
* @return     None
*******************************************************************************/
void CMP_SetOutputMonitoring(FunctionalState OutMonMode)
{
	volatile uint32_t Reg32;
	
	Reg32 = CMP->IEN;
	Reg32 &= ~(CMP_IEN_COUTMON_Msk);
	Reg32 |= (OutMonMode << CMP_IEN_COUTMON_Pos);
	
	CMP->IEN = Reg32;
}

/***************************************************************************//**
* @brief      Get Comparator Output Monitoring Edge Status
* @param      None
* @return     Comparator current output and edge polarity status
*******************************************************************************/
uint32_t CMP_GetMonitoringEdgeStatus(void)
{
	return (CMP->IST & (0x000F3000));
}

/***************************************************************************//**
* @brief      Comparator Output Monitoring Edge Status Clear
* @param      None
* @return     None
*******************************************************************************/
void CMP_ClearMonitoringEdgeStatus(void)
{	
	CMP->ICLR |= (CMP0_OUTPUT_RISING_EDGE | CMP0_OUTPUT_FALLING_EDGE | CMP1_OUTPUT_RISING_EDGE | CMP1_OUTPUT_FALLING_EDGE);	
}


/***************************************************************************//**
* @brief      Comparator Output Monitoring Edge Status Clear
* @param      None
* @return     FlagStatus
*             - SET : Enabled comparator Output Monitoring Function, 
*             - RESET : Not occurred
*******************************************************************************/
FlagStatus CMP_GetMonitoringStatus(void)
{
	return (CMP->IEN & CMP_OUTPUT_MONITORING) ? SET : RESET;
}

/*******************************************************************************
* Private Function
*******************************************************************************/

/* --------------------------------- End Of File ------------------------------ */
