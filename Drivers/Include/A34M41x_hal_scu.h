/**********************************************************************
* @file		A34M41x_scu.h
* @brief	Contains all macro definitions and function prototypes
* 			support for SCU firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_SCU_H_
#define _A34M41x_SCU_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

// SMR Define
#define SCU_SMR_PREVMODE_Pos                  4                                                       /*!< SCU SMR: PREVMODE Position              */
#define SCU_SMR_PREVMODE_Msk                  (0x03UL << SCU_SMR_PREVMODE_Pos)                        /*!< SCU SMR: PREVMODE Mask                  */
#define SCU_SMR_VDCAON_Pos                    8                                                       /*!< SCU SMR: VDCAON Position                */
#define SCU_SMR_VDCAON_Msk                    (0x01UL << SCU_SMR_VDCAON_Pos)                          /*!< SCU SMR: VDCAON Mask                    */
#define SCU_SMR_LSIAON_Pos                   9                                                     /*!< SCU SMR: LSIAON Position               */
#define SCU_SMR_LSIAON_Msk                   (0x01UL << SCU_SMR_LSIAON_Pos)                         /*!< SCU SMR: LSIAON Mask                   */

#define SCU_SMR_LSEAON		(1UL<<13)
#define SCU_SMR_LSEAOFF		(0UL<<13)
#define SCU_SMR_HSEAON		(1UL<<12)
#define SCU_SMR_HSEAOFF	(0UL<<12)
#define SCU_SMR_PLLAON		(1UL<<11)
#define SCU_SMR_PLLAOFF		(0UL<<11)
#define SCU_SMR_HSIAON		(1UL<<10)
#define SCU_SMR_HSIAOFF		(0UL<<10)
#define SCU_SMR_LSIAON		(1UL<<9)
#define SCU_SMR_LSIAOFF		(0UL<<9)
#define SCU_SMR_VDCAON		(1UL<<8)
#define SCU_SMR_VDCAOFF		(0UL<<8)	

// SYSTEM Enable
#define SYST_ACCESS_EN()  						do { SCU->SYSTEN=0x57; SCU->SYSTEN=0x75; } while(0)
#define SYST_ACCESS_DIS()  						do { SCU->SYSTEN=0x00; } while(0) 

// RSER Define
#define RST_LOCKUPRST     (1UL<<9)
#define RST_PORST     (1UL<<8)
#define RST_PINRST     (1UL<<7)
#define RST_CPURST     (1UL<<6)
#define RST_SWRST     (1UL<<5)
#define RST_WDTRST     (1UL<<4)
#define RST_MCLKFRST        (1UL<<3)
#define RST_LSEFRST        (1UL<<2)
#define RST_HSEFRST        (1UL<<1)
#define RST_LVDRST        (1UL<<0)

// WUER Define
#define GPIOGWUE	 (1UL<<14)
#define GPIOFWUE	 (1UL<<13)
#define GPIOEWUE	 (1UL<<12)
#define GPIODWUE     (1UL<<11)
#define GPIOCWUE     (1UL<<10)
#define GPIOBWUE     (1UL<<9)
#define GPIOAWUE     (1UL<<8)
#define FRT1WUE        (1UL<<3)
#define FRT0WUE        (1UL<<2)
#define WDTWUE        (1UL<<1)
#define LVIWUE        (1UL<<0)
	
// PRER1 Define
#define PERI_QEI1		(1UL<<29)
#define PERI_QEI0	 	(1UL<<28)
#define PERI_TIMER9		(1UL<<25)
#define PERI_TIMER8	 	(1UL<<24)
#define PERI_TIMER7		(1UL<<23)
#define PERI_TIMER6	 	(1UL<<22)
#define PERI_TIMER5		(1UL<<21)
#define PERI_TIMER4	 	(1UL<<20)
#define PERI_TIMER3		(1UL<<19)
#define PERI_TIMER2	 	(1UL<<18)
#define PERI_TIMER1		(1UL<<17)
#define PERI_TIMER0	 	(1UL<<16)
#define PERI_GPIOG		(1UL<<14)
#define PERI_GPIOF	 	(1UL<<13)
#define PERI_GPIOE		(1UL<<12)
#define PERI_GPIOD	 	(1UL<<11)
#define PERI_GPIOC		(1UL<<10)
#define PERI_GPIOB	 	(1UL<<9)
#define PERI_GPIOA		(1UL<<8)
#define PERI_FRT1	 	(1UL<<7)
#define PERI_FRT0	 	(1UL<<6)
#define PERI_DMA	 	(1UL<<4)
#define PERI_DFMC	 	(1UL<<3)
#define PERI_WDT	 	(1UL<<2)
#define PERI_CFMC	 	(1UL<<1)
#define PERI_SCU	 	(1UL<<0)

// PRER2 Define
#define PERI_RNG		(1UL<<31)			// 20191210 오타 수정
#define PERI_AES	 	(1UL<<30)
#define PERI_CRC		(1UL<<29)
#define PERI_COMPARATOR	 	(1UL<<28)
#define PERI_CAN		(1UL<<26)
#define PERI_PGA	 	(1UL<<24)
#define PERI_ADC2		(1UL<<22)
#define PERI_ADC1	 	(1UL<<21)
#define PERI_ADC0	 	(1UL<<20)
#define PERI_MPWM1		(1UL<<17)
#define PERI_MPWM0	 	(1UL<<16)
#define PERI_UART5		(1UL<<13)
#define PERI_UART4	 	(1UL<<12)
#define PERI_UART3		(1UL<<11)
#define PERI_UART2 	(1UL<<10)
#define PERI_UART1		(1UL<<9)
#define PERI_UART0	 	(1UL<<8)
#define PERI_I2C1		(1UL<<5)
#define PERI_I2C0	 	(1UL<<4)
#define PERI_SPI2		(1UL<<2)
#define PERI_SPI1	 	(1UL<<1)
#define PERI_SPI0	 	(1UL<<0)

// CSCR Define
#define SCU_CSCR_LSE_ON							(0x01<<7)
#define SCU_CSCR_LSI_ON							(0x01<<5)
#define SCU_CSCR_HSI_ON							(0x01<<3)
#define SCU_CSCR_HSE_ON							(0x01<<1)


// SCCR Define
#define SCU_SCCR_PCLKDIV_POS						(16)
#define SCU_SCCR_PLLCLKSEL_POS						(12)
#define SCU_SCCR_PLLPREDIV_POS						(8)
#define SCU_SCCR_HCLKDIV_POS						(24)
#define SCU_SCCR_MCLKSEL_POS						(0)

#define SCU_SCCR_PCLKDIV_MSK						(0x07 << SCU_SCCR_PCLKDIV_POS)
#define SCU_SCCR_PLLCLKSEL_MSK						(0x01 << SCU_SCCR_PLLCLKSEL_POS)
#define SCU_SCCR_PLLPREDIV_MSK						(0x03 << SCU_SCCR_PLLPREDIV_POS)
#define SCU_SCCR_HCLKDIV_MSK						(0x0F << SCU_SCCR_HCLKDIV_POS)
#define SCU_SCCR_MCLKSEL_MSK						(0x07 << SCU_SCCR_MCLKSEL_POS)

#define HCLKDIV_MCLK_1	0
#define HCLKDIV_MCLK_2	1
#define HCLKDIV_MCLK_4	2
#define HCLKDIV_MCLK_8	3
#define HCLKDIV_MCLK_16	4
#define HCLKDIV_MCLK_32	5
#define HCLKDIV_MCLK_64	6
#define HCLKDIV_MCLK_128	7
#define HCLKDIV_MCLK_256	8
#define HCLKDIV_MCLK_512	9

#define PCLKDIV_HCLK_1		0
#define PCLKDIV_HCLK_2		1
#define PCLKDIV_HCLK_4		2
#define PCLKDIV_HCLK_8		3
#define PCLKDIV_HCLK_16	4

#define PLLPREDIV_PLLINCLK_1		0
#define PLLPREDIV_PLLINCLK_2		1
#define PLLPREDIV_PLLINCLK_4		2
#define PLLPREDIV_PLLINCLK_8		3

#define SCU_SCCR_PLLCLKSEL_HSI						(0x00<<12)
#define SCU_SCCR_PLLCLKSEL_HSE						(0x01<<12)

#define SCU_SCCR_MCLKSEL_LSI						(0x00<<0)
#define SCU_SCCR_MCLKSEL_HSI						(0x02<<0)
#define SCU_SCCR_MCLKSEL_PLL						(0x07<<0)
#define SCU_SCCR_MCLKSEL_HSE						(0x06<<0)
#define SCU_SCCR_MCLKSEL_LSE						(0x01<<0)


// CMR Define
#define SCU_CMR_MCLKREC								(0x01<<15)

#define SCU_CMR_LSEMNT								(0x01<<11)
#define SCU_CMR_LSEIE								(0x01<<10)
#define SCU_CMR_LSEFAIL							(0x01<<9)

#define SCU_CMR_MCLKMNT								(0x01<<7)
#define SCU_CMR_MCLKIE								(0x01<<6)
#define SCU_CMR_MCLKFAIL							(0x01<<5)
#define SCU_CMR_MCLKSTS								(0x01uL<<4)

#define SCU_CMR_HSEMNT								(0x01<<3)
#define SCU_CMR_HSEIE								(0x01<<2)
#define SCU_CMR_HSEFAIL							(0x01<<1)


// NMICR Define
#define SCU_NMICR_NMIINEN							(0x01<<15)
#define SCU_NMICR_PROT1EN							(0x01<<6)
#define SCU_NMICR_OVP1EN							(0x01<<5)
#define SCU_NMICR_PROT0EN							(0x01<<4)
#define SCU_NMICR_OVP0EN							(0x01<<3)
#define SCU_NMICR_WDTINTEN							(0x01<<2)
#define SCU_NMICR_MCLKFAILEN						(0x01<<1)
#define SCU_NMICR_LVDEN								(0x01<<0)

// NMISR Define
#define SCU_NMISR_WTIDKY							(0x8CUL<<24)
#define SCU_NMISR_WTIDKY_NOR						(0x73UL<<24)

#define SCU_NMISR_NMIINTSTS							(0x01<<15)
#define SCU_NMISR_PROT1								(0x01<<6)
#define SCU_NMISR_OVP1								(0x01<<5)
#define SCU_NMISR_PROT0								(0x01<<4)
#define SCU_NMISR_OVP0								(0x01<<3)
#define SCU_NMISR_WDTINT							(0x01<<2)
#define SCU_NMISR_MCLKFAIL							(0x01<<1)
#define SCU_NMISR_LVDSTS							(0x01<<0)


// COR Define
#define SCU_COR_CLKOEN_ENABLE						(0x01<<4)
#define SCU_COR_CLKOEN_DISABLE						(0x00<<4)

#define SCU_COR_CLKOINSEL_LSI						(0x00<<5)
#define SCU_COR_CLKOINSEL_LSE						(0x01<<5)
#define SCU_COR_CLKOINSEL_MCLK						(0x04<<5)
#define SCU_COR_CLKOINSEL_HSI						(0x05<<5)
#define SCU_COR_CLKOINSEL_HSE						(0x06<<5)
#define SCU_COR_CLKOINSEL_PLL						(0x07<<5)


// PLLCON Define
#define SCU_PLLCON_LOCKSTS							(0x01uL<<31)

#define SCU_PLLCON_PLLRSTB							(0x01<<23)
#define SCU_PLLCON_PLLEN							(0x01<<22)
#define SCU_PLLCON_BYPASSB							(0x01<<21)

#define SCU_PLLCON_PLLMODE_FOUT						(0x00<<20)
#define SCU_PLLCON_PLLMODE_2xFOUT					(0x01UL<<20)




// MCCR
enum {
	SYSTICK_TYPE=0,
	WDT_TYPE,
	MPWM0_TYPE,
	MPWM1_TYPE,
	TIMER04_TYPE,
	TIMER59_TYPE,
	ADC_TYPE,
	PGA_TYPE,
	PGB_TYPE,
	PGC_TYPE,
	FRT0_TYPE,
	FRT1_TYPE,
	CAN_TYPE,
	UART_TYPE
};

#define SCU_MCCR_CSEL_MSK1						(0x07<<8)
#define SCU_MCCR_CSEL_MSK2						(0x07<<24)

#define SCU_MCCR_CDIV_MSK1						(0xFF<<0)
#define SCU_MCCR_CDIV_MSK2						(0xFF<<16)
	
#define SCU_MCCR_CSEL_LSI						(0x00)
#define SCU_MCCR_CSEL_LSE						(0x01)
#define SCU_MCCR_CSEL_MCLK						(0x04)
#define SCU_MCCR_CSEL_HSI						(0x05)
#define SCU_MCCR_CSEL_HSE						(0x06)
#define SCU_MCCR_CSEL_PLL						(0x07)



	
/* Private macros ------------------------------------------------------------- */

/* Public Functions ----------------------------------------------------------- */
void HAL_SCU_SMRCmd(uint16_t operating_clk, FunctionalState NewState);
uint16_t HAL_SCU_GetPREVStatus (void);
void HAL_SCU_WakeUpSRCCmd(uint16_t WakeUpSrc, FunctionalState NewState);
uint16_t HAL_SCU_GetWakeUpSRCStatus(void);
void HAL_SCU_ResetSRCCmd(uint16_t ResetSRC, FunctionalState NewState);
uint16_t HAL_SCU_GetResetSRCStatus(void);
void HAL_SCU_PeriSRCCmd_1(uint32_t PeriSRC, FunctionalState NewState);
void HAL_SCU_PeriSRCCmd_2(uint32_t PeriSRC, FunctionalState NewState);
void HAL_SCU_PeriCLKCmd_1(uint32_t PeriSRC, FunctionalState NewState);
void HAL_SCU_PeriCLKCmd_2(uint32_t PeriSRC, FunctionalState NewState);
void HAL_SCU_ClockSRCCmd (uint8_t ClockSRC, FunctionalState NewState);
void HAL_SCU_PCLKDIVCmd (uint8_t PCLKDIV);
void HAL_SCU_PLLINCLKCmd (uint32_t PLLINSRC);
void HAL_SCU_PLLPREDIVCmd (uint8_t PLLPREDIV);
void HAL_SCU_HCLKDIVCmd (uint8_t HCLKDIV);
void HAL_SCU_MainCLKCmd (uint8_t MCLKSRC);
void HAL_SCU_CLKMNTCmd (uint16_t MonioringSRC, FunctionalState NewState);
void HAL_SCU_ClearCLKMNTFLG (uint16_t ClearMNTSRC);
uint16_t HAL_SCU_GetCLKMNTStatus(void);
void HAL_SCU_NMICmd (uint8_t NMISRC, uint16_t NMIINTB, FunctionalState NewState);
uint32_t HAL_SCU_GetNMIStatus(void);
void HAL_SCU_ClearNMIFLG (uint16_t ClearNMISRC);
void HAL_SCU_CORCmd (uint8_t CLKOSRC, uint8_t CLKODIV, FunctionalState NewState);
void HAL_SCU_PLLCmd (uint32_t PLLMODE, uint8_t PREDIV, uint8_t POSTDIV1, uint8_t POSTDIV2, uint8_t OUTDIV, FunctionalState NewState);
void HAL_SCU_SetMCCRx(uint8_t mccrnum, uint8_t type, uint8_t clksrc, uint8_t clkdiv);
uint32_t HAL_SCU_GetPLLStatus(void);
uint32_t HAL_SCU_GetCMRStatus(void);
void HAL_SCU_ClearResetFlag(uint16_t ResetEvent);
void HAL_SCU_ClockInit(void);



#ifdef __cplusplus
}
#endif

#endif /* end _A34M41x_SCU_H_ */

/* --------------------------------- End Of File ------------------------------ */
