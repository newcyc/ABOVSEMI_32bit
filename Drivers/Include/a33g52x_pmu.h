/**********************************************************************
* @file		a33g52x_pmu.h
* @brief	Contains all macro definitions and function prototypes
* 			support for SCU firmware library on AC34Mx256
* @version	1.0
* @date		
* @author	ABOV M team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A33G52X_PMU_H_
#define _A33G52X_PMU_H_

/* Includes ------------------------------------------------------------------- */
#include "A33G52x.h"

#ifdef __cplusplus
extern "C"
{
#endif


	
//===================================================================
// PMUIDR 
//
//				@ addr = 0x4000_0000
//
//===================================================================




//===================================================================
// PMUMR
//
//				@ addr = 0x4000_0004
//
//===================================================================
#define PMUMR_PVDCLP					(0x0001UL<<6)

#define PMUMR_PREVMODE_RUN				(0x0000UL<<4)
#define PMUMR_PREVMODE_SLEEP			(0x0001UL<<4)
#define PMUMR_PREVMODE_PWDN			(0x0002UL<<4)
#define PMUMR_PREVMODE_INIT				(0x0003UL<<4)
#define PMUMR_PREVMODE_MASK			(0x0003UL<<4)

#define PMUMR_ECLKMD							(0x0001UL<<3)
#define PMUMR_ECLKMD_AUTOOFF			(0x0000UL<<3)
#define PMUMR_ECLKMD_MANOFF 			(0x0001UL<<3)

#define PMUMR_VDCLP							(0x0001UL<<2)
#define PMUMR_VDCLP_EN						(0x0001UL<<2)
#define PMUMR_VDCLP_DIS					(0x0000UL<<2)

#define PMUMR_PMUMODE_RUN				(0x0000UL<<0)
#define PMUMR_PMUMODE_SLEEP			(0x0001UL<<0)
#define PMUMR_PMUMODE_PWDN			(0x0002UL<<0)
#define PMUMR_PMUMODE_INIT				(0x0003UL<<0)
#define PMUMR_PMUMODE_MASK			(0x0003UL<<0)


//===================================================================
// PMUCFGR 
//
//				@ addr = 0x4000_0008
//
//===================================================================
#define PMUCFGR_STBYOP							(0x0001UL<<4)
#define PMUCFGR_STBYOP_LOW_ACTIVE		(0x0000UL<<4)
#define PMUCFGR_STBYOP_HIGH_ACTIVE 	(0x0001UL<<4)

#define PMUCFGR_SOFTRST						(0x0001UL<<0)



//===================================================================
// PMUWSER 
//
//				@ addr = 0x4000_0010
//
//===================================================================
#define PMUWSER_GPIOFE					(0x0001UL<<10)
#define PMUWSER_GPIOEE					(0x0001UL<<9)
#define PMUWSER_GPIODE				(0x0001UL<<8)
#define PMUWSER_GPIOCE					(0x0001UL<<7)
#define PMUWSER_GPIOBE					(0x0001UL<<6)
#define PMUWSER_GPIOAE				(0x0001UL<<5)
#define PMUWSER_FRTE					(0x0001UL<<4)
#define PMUWSER_WDTE					(0x0001UL<<3)
#define PMUWSER_SXFAILE				(0x0001UL<<2)
#define PMUWSER_MXFAILE				(0x0001UL<<1)
#define PMUWSER_LVDE					(0x0001UL<<0)




//===================================================================
// PMUWSSR
//
//				@ addr = 0x4000_0014
//
//===================================================================
#define PMUWSSR_GPIOF					(0x0001UL<<10)
#define PMUWSSR_GPIOE					(0x0001UL<<9)
#define PMUWSSR_GPIOD					(0x0001UL<<8)
#define PMUWSSR_GPIOC					(0x0001UL<<7)
#define PMUWSSR_GPIOB					(0x0001UL<<6)
#define PMUWSSR_GPIOA					(0x0001UL<<5)
#define PMUWSSR_FRT					(0x0001UL<<4)
#define PMUWSSR_WDT					(0x0001UL<<3)
#define PMUWSSR_SXFAIL					(0x0001UL<<2)
#define PMUWSSR_MXFAIL					(0x0001UL<<1)
#define PMUWSSR_LVD					(0x0001UL<<0)




//===================================================================
// PMURSER
//
//				@ addr = 0x4000_0018
//
//===================================================================
#define PMURSER_MCKFAILRSTE			(0x0001UL<<7)
#define PMURSER_RSTRSTE				(0x0001UL<<6)
#define PMURSER_SYSRSTE				(0x0001UL<<5)
#define PMURSER_SWRSTE					(0x0001UL<<4)
#define PMURSER_WDTRSTE				(0x0001UL<<3)
#define PMURSER_SXFAILRSTE				(0x0001UL<<2)
#define PMURSER_MXFAILRSTE				(0x0001UL<<1)
#define PMURSER_LVDRSTE				(0x0001UL<<0)




//===================================================================
// PMURSSR 
//
//				@ addr = 0x4000_001C
//
//===================================================================
#define PMURSSR_MCKFAILRST				(0x0001UL<<7)
#define PMURSSR_RSTRST					(0x0001UL<<6)
#define PMURSSR_SYSRST					(0x0001UL<<5)
#define PMURSSR_SWRST					(0x0001UL<<4)
#define PMURSSR_WDTRST				(0x0001UL<<3)
#define PMURSSR_SXFAILRST				(0x0001UL<<2)
#define PMURSSR_MXFAILRST				(0x0001UL<<1)
#define PMURSSR_LVDRST					(0x0001UL<<0)



//===================================================================
// PMUPERR 
//
//				@ addr = 0x4000_0020
//
//===================================================================
#define PMUPERR_JTAGEN					(0x0001UL<<31)
#define PMUPERR_PMC					(0x0001UL<<29)
#define PMUPERR_ADC					(0x0001UL<<28)
//------------------------------------------------------------------------
#define PMUPERR_PWM4_7				(0x0001UL<<25)
#define PMUPERR_PWM0_3				(0x0001UL<<24)
//------------------------------------------------------------------------
#define PMUPERR_UART3					(0x0001UL<<23)
#define PMUPERR_UART2					(0x0001UL<<22)
#define PMUPERR_UART1					(0x0001UL<<21)
#define PMUPERR_UART0					(0x0001UL<<20)
//------------------------------------------------------------------------
#define PMUPERR_I2C1					(0x0001UL<<19)
#define PMUPERR_I2C0					(0x0001UL<<18)
#define PMUPERR_SPI1					(0x0001UL<<17)
#define PMUPERR_SPI0					(0x0001UL<<16)
//------------------------------------------------------------------------
#define PMUPERR_CRC16					(0x0001UL<<14)
#define PMUPERR_GPIOF					(0x0001UL<<13)
#define PMUPERR_GPIOE					(0x0001UL<<12)
#define PMUPERR_GPIOD					(0x0001UL<<11)
#define PMUPERR_GPIOC					(0x0001UL<<10)
#define PMUPERR_GPIOB					(0x0001UL<<9)
#define PMUPERR_GPIOA					(0x0001UL<<8)
//------------------------------------------------------------------------
#define PMUPERR_TC6_9					(0x0001UL<<7)
#define PMUPERR_TC2_5					(0x0001UL<<6)
#define PMUPERR_TC0_1					(0x0001UL<<5)
#define PMUPERR_FRT						(0x0001UL<<4)
//------------------------------------------------------------------------
#define PMUPERR_WDT					(0x0001UL<<3)



#define PMUPERR_ALL_PERI				(PMUPERR_PMC|PMUPERR_ADC|PMUPERR_PWM4_7|PMUPERR_PWM0_3|\
										PMUPERR_UART3|PMUPERR_UART2|PMUPERR_UART1|PMUPERR_UART0|\
										PMUPERR_I2C1|PMUPERR_I2C0|PMUPERR_SPI1|PMUPERR_SPI0|\
										PMUPERR_CRC16|\
										PMUPERR_GPIOF|PMUPERR_GPIOE|PMUPERR_GPIOD|PMUPERR_GPIOC|PMUPERR_GPIOB|PMUPERR_GPIOA|\
										PMUPERR_TC6_9|PMUPERR_TC2_5|PMUPERR_TC0_1|PMUPERR_FRT|PMUPERR_WDT)


#define PMUPERR_ALL_PERI_WITH_JTAG	(PMUPERR_JTAGEN|PMUPERR_ALL_PERI)


//===================================================================
// PMUPER 
//
//				@ addr = 0x4000_0024
//
//===================================================================
#define PMUPER_JTAGEN				(0x0001UL<<31)
#define PMUPER_PMC						(0x0001UL<<29)
#define PMUPER_ADC						(0x0001UL<<28)
//------------------------------------------------------------------------
#define PMUPER_PWM4_7				(0x0001UL<<25)
#define PMUPER_PWM0_3				(0x0001UL<<24)
//------------------------------------------------------------------------
#define PMUPER_UART3					(0x0001UL<<23)
#define PMUPER_UART2					(0x0001UL<<22)
#define PMUPER_UART1					(0x0001UL<<21)
#define PMUPER_UART0					(0x0001UL<<20)
//------------------------------------------------------------------------
#define PMUPER_I2C1						(0x0001UL<<19)
#define PMUPER_I2C0						(0x0001UL<<18)
#define PMUPER_SPI1						(0x0001UL<<17)
#define PMUPER_SPI0						(0x0001UL<<16)
//------------------------------------------------------------------------
#define PMUPER_CRC16 					(0x0001UL<<14)
#define PMUPER_GPIOF					(0x0001UL<<13)
#define PMUPER_GPIOE					(0x0001UL<<12)
#define PMUPER_GPIOD					(0x0001UL<<11)
#define PMUPER_GPIOC					(0x0001UL<<10)
#define PMUPER_GPIOB					(0x0001UL<<9)
#define PMUPER_GPIOA					(0x0001UL<<8)
//------------------------------------------------------------------------
#define PMUPER_TC6_9					(0x0001UL<<7)
#define PMUPER_TC2_5					(0x0001UL<<6)
#define PMUPER_TC0_1					(0x0001UL<<5)
#define PMUPER_FRT						(0x0001UL<<4)
//------------------------------------------------------------------------
#define PMUPER_WDT						(0x0001UL<<3)



#define PMUPER_MODULE_OFF_FOR_GPIO_WKUP 	(PMUPER_ADC|PMUPER_PWM4_7|PMUPER_PWM0_3|\
										PMUPER_UART3|PMUPER_UART2|PMUPER_UART1|PMUPER_UART0|\
										PMUPER_I2C1|PMUPER_I2C0|PMUPER_SPI1|PMUPER_SPI0|\
										PMUPER_TC6_9|PMUPER_TC2_5|PMUPER_TC0_1|PMUPER_FRT|PMUPER_WDT)



#define PMUPER_MODULE_OFF_FOR_FRT_WKUP 	(PMUPER_ADC|PMUPER_PWM4_7|PMUPER_PWM0_3|\
										PMUPER_UART3|PMUPER_UART2|PMUPER_UART1|PMUPER_UART0|\
										PMUPER_I2C1|PMUPER_I2C0|PMUPER_SPI1|PMUPER_SPI0|\
										PMUPER_GPIOF|PMUPER_GPIOD|PMUPER_GPIOC|PMUPER_GPIOB|PMUPER_GPIOA|\
										PMUPER_TC6_9|PMUPER_TC2_5|PMUPER_TC0_1|PMUPER_WDT)


#define PMUPER_MODULE_OFF_FOR_WDT_WKUP 	(PMUPER_ADC|PMUPER_PWM4_7|PMUPER_PWM0_3|\
										PMUPER_UART3|PMUPER_UART2|PMUPER_UART1|PMUPER_UART0|\
										PMUPER_I2C1|PMUPER_I2C0|PMUPER_SPI1|PMUPER_SPI0|\
										PMUPER_GPIOF|PMUPER_GPIOD|PMUPER_GPIOC|PMUPER_GPIOB|PMUPER_GPIOA|\
										PMUPER_TC6_9|PMUPER_TC2_5|PMUPER_TC0_1|PMUPER_FRT)


//===================================================================
// PMUPCCR
//
//				@ addr = 0x4000_0028
//
//===================================================================
#define PMUPCCR_ADC					(0x0001UL<<28)
//------------------------------------------------------------------------
#define PMUPCCR_PWM4_7				(0x0001UL<<25)
#define PMUPCCR_PWM0_3				(0x0001UL<<24)
//------------------------------------------------------------------------
#define PMUPCCR_UART3					(0x0001UL<<23)
#define PMUPCCR_UART2					(0x0001UL<<22)
#define PMUPCCR_UART1					(0x0001UL<<21)
#define PMUPCCR_UART0					(0x0001UL<<20)
//------------------------------------------------------------------------
#define PMUPCCR_I2C1					(0x0001UL<<19)
#define PMUPCCR_I2C0					(0x0001UL<<18)
#define PMUPCCR_SPI1					(0x0001UL<<17)
#define PMUPCCR_SPI0					(0x0001UL<<16)
//------------------------------------------------------------------------
#define PMUPCCR_CRC16 					(0x0001UL<<14)
#define PMUPCCR_GPIO					(0x0001UL<<8)
//------------------------------------------------------------------------
#define PMUPCCR_TC6_9					(0x0001UL<<7)
#define PMUPCCR_TC2_5					(0x0001UL<<6)
#define PMUPCCR_TC0_1					(0x0001UL<<5)
#define PMUPCCR_FRT						(0x0001UL<<4)
//------------------------------------------------------------------------
#define PMUPCCR_WDT					(0x0001UL<<3)




#define PMUPCCR_MODULE_OFF_FOR_GPIO_WKUP 	(PMUPCCR_ADC|PMUPCCR_PWM4_7|PMUPCCR_PWM0_3|\
												PMUPCCR_UART3|PMUPCCR_UART2|PMUPCCR_UART1|PMUPCCR_UART0|\
												PMUPCCR_I2C1|PMUPCCR_I2C0|PMUPCCR_SPI1|PMUPCCR_SPI0|\
												PMUPCCR_TC6_9|PMUPCCR_TC2_5|PMUPCCR_TC0_1|PMUPCCR_FRT|PMUPCCR_WDT)



#define PMUPCCR_MODULE_OFF_FOR_FRT_WKUP 	(PMUPCCR_ADC|PMUPCCR_PWM4_7|PMUPCCR_PWM0_3|\
												PMUPCCR_UART3|PMUPCCR_UART2|PMUPCCR_UART1|PMUPCCR_UART0|\
												PMUPCCR_I2C1|PMUPCCR_I2C0|PMUPCCR_SPI1|PMUPCCR_SPI0|\
												PMUPCCR_GPIO|\
												PMUPCCR_TC6_9|PMUPCCR_TC2_5|PMUPCCR_TC0_1|PMUPCCR_WDT)


#define PMUPCCR_MODULE_OFF_FOR_WDT_WKUP 	(PMUPCCR_ADC|PMUPCCR_PWM4_7|PMUPCCR_PWM0_3|\
												PMUPCCR_UART3|PMUPCCR_UART2|PMUPCCR_UART1|PMUPCCR_UART0|\
												PMUPCCR_I2C1|PMUPCCR_I2C0|PMUPCCR_SPI1|PMUPCCR_SPI0|\
												PMUPCCR_GPIO|\
												PMUPCCR_TC6_9|PMUPCCR_TC2_5|PMUPCCR_TC0_1|PMUPCCR_FRT)


//===================================================================
// PMUCCR 
//
//				@ addr = 0x4000_0030
//				
//===================================================================
#define PMUCCR_ROSCEN_DIV_BY_1		(0x0002UL<<6)
#define PMUCCR_ROSCEN_DIV_BY_2		(0x0003UL<<6)
#define PMUCCR_ROSCEN_MASK			(0x0003UL<<6)

#define PMUCCR_IOSC16EN_STOP			(0x0000UL<<4)
#define PMUCCR_IOSC16EN_DIV_BY_1		(0x0002UL<<4)
#define PMUCCR_IOSC16EN_DIV_BY_2		(0x0003UL<<4)
#define PMUCCR_IOSC16EN_MASK 			(0x0003UL<<4)

#define PMUCCR_SXOSCEN_STOP			(0x0000UL<<2)
#define PMUCCR_SXOSCEN_DIV_BY_1		(0x0002UL<<2)
#define PMUCCR_SXOSCEN_DIV_BY_2		(0x0003UL<<2)
#define PMUCCR_SXOSCEN_MASK			(0x0003UL<<2)

#define PMUCCR_MXOSCEN_STOP			(0x0000UL<<0)
#define PMUCCR_MXOSCEN_DIV_BY_1		(0x0002UL<<0)
#define PMUCCR_MXOSCEN_DIV_BY_2		(0x0003UL<<0)
#define PMUCCR_MXOSCEN_MASK			(0x0003UL<<0)
//--------------------------------------------------------




//===================================================================
// PMUCMR
//
//				@ addr = 0x4000_0034
//
//===================================================================
#define PMUCMR_SXOSCIE					(0x0001UL<<15)
#define PMUCMR_MXOSCIE					(0x0001UL<<14)
#define PMUCMR_SXOSCMNT				(0x0001UL<<9)
#define PMUCMR_MXOSCMNT				(0x0001UL<<8)
#define PMUCMR_SXOSCIF					(0x0001UL<<7)
#define PMUCMR_MXOSCIF					(0x0001UL<<6)
#define PMUCMR_SXOSCSTS				(0x0001UL<<1)
#define PMUCMR_MXOSCSTS				(0x0001UL<<0)



//===================================================================
// PMUMCMR
//
//				@ addr = 0x4000_0038
//
//===================================================================
#define PMUMCMR_RECOVER 				(0x0001UL<<15)
#define PMUMCMR_MCKIE					(0x0001UL<<14)
#define PMUMCMR_MCKMNT				(0x0001UL<<8)
#define PMUMCMR_MCKIF 					(0x0001UL<<6)
#define PMUMCMR_MCKSTS 				(0x0001UL<<0)






//===================================================================
// PMUBCCR 
//
//				@ addr = 0x4000_003C
//
//===================================================================
#define PMUBCCR_PCLKDIV				(0x0001UL<<10)
#define PMUBCCR_PCLKDIV_DIV_BY_1		(0x0000UL<<10)
#define PMUBCCR_PCLKDIV_DIV_BY_2		(0x0001UL<<10)
#define PMUBCCR_PCLKDIV_MASK			(0x0001UL<<10)


#define PMUBCCR_HCLKDIV_DIV_BY_1		(0x0000UL<<8)
#define PMUBCCR_HCLKDIV_DIV_BY_2		(0x0002UL<<8)
#define PMUBCCR_HCLKDIV_DIV_BY_4		(0x0003UL<<8)
#define PMUBCCR_HCLKDIV_MASK			(0x0003UL<<8)

#define PMUBCCR_PLLCLKDIV					(0x0001UL<<5)
#define PMUBCCR_PLLCLKDIV_DIV_BY_1	(0x0000UL<<5)
#define PMUBCCR_PLLCLKDIV_DIV_BY_2	(0x0001UL<<5)

#define PMUBCCR_PLLCLKSEL					(0x0001UL<<4)
#define PMUBCCR_PLLCLKSEL_IOSC16M	(0x0000UL<<4)
#define PMUBCCR_PLLCLKSEL_XTAL			(0x0001UL<<4)


#define PMUBCCR_MCLKSEL_RING_OSC		(0x0000UL<<0)
#define PMUBCCR_MCLKSEL_SUB_OSC			(0x0001UL<<0)
#define PMUBCCR_MCLKSEL_PLL					(0x0002UL<<0)
#define PMUBCCR_MCLKSEL_PLL_BYPASS	(0x0003UL<<0)
#define PMUBCCR_MCLKSEL_MASK				(0x0003UL<<0)





//===================================================================
// PMUPCSR
//
//				@ addr = 0x4000_0040
//
//===================================================================
#define PMUPCSR_T69CS_MAIN_XTAL		(0x0000UL<<8)
#define PMUPCSR_T69CS_IOSC16M			(0x0001UL<<8)
#define PMUPCSR_T69CS_SUB_XTAL		(0x0002UL<<8)
#define PMUPCSR_T69CS_IOSC1M			(0x0003UL<<8)
#define PMUPCSR_T69CS_MASK			(0x0003UL<<8)

#define PMUPCSR_T25CS_MAIN_XTAL		(0x0000UL<<6)
#define PMUPCSR_T25CS_IOSC16M			(0x0001UL<<6)
#define PMUPCSR_T25CS_SUB_XTAL		(0x0002UL<<6)
#define PMUPCSR_T25CS_IOSC1M			(0x0003UL<<6)
#define PMUPCSR_T25CS_MASK			(0x0003UL<<6)

#define PMUPCSR_T01CS_MAIN_XTAL		(0x0000UL<<4)
#define PMUPCSR_T01CS_IOSC16M			(0x0001UL<<4)
#define PMUPCSR_T01CS_SUB_XTAL		(0x0002UL<<4)
#define PMUPCSR_T01CS_IOSC1M			(0x0003UL<<4)
#define PMUPCSR_T01CS_MASK			(0x0003<<4)

#define PMUPCSR_FRTCS_MAIN_XTAL		(0x0000UL<<2)
#define PMUPCSR_FRTCS_IOSC16M			(0x0001UL<<2)
#define PMUPCSR_FRTCS_SUB_XTAL		(0x0002UL<<2)
#define PMUPCSR_FRTCS_IOSC1M			(0x0003UL<<2)
#define PMUPCSR_FRTCS_MASK			(0x0003UL<<2)

#define PMUPCSR_WDTCS_MAIN_XTAL		(0x0000UL<<0)
#define PMUPCSR_WDTCS_IOSC16M		(0x0001UL<<0)
#define PMUPCSR_WDTCS_SUB_XTAL		(0x0002UL<<0)
#define PMUPCSR_WDTCS_RINGOSC		(0x0003UL<<0)
#define PMUPCSR_WDTCS_MASK			(0x0003UL<<0)



//===================================================================
// PMUCOR
//
//				@ addr = 0x4000_0044
//
//===================================================================
#define PMUCOR_TRACEKEY_B 				(0x000BUL<<12)
#define PMUCOR_TRACEKEY_A 				(0x000AUL<<12) 

#define PMUCOR_TRACEDIV_VAL(n)			(((n)&0x07UL)<<8)
#define PMUCOR_TRACEDIV_MASK			(0x0007UL<<8)

#define PMUCOR_TRACECLK_INV			(0x0001UL<<7)
#define PMUCOR_TRACECLK_DELAY			(0x0001UL<<6)

#define PMUCOR_CLKOSEL 					(0x0001UL<<5)
#define PMUCOR_CLKOSEL_PLL 			(0x0000UL<<5)
#define PMUCOR_CLKOSEL_MCLK 			(0x0001UL<<5) 


#define PMUCOR_CLKOEN 					(0x0001UL<<4)
#define PMUCOR_CLKOEN_CLKO_OFF		(0x0000UL<<4)
#define PMUCOR_CLKOEN_CLKO_ON			(0x0001UL<<4)

#define PMUCOR_CLKODIV_VAL(n)			(((n)&0x000FUL)<<0)
#define PMUCOR_CLKODIV_MASK			(0x000FUL<<0)


#define PMUCOR_MASK					(0x003FUL<<0)			// For both legacy mode and normal mode 





//===================================================================
// PLLCON
//
//				@ addr = 0x4000_0050 
//
//===================================================================
#define PLL_SOURCE_XTAL					
#define PLLCON_VCOMODE					(0x0001UL<<28)
#define PLLCON_VCOMODE_NORMAL	(0x0000UL<<28)
#define PLLCON_VCOMODE_DOUBLE	(0x0001UL<<28)

#define PLLCON_MULTI_VAL(n) 			(((n)&0x00FFUL)<<20)
#define PLLCON_MULTI_MASK 				(0x00FFUL<<20) 

#define PLLCON_DIV_VAL(n) 				(((n)&0x000FUL)<<16)
#define PLLCON_DIV_MASK 				(0x000FUL<<16) 


#define PLLCON_PLLnRESB					(0x0001UL<<15)
#define PLLCON_PLLEN						(0x0001UL<<14)
#define PLLCON_BYPASS					(0x0001UL<<13)
#define PLLCON_BYPASS_DISABLE		(0x0001UL<<13)
#define PLLCON_LOCKSTS					(0x0001UL<<12)

#define PLLCON_PREDIV_VAL(n)			(((n)&0x0007UL)<<8)
#define PLLCON_PREDIV_MASK 			(0x0007UL<<8) 

#define PLLCON_POSTDIV_VAL(n) 		(((n)&0x000FUL)<<0)
#define PLLCON_POSTDIV_MASK  		(0x000FUL<<0) 

typedef enum _pllStatus {
	PLL_OK = 0,
	PLL_WRONG = -1,
} pllStatus;

typedef enum _pllInClk {
	XTAL4MHz = 4,
	XTAL8MHz = 8,
	XTAL10MHz = 10,
	IOSC16MHz= 16,
} pllInClk;

typedef enum _pllFreq {
	PLL1MHz = 1,
	PLL2MHz = 2,
	PLL3MHz = 3,
	PLL4MHz = 4,
	PLL5MHz = 5,
	PLL6MHz = 6,
	PLL7MHz = 7,
	PLL8MHz = 8,
	PLL9MHz = 9,
	PLL10MHz = 10,
	PLL11MHz = 11,
	PLL12MHz = 12,
	PLL13MHz = 13,
	PLL14MHz = 14,
	PLL15MHz = 15,
	PLL16MHz = 16,
	PLL17MHz = 17,
	PLL18MHz = 18,
	PLL19MHz = 19,
	PLL20MHz = 20,
	PLL21MHz = 21,
	PLL22MHz = 22,
	PLL23MHz = 23,
	PLL24MHz = 24,
	PLL25MHz = 25,
	PLL26MHz = 26,
	PLL27MHz = 27,
	PLL28MHz = 28,
	PLL29MHz = 29,
	PLL30MHz = 30,
	PLL31MHz = 31,
	PLL32MHz = 32,
	PLL33MHz = 33,
	PLL34MHz = 34,
	PLL35MHz = 35,
	PLL36MHz = 36,
	PLL37MHz = 37,
	PLL38MHz = 38,
	PLL39MHz = 39,
	PLL40MHz = 40,
	PLL41MHz = 41,
	PLL42MHz = 42,
	PLL43MHz = 43,
	PLL44MHz = 44,
	PLL45MHz = 45,
	PLL46MHz = 46,
	PLL47MHz = 47,
	PLL48MHz = 48,
	PLL49MHz = 49,
	PLL50MHz = 50,
	PLL51MHz = 51,
	PLL52MHz = 52,
	PLL53MHz = 53,
	PLL54MHz = 54,
	PLL55MHz = 55,
	PLL56MHz = 56,
	PLL57MHz = 57,
	PLL58MHz = 58,
	PLL59MHz = 59,
	PLL60MHz = 60,
	PLL61MHz = 61,
	PLL62MHz = 62,	
	PLL63MHz = 63,
	PLL64MHz = 64,
	PLL65MHz = 65,
	PLL66MHz = 66,
	PLL67MHz = 67,
	PLL68MHz = 68,
	PLL69MHz = 69,
	PLL70MHz = 70,
	PLL71MHz = 71,	
	PLL72MHz = 72,
	PLL73MHz = 73,	
	PLL74MHz = 74,
	PLL75MHz = 75,
} pllFreq;

//===================================================================
// LVDCON 
//
//				@ addr = 0x4000_0054
//
//===================================================================
#define LVDCON_LVDEN					(0x0001UL<<15)
#define LVDCON_LVDRF					(0x0001UL<<14)
#define LVDCON_LVDREN					(0x0001UL<<11)

#define LVDCON_LVDRL_Pos					(8)
#define LVDCON_LVDRL_2_60V				(0x0000UL<<8)
#define LVDCON_LVDRL_2_80V				(0x0001UL<<8)
#define LVDCON_LVDRL_3_00V				(0x0002UL<<8)
#define LVDCON_LVDRL_3_30V				(0x0003UL<<8)
#define LVDCON_LVDRL_3_75V				(0x0004UL<<8)
#define LVDCON_LVDRL_4_00V				(0x0005UL<<8)
#define LVDCON_LVDRL_4_25V				(0x0006UL<<8)
#define LVDCON_LVDRL_4_50V				(0x0007UL<<8)
#define LVDCON_LVDRL_MASK				(0x0007UL<<8)
#define LVDCON_LVDRL_VAL(n)				(((n)&0x0007UL)<<8)

#define LVDCON_LVDIF					(0x0001UL<<6)
#define LVDCON_LVDICS					(0x0001UL<<5)
#define LVDCON_LVDIEN					(0x0001UL<<3)

#define LVDCON_LVDIL_2_60V				(0x0000UL<<0)
#define LVDCON_LVDIL_2_80V				(0x0001UL<<0)
#define LVDCON_LVDIL_3_00V				(0x0002UL<<0)
#define LVDCON_LVDIL_3_30V				(0x0003UL<<0)
#define LVDCON_LVDIL_3_75V				(0x0004UL<<0)
#define LVDCON_LVDIL_4_00V				(0x0005UL<<0)
#define LVDCON_LVDIL_4_25V				(0x0006UL<<0)
#define LVDCON_LVDIL_4_50V				(0x0007UL<<0)
#define LVDCON_LVDIL_MASK				(0x0007UL<<0)
#define LVDCON_LVDIL_VAL(n)				(((n)&0x0007UL)<<0)


//===================================================================
// VDCCON
//
//				@ addr = 0x4000_0058
//
//===================================================================
#define VDCCON_DFLVL 					(0x0001UL<<31)

#define VDCCON_DFLVL_EN_VAL(n)			(((n)&0x0003UL)<<22)




//===================================================================
// IOSC16TRIM 
//
//				@ addr = 0x4000_005C
//
//===================================================================
#define IOSC16TRIM_LT_EN 				(0x0001UL<<20)
#define IOSC16TRIM_LTM_EN 				(0x0001UL<<19)
#define IOSC16TRIM_TSL_EN 				(0x0001UL<<18)
#define IOSC16TRIM_UDCH_EN 			(0x0001UL<<17)
#define IOSC16TRIM_TCAL_EN 			(0x0001UL<<16)

#define IOSC16TRIM_LT_VAL(n) 			(((n)&0x000FUL)<<10)
#define IOSC16TRIM_LTM_VAL(n) 			(((n)&0x0003UL)<<8)
#define IOSC16TRIM_TSL_VAL(n) 			(((n)&0x0007UL)<<5)
#define IOSC16TRIM_UDCH_VAL(n) 		(((n)&0x0003UL)<<3)
#define IOSC16TRIM_TCAL_VAL(n) 			(((n)&0x0007UL)<<0)



//===================================================================
// EOSCCON
//
//				@ addr = 0x4000_0060
//
//===================================================================
#define EOSCCON_MAGIC_CODE 			(0x18A2UL<<16)

#define EOSCCON_SOSCNF					(0x0001UL<<15)
#define EOSCCON_SOSCNF_ON				(0x0000UL<<15)
#define EOSCCON_SOSCNF_OFF			(0x0001UL<<15)

#define EOSCCON_SOSCISEL_VAL(n)		(((n)&0x0003UL)<<12)
#define EOSCCON_SOSCISEL_MASK 		(0x0003UL<<12)

#define EOSCCON_MOSCNF 				(0x0001UL<<7)
#define EOSCCON_MOSCNF_ON 			(0x0000UL<<7)
#define EOSCCON_MOSCNF_OFF 			(0x0001UL<<7)

#define EOSCCON_MOSCNFSEL_VAL(n) 		(((n)&0x0003UL)<<4)
#define EOSCCON_MOSCNFSEL_MASK 		(0x0003UL<<4)

#define EOSCCON_MOSCISEL_VAL(n) 		(((n)&0x0003UL)<<0)
#define EOSCCON_MOSCISEL_MASK 		(0x0003UL<<0)




//===================================================================
// EXTMODER
//
//				@ addr = 0x4000_0070
//
//===================================================================



//===================================================================
//	PMULEGACY
//
//		@ address		= 0x4000_00F8
//
//===================================================================
#define PMULEGACY_MODE						(0x0001UL<<31)
#define PMULEGACY_MODE_NORMAL				(0x0000UL<<31)
#define PMULEGACY_MODE_LEGACY 				(0x0001UL<<31) 


#define PMULEGACY_LEGACY_STEP1 				(0xDADAUL<<0)
#define PMULEGACY_LEGACY_STEP2 				(0xDA00UL<<0)

#define PMULEGACY_NORMAL_STEP1 				(0xDADAUL<<0)
#define PMULEGACY_NORMAL_STEP2 				(0xDAADUL<<0)




//===================================================================
// macros
//===================================================================

//===================================================================
// function declarations
//===================================================================
void PMU_PLLEnable (PMU_Type *pmu, uint32_t prediv, uint32_t mul, uint32_t div, uint32_t postdiv, uint32_t vco_mode); 
int32_t PMU_SetPLLFreq(PMU_Type *pmu, uint32_t pllInClk, uint32_t setPllFreq);			// PLL Calcuator
void PMU_ConfigureInterrupt (PMU_Type *pmu, uint32_t intr_mask, uint32_t enable);
void PMU_CheckResetEvent (PMU_Type *pmu);

#ifdef __cplusplus
}
#endif

#endif /* end _a33g52x_PMU_H_ */

/* --------------------------------- End Of File ------------------------------ */
