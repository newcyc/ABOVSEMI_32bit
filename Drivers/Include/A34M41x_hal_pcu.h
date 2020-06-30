/**********************************************************************
* @file		A34M41x_pcu.h
* @brief	Contains all macro definitions and function prototypes
* 			support for PCU firmware library on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application3 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

#ifndef _A34M41x_PCU_H_
#define _A34M41x_PCU_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Private macros ------------------------------------------------------------- */

#define PORT_ACCESS_EN()  						do { PORTEN->EN=0x15; PORTEN->EN=0x51; } while(0)
#define PORT_ACCESS_DIS()  						do { PORTEN->EN=0x00; } while(0) 

#define PCU_MR_FUNC_Msk                        (0x07UL)
#define PCU_CR_MODE_Msk                        (0x03UL)
#define PCU_PRCR_Msk						   (0x03UL)
#define PCU_STR_Msk							   (0x01UL)

/* ================================================================================ */
/* ================          struct 'PCU' Position & Mask          ================ */
/* ================================================================================ */


/* Pin function */
#define FUNC0 			0x0				/** Function 0 	*/
#define FUNC1 			0x1				/** Function 1 	*/
#define FUNC2 			0x2				/** Function 2	*/
#define FUNC3 			0x3				/** Function 3	*/
#define FUNC4			0x7				/** Function 4  */


//==========================================================================
// 	PAMR
//==========================================================================
#define PA0_MUX_PA0							(0)
#define PA0_MUX_AN0							(7)

#define PA1_MUX_PA1							(0)
#define PA1_MUX_AN1							(7)

#define PA2_MUX_PA2							(0)
#define PA2_MUX_AN2							(7)

#define PA3_MUX_PA3							(0)
#define PA3_MUX_AN3							(7)

#define PA4_MUX_PA4							(0)
#define PA4_MUX_T0IO						(2)
#define PA4_MUX_CP0A						(7)
#define PA4_MUX_AN4							(7)

#define PA5_MUX_PA5							(0)
#define PA5_MUX_T1IO						(2)
#define PA5_MUX_CAPEU						(3)
#define PA5_MUX_CP0B						(7)
#define PA5_MUX_AN5							(7)

#define PA6_MUX_PA6							(0)
#define PA6_MUX_T2IO						(2)
#define PA6_MUX_CAPEV						(3)
#define PA6_MUX_CP0C						(7)
#define PA6_MUX_AN6							(7)

#define PA7_MUX_PA7							(0)
#define PA7_MUX_T3IO						(2)
#define PA7_MUX_CAPEW						(3)
#define PA7_MUX_CREF0						(7)
#define PA7_MUX_AN7							(7)

#define PA8_MUX_PA8							(0)
#define PA8_MUX_CAN_RX						(1)
#define PA8_MUX_CP1A						(7)
#define PA8_MUX_AN8							(7)

#define PA9_MUX_PA9							(0)
#define PA9_MUX_CAN_TX						(1)
#define PA9_MUX_CP1B						(7)
#define PA9_MUX_AN9							(7)

#define PA10_MUX_PA10						(0)
#define PA10_MUX_RXD1						(1)
#define PA10_MUX_SCAPEU						(3)
#define PA10_MUX_CP1C						(7)
#define PA10_MUX_AN10						(7)

#define PA11_MUX_PA11						(0)
#define PA11_MUX_TXD1						(1)
#define PA11_MUX_SCAPEV						(3)
#define PA11_MUX_CREF1						(7)
#define PA11_MUX_AN11						(7)

#define PA12_MUX_PA12						(0)
#define PA12_MUX_SS0						(1)
#define PA12_MUX_QEI0_UPDN					(2)
#define PA12_MUX_SCAPEW						(3)
#define PA12_MUX_AN12						(7)

#define PA13_MUX_PA13						(0)
#define PA13_MUX_SCK0						(1)
#define PA13_MUX_QEI0_A						(2)
#define PA13_MUX_AN13						(7)

#define PA14_MUX_PA14						(0)
#define PA14_MUX_MOSI0						(1)
#define PA14_MUX_QEI0_B						(2)
#define PA14_MUX_PRTINEV					(3)
#define PA14_MUX_AN14						(7)

#define PA15_MUX_PA15						(0)
#define PA15_MUX_MISO0						(1)
#define PA15_MUX_QEI0_IDX					(2)
#define PA15_MUX_OVINEV						(3)
#define PA15_MUX_AN15						(7)


//==========================================================================
// 	PBMR
//==========================================================================
#define PB0_MUX_PB0					(0)
#define PB0_MUX_MPWM0UH				(3)

#define PB1_MUX_PB1					(0)
#define PB1_MUX_MPWM0UL				(3)

#define PB2_MUX_PB2					(0)
#define PB2_MUX_MPWM0VH				(3)				

#define PB3_MUX_PB3					(0)
#define PB3_MUX_MPWM0VL				(3)				

#define PB4_MUX_PB4					(0)
#define PB4_MUX_T8IO				(2)
#define PB4_MUX_MPWM0WH				(3)

#define PB5_MUX_PB5					(0)
#define PB5_MUX_T9IO				(2)
#define PB5_MUX_MPWM0WL				(3)

#define PB6_MUX_PB6					(0)
#define PB6_MUX_WDTO				(2)
#define PB6_MUX_PRTIN0U				(3)

#define PB7_MUX_PB7					(0)
#define PB7_MUX_STBYO				(2)
#define PB7_MUX_OVIN0U				(3)

#define PB8_MUX_PB8					(0)
#define PB8_MUX_RXD3				(1)
#define PB8_MUX_PRTIN1U				(3)

#define PB9_MUX_PB9					(0)
#define PB9_MUX_TXD3				(1)
#define PB9_MUX_OVIN1U				(3)

#define PB10_MUX_PB10				(0)
#define PB10_MUX_MPWM1UH			(3)

#define PB11_MUX_PB11				(0)
#define PB11_MUX_MPWM1UL			(3)

#define PB12_MUX_PB12				(0)
#define PB12_MUX_MPWM1VH			(3)

#define PB13_MUX_PB13				(0)
#define PB13_MUX_MPWM1VL			(3)

#define PB14_MUX_PB14				(0)
#define PB14_MUX_MPWM1WH			(3)

#define PB15_MUX_PB15				(0)
#define PB15_MUX_MPWM1WL			(3)


//==========================================================================
// 	PCMR
//
//==========================================================================
#define PC0_MUX_PC0							(0)
#define PC0_MUX_RXD0						(1)
#define PC0_MUX_TCK_SWCLK					(3)

#define PC1_MUX_PC1							(0)
#define PC1_MUX_TXD0						(1)
#define PC1_MUX_TMS_SWDIO					(3)

#define PC2_MUX_PC2							(0)
#define PC2_MUX_TDO_SWO						(3)

#define PC3_MUX_PC3							(0)
#define PC3_MUX_TDI							(3)

#define PC4_MUX_PC4							(0)
#define PC4_MUX_T0IO						(2)
#define PC4_MUX_nTRST						(3)

#define PC5_MUX_PC5							(0)
#define PC5_MUX_RXD1						(1)
#define PC5_MUX_T1IO						(2)

#define PC6_MUX_PC6							(0)
#define PC6_MUX_TXD1						(1)
#define PC6_MUX_T2IO						(2)

#define PC7_MUX_PC7							(0)
#define PC7_MUX_SCL0						(1)
#define PC7_MUX_T3IO						(2)

#define PC8_MUX_PC8							(0)
#define PC8_MUX_SDA0						(1)
#define PC8_MUX_T4IO						(2)

#define PC9_MUX_PC9							(0)
#define PC9_MUX_T8IO						(2)
#define PC9_MUX_CLKO						(3)

#define PC10_MUX_PC10						(0)
#define PC10_MUX_nRESET						(3)

#define PC11_MUX_PC11						(0)
#define PC11_MUX_T9IO						(2)
#define PC11_MUX_BOOT 						(3)

#define PC12_MUX_PC12						(0)
#define PC12_MUX_XIN						(7)

#define PC13_MUX_PC13						(0)
#define PC13_MUX_XOUT						(7)

#define PC14_MUX_PC14						(0)
#define PC14_MUX_MOSI0						(1)
#define PC14_MUX_RXD0						(3)

#define PC15_MUX_PC15						(0)
#define PC15_MUX_MISO0						(1)
#define PC15_MUX_TXD0						(3)


//==========================================================================
// 	PDMR
//==========================================================================
#define PD0_MUX_PD0							(0)
#define PD0_MUX_SS1							(1)
#define PD0_MUX_SXIN						(7)

#define PD1_MUX_PD1							(0)
#define PD1_MUX_SCK1						(1)
#define PD1_MUX_SXOUT						(7)

#define PD2_MUX_PD2							(0)
#define PD2_MUX_MOSI1						(1)

#define PD3_MUX_PD3							(0)
#define PD3_MUX_MISO1						(1)

#define PD4_MUX_PD4							(0)
#define PD4_MUX_SCL1						(1)
#define PD4_MUX_AN16						(7)

#define PD5_MUX_PD5							(0)
#define PD5_MUX_SDA1						(1)
#define PD5_MUX_AN17						(7)

#define PD6_MUX_PD6							(0)
#define PD6_MUX_TXD2						(1)
#define PD6_MUX_AN18						(7)

#define PD7_MUX_PD7							(0)
#define PD7_MUX_RXD2						(1)
#define PD7_MUX_AN19						(7)

#define PD8_MUX_PD8							(0)
#define PD8_MUX_T6IO						(2)
#define PD8_MUX_WDTO						(3)

#define PD9_MUX_PD9							(0)
#define PD9_MUX_T7IO						(2)
#define PD9_MUX_STBYO						(3)

#define PD10_MUX_PD10						(0)
#define PD10_MUX_T0IO						(2)
#define PD10_MUX_AD0S						(3)

#define PD11_MUX_PD11						(0)
#define PD11_MUX_T1IO						(2)
#define PD11_MUX_AD0E						(3)

#define PD12_MUX_PD12						(0)
#define PD12_MUX_T2IO						(2)
#define PD12_MUX_AD1S						(3)

#define PD13_MUX_PD13						(0)
#define PD13_MUX_T3IO						(2)
#define PD13_MUX_AD1E						(3)

#define PD14_MUX_PD14						(0)
#define PD14_MUX_SS0						(1)
#define PD14_MUX_AD2S						(3)

#define PD15_MUX_PD15						(0)
#define PD15_MUX_SCK0						(1)
#define PD15_MUX_AD2E						(3)


//==========================================================================
// 	PEMR
//==========================================================================
#define PE0_MUX_PE0							(0)

#define PE1_MUX_PE1							(0)
#define PE1_MUX_CP2							(7)
#define PE1_MUX_AN20						(7)

#define PE2_MUX_PE2							(0)
#define PE2_MUX_CREF2						(7)
#define PE2_MUX_AN21						(7)

#define PE3_MUX_PE3							(0)
#define PE3_MUX_SCL0						(1)

#define PE4_MUX_PE4							(0)
#define PE4_MUX_SDA0						(1)

#define PE5_MUX_PE5							(0)
#define PE5_MUX_T5IO						(2)

#define PE6_MUX_PE6							(0)
#define PE6_MUX_T5IO						(2)
#define PE6_MUX_QEI1_UPDN					(3)

#define PE7_MUX_PE7							(0)
#define PE7_MUX_T6IO						(2)
#define PE7_MUX_QEI1_A						(3)

#define PE8_MUX_PE8							(0)
#define PE8_MUX_T7IO						(2)
#define PE8_MUX_QEI1_B						(3)

#define PE9_MUX_PE9							(0)
#define PE9_MUX_T8IO						(2)
#define PE9_MUX_QEI1_IDX					(3)

#define PE10_MUX_PE10						(0)
#define PE10_MUX_T9IO						(2)

#define PE11_MUX_PE11						(0)
#define PE11_MUX_SCL1						(1)
#define PE11_MUX_T0IO						(2)

#define PE12_MUX_PE12						(0)
#define PE12_MUX_SDA1						(1)
#define PE12_MUX_T1IO						(2)

#define PE13_MUX_PE13						(0)
#define PE13_MUX_TXD4						(1)
#define PE13_MUX_T2IO						(2)

#define PE14_MUX_PE14						(0)
#define PE14_MUX_RXD4						(1)
#define PE14_MUX_T3IO						(2)

#define PE15_MUX_PE15						(0)


//==========================================================================
// 	PFMR
//==========================================================================
#define PF0_MUX_PF0							(0)

#define PF1_MUX_PF1							(0)

#define PF2_MUX_PF2							(0)
#define PF2_MUX_CP3							(7)
#define PF2_MUX_AN22						(7)

#define PF3_MUX_PF3							(0)
#define PF3_MUX_CREF3						(7)
#define PF3_MUX_AN23						(7)

#define PF4_MUX_PF4							(0)
#define PF4_MUX_TXD5						(1)

#define PF5_MUX_PF5							(0)
#define PF5_MUX_RXD5						(1)

#define PF6_MUX_PF6							(0)
#define PF6_MUX_CAN_RX						(1)
#define PF6_MUX_PRTINEW						(3)

#define PF7_MUX_PF7							(0)
#define PF7_MUX_CAN_TX						(1)
#define PF7_MUX_OVINEW						(3)

#define PF8_MUX_PF8							(0)

#define PF9_MUX_PF9							(0)

#define PF10_MUX_PF10						(0)

#define PF11_MUX_PF11						(0)

#define PF12_MUX_PF12						(0)

#define PF13_MUX_PF13						(0)

#define PF14_MUX_PF14						(0)

#define PF15_MUX_PF15						(0)


//==========================================================================
// 	PGMR
//==========================================================================
#define PG0_MUX_PG0							(0)
#define PG0_MUX_SS2							(1)

#define PG1_MUX_PG1							(0)
#define PG1_MUX_SCK2						(1)

#define PG2_MUX_PG2							(0)
#define PG2_MUX_MOSI2						(1)

#define PG3_MUX_PG3							(0)
#define PG3_MUX_MISO2						(1)

#define PG4_MUX_PG4							(0)

#define PG5_MUX_PG5							(0)

#define PG6_MUX_PG6							(0)

#define PG7_MUX_PG7							(0)

#define PG8_MUX_PG8							(0)
#define PG8_MUX_RXD3						(1)

#define PG9_MUX_PG9							(0)
#define PG9_MUX_TXD3						(1)

#define PG10_MUX_PG10						(0)




/*
 * @brief 	PCU port mode enumerate definition
 */
typedef enum {
	PUSH_PULL_OUTPUT = 0,
	OPEN_DRAIN_OUTPUT,
	INPUT
}PCU_PORT_MODE;

typedef enum {
	PULL_UP_DOWN_DISABLE = 0,
	PULL_UP_ENABLE = 2,
	PULL_DOWN_ENABLE = 3
}PCU_PULLUP_MODE;

typedef enum {
	LEVEL1 = 0,
	LEVEL2,
	LEVEL3,
	LEVEL4
}PCU_STR_LEVEL;


enum {
	IER_DISABLE = 0,
	IER_LEVEL_NON_PENDING,
	IER_LEVEL_PENDING,
	IER_EDGE
};

enum {
	ICR_PROHIBIT_INT = 0,
	ICR_LOW_LEVEL_INT = 1,
	ICR_FALLING_EDGE_INT = ICR_LOW_LEVEL_INT,
	ICR_HIGH_LEVEL_INT =2,
	ICR_RISING_EDGE_INT = ICR_HIGH_LEVEL_INT,
	ICR_BOTH_EDGE_INT =3	
};


/* Public Functions ----------------------------------------------------------- */

void HAL_GPIO_ConfigureFunction(PCU_Type *PCx, uint8_t pin_no, uint32_t func);

void HAL_GPIO_ConfigOutput(PCU_Type *PCx, uint8_t pin_no, PCU_PORT_MODE dir_type);
void HAL_GPIO_ConfigPullup (PCU_Type *PCx, uint8_t pin_no, PCU_PULLUP_MODE pullup);

void HAL_GPIO_ConfigureStrength (PCU_Type *PCx, uint8_t pin_no, PCU_STR_LEVEL str_level);

void HAL_GPIO_EXTI_Config(PCU_Type *PCx, uint8_t pin_no, uint8_t pin_en, uint8_t int_mode);
uint32_t HAL_GPIO_EXTI_GetStatus(PCU_Type *PCx);
void HAL_GPIO_EXTI_ClearPin(PCU_Type *PCx, uint32_t value);

void HAL_GPIO_SetDebouncePin (PCU_Type *PCx, uint8_t pin_no, FunctionalState debounce);


/* Public Functions ----------------------------------------------------------- */
void HAL_GPIO_SetPin(PCU_Type *Px, uint16_t bitValue);
void HAL_GPIO_ClearPin(PCU_Type *Px, uint16_t bitValue);
uint16_t HAL_GPIO_ReadPin(PCU_Type *Px);
void HAL_GPIO_WritePin(PCU_Type *Px, uint16_t Value);

uint32_t HAL_PCU_GetIntMode(PCU_Type *PCx);
uint32_t HAL_PCU_GetIntModeStatus(PCU_Type *PCx);

#ifdef __cplusplus
}
#endif


#endif /* end _A34M41x_PCU_H_ */

/* --------------------------------- End Of File ------------------------------ */
