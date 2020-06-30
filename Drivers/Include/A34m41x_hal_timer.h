/**********************************************************************
* @file		A34M41x_timer.h
* @brief	Contains all macro definitions and function prototypes
* 			support for timer firmware library on A34M41x
* @version	1.0
* @date		
* @author ABOV Application2 team
*
* Copyright(C)  2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
#ifndef _AC34M418_TIMER_H_
#define _AC34M418_TIMER_H_

/* Includes ------------------------------------------------------------------- */
#include "A34M41x.h"
#include "A34M41x_aa_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**********************************************************************
* Timer Control1 Register definitions
**********************************************************************/
/** Counter/timer mode bits */
#define TIMER_CR1_MODE_MASK  (0x3)

/** clear mode select bits when capture */
#define TIMER_CR1_CLRMD_MASK		(0x03<<2)
#define TIMER_CR1_CLRMD_SET(n)		(n<<2)

/** counter clock select bits */
#define TIMER_CR1_CKSEL_MASK		(0x07<<4)
#define TIMER_CR1_CKSEL_SET(n)		(n<<4)

/** counter clock select bits */
#define TIMER_CR1_STARTLVL_MASK		(0x01<<7)
#define TIMER_CR1_STARTLVL_SET(n)	(n<<7)

/** adc triger source enable bits */
#define TIMER_CR1_ADCTRGEN_MASK		(0x01<<8)
#define TIMER_CR1_ADCTRGEN_SET(n)	(n<<8)

/** timer pin input/output selection bits */
#define TIMER_CR1_IOSEL_MASK		(0x01<<11)
#define TIMER_CR1_IOSEL_SET(n)		(n<<11)

/** adc triger source enable bits */
#define TIMER_CR1_OUTPOL_MASK		(0x01<<12)
#define TIMER_CR1_OUTPOL_SET(n)		(n<<12)

/** adc triger source enable bits */
#define TIMER_CR1_UAO_MASK			(0x01<<13)
#define TIMER_CR1_UAO_SET(n)		(n<<13)

/** adc triger source enable bits */
#define TIMER_CR1_CSYNC_MASK		(0x01<<14)
#define TIMER_CR1_CSYNC_SET(n)		(n<<14)

/** adc triger source enable bits */
#define TIMER_CR1_SSYNC_MASK		(0x01<<15)
#define TIMER_CR1_SSYNC_SET(n)		(n<<15)


/**********************************************************************
* Timer control2 register definitions
**********************************************************************/
/** Timer/counter enable bit */
#define TIMER_ENABLE			((uint8_t)(1<<0))
/** Timer/counter reset bit */
#define TIMER_CLEAR			((uint8_t)(1<<1))
/** Timer control bit mask */

/**********************************************************************
* Timer Prescaler Register definitions (PRS)
**********************************************************************/
#define TIMER_PRS_MASK	0x03FF

/**********************************************************************
* Timer Status Register definitions (SR)
**********************************************************************/
#define TIMER_SR_OVF		((uint8_t)(1<<0))
#define TIMER_SR_MFB		((uint8_t)(1<<1))
#define TIMER_SR_MFA		((uint8_t)(1<<2))

/**********************************************************************
* Timer Interrupt Enable Register definitions (IER)
**********************************************************************/
#define TIMER_IER_OVIE			((uint8_t)(1<<0))
#define TIMER_IER_MBIE			((uint8_t)(1<<1))
#define TIMER_IER_MAIE			((uint8_t)(1<<2))

#define TIMER_IER_BITMASK		(0x07)


/**********************************************************************
* Timer sync setting register (SYNC)
**********************************************************************/
#define TIMER_SYNC_SYNCB_9		(1<<29)
#define TIMER_SYNC_SYNCB_8		(1<<28)
#define TIMER_SYNC_SYNCB_7		(1<<27)
#define TIMER_SYNC_SYNCB_6		(1<<26)
#define TIMER_SYNC_SYNCB_5		(1<<25)
#define TIMER_SYNC_SYNCB_4		(1<<24)
#define TIMER_SYNC_SYNCB_3		(1<<23)
#define TIMER_SYNC_SYNCB_2		(1<<22)
#define TIMER_SYNC_SYNCB_1		(1<<21)
#define TIMER_SYNC_SYNCB_0		(1<<20)

#define TIMER_SYNC_SSYNC		(1<<17)
#define TIMER_SYNC_CSYNC		(1<<16)



/* ================================================================================ */
/* ================         struct 'TGECR' Position & Mask         ================ */
/* ================================================================================ */

/* ----------------------------------  TGECR_CR  ---------------------------------- */
#define TGECR_CR_QDMOD_Pos                    0					/*!< TGECR CR: QDMOD Position                */
#define TGECR_CR_QDMOD_Msk                    (0x01UL << 0)		/*!< TGECR CR: QDMOD Mask                    */
#define TGECR_CR_QDPHSWAP_Pos                 2					/*!< TGECR CR: QDPHSWAP Position             */
#define TGECR_CR_QDPHSWAP_Msk                 (0x01UL << 2)		/*!< TGECR CR: QDPHSWAP Mask                 */
#define TGECR_CR_QDPHZEG_Pos                  3					/*!< TGECR CR: QDPHZEG Position              */
#define TGECR_CR_QDPHZEG_Msk                  (0x01UL << 3)		/*!< TGECR CR: QDPHZEG Mask                  */
#define TGECR_CR_PDPHAEG_Pos                  4					/*!< TGECR CR: PDPHAEG Position              */
#define TGECR_CR_PDPHAEG_Msk                  (0x03UL << 4)		/*!< TGECR CR: PDPHAEG Mask                  */
#define TGECR_CR_QDPHBEG_Pos                  6					/*!< TGECR CR: QDPHBEG Position              */
#define TGECR_CR_QDPHBEG_Msk                  (0x03UL << 6)		/*!< TGECR CR: QDPHBEG Mask                  */
#define TGECR_CR_ADIRCON_Pos                  8					/*!< TGECR CR: ADIRCON Position              */
#define TGECR_CR_ADIRCON_Msk                  (0x01UL << 8)		/*!< TGECR CR: ADIRCON Mask                  */
#define TGECR_CR_BDIRCON_Pos                  9					/*!< TGECR CR: BDIRCON Position              */
#define TGECR_CR_BDIRCON_Msk                  (0x01UL << 9)		/*!< TGECR CR: BDIRCON Mask                  */
#define TGECR_CR_PDIRCON_Pos                  10				/*!< TGECR CR: PDIRCON Position              */
#define TGECR_CR_PDIRCON_Msk                  (0x01UL << 10)	/*!< TGECR CR: PDIRCON Mask                  */
#define TGECR_CR_RDIRCON_Pos                  11				/*!< TGECR CR: RDIRCON Position              */
#define TGECR_CR_RDIRCON_Msk                  (0x01UL << 11)	/*!< TGECR CR: RDIRCON Mask                  */



/* Public Types --------------------------------------------------------------- */
/***********************************************************************
 * @brief Timer device enumeration
**********************************************************************/
/* Timer n Control register 1 */
/** @brief Timer operating mode */
typedef enum
{
	PERIODIC_MODE = 0,	/*!< PERIODIC mode */
	PWM_MODE,				/*!< PWM mode */
	ONESHOT_MODE,		/*!< ONE SHOT mode */
	CAPTURE_MODE			/*!< CAPTURE mode */
} TIMER_MODE_OPT;

/** @brief clear select when capture mode */
typedef enum
{
	RISING_EGDE = 0,	/*!< rising edge clear mode */
	FALLING_EGDE,		/*!< falling edge clear mode */
	BOTH_EGDE,			/*!< both edge clear  mode */
	NONE						/*!< none clear mode */
} TIMER_CLR_MODE_OPT;

/** @brief counter clock source select */
typedef enum
{
	PCLK_2 = 0,		/*!< clock source from pclk div 2 */
	PCLK_4,			/*!< clock source from pclk div 4 */
	PCLK_16,			/*!< clock source from pclk div 16 */
	PCLK_64,			/*!< clock source from pclk div 64 */
	EXT = 4,			/*!< clock source from MCCR3(TEXT) clock setting */
	TnC = 6			/*!< clock source from TnC pin input. before setting, have to set TnC pin mode */	
} TIMER_CKSEL_MODE_OPT;

/** @brief start default level select: initial output value. */
typedef enum
{
	START_LOW = 0,		/*!< clock source from pclk div 2 */
	START_HIGH			/*!< clock source from pclk div 4 */
} TIMER_STARTLVL_OPT;

/*********************************************************************
* @brief TIMER Interrupt Type definitions
**********************************************************************/
typedef enum {
	TIMER_INTCFG_OVIE = 0,		/*!< OVIE Interrupt enable*/
	TIMER_INTCFG_MBIE,			/*!< MBIE Interrupt enable*/
	TIMER_INTCFG_MAIE,			/*!< MAIE interrupt enable*/
} TIMER_INT_Type;

/***********************************************************************
 * @brief Timer structure definitions
**********************************************************************/
/** @brief Configuration structure in TIMER mode */
typedef struct
{
	uint16_t GRA;	
	uint16_t GRB;	
	uint16_t Cnt;	
	uint16_t Prescaler;		/**< Timer Prescaler(TnPRS), should be:
									- 0~1023  value range  
									*/
	uint8_t CkSel;			/**< Counter clock source select, should be:
									- PCLK_2  : PCLK / 2
									- PCLK_4  : PCLK / 4
									- PCLK_16: PCLK / 16
									- PCLK_64: PCLK / 64
									- EXT = 4  : EXT,  clock source from MCCR3(TEXT) clock
									- TnC 	
									*/
	uint8_t StartLevel;			/**< set initial output value, should be:
									- START_LOW 
									- START_HIGH
									*/	
	uint8_t AdcTrgEn;			/**< Adc triger source enable, should be:
									- DISABLE 
									- ENABLE
									*/
} TIMER_PERIODICCFG_Type;

/** @brief Configuration structure in COUNTER mode */
typedef struct {
	uint16_t GRA;		
	uint16_t GRB;			
	uint16_t Cnt;		
	uint16_t Prescaler;		/**< Timer Prescaler(TnPRS), should be:
									- 0~1023  value range  
									*/
	uint8_t CkSel;			/**< Counter clock source select, should be:
									- PCLK_2  : PCLK / 2
									- PCLK_4  : PCLK / 4
									- PCLK_16: PCLK / 16
									- PCLK_64: PCLK / 64
									- EXT = 4  : EXT,  clock source from MCCR3(TEXT) clock
									- TnC 	
									*/	
	uint8_t StartLevel;			/**< set initial output value, should be:
									- START_LOW 
									- START_HIGH
									*/	
	uint8_t AdcTrgEn;			/**< Adc triger source enable, should be:
									- DISABLE 
									- ENABLE
									*/		
} TIMER_PWMCFG_Type,TIMER_ONESHOTCFG_Type;

/** @brief Capture Input configuration structure */
typedef struct {	
	uint16_t Prescaler;		/**< Timer Prescaler(TnPRS), should be:
									- 0~1023  value range  
									*/
	uint8_t ClrMode;		/**< clear select when capture, should be:
									- RISING_EGDE
									- FALLING_EGDE
									- BOTH_EGDE
									- NONE	
									*/
	uint8_t CkSel;			/**< Counter clock source select, should be:
									- PCLK_2  : PCLK / 2
									- PCLK_4  : PCLK / 4
									- PCLK_16: PCLK / 16
									- PCLK_64: PCLK / 64
									- EXT = 4  : EXT,  clock source from MCCR3(TEXT) clock
									- TnC 	
									*/	
	uint8_t AdcTrgEn;			/**< Adc triger source enable, should be:
									- DISABLE 
									- ENABLE
									*/	
	uint8_t Reserved[2];	/** Reserved */
	
} TIMER_CAPTURECFG_Type;


/* Public Functions ----------------------------------------------------------- */
/* Init/DeInit TIM functions -----------*/
HAL_Status_Type HAL_TIMER_Init(TIMER_Type *TIMERx, TIMER_MODE_OPT TimerCounterMode, void *TIMER_ConfigStruct);
HAL_Status_Type HAL_TIMER_DeInit (TIMER_Type *TIMERx);

/* TIM configuration functions --------*/
HAL_Status_Type HAL_TIMER_ClearStatus(TIMER_Type *TIMERx, uint8_t value);
uint8_t HAL_TIMER_GetStatus(TIMER_Type *TIMERx);
HAL_Status_Type HAL_TIMER_Cmd(TIMER_Type *TIMERx, FunctionalState NewState);
HAL_Status_Type HAL_TIMER_ClearCounter(TIMER_Type *TIMERx);
HAL_Status_Type HAL_TIMER_UpdateCountValue(TIMER_Type *TIMERx, uint8_t CountCh, uint16_t Value);
uint16_t HAL_TIMER_GetCountValue(TIMER_Type *TIMERx, uint8_t CountCh);
HAL_Status_Type HAL_TIMER_ConfigInterrupt(TIMER_Type *TIMERx, TIMER_INT_Type TIMERIntCfg, FunctionalState NewState);

HAL_Status_Type HAL_TIMER_SYNCConfig(TIMER_Type* TIMERa, TIMER_Type* TIMERb, uint32_t CMD, uint32_t DLYVAL);


#ifdef __cplusplus
}
#endif

#endif /* _A34M41x_TIMER_H_ */

/* --------------------------------- End Of File ------------------------------ */
