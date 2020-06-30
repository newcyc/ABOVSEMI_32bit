/**********************************************************************
* @file		A34M41x_it.c
* @brief	Contains all macro definitions and function prototypes
* 			support for PCU firmware library
* @version	1.0
* @date		
* @author	ABOV M team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/
#include "A34M41x_it.h"
#include "main_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void SysTick_Handler_IT(void);
extern void TIMER1_IRQHandler_IT(void);
extern void QEI0_IRQHandler_IT(void);
/* Private variables ---------------------------------------------------------*/


/******************************************************************************/
/*            Cortex M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**********************************************************************
 * @brief		This function handles NMI exception.
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void NMI_Handler(void)
{
}


/**********************************************************************
 * @brief		This function handles Hard Fault exception.
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}


/**********************************************************************
 * @brief		This function handles SVCall exception
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SVC_Handler(void)
{
}

/**********************************************************************
 * @brief		This function handles PendSVC exception
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void PendSV_Handler(void)
{
}

/******************************************************************************/
/*                 A34M41x Peripherals Interrupt Handlers                     */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_A34M41x.s).                                                 */
/******************************************************************************/
/**********************************************************************
 * @brief		ADC0_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void ADC0_IRQHandler(void)
{

}


/**********************************************************************
 * @brief		LVI_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void LVI_IRQHandler(void)
{

}

/**********************************************************************
 * @brief		SysTick_Handler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SysTick_Handler(void)
{
	SysTick_Handler_IT();
}


/**********************************************************************
 * @brief		TIMER1_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void TIMER1_IRQHandler(void)
{
	TIMER1_IRQHandler_IT();
}


/**********************************************************************
 * @brief		GPIOB_IRQHandler
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void QEI0_IRQHandler(void)
{
	QEI0_IRQHandler_IT();
}



