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
extern void	NMI_Handler_IT(void);
extern void	MPWM0PROT_IRQHandler_IT(void);
extern void	MPWM0OVV_IRQHandler_IT(void);
extern void	MPWM0U_IRQHandler_IT(void);
extern void	MPWM0V_IRQHandler_IT(void);
extern void	MPWM0W_IRQHandler_IT(void);
extern void ADC0_IRQHandler_IT(void);
/* Private variables ---------------------------------------------------------*/


/******************************************************************************/
/*            Cortex M4 Processor Exceptions Handlers                         */
/******************************************************************************/

void	NMI_Handler(void)
{
	NMI_Handler_IT();
}

void	MPWM0PROT_IRQHandler(void)
{
	MPWM0PROT_IRQHandler_IT();
}

void	MPWM0OVV_IRQHandler(void)
{
	MPWM0OVV_IRQHandler_IT();
}

void	MPWM0U_IRQHandler(void)
{
	MPWM0U_IRQHandler_IT();
}

void	MPWM0V_IRQHandler(void)
{
	MPWM0V_IRQHandler_IT();
}

void	MPWM0W_IRQHandler(void)
{
	MPWM0W_IRQHandler_IT();
}

void ADC0_IRQHandler(void)
{
	ADC0_IRQHandler_IT();
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

/**********************************************************************
 * @brief		This function handles SysTick Handler.
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SysTick_Handler(void)
{
}
