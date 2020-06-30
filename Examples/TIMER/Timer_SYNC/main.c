/**********************************************************************
* @file		main.c
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
#include "main_conf.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void TIMER_SYNCRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
" A34M41x_TEST_EXAMPLE \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
" Using Timer in SYNC Mode\n\r"
"\t 1. SYNC Init \n\r"
"\t 2. SYNC Test \n\r"
"************************************************\n\r";
const uint8_t cmdm_t_s[] =
"A34M41x> ";

const uint8_t test_sel_t_s[]=
"\t SYNC Test Selection\r\n"
"\t - 0: SSYNC(delay : 256us) Setting \n\r"
"\t        Master : Timer0, Slave : Timer1 \n\r"
"\t        Master : Timer1, Slave : Timer3 \n\r"
"\t - 1: CSYNC Setting \n\r"
"\t        Master : Timer0, Slave : Timer1 \n\r"
"\t - 2: SSYNC(delay:256us) + CSYNC Setting \n\r"
"\t        Master : Timer0, Slave : Timer1 \n\r";

TIMER_PWMCFG_Type Ta_Config;
TIMER_PWMCFG_Type Tb_Config;
TIMER_PWMCFG_Type Tc_Config;


/**********************************************************************
 * @brief		Print menu
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void DEBUG_MenuPrint(void)
{
	#ifdef _DEBUG_MSG
	_DBG(menu);
	#endif
}


/**********************************************************************
 * @brief		DEBUG_Init
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void DEBUG_Init(void)
{
	#ifdef _DEBUG_MSG
	debug_frmwrk_init();
	#endif
}


/**********************************************************************
 * @brief		PCU_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_Configure(void)
{
			// T0O pin config set if need
			HAL_GPIO_ConfigureFunction(PA, 4, PA4_MUX_T0IO);
			HAL_GPIO_ConfigOutput(PA, 4, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PA, 4, PULL_UP_DOWN_DISABLE);

			// T1O pin config set if need
			HAL_GPIO_ConfigureFunction(PA, 5, PA5_MUX_T1IO);
			HAL_GPIO_ConfigOutput(PA, 5, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PA, 5, PULL_UP_DOWN_DISABLE);

			// T3O pin config set if need
			HAL_GPIO_ConfigureFunction(PA, 7, PA7_MUX_T3IO);
			HAL_GPIO_ConfigOutput(PA, 7, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PA, 7, PULL_UP_DOWN_DISABLE);
}


/**********************************************************************
 * @brief		TIMER_SyncRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void TIMER_SYNCRun(void)
{
	uint8_t		cmdn_cnt=0, ch_rtn, ch_mtn, mod_check[3] = {0x30, 0x31, 0x32};
		
	while(1)
	{
		if (cmdn_cnt == 0)
		{
			_DBG(cmdm_t_s);
			cmdn_cnt=1;
		}
		
		ch_rtn = UARTGetChar(UART0);
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		if (ch_rtn == 0x0D)
		{
			_DBG("\r\n");
		}
		
		if (ch_rtn == mod_check[1])
		{			
			_DBG("SYNC Init OK\r\n");
			// TIMERa(TIMER0) Setting
			Ta_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
			Ta_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

			Ta_Config.GRA = (500); // 0.5msec	DUTY = GRA
			Ta_Config.GRB = (2000); // 2msec	PERIOD = GRB
			Ta_Config.Cnt=0;		
			Ta_Config.StartLevel=START_LOW;
			Ta_Config.AdcTrgEn=DISABLE;
			
			
			// TIMERb(TIMER1) Setting
			Tb_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
			Tb_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

			Tb_Config.GRA = (500); // 0.5msec	DUTY = GRA
			Tb_Config.GRB = (2500); // 2.1msec	PERIOD = GRB
			Tb_Config.Cnt=0;		
			Tb_Config.StartLevel=START_LOW;
			Tb_Config.AdcTrgEn=DISABLE;
			
			// TIMERc(TIMER3) Setting
			Tc_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
			Tc_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

			Tc_Config.GRA = (500); // 0.5msec	DUTY = GRA
			Tc_Config.GRB = (3000); // 2.2msec	PERIOD = GRB
			Tc_Config.Cnt=0;		
			Tc_Config.StartLevel=START_LOW;
			Tc_Config.AdcTrgEn=DISABLE;
			
	
			// TIMERa(TIMER0) Setting
			if(HAL_TIMER_Init(TIMER0, PWM_MODE, &Ta_Config) != HAL_OK)
			{
				Error_Handler();
			}

			// TIMERb(TIMER1) Setting
			if(HAL_TIMER_Init(TIMER1, PWM_MODE, &Tb_Config) != HAL_OK)
			{
				Error_Handler();
			}

			// TIMERc(TIMER3) Setting
			if(HAL_TIMER_Init(TIMER3, PWM_MODE, &Tc_Config) != HAL_OK)
			{
				Error_Handler();
			}

			_DBG(test_sel_t_s);		// SYNC Mode Select
modesel:
			ch_mtn = UARTGetChar(UART0);
			_DBC(ch_mtn);
			_DBG("\r\n");

			/* SYNC Mode Initialization */
			if (ch_mtn == mod_check[0])		// SYNC Setting
			{			
				HAL_TIMER_SYNCConfig(TIMER0, TIMER1, (TIMER_SYNC_SSYNC), 0x0100);
				HAL_TIMER_SYNCConfig(TIMER1, TIMER3, (TIMER_SYNC_SSYNC), 0x0100);
			}
			else if (ch_mtn == mod_check[1])		// CSYNC Setting 
			{
				HAL_TIMER_SYNCConfig(TIMER0, TIMER1, TIMER_SYNC_CSYNC, 0);
			}
			else if (ch_mtn == mod_check[2])		// SYNC + CSYNC Setting
			{
				HAL_TIMER_SYNCConfig(TIMER0, TIMER1, (TIMER_SYNC_SSYNC|TIMER_SYNC_CSYNC), 0x100);
			}
			else
			{
				_DBG("...Please input character Check!  \n\r");
				goto modesel;
			}
			cmdn_cnt = 0;
		}

		else if (ch_rtn == mod_check[2])
		{
			_DBG("SYNC Test start!!\r\n");
			/* preemption = 1, sub-priority = 1 */
			NVIC_SetPriority(TIMER0_IRQn, ((0x01<<1)|0x01));
			/* Enable Interrupt for TIMER0 channel */
			NVIC_EnableIRQ(TIMER0_IRQn);

			/* preemption = 1, sub-priority = 1 */
			NVIC_SetPriority(TIMER1_IRQn, ((0x01<<1)|0x01));
			/* Enable Interrupt for TIMER0 channel */
			NVIC_EnableIRQ(TIMER1_IRQn);
			
			/* preemption = 1, sub-priority = 1 */
			NVIC_SetPriority(TIMER3_IRQn, ((0x01<<1)|0x01));
			/* Enable Interrupt for TIMER0 channel */
			NVIC_EnableIRQ(TIMER3_IRQn);
			
			/* SYNC Test Start */
			if (ch_mtn == mod_check[0])		// SYNC Setting
			{			
				HAL_TIMER_Cmd(TIMER0, ENABLE); // Master(TIMER0) start
			}
			else if (ch_mtn == mod_check[1])		// CSYNC Setting 
			{
				HAL_TIMER_Cmd(TIMER0, ENABLE); // Master(TIMER0) start
				HAL_TIMER_Cmd(TIMER1, ENABLE); // Slave(TIMER1) start
			}
			else if (ch_mtn == mod_check[2])		// SYNC + CSYNC Setting
			{
				HAL_TIMER_Cmd(TIMER0, ENABLE); // Master(TIMER0) start
			}
			
			__enable_irq();
			
			cmdn_cnt = 0;
		}
		
		else
		{
			_DBG("...Please input character 1:Init, 2:Test!  \n\r");
			cmdn_cnt=0;
		}
	}	
}


/**********************************************************************
 * @brief		Main loop
 * @param[in]	None
 * @return	None
 **********************************************************************/
void mainloop(void)
{
	/*Configure menu prinf*/
	DEBUG_MenuPrint();

	/*Configure port peripheral*/
	GPIO_Configure();
  
	/* Enable IRQ Interrupts */
	__enable_irq();

	TIMER_SYNCRun();
	
	while(1)
	{

	}
}

/**********************************************************************
 * @brief		Main program
 * @param[in]	None
 * @return	None
 **********************************************************************/
int main (void)
{

	 /* Initialize all port */
	Port_Init(); 

	/* Configure the system clock to HSE 8 MHz */
	SystemClock_Config();
	
	/* Initialize Debug frame work through initializing USART port  */
	DEBUG_Init();		
	
	/* Infinite loop */
	mainloop();  


	return (0);
}

/**********************************************************************
  * @brief  Reports the name of the source file and the source line number
  *   where the check_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: check_param error line source number
  * @retval : None
 **********************************************************************/
void Error_Handler(void)
{
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**********************************************************************
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
 **********************************************************************/
void check_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

   /* Infinite loop */
   while (1)
   {
   }
}
#endif

