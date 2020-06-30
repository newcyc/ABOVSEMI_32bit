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
void NMI_Handler_IT(void);
void MPWM0PROT_IRQHandler_IT(void);
void MPWM0OVV_IRQHandler_IT(void);
void MPWM0U_IRQHandler_IT(void);
void MPWM0V_IRQHandler_IT(void);
void MPWM0W_IRQHandler_IT(void);
void ADC0_IRQHandler_IT(void);
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void mainloop(void);
int main (void);
void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
"Motor PWM interrupt demo \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
" Using MPWM\n\r"
"************************************************\n\r";
const uint8_t psel_menu_m_t[] =
"************************************************\n\r"
"Protection Mode Selection \n\r"
"\t - 0 :  Protection L-activie \n\r"
"\t - 1 :  Protection H-activie \n\r"
"\t - 2 :  Overvoltage L-active \n\r"
"\t - 3 :  Overvoltage H-active \n\r"
"************************************************\n\r";

const uint8_t protc_menu_m_t[] =
"************************************************\n\r"
"Protection Status Clear Selection \n\r"
"\t - 0 :  ULPROT \n\r"
"\t - 1 :  VLPROT  \n\r"
"\t - 2 :  WLPROT  \n\r"
"\t - 3 :  UHPROT  \n\r"
"\t - 4 :  VHPROT  \n\r"
"\t - 5 :  WHPROT  \n\r"
"\t - 6 :  PROTIF  \n\r"
"************************************************\n\r";

const uint8_t int_menu_m_t[] =
"************************************************\n\r"
"Interrupt Enable Selection \n\r"
"\t - 0 :  UL Duty or ATR1 Interrupt \n\r"
"\t - 1 :  VL Duty or ATR2 Interrupt \n\r"
"\t - 2 :  WL Duty or ATR3 Interrupt \n\r"
"\t - 3 :  UH Duty or ATR4 Interrupt \n\r"
"\t - 4 :  VH Duty or ATR5 Interrupt \n\r"
"\t - 5 :  WH Duty or ATR6 Interrupt \n\r"
"\t - 6 :  Bottom Interrupt \n\r"
"\t - 7 :  Period Interrupt \n\r"
"************************************************\n\r";
const uint8_t modsel_m_t[] =
"------------------------------------------------\n\r"
"Motor PWM Mode Selection \n\r"
"\t - 0 : Motor Mode \n\r"
"\t - 1 : PWM Mode \n\r"
"\t - 2 : Extend Mode \n\r"
"------------------------------------------------\n\r";

const uint8_t test_menu_m_t[] =
"************************************************\n\r"
"Test Command  \n\r"
"\t 'test adc' : ADC ATR Test Mode \n\r"
"\t 'n' : W/V/U Phase Counter Halt \n\r"
"\t 'm' : W/V/U Phase Counter Continue \n\r"
"\t 'x' : W/V/U Phase Counter Start \n\r"
"\t 'i' : Interrupt Enable Mode \n\r"
"\t 'q' : Protection Status Clear Mode \n\r"
"\t 'p' : Protection Setting Mode \n\r"
"\t ' ' : Protection Setting and Status Check \n\r"
"\t '0' : MPWM Set Delta Duty value to 1\n\r"
"\t '1' : MPWM Set DeltaDuty value to 0x10\n\r"
"\t 's' : MPWM Mode(Motor/Normal/Individual) Change \n\r"
"\t 'u' : MPWM Up/Down Counter Mode Change \n\r"
"\t '[' : xH Phase Counter Duty Change (DutyH-=Delta) \n\r"
"\t ']' : xH Phase Counter Duty Change (DutyH+=Delta) \n\r"
"\t '{' : xL Phase Counter Duty Change (DutyL-=Delta) \n\r"
"\t '}' : xL Phase Counter Duty Change (DutyL+=Delta) \n\r"
"\t ',' : ATR1 Counter Change (trg-=Delta) \n\r"
"\t '.' : ATR1 Counter Change (trg+=Delta) \n\r"
"\t 'd' : Enter \n\r"
"************************************************\n\r";

static	int	_prot0 = 0;
static	int	_prot1 = 0;

uint32_t count_m_t;

uint32_t fflag_m_t;
uint16_t adcval_m_t[8];

/**********************************************************************
 * @brief		NMI_Handler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void	NMI_Handler_IT(void)
{
	int	sts;
	
	sts = SCU->NMISR;
	if(sts & (1<<4)) { //BV_MP0PROTSTS
		SCU->NMISR = 0x8C000010;
		MPWM0->PSR = 0xCA80;
		_prot0+= 100;
	}
	if(sts & (1<<3)) { //BV_MP0OVPSTS
		SCU->NMISR = 0x8C000008;
		MPWM0->OSR = 0xAC80;
		_prot1+= 100;
	}
	if(sts & (1<<6)) { //BV_MP1PROTSTS
		SCU->NMISR = 0x8C000040;
		MPWM0->PSR = 0xCA80;
		_prot0+= 100;
	}
	if(sts & (1<<5)) { //BV_MP1OVPSTS
		SCU->NMISR = 0x8C000020;
		MPWM0->OSR = 0xAC80;
		_prot1+= 100;
	}
}


/**********************************************************************
 * @brief		MPWM0PROT_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void	MPWM0PROT_IRQHandler_IT(void)
{
	MPWM0->PSR = 0xCA80;
	_prot0++;
}


/**********************************************************************
 * @brief		MPWM0OVV_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void	MPWM0OVV_IRQHandler_IT(void)
{
	MPWM0->OSR = 0xAC80;
	_prot1++;
}


/**********************************************************************
 * @brief		MPWM0U_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void	MPWM0U_IRQHandler_IT(void)
{
	int	irq;
	irq=MPWM0->SR;// & MPnIER;
	
	
	if((irq&(1<<7))==(1<<7)) { //BV_PRDIF_PH_U
		MPWM0->SR = (1<<7);
		PA->ODR^=(1<<7);
	}
	if((irq&(1<<6))==(1<<6)) { //BV_BOTIF_PH_U
		MPWM0->SR = (1<<6);
		PA->ODR^=(1<<6);
	}
	if((irq&(1<<5))==(1<<5)) { //BV_WHIF
		MPWM0->SR = (1<<5);
		PA->ODR^=(1<<5);
	}
	if((irq&(1<<4))==(1<<4)) { //BV_VHIF
		MPWM0->SR = (1<<4);
		PA->ODR^=(1<<4);
	}
	if((irq&(1<<3))==(1<<3)) { //BV_UHIF
		MPWM0->SR = (1<<3);
		PA->ODR^=(1<<3);
	}
	if((irq&(1<<2))==(1<<2)) { //BV_WLIF
		MPWM0->SR = (1<<2);
		PA->ODR^=(1<<2);
	}
	if((irq&(1<<1))==(1<<1)) {  //BV_VLIF
		MPWM0->SR = (1<<1);
		PA->ODR^=(1<<1);
	}
	if((irq&(1<<0))==(1<<0)) { //BV_ULIF
		MPWM0->SR = (1<<0);
		PA->ODR^=(1<<0);
	}
}

/**********************************************************************
 * @brief		MPWM0V_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void	MPWM0V_IRQHandler_IT(void)
{
	int	irq;
	irq=MPWM0->SR;// & MPnIER;
	

	if((irq&(1<<9))==(1<<9)) { //BV_PRDIF_PH_V
		MPWM0->SR = (1<<9);
		PA->ODR^=(1<<9);
	}
	if((irq&(1<<8))==(1<<8)) { //BV_BOTIF_PH_V
		MPWM0->SR = (1<<8);
		PA->ODR^=(1<<8);
	}
	if((irq&(1<<5))==(1<<5)) { //BV_WHIF
		MPWM0->SR = (1<<5);
		PA->ODR^=(1<<5);			
	}
	if((irq&(1<<4))==(1<<4)) { //BV_VHIF
		MPWM0->SR = (1<<4);
		PA->ODR^=(1<<4);			
	}
	if((irq&(1<<3))==(1<<3)) { //BV_UHIF
		MPWM0->SR = (1<<3);
		PA->ODR^=(1<<3);			
	}
	if((irq&(1<<2))==(1<<2)) { //BV_WLIF
		MPWM0->SR = (1<<2);
		PA->ODR^=(1<<2);			
	}
	if((irq&(1<<1))==(1<<1)) {  //BV_VLIF
		MPWM0->SR = (1<<1);
		PA->ODR^=(1<<1);			
	}
	if((irq&(1<<0))==(1<<0)) { //BV_ULIF
		MPWM0->SR = (1<<0);
		PA->ODR^=(1<<0);			
	}
}


/**********************************************************************
 * @brief		MPWM0W_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void	MPWM0W_IRQHandler_IT(void)
{
	int	irq;
	irq=MPWM0->SR;// & MPnIER;
	

	if((irq&(1<<11))==(1<<11)) { //BV_PRDIF_PH_W
		MPWM0->SR = (1<<11);
		PA->ODR^=(1<<11);
	}
	if((irq&(1<<10))==(1<<10)) { //BV_BOTIF_PH_W
		MPWM0->SR = (1<<10);
		PA->ODR^=(1<<10);
	}
	if((irq&(1<<5))==(1<<5)) { //BV_WHIF
		MPWM0->SR = (1<<5);
		PA->ODR^=(1<<5);			
	}
	if((irq&(1<<4))==(1<<4)) { //BV_VHIF
		MPWM0->SR = (1<<4);
		PA->ODR^=(1<<4);			
	}
	if((irq&(1<<3))==(1<<3)) { //BV_UHIF
		MPWM0->SR = (1<<3);
		PA->ODR^=(1<<3);			
	}
	if((irq&(1<<2))==(1<<2)) { //BV_WLIF
		MPWM0->SR = (1<<2);
		PA->ODR^=(1<<2);			
	}
	if((irq&(1<<1))==(1<<1)) {  //BV_VLIF
		MPWM0->SR = (1<<1);
		PA->ODR^=(1<<1);			
	}
	if((irq&(1<<0))==(1<<0)) { //BV_ULIF
		MPWM0->SR = (1<<0);
		PA->ODR^=(1<<0);			
	}
}

/**********************************************************************
 * @brief		ADC0_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void ADC0_IRQHandler_IT(void)
{
	uint16_t status;
	status = HAL_ADC_GetStatus(ADC0);

	if ((status & ADC_STAT_SEQ)==ADC_STAT_SEQ){
		PA->ODR^=(1<<2);
		HAL_ADC_ClearStatus(ADC0, (1<<2));
		adcval_m_t[0] = ADC0->DR0;
		adcval_m_t[1] = ADC0->DR1;		
		adcval_m_t[2] = ADC0->DR2;		
		ADC0->CSCR =0; // !! 
	}
	
	if ((status & ADC_STAT_SINGLE)==ADC_STAT_SINGLE){
		fflag_m_t=1;
		HAL_ADC_ClearStatus(ADC0, 1);	
		PA->ODR^=(1<<8);
		adcval_m_t[0] = ADC0->DR0;
		adcval_m_t[1] = ADC0->DR1;		
	}	
}


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
	// Inerrupt Checking Pin
	HAL_GPIO_ConfigureFunction(PA, 0, PA0_MUX_PA0);
	HAL_GPIO_ConfigOutput(PA, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PA, 0, PULL_UP_DOWN_DISABLE);
	
	HAL_GPIO_ConfigureFunction(PA, 1, PA1_MUX_PA1);
	HAL_GPIO_ConfigOutput(PA, 1, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PA, 1, PULL_UP_DOWN_DISABLE);
	
	HAL_GPIO_ConfigureFunction(PA, 2, PA2_MUX_PA2);
	HAL_GPIO_ConfigOutput(PA, 2, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PA, 2, PULL_UP_DOWN_DISABLE);
	
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_MPWM0UH);
	HAL_GPIO_ConfigureFunction(PB, 1, PB1_MUX_MPWM0UL);
	HAL_GPIO_ConfigureFunction(PB, 2, PB2_MUX_MPWM0VH);
	HAL_GPIO_ConfigureFunction(PB, 3, PB3_MUX_MPWM0VL);
	HAL_GPIO_ConfigureFunction(PB, 4, PB4_MUX_MPWM0WH);
	HAL_GPIO_ConfigureFunction(PB, 5, PB5_MUX_MPWM0WL);
	HAL_GPIO_ConfigureFunction(PB, 6, PB6_MUX_PRTIN0U);
	HAL_GPIO_ConfigureFunction(PB, 7, PB7_MUX_OVIN0U);
}


void ADC_Configure(void)
{
	//ADC_SINGLE_MODE
	ADC_CFG_Type AD0_config;

	AD0_config.Mode = ADC_SINGLE_MODE;
	AD0_config.SamplingTime=0x1f;
	AD0_config.SeqCnt = 3;
	AD0_config.RestartEn = 1;
	AD0_config.TrgSel = ADC_TRIGGER_MPWM0;
	AD0_config.UseClk = ADC_EXTERNAL_CLK;
	AD0_config.InClkDiv = 1;

	HAL_SCU_SetMCCRx(4, ADC_TYPE, SCU_MCCR_CSEL_HSI, 8);
	
	if( HAL_ADC_Init(ADC0, &AD0_config) != HAL_OK)
	{
		/* Initialization Error */
     Error_Handler();
	}	
	
	ADC0->CSCR =(0<<4);	

//	ADC_TriggerlSel(ADC0, 0x00000000); // use ATR1
//	ADC_TriggerlSel(ADC0, 0x00000010); //use ATR1, ATR2
	HAL_ADC_TriggerlSel(ADC0, 0x00000210); //use ATR1, ATR2, ATR3	

	HAL_ADC_ChannelSel(ADC0, 0x00000210);		
	
	HAL_ADC_ClearStatus(ADC0, 0x0f); // before adc interrupt enable, you must clear adc status!! 
	HAL_ADC_ConfigInterrupt(ADC0, (ADC_INTEN_SINGLE | ADC_INTEN_SEQ), ENABLE);

	NVIC_SetPriority(ADC0_IRQn, 3);
	/* Enable Interrupt for ADC channel */
        NVIC_EnableIRQ(ADC0_IRQn);
}

/**********************************************************************
 * @brief		check_prot
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void	check_prot(void)
{
	if(_prot0) {
		_DBG("\n\r");
		_DBG("PROT0="); _DBH(_prot0);_DBG("\n\r");		
		_prot0=0;
	}
	if(_prot1) {
		_DBG("\n\r");
		_DBG("PROT1="); _DBH(_prot1);_DBG("\n\r");			
		_prot1=0;
	}
}

/**********************************************************************
 * @brief		MPWM_PWMRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void MPWM_PWMRun(void)
{
	char 		cmd, ch, get_md;
	uint16_t 	dutyh, dutyl;
	uint16_t 	delta=0x10;
	int			trg, mode;
	
	IRQn_Type	M_MPWMx_IRQn, E_MPWMxU_IRQn, E_MPWMxV_IRQn, E_MPWMxW_IRQn;
	IRQn_Type	MPWMxPROT_IRQn, MPWMxOVV_IRQn;
	
	__enable_irq();
		
	_DBG(modsel_m_t);
	_DBG("A34M41x >");
	
	get_md = UARTGetChar(UART0);
	
	_DBC(get_md);
	_DBG("\r\n");
	
	__disable_irq();
	
		
//	SCU_SetMCCRx(2,MPWM0_TYPE,SCU_MCCR_CSEL_PLL,2); // select clock source
	HAL_SCU_SetMCCRx(2,MPWM0_TYPE,SCU_MCCR_CSEL_MCLK,2); // select clock source
//	SCU_SetMCCRx(2,MPWM0_TYPE,SCU_MCCR_CSEL_HSI,1); // select clock source
//	SCU_SetMCCRx(2,MPWM0_TYPE,SCU_MCCR_CSEL_HSE,1); // select clock source
		
	M_MPWMx_IRQn = MPWM0U_IRQn;
		
	E_MPWMxU_IRQn = MPWM0U_IRQn;
	E_MPWMxV_IRQn = MPWM0V_IRQn;
	E_MPWMxW_IRQn = MPWM0W_IRQn;
		
	MPWMxPROT_IRQn = MPWM0PROT_IRQn;
	MPWMxOVV_IRQn = MPWM0OVV_IRQn;
	
	if (get_md == 0x30)
	{
		HAL_MPWM_Init(MPWM0);
		
//		MPWM_Cmd(MPWM0,0,0,0,1,0,1);	// MPWM mode reg setting - motor mode (2-ch symmetric)
		
		HAL_MPWM_Cmd(MPWM0,0,1,1,0,1,1);	// MPWM mode reg setting - motor mode (1-ch asymmetric)
		
//		MPWM_Cmd(MPWM0,0,1,0,1,2,1);	// MPWM mode reg setting - motor mode (1-ch symmetric)
	}
	else if (get_md == 0x31)
	{
		HAL_MPWM_Init(MPWM0);
		
//		MPWM_Cmd(MPWM0,1,1,1,0,0,0);	// MPWM mode reg setting - pwm mode
		
		HAL_MPWM_Cmd(MPWM0,1,0,0,0,0,1);	// MPWM mode reg setting - pwm mode (up/down mode, Not use update timing)
		
//		MPWM_Cmd(MPWM0,1,0,1,0,0,1);	// MPWM mode reg setting - pwm mode (up/down mode, period udate)
		
//		MPWM_Cmd(MPWM0,1,0,0,1,0,1);	// MPWM mode reg setting - pwm mode (up/down mode, bottom udate)

	}
	else if (get_md == 0x32)
	{
		HAL_MPWM_ExtInit(MPWM0);
		
//		MPWM_Cmd(MPWM0,3,1,0,1,0,1);	// MPWM mode reg setting - extend mode (2-ch symmetric)

//		MPWM_Cmd(MPWM0,3,1,0,1,1,1);	// MPWM mode reg setting - extend mode (1-ch asymmetric)
		
		HAL_MPWM_Cmd(MPWM0,3,1,0,1,2,1);	// MPWM mode reg setting - extend mode (1-ch symmetric)
		
		HAL_GPIO_ConfigureFunction(PB, 6, PB6_MUX_PRTIN0U);
		HAL_GPIO_ConfigureFunction(PB, 7, PB7_MUX_OVIN0U);
		
		HAL_GPIO_ConfigureFunction(PA, 14, PA14_MUX_PRTINEV);
		HAL_GPIO_ConfigureFunction(PA, 15, PA15_MUX_OVINEV);
	
		HAL_GPIO_ConfigureFunction(PF, 6, PF6_MUX_PRTINEW);
		HAL_GPIO_ConfigureFunction(PF, 7, PF7_MUX_OVINEW);
		
	}
	
	if (get_md == 0x30 || get_md == 0x31)
	{
		/* preemption = 1, sub-priority = 1 */
		NVIC_SetPriority(M_MPWMx_IRQn, 3);
		/* Enable Interrupt for MPWM channel */
		NVIC_EnableIRQ(M_MPWMx_IRQn);
	}
	else if (get_md == 0x32)
	{
		/* preemption = 1, sub-priority = 1 */
		NVIC_SetPriority(E_MPWMxU_IRQn, 3);
		NVIC_SetPriority(E_MPWMxV_IRQn, 3);
		NVIC_SetPriority(E_MPWMxW_IRQn, 3);
		/* Enable Interrupt for MPWM channel */
		NVIC_EnableIRQ(E_MPWMxU_IRQn);
		NVIC_EnableIRQ(E_MPWMxV_IRQn);
		NVIC_EnableIRQ(E_MPWMxW_IRQn);
	}
	
	// Over Voltage, Protection Interrupt
	NVIC_SetPriority(MPWMxPROT_IRQn, 1);
	NVIC_SetPriority(MPWMxOVV_IRQn, 1);
	
	// Enable IRQ
	NVIC_EnableIRQ(MPWMxPROT_IRQn);
	NVIC_EnableIRQ(MPWMxOVV_IRQn);

	//===========================================
	// Duty 50% Setting
	//===========================================
	dutyh=0x5000;
	dutyl=0x5000;
	
	if (get_md == 0x32)
	{
		HAL_MPWM_ExtSetPeriod(MPWM0, PH_U, 0xa000);
		HAL_MPWM_ExtSetPeriod(MPWM0, PH_V, 0xa000);
		HAL_MPWM_ExtSetPeriod(MPWM0, PH_W, 0xa000);
	}
	else
	{
		HAL_MPWM_SetPeriod(MPWM0,0xa000);	
	}
	
	HAL_MPWM_SetUDuty(MPWM0,dutyh,dutyl);
	HAL_MPWM_SetVDuty(MPWM0,dutyh,dutyl);
	HAL_MPWM_SetWDuty(MPWM0,dutyh,dutyl);
	
	if (get_md == 0x32)
	{	
	//=============================================================
	//			DEADTIME & P - SHORT ======>>>> OFF
	//=============================================================
		/* deadtime&p-short not use */
		HAL_MPWM_ExtSetDeadTime(MPWM0, PH_U, 0, 1, 0, 0, 0, 0);
		HAL_MPWM_ExtSetDeadTime(MPWM0, PH_V, 0, 1, 0, 0, 0, 0);
		HAL_MPWM_ExtSetDeadTime(MPWM0, PH_W, 0, 1, 0, 0, 0, 0);
	}
	else
	{
	//=============================================================
	//			DEADTIME & P - SHORT ======>>>> OFF
	//=============================================================
		HAL_MPWM_SetDeadTime(MPWM0,0,1,0,0,0); // dead time setting 		
	}

	mode=0;

	__enable_irq();

	_DBG(test_menu_m_t);
	
	while(1){
		_DBG("*");	
		getstring();
    
		if (InFlag) {
			_DBG("\n\r");
			cmd=InData[0];
			
			if(strncmp(InData+5,"adc",3)==0) {
					_DBG("adc ");

					ADC_Configure();
				
					_DBG("\n\rADC ATR test \n\r");
					_DBG("\n\rATR1 : 0x3000, ATR2 : 0x5000, ATR3 : 0x8900 - up counter \n\r");						
					_DBG("\n\rATR4 : 0x8900, ATR5 : 0x5000, ATR6 : 0x3000 - down counter \n\r");			
				
					//=============================================================
					//			ATR Use Mode (up&down counter)
					//=============================================================
					MPWM0->ATR1 = (1<<16) | (1<<19) | (0x3000);
					MPWM0->ATR2 = (1<<16) | (1<<19) | (0x5000);
					MPWM0->ATR3 = (1<<16) | (1<<19) | (0x8900);
					MPWM0->ATR4 = (2<<16) | (1<<19) | (0x8900);
					MPWM0->ATR5 = (2<<16) | (1<<19) | (0x5000);
					MPWM0->ATR6 = (2<<16) | (1<<19) | (0x3000);	
					
					/* ATR Interrupt Enable */
					MPWM0->IER = 0x3F;			
				}					
			else {
			switch(cmd) {
				case 'n':	//halt
					if (get_md == 0x32)
					{
						MPWM0->CR3 ^= 0x80000;	// Phase-W HALT
						MPWM0->CR3 ^= 0x800;	// Phase-V HALT
						MPWM0->CR3 ^= 0x8;		// Phase-U HALT
						_DBG("\n\r");
						_DBG("MPWM0->CR3="); _DBH(MPWM0->CR3);_DBG("\n\r");
					}
					else
					{
						MPWM0->CR2 ^=0x80; // PWM HALT
						_DBG("\n\r");
						_DBG("MPWM0->CR2="); _DBH(MPWM0->CR2);_DBG("\n\r");
					}
					break;
				case 'm':	//Continue
					if (get_md == 0x32)
					{
						MPWM0->CR4 |= (0x80000);	// Phase-W Cont
						MPWM0->CR4 |= (0x800);		// Phase-V Cont
						MPWM0->CR4 |= (0x8);		// Phase-U Cont
					}
					break;
				case 'x':	//start
					if (get_md == 0x32)
					{
						PA->ODR = 0x0;

						MPWM0->CR3 ^= 0x2;		// Phase-U Start
						MPWM0->CR3 ^= 0x200;	// Phase-V Start
						MPWM0->CR3 ^= 0x20000;	// Phase-W Start

						_DBG("\n\r");
						_DBG("MPWM0->CR3="); _DBH(MPWM0->CR3);_DBG("\n\r");
						
					}
					else
					{
						PA->ODR = 0x0;
						
						MPWM0->CR2 ^=1;	// PWM Start

						_DBG("\n\r");
						_DBG("MPWM0->CR2="); _DBH(MPWM0->CR2);_DBG("\n\r");
					}
					break;	
				case 'p':	//protection mode select
					_DBG(psel_menu_m_t);
					ch=_DG;
					_DBC(ch);
					_DBG("\n\r");
					if (get_md == 0x32)
					{
						switch(ch){
						case '0':	HAL_MPWM_SetProtCtrl(MPWM0, 0x818181BF, 0);	break; // U/V/W Phase Protection L-active
						case '1':	HAL_MPWM_SetProtCtrl(MPWM0, 0xC1C1C1BF, 0);	break; // U/V/W Phase Protection H-active
						case '2':	HAL_MPWM_SetProtCtrl(MPWM0, 0x818181BF, 1);	break; // U/V/W Phase Overvoltage L-active
						case '3':	HAL_MPWM_SetProtCtrl(MPWM0, 0xC1C1C1BF, 1);	break; // U/V/W Phase Overvoltage H-active
						}							
					}
					else
					{
						switch(ch) {
						case '0':	HAL_MPWM_SetProtCtrl(MPWM0, 0x8189, 0);	break; // U Phase Protection L-active
						case '1':	HAL_MPWM_SetProtCtrl(MPWM0, 0xC189, 0);	break; // U Phase Protection H-active
						case '2':	HAL_MPWM_SetProtCtrl(MPWM0, 0x8189, 1);	break; // U Phase Overvoltage L-active
						case '3':	HAL_MPWM_SetProtCtrl(MPWM0, 0xC189, 1);	break; // U Phase Overvoltage H-active
							}
					}			
					break;	
				case 'i':	
					_DBG(int_menu_m_t);
					ch=_DG;
					_DBC(ch);
					_DBG("\n\r");
					switch(ch) {
						case '0':	MPWM0->IER^= 0x01;	break; //BV_ULIE
						case '1':	MPWM0->IER^= 0x02;	break; //BV_VLIE
						case '2':	MPWM0->IER^= 0x04;	break; //BV_WLIE
						case '3':	MPWM0->IER^= 0x08;	break; //BV_UHIE
						case '4':	MPWM0->IER^= 0x10;	break; //BV_VHIE
						case '5':	MPWM0->IER^= 0x20;	break; //BV_WHIE
						case '6':	MPWM0->IER^= 0x40;	break; //BV_BOTIE
						case '7':	MPWM0->IER^= 0x80;	break; //BV_PRDIE
					}
					_DBG("\n\r");
					_DBG("MPWM0->IER="); _DBH(MPWM0->IER);_DBG("\n\r");						
					break;
				case 'q':	
					_DBG(protc_menu_m_t);
					ch=_DG;
					switch(ch) {
					case '0': 	MPWM0->PSR^= 0xCA01;	break;
					case '1': 	MPWM0->PSR^= 0xCA02;	break;
					case '2': 	MPWM0->PSR^= 0xCA04;	break;
					case '3': 	MPWM0->PSR^= 0xCA08;	break;
					case '4': 	MPWM0->PSR^= 0xCA10;	break;
					case '5': 	MPWM0->PSR^= 0xCA20;	break;
					case '6': 	MPWM0->PSR^= 0xCA80;	break;
					}
					break;
				case ' ':
					_DBG("\n\r");
					_DBG("PB->IDR="); _DBH(PB->IDR);	
					_DBG("\n\r");
					_DBG("MPWM0->PCR="); _DBH(MPWM0->PCR);	
					_DBG("\n\r");
					_DBG("MPWM0->PSR="); _DBH(MPWM0->PSR);_DBG("\n\r");							
					break;
				case '1':	delta=1; break;
				case '0':	delta=0x10; break;
				case 's':	if(++mode>3) mode=0;
					MPWM0->MR = (MPWM0->MR&~(3<<1))|(mode<<1);
					_DBG("\n\r");
					_DBG("MCHMOD="); _DBD(mode);_DBG("\n\r");					
					break;

				case 'u':	if(MPWM0->MR&0x0001) {
								MPWM0->MR&=~0x0001;
								_DBG("(Up-count Mode)");_DBG("\n\r");
							}
							else {
								MPWM0->MR|= 0x0001;
								_DBG("(Up-down Mode)");_DBG("\n\r");
							}
							break;						
				case '[':	dutyh-=delta;
					MPWM0->DUH= dutyh;
					MPWM0->DVH= dutyh;
					MPWM0->DWH= dutyh;		
					_DBG("\n\r");
					_DBG("MPWM0->MR="); _DBH(MPWM0->MR);	
					_DBG(" dutyh="); _DBD16(dutyh);	_DBG("\n\r");	
					break;
				case ']':	dutyh+=delta;
					MPWM0->DUH= dutyh;
					MPWM0->DVH= dutyh;
					MPWM0->DWH= dutyh;
					_DBG("\n\r");
					_DBG("MPWM0->MR="); _DBH(MPWM0->MR);	
					_DBG(" dutyh="); _DBD16(dutyh);_DBG("\n\r");							
					break;
				case '{':	dutyl-=delta;
					MPWM0->DUL= dutyl;
					MPWM0->DVL= dutyl;
					MPWM0->DWL= dutyl;
					_DBG("\n\r");
					_DBG("MPWM0->MR="); _DBH(MPWM0->MR);	
					_DBG(" dutyl="); _DBD16(dutyl);_DBG("\n\r");							
					break;
				case '}':	dutyl+=delta;
					MPWM0->DUL= dutyl;
					MPWM0->DVL= dutyl;
					MPWM0->DWL= dutyl;
					_DBG("\n\r");
					_DBG("MPWM0->MR="); _DBH(MPWM0->MR);	
					_DBG(" dutyl="); _DBD16(dutyl);_DBG("\n\r");					
					break;
				case ',':	trg-= delta;
					MPWM0->ATR1=trg;
					_DBG("\n\r");
					_DBG("MPWM0->MR="); _DBH(MPWM0->MR);	
					_DBG(" trg="); _DBD16(trg);_DBG("\n\r");							
					break;
				case '.':	trg+= delta;
					MPWM0->ATR1=trg;
					_DBG("\n\r");
					_DBG("MPWM0->MR="); _DBH(MPWM0->MR);	
					_DBG(" trg="); _DBD16(trg);_DBG("\n\r");							
					break;						
				case 'd':
					_DBG("\n\r");							
					break;
				}
			}
		}			
		else {
			_DBG("\n\r");
			InFlag=0;
			InCount=0;	
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

	/*MPWM strt */
	MPWM_PWMRun();
	
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

