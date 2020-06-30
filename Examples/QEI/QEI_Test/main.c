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
void QEI_TestRun(void);
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
"\t QEI Test Example \n\r"
"\t 1. QEI Init \n\r"
"\t 2. QEI Test \n\r"
"************************************************\n\r";
const uint8_t cmdm_q_t[] =
"A34M41x> ";
const uint8_t tmenu_q_t[] =
"************************************************\n\r"
" TEST Mode Selection \n\r"
"\t QEI Test Example \n\r"
"\t 1. Maximum reset Check \n\r"
"\t 2. Position, Index Counter Direction Check \n\r"
"\t 3. Direction Interrupt Check \n\r"
"\t 4. Signal Mode Check \n\r"
"\t 5. Capture Mode(2X, 4X) Check \n\r"
"\t 6. Position Counter Test(1, 2, 3, ..., MAX, Reset & Reverse \n\r"
"\t 7. Index Counter Test(1, 2, 3, ..., MAX, Reset & Reverse \n\r"
"\t 8. Velocity Counter Test(Reset) \n\r"
"\t 9. Index, Position0~2 Interrupt Test \n\r"
"************************************************\n\r";

const uint8_t dp_menu[] =
"************************************************\n\r"
" Position Counter Direction Selection \n\r"
"\t 0. None \n\r"
"\t 1. Change POSCNT \n\r"
"************************************************\n\r";

const uint8_t di_menu[] =
"************************************************\n\r"
" Index Counter Direction Selection \n\r"
"\t 0. None \n\r"
"\t 1. Change IDXCNT \n\r"
"************************************************\n\r";

const uint8_t smenu[] =
"************************************************\n\r"
" SWAP  Selection \n\r"
"\t 0. No Swap \n\r"
"\t 1. SWAP \n\r"
"************************************************\n\r";
const uint8_t cmenu[] =
"************************************************\n\r"
"Capture Time Measure .. POS : 0xFFFF \n\r"
" Capture  Selection \n\r"
"\t 0. only Ph-A(PA4) edge is counted \n\r"
"\t 1. Ph-A(PA4) and Ph-B(PA5) edge are counted \n\r"
"************************************************\n\r";

const uint8_t d_mode[] =
"************************************************\n\r"
" Direction Interrupt Test Mode \n\r"
"   To test the Direction Interrupt, \n\r"
"    replace the Ph-A(PA4) and Ph-B(PA5) Signal \n\r"
"************************************************\n\r";

const uint8_t s_mode[] =
"************************************************\n\r"
" Position Counter Direction Signal Output Test  \n\r"
"   Ph-A(PA4) : Clock, Ph-B(PA5) :Direction \n\r"
"   0 : Position counter direction is negative(-) \n\r"
"   1 : Position counter direction is positive(+) \n\r"
"************************************************\n\r";

const uint8_t sig_mode[] =
"************************************************\n\r"
" QEI Input Signal Setting \n\r"
"\t - PhA(PA4)  DUTY : 10us , PERIOD : 20us \n\r"
"\t - PhB(PA5)  DUTY : 10us, PERIOD : 20us \n\r"
"\t - IDX(PA6) DUTY : 10us, PERIOD : 20us (Every :1s) \n\r"
"************************************************\n\r";

const uint8_t idx_sel[] =
"************************************************\n\r"
" Index Gating configuration \n\r"
"\t 1. Pass the index  \n\r"
"\t 2. Pass the index when Ph-A=0, Ph-B=0 \n\r"
"\t 3. Pass the index when Ph-A=0, Ph-B=1 \n\r"
"\t 4. Pass the index when Ph-A=1, Ph-B=1 \n\r"
"\t 5. Pass the index when Ph-A=1, Ph-B=0 \n\r"
"************************************************\n\r";

uint32_t msec_q_t;
uint32_t timer_val;
uint32_t cap_check,idx_check,dir_check,pos_check,vel_check,int_check;
uint32_t matchintcnt;
uint32_t overflowintcnt;


/**********************************************************************
 * @brief		SysTick_Handler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SysTick_Handler_IT(void)
{
	if(msec_q_t)msec_q_t--;	
}


/**********************************************************************
 * @brief		TIMER1_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void TIMER1_IRQHandler_IT(void)
{
	uint32_t status;
	status = HAL_TIMER_GetStatus(TIMER1);
	if((status & TIMER_SR_MFB) == TIMER_SR_MFB)
	{
		matchintcnt++;
		if(matchintcnt>50) {
			HAL_TIMER_Cmd(TIMER2,ENABLE);
		   matchintcnt=0;
		}
	}
	HAL_TIMER_ClearStatus(TIMER1, status);	
}


/**********************************************************************
 * @brief		QEI0_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void QEI0_IRQHandler_IT(void)
{
	uint32_t status;
	
	status = HAL_QEI_GetInterruptStatus(QEI0);
	
	if((status&QEInISR_POS0_FLAG) == QEInISR_POS0_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);
		if(cap_check==1){
			HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), DISABLE);
			cap_check=0;
		}
		if(pos_check==2)
		{
			pos_check=3;
			HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), DISABLE);	
		}
		if(int_check==0){
			int_check=1;
		}
	}
	
	if((status&QEInISR_POS1_FLAG) == QEInISR_POS1_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);
		if(int_check==1){
			int_check=2;
		}
	}
	
	if((status&QEInISR_POS2_FLAG) == QEInISR_POS2_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);	
		if(int_check==2)
		{
			int_check=3;
		}
	}
	
	if((status&QEInISR_IDX_FLAG) == QEInISR_IDX_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);
		if(idx_check==0)
		{
			idx_check=1;
			// Mode Selection
			HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), DISABLE);	
		}
	}
	
	if((status&QEInISR_MAX_FLAG) == QEInISR_MAX_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);
		if(pos_check==0)
		{
			pos_check=1;
		}
		else if(pos_check==1)
		{
			pos_check=2;
			// Mode Selection
			HAL_QEI_ModeCmd(QEI0, (QEInMR_DIRPC_ON | QEInMR_QDSWAP_ON), ENABLE);	
		}
		if(int_check==3)
		{
			int_check=4;
		}
	}
	
	if((status&QEInISR_INX_FLAG) == QEInISR_INX_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);
		if(int_check==4){
			int_check=5;
		}
	}
	
		if((status&QEInISR_DIR_FLAG) == QEInISR_DIR_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);
		if(dir_check==0)
		{
			HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), DISABLE);
			dir_check=1;
		}
	}
	
	if((status&QEInISR_VELT_FLAG) == QEInISR_VELT_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);
		if(vel_check==0)
		{
			HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON |QEInMR_QDVEL_ON ), DISABLE);
			vel_check=1;
		}
	}
	
	if((status&QEInISR_VELC_FLAG) == QEInISR_VELC_FLAG)
	{
		HAL_QEI_ClearInterruptFlag(QEI0, status);	
		if(vel_check==1)
		{
			vel_check=2;
			HAL_QEI_InterruptCmd(QEI0,QEInIER_VELC_ON, DISABLE);
		}
	}

	HAL_QEI_ClearInterruptFlag(QEI0, status);	
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
		/* TIMER Setting */
		// T0O pin config set if need
		HAL_GPIO_ConfigureFunction(PA, 4, PA4_MUX_T0IO);
		HAL_GPIO_ConfigOutput(PA, 4, PUSH_PULL_OUTPUT);
		HAL_GPIO_ConfigPullup(PA, 4, PULL_UP_DOWN_DISABLE);
		
		HAL_GPIO_ConfigureFunction(PA, 5, PA5_MUX_T1IO);
		HAL_GPIO_ConfigOutput(PA, 5, PUSH_PULL_OUTPUT);
		HAL_GPIO_ConfigPullup(PA, 5, PULL_UP_DOWN_DISABLE);
		
		HAL_GPIO_ConfigureFunction(PA, 6, PA6_MUX_T2IO);
		HAL_GPIO_ConfigOutput(PA, 6, PUSH_PULL_OUTPUT);
		HAL_GPIO_ConfigPullup(PA, 6, PULL_UP_DOWN_DISABLE);
		
		HAL_GPIO_ConfigureFunction(PA, 7, PA7_MUX_T3IO);
		HAL_GPIO_ConfigOutput(PA, 7, PUSH_PULL_OUTPUT);
		HAL_GPIO_ConfigPullup(PA, 7, PULL_UP_DOWN_DISABLE);
}


/**********************************************************************
 * @brief		QEI_TestRun
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void QEI_TestRun(void)
{
	uint8_t		cmdn_cnt=0, ch_rtn,ch_itn, ch_p, ch_i, ch_s, mod_check[3] = {0x30, 0x31, 0x32};
	uint8_t		tmd_check[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
	TIMER_PWMCFG_Type Tx_Config;

	while(1)
	{
		if (cmdn_cnt == 0)
		{
			_DBG(cmdm_q_t);
			cmdn_cnt=1;
		}
		
		ch_rtn = _DG;
		
		_DBC(ch_rtn);
		_DBG("\r\n");
		
		if (ch_rtn == 0x0D)
		{
			_DBG("\r\n");
		}
				
		Tx_Config.CkSel = PCLK_2; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/2=4Mhz 
		Tx_Config.Prescaler = 3;    // TCLK = (CLOCK_IN / (PRS+1))  = 4Mhz / 4 = 1Mhz ->1us

		Tx_Config.GRA = (10); // 1us * 10 = 10us	DUTY = GRA
		Tx_Config.GRB = (20); // 1us * 20 = 20us	PERIOD = GRB
		Tx_Config.Cnt=0;		
		Tx_Config.StartLevel=START_LOW;
		Tx_Config.AdcTrgEn=DISABLE;
		
		if(HAL_TIMER_Init(TIMER0, PWM_MODE, &Tx_Config) != HAL_OK) // timer0 setting 
		{
			Error_Handler();
		}
		if(HAL_TIMER_Init(TIMER1, PWM_MODE, &Tx_Config) != HAL_OK) // timer1 setting 
		{
			Error_Handler();
		}
		if(HAL_TIMER_Init(TIMER2, ONESHOT_MODE, &Tx_Config) != HAL_OK) // timer2 setting 
		{
			Error_Handler();
		}
		
		Tx_Config.CkSel = PCLK_64; // SystemPeriClock=8Mhz=PCLK  CLOCK_IN = PCLK/64=125khz 
		Tx_Config.Prescaler = 124;    // TCLK = (CLOCK_IN / (PRS+1))  = 125khz / (124+1)= 1khz ->1ms
		Tx_Config.GRA = (0); // 1ms * 0 = 0us	DUTY = GRA
		Tx_Config.GRB = (10000); // 1ms * 10000 = 10s	PERIOD = GRB
		Tx_Config.Cnt=0;		
		Tx_Config.StartLevel=START_LOW;
		Tx_Config.AdcTrgEn=DISABLE;
		if(HAL_TIMER_Init(TIMER3, PERIODIC_MODE, &Tx_Config) != HAL_OK) // timer3 setting 
		{
			Error_Handler();			
		}
		
		HAL_TIMER_SYNCConfig(TIMER0, TIMER1, (TIMER_SYNC_SSYNC), 5);
		
		HAL_TIMER_ConfigInterrupt(TIMER1, TIMER_INTCFG_MBIE, ENABLE);	// Match Interrupt Enable
		
		/* preemption = 1, sub-priority = 1 */
		NVIC_SetPriority(TIMER1_IRQn, ((0x01<<1)|0x01));
		/* Enable Interrupt for TIMER0 channel */
		NVIC_EnableIRQ(TIMER1_IRQn);		
		
		__enable_irq();
		
		HAL_TIMER_Cmd(TIMER0, ENABLE);

		
		if (ch_rtn == mod_check[1])	// QEI0 Init
		{
			// QEI0(UPDN, Ph-A, Ph-B, IDX) Pin Setting 
			HAL_GPIO_ConfigureFunction(PA, 12, PA12_MUX_QEI0_UPDN);
			HAL_GPIO_ConfigOutput(PA, 12, PUSH_PULL_OUTPUT);
			HAL_GPIO_ConfigPullup(PA, 12, PULL_UP_DOWN_DISABLE);
			
			HAL_GPIO_ConfigureFunction(PA, 13, PA13_MUX_QEI0_A);
			HAL_GPIO_ConfigOutput(PA, 13, INPUT);
			HAL_GPIO_ConfigPullup(PA, 13, PULL_UP_DOWN_DISABLE);
			
			HAL_GPIO_ConfigureFunction(PA, 14, PA14_MUX_QEI0_B);
			HAL_GPIO_ConfigOutput(PA, 14, INPUT);
			HAL_GPIO_ConfigPullup(PA, 14, PULL_UP_DOWN_DISABLE);
			
			HAL_GPIO_ConfigureFunction(PA, 15, PA15_MUX_QEI0_IDX);
			HAL_GPIO_ConfigOutput(PA, 15, INPUT);
			HAL_GPIO_ConfigPullup(PA, 15, PULL_UP_DOWN_DISABLE);
		
			
			NVIC_SetPriority(QEI0_IRQn, 7);
			NVIC_EnableIRQ(QEI0_IRQn);
					
			_DBG(sig_mode);
			cmdn_cnt=0;
			
		}
		else if (ch_rtn == mod_check[2])
		{
					
			while(1)
			{
				if (cmdn_cnt == 0)
				{
					HAL_SCU_PeriSRCCmd_1(PERI_QEI0 |PERI_QEI1 , ENABLE);
					_DBG("\r\n");
					cmdn_cnt=1;
				}
				
				/* index setting */
				_DBG(idx_sel);
				
				ch_itn = _DG;
				_DBC(ch_itn);
				_DBG("\r\n");
				
				while(ch_itn<'1' || ch_itn>'5')
				{
						_DBG("repeat input character !! \r\n"); 
						ch_itn = _DG;
						_DBC(ch_itn);
						_DBG("\r\n");
				}
				
				if(ch_itn == '1') HAL_QEI_ModeInit(QEI0, QEInMR_INXGATE_PASS_ALL, 0);
				else if(ch_itn == '2')HAL_QEI_ModeInit(QEI0, QEInMR_INXGATE_PASS_PHA0_PHB0, 0);
				else if(ch_itn == '3') HAL_QEI_ModeInit(QEI0, QEInMR_INXGATE_PASS_PHA0_PHB1, 0);
				else if(ch_itn == '4') HAL_QEI_ModeInit(QEI0, QEInMR_INXGATE_PASS_PHA1_PHB1, 0);
				else if(ch_itn == '5') HAL_QEI_ModeInit(QEI0, QEInMR_INXGATE_PASS_PHA1_PHB0, 0);			
				
				_DBG(tmenu_q_t);
				
				ch_rtn = _DG;
				
				_DBC(ch_rtn);
				_DBG("\r\n");
				
				if (ch_rtn == 0x0D)
				{
					_DBG("\r\n");
				}
				
				if (ch_rtn == tmd_check[1])	// Maximum Reset Check
				{
					_DBG("Maximum Reset Test !!\r\n");
					_DBG("[VLT : 0xFFFF, POS : 0xFFFFFFFF, IDX : 0xFFFF]\r\n");
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDVEL_ON | QEInMR_DIRI_ON |QEInMR_DIRPC_ON |  QEInMR_QDSWAP_ON | QEInMR_QDMOD_ON), ENABLE);
					
					while(1)
					{
						_DBG("VLT : 0x"); _DBH16(QEI0->VLT);  _DBG("\t");
						_DBG("POS : 0x"); _DBH32(QEI0->POS);  _DBG("\t");
						_DBG("IDX : 0x"); _DBH16(QEI0->IDX); _DBG("\r");
					}	
				}
				else if (ch_rtn == tmd_check[2])		// Position, Index Counter Direction Check
				{
					_DBG(di_menu);
					
					/* Index counter Direction Setting (0:None, 1:change direction) */
					ch_i = _DG;
					_DBC(ch_i);
					_DBG("\r\n");
					
					while(ch_i<'0' || ch_i>'1')
					{
						_DBG("repeat input character !! \r\n"); 
						ch_i = _DG;
						_DBC(ch_i);
						_DBG("\r\n");
					}
					
					if(ch_i=='0')
					{
						__NOP();
					}
					else if(ch_i=='1')
					{
							HAL_QEI_ModeCmd(QEI0, (QEInMR_DIRI_ON), ENABLE);
					}		
					
					_DBG(dp_menu);
					
					/* Position counter Direction Setting (0:None, 1:change direction) */
					ch_p = _DG;
					_DBC(ch_p);
					_DBG("\r\n");
					
					while(ch_p<'0' || ch_p>'1')
					{
						_DBG("repeat input character !! \r\n"); 
						ch_p = _DG;
						_DBC(ch_p);
						_DBG("\r\n");
					}
					
					if(ch_p=='0')
					{
						__NOP();
					}
					else if(ch_p=='1')
					{
							HAL_QEI_ModeCmd(QEI0, (QEInMR_DIRPC_ON), ENABLE);
					}		

					_DBG(smenu);
					
					/* SWAP Setting (0:None, 1:Ph-a, Ph-b Signal Swap) */
					ch_s = _DG;
					_DBC(ch_s);
					_DBG("\r\n");
					
					while(ch_s<'0' || ch_s>'1')
					{
						_DBG("repeat input character !! \r\n"); 
						ch_s = _DG;
						_DBC(ch_s);
						_DBG("\r\n");
					}
					
					if(ch_s=='0')
					{
						__NOP();
					}
					else if(ch_s=='1')
					{
							HAL_QEI_ModeCmd(QEI0, (QEInMR_QDSWAP_ON), ENABLE);
					}	
					
					// Position Counter Max Value Setting
					HAL_QEI_SetMaximumCNT(QEI0, 0x50000);
					
					_DBG(" [ POSMAX : 0x50000 / "); _DBG("DIRI : ");
					if(ch_i=='0'){
						_DBG("0 / ");
					}
					else{
						_DBG("1 / ");
					}
					_DBG("DIRPC : ");
					if(ch_p=='0')
					{
						_DBG("0 / ");
					}
					else{
						_DBG("1 / ");
					}
					_DBG("SWAP : ");
					if(ch_s=='0')
					{
						_DBG("0 ] \r\n");
					}
					else{
						_DBG("1 ] \r\n");
					}
					
					// QEI Enable
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), ENABLE);
					
					while(1)
					{
						_DBG("   Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward \t");
						}
						else{
							_DBG("Reverse \t");
						}
						_DBG("POS : 0x"); _DBH32(QEI0->POS);  _DBG("\t");
						_DBG("IDX : 0x"); _DBH32(QEI0->IDX); _DBG("\r");
					}	
				}
				else if (ch_rtn == tmd_check[3])		// Direction Interrupt Check
				{
					// Direction Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_DIR_ON), ENABLE);	
					
					// QEI Enable
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), ENABLE);
					
					_DBG(d_mode);
					
					while(1)
					{
						_DBG("   Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward \t");
						}
						else{
							_DBG("Reverse \t");
						}
						_DBG("POS : 0x"); _DBH32(QEI0->POS);  _DBG("\r");
						
						if(dir_check==1)
						{
							_DBG("\r\n Dircetion Change Interrupt Ok !! \r\n");
							dir_check=0;
							cmdn_cnt=0;
							break;
						}
					}	
				}
				else if (ch_rtn == tmd_check[4]) 					// Signal Mode Check
				{
					_DBG(s_mode);
						
					// Signal Mode Setting and QEI Enable
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDSIG_ON |QEInMR_QDMOD_ON), ENABLE); 		
					
					while(1)
					{
						_DBG("   Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward \t");
						}
						else{
							_DBG("Reverse \t");
						}
						
						_DBG("UPDN : "); 
						if((PA->IDR&(1<<12))==(1<<12)){
							_DBG("1 \r");
						}
						else{
							_DBG("0 \r");
						}
					}
				}
				
				else if (ch_rtn == tmd_check[5])					// Capture Mode Check
				{
						_DBG(cmenu);
									
					// Position Compare Bit Setting
					HAL_QEI_SetCompareCNT(QEI0, 0xFFFF,0,0);	
					
					// Position0 Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_POS0_ON), ENABLE);				

					// Capture Setting (0:2X, 1:4X)
					ch_s = _DG;
					_DBC(ch_s);
					_DBG("\r\n");
					
					if(ch_s=='0')
					{
							HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), ENABLE);
					}
					else
					{
							HAL_QEI_ModeCmd(QEI0, (QEInMR_QDCAP_ON | QEInMR_QDMOD_ON), ENABLE);
					}
					
					cap_check=1;
					
					HAL_TIMER_Cmd(TIMER3,ENABLE);
					
					while(1)
					{
						_DBG("Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward	");
						}
						else{
							_DBG("Reverse	");
						}
						
						_DBG("POS : 0x"); _DBH32(QEI0->POS); _DBG("\r");
						
						if(cap_check==0) {
							timer_val = TIMER3->CNT;
							HAL_TIMER_Cmd(TIMER3,DISABLE);
							_DBG("\r\nTIME : "); _DBH32(timer_val); _DBG("\r\n");
							break;
						}
					}	
					cmdn_cnt=0;
				}
				else if (ch_rtn == tmd_check[6])		// Position Conter Test
				{
					// Counter - 0, 1, 2, 3, ..., MAX, Reset with MAXCNT
					_DBG("Direction : Forward, Reset with MAXCNT[0xB0000]\r\n");
							
					// MAX Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_MAX_ON), ENABLE);
				
					// Max Counter Setting
					HAL_QEI_SetMaximumCNT(QEI0, 0xB0000);
					
					// QEI Enable
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), ENABLE);
					
					while(pos_check!=1){
						_DBG(" Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward	");
						}
						else{
							_DBG("Reverse	");
						}
						_DBG("POS : 0x"); _DBH32(QEI0->POS); _DBG("\r");
						}
			
					_DBG("\r\n MAXCNT Interrupt ok !! \r\n\r\n");
						
					// QEI Disable
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), DISABLE);
					
					// Position conter Reset
					HAL_QEI_FunctionCmd(QEI0, QEInCON_RESP_ON, ENABLE);
			
					// Counter - 0, 1, 2, 3, ..., MAX, Reverse Counter
					_DBG("Direction : Forward, Reverse with MAXCNT[0xB0000]\r\n");
					_DBG("1, 2, 3, ..., MAX, MAX-1, MAX-2, ..., 0\r\n");
					
					// Max counter Setting
					HAL_QEI_SetMaximumCNT(QEI0, 0xB0000);
					
					// MAX Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_MAX_ON), ENABLE);
					
					//QEI Enable
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), ENABLE);		
					
					while(pos_check!=2){
						_DBG(" Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward	");
						}
						else{
							_DBG("Reverse	");
						}
						_DBG("POS : 0x"); _DBH32(QEI0->POS); _DBG("\r");
						}
					
						_DBG("\r\n MAXCNT Interrupt ok !! \r\n\r\n");
						_DBG("Direction : Reverse, QEI Input Signal[Ph-A ¡Ë¢çe Ph-B]\r\n");
						
					// POS Compare Setting 
					HAL_QEI_SetCompareCNT(QEI0, 0x0, 0x0, 0x0);
						
					// POS0 Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_POS0_ON), ENABLE);	
					
						while(pos_check!=3){
						_DBG(" Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward	");
						}
						else{
							_DBG("Reverse	");
						}
						_DBG("POS : 0x"); _DBH32(QEI0->POS); _DBG("\r");
					}
						_DBG("\r\n POS0 Interrupt ok !! \r\n\r\n");
						_DBG ("\r\n***********FINISH POSITION RESET TEST OK*****************\r\n");		
							pos_check=0;
					
							cmdn_cnt=0;

				}
				else if (ch_rtn == tmd_check[7])							// Index Pulse Reset
				{		
							
					_DBG ("Direction : Forward, POSCNT Reset Value[0] \r\n");
					_DBG(" - Index Count up to 0x1000 \r\n");
					_DBG ("  [Poscnt value is reset to 0 when Index Pulse Occurs]\r\n");					
					
					// Index gating Setting
					HAL_QEI_IDXGatingCmd(QEI0, QEInMR_INXGATE_PASS_ALL);
					
					// Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_IDX_ON), ENABLE);
					
					// Index counter init
					HAL_QEI_FunctionCmd(QEI0, QEInCON_RESI_ON, ENABLE);
					
					// Compare Index Setting
					HAL_QEI_SetIndexCompareCNT(QEI0, 0x1000);
					
					// Mode Setting
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDRST_ON | QEInMR_QDMOD_ON), ENABLE);
					
					while(idx_check!=1)
					{
						_DBG("   Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward \t");
						}
						else{
							_DBG("Reverse \t");
						}
						_DBG("POS : 0x"); _DBH32(QEI0->POS);  _DBG(" ");
						_DBG("IDX : 0x"); _DBH32(QEI0->IDX); _DBG("\r");
					}	
					
					idx_check=0;
					
					_DBG ("\r\n   Index Pulse 0x1000 Occured !!  \r\n\r\n");
							
					_DBG ("Direction : Reverse, POSCNT Reset Value[POSMAX] \r\n");
					_DBG(" - POSMAX : 0x5000 \r\n");
					_DBG(" - Index Count up to 0x1000 \r\n");
					_DBG ("  [Poscnt value is reset to POSMAX when Index Pulse Occurs]\r\n");	
				
					// Index Counter Reverse
					
					// Max counter Setting
					HAL_QEI_SetMaximumCNT(QEI0, 0x5000);
					HAL_QEI_SetIndexCompareCNT(QEI0, 0x1000);
								
					// Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_IDX_ON), ENABLE);
					
					// Function Setting
					HAL_QEI_FunctionCmd(QEI0, (QEInCON_RESI_ON | QEInCON_RESP_ON) , ENABLE);
					
					// Mode Setting
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDRST_ON | QEInMR_DIRPC_ON | QEInMR_QDSWAP_ON | QEInMR_QDMOD_ON), ENABLE);
					
					while(idx_check!=1)
					{
						_DBG("   Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward \t");
						}
						else{
							_DBG("Reverse \t");
						}
						_DBG("POS : 0x"); _DBH32(QEI0->POS);  _DBG(" ");
						_DBG("IDX : 0x"); _DBH32(QEI0->IDX); _DBG("\r");
					}	
					
					idx_check=0;
					
					_DBG ("\r\n   Index Pulse 0x1000 Occured !!  \r\n\r\n");
					_DBG ("***********INDEX PULSE RESET TEST OK*****************\r\n\r\n");
				
					cmdn_cnt=0;
	
				}
				else if (ch_rtn == tmd_check[8])		// Velocity Counter Test
				{
					_DBG ("***********Velocify Conter Test*****************\r\n");
					_DBG (" * VLR : 0xFFFF, reloaded VLT when the VLT value becomes zero \r\n");
					_DBG(" * VLCOM = 0x0400, Interrupt occurs when VLC<VLCOM  \r\n");
					
					// Velocity Pulse Counter
					HAL_QEI_SetVelocityPulse(QEI0, 0x0000);
					
					// Velocity Reload Counter
					HAL_QEI_SetVelocityReload(QEI0, 0xFFFF);
					
					// Function Setting
					HAL_QEI_FunctionCmd(QEI0, QEInCON_RESV_ON, ENABLE);
					
					// Velocity Capture Counter
					HAL_QEI_SetVelocityCapture(QEI0, 0xffff);
					
					// Velocity Compare Setting
					HAL_QEI_SetVelocityCompare(QEI0, 0x400);
					
					// Velocity Timer Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_VELT_ON), ENABLE);								
					
					// Velocity Timer Mode Setting and QEI Enable
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDVEL_ON | QEInMR_QDMOD_ON), ENABLE);

					while(vel_check!=1)
					{
						_DBG("    - VLT : 0x"); _DBH32(QEI0->VLT); _DBG(" ");
						_DBG("   VLP : 0x"); _DBH32(QEI0->VLP); _DBG(" ");
						_DBG("   VLC : 0x"); _DBH32(QEI0->VLC); _DBG("\r");
					}	
					
					_DBG("\r\nVELOCITY TIMER INTERRUPT OK !! \r\n");
					
					// Velocity Compare Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_VELC_ON), ENABLE);	
					
					while(vel_check!=2);
					
					vel_check=0;
					
					_DBG("    - VLT : 0x"); _DBH32(QEI0->VLT); _DBG(" ");
					_DBG("   VLP : 0x"); _DBH32(QEI0->VLP); _DBG(" ");
					_DBG("   VLC : 0x"); _DBH32(QEI0->VLC); _DBG("\r");
					_DBG("\r\nVELOCITY COMPARE INTERRUPT ENABLE . . . \r\n");
					_DBG("VELOCITY COMPARE INTERRUPT OK!!\r\n");
						
					// Velocity Counter Reset
					HAL_QEI_FunctionCmd(QEI0, QEInCON_RESV_ON, ENABLE);
					
					cmdn_cnt=0;
					
				}
				else if (ch_rtn == '9')
				{
					_DBG("POS0, POS1, POS2, MAX, INX  Interrupt Test \r\n");
					_DBG("POS0 : 0x50000, POS1 : 0x90000, POS2 : 0xB0000, MAX : 0xF0000 \r\n");
								
					// Compare Bit Setting
					HAL_QEI_SetCompareCNT(QEI0, 0x50000, 0x90000, 0xB0000);
					
					// QEI Max Counter Setting
					HAL_QEI_SetMaximumCNT(QEI0, 0xF0000);
								
					// POS0, POS1, POS2, MAX Interrupt Enable
					HAL_QEI_InterruptCmd(QEI0, (QEInIER_POS0_ON | QEInIER_POS1_ON | QEInIER_POS2_ON | QEInIER_MAX_ON), ENABLE);

					// QEI Enable
					HAL_QEI_ModeCmd(QEI0, (QEInMR_QDMOD_ON), ENABLE);
					
					while(1)
					{
						_DBG("   Direction : ");
						if((QEI0->SR&(1<<1))==(1<<1)){
							_DBG("Forward \t");
						}
						else{
							_DBG("Reverse \t");
						}
						_DBG("POS : 0x"); _DBH32(QEI0->POS); 
						if(int_check==1) 	_DBG(" POS0 INTERRUPT OK !! ");
						else if(int_check==2) 	_DBG(" POS1 INTERRUPT OK !! ");
						else if(int_check==3) 	_DBG(" POS2 INTERRUPT OK !! ");
						else if(int_check==4) 	{
							_DBG(" MAX INTERRUPT OK !!");
							
							// POS0, POS1, POS2, MAX Interrupt Disable
							HAL_QEI_InterruptCmd(QEI0, (QEInIER_POS0_ON | QEInIER_POS1_ON | QEInIER_POS2_ON | QEInIER_MAX_ON), DISABLE);

						// INX Interrupt Enable
							HAL_QEI_InterruptCmd(QEI0, QEInIER_INX_ON, ENABLE);
						}
						else if(int_check==5) {
							// INX Interrupt Disable
							HAL_QEI_InterruptCmd(QEI0, QEInIER_INX_ON, DISABLE);
							_DBG(" INX INTERRUPT OK !! ");
							break;
						}
						_DBG("\r");
					}	

					_DBG("\r\n--POS0, POS1, POS2, MAX, INX  INTERRUPT TEST OK--\r\n");
					
					int_check=0;
					
					cmdn_cnt=0;
										
					}
				
				HAL_SCU_PeriSRCCmd_1(PERI_QEI0 | PERI_QEI1, DISABLE);
			}
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

	QEI_TestRun();
	
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

