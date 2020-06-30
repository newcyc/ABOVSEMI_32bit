/***************************************************************************//**
* @file     main.c
* @brief    An example of I2C on A31G22x
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

/* Includes ------------------------------------------------------------------- */
#include "A31G22x_i2c.h"
#include "A31G22x_pcu.h"
#include "A31G22x_scu.h"

#include "debug_frmwrk.h"


//-------------------------
#define USED_CLKO
//-------------------------
#define USED_HSI
//#define USED_LSI
//#define USED_MOSC
//#define USED_SOSC
//#define USED_MOSCPLL
//#define USED_HSIPLL

uint32_t msec;

// system default I/O setting
void PCU_Init(void);
void SCU_ClockInit(void);

#define M24C02_ADDR    (0xA0>>1)
/** Own Slave address in Slave I2C device */
#define I2CDEV_S_ADDR	(0xC0>>1)

/** Max buffer length */
#define BUFFER_SIZE			10

Bool complete;
Bool isMasterMode = TRUE;
/** These global variables below used in interrupt mode - Slave device ----------------*/
uint8_t Master_Buf[BUFFER_SIZE];

const uint8_t menu[] =
"\r\n************************************************\r\n"
"I2C demo\r\n"
"\t - MCU: A31G22x\r\n"
"\t - Core: ARM Cortex-M0+\r\n"
"\t - Communicate via: USART10 - 38400 bps\r\n"
"\r\nPress 1-4 to select I2C running mode:\r\n\r\n"
"\t 1 - I2C Polling   Master\r\n"
"\t 2 - I2C Interrupt Master\r\n"
"\t 3 - I2C Polling   Slave\r\n"
"\t 4 - I2C Interrupt Slave\r\n"
"************************************************\r\n";
const uint8_t  menu1[] = "\r\n \t - Press x to exit this mode!\r\n";
const uint8_t  menu3[] = "\t - Press c to continue...\r\n";

void I2C0_IRQHandler(void)
{
	if (isMasterMode)// Master Mode
	{
		I2C_Interrupt_MasterHandler(I2C0);
		if (I2C_MasterTransferComplete(I2C0)){
			complete = TRUE;
		}
	}
	else// Slave Mode
	{
		I2C_Interrupt_SlaveHandler(I2C0);
		if (I2C_SlaveTransferComplete(I2C0))
		{
			complete = TRUE;
		}
	}	
}

/**********************************************************************
 * @brief		Print menu
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void print_menu(void)
{
	_DBG(menu);
}

/*********************************************************************//**
 * @brief		Initialize buffer
 * @param[in]	buffer	buffer to initialize
 * @param[in]	type:
 * 				- 0: Initialize buffer with 0
 * 				- 1: Initialize buffer with increment value from 0
 * @return 		None
 **********************************************************************/
void Buffer_Init(uint8_t* buffer, uint8_t type)
{
	uint8_t i;

	if (type) {
		for (i = 0; i < BUFFER_SIZE; i++) {
			buffer[i] = i;
		}
	}
	else	{
		for (i = 0; i < BUFFER_SIZE; i++) {
			buffer[i] = 0;
		}
	}
}



/*********************************************************************//**
 * @brief		I2C master application in polling mode
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void App_I2C_Polling_Master(void)
{
	I2C_M_SETUP_Type transferMCfg;
	uint8_t buffer = 0xFF, regAddr = 0x00;
	uint8_t Master_Buf[BUFFER_SIZE];

	while (1) {
		_DBG_("\r\n\r\nI2C Polling Master is running...\r\n");
		_DBG_(menu1);
		_DBG_(menu3);

		while (1)
		{
			buffer = _DG;
			if (buffer == 'x' || buffer == 'X')
			{
				return;
			}
			else if (buffer == 'c' || buffer == 'C')
			{
				break;
			}
		}
		/* Transmit -------------------------------------------------------- */
		_DBG_("\r\nPress '1' to transmit");
		while (1)
		{
			buffer = _DG;
			if (buffer == '1')
				break;
		}
		_DBG_("\r\nTransmitting...");

		/* Initialize buffer */
		Buffer_Init(Master_Buf, 0);
		Master_Buf[0]=0x00; //address
		Master_Buf[1]=0xa2; //data	
		Master_Buf[2]=0xa4; //data	
		Master_Buf[3]=0xa6; //data	
		Master_Buf[4]=0xa8; //data		
		Master_Buf[5]=0xaA; //data		
		transferMCfg.sl_addr7bit = M24C02_ADDR;
		transferMCfg.tx_data = Master_Buf;
		transferMCfg.tx_length = 6;
		I2C_MasterTransmitData(I2C0, &transferMCfg, I2C_TRANSFER_POLLING);

		/* Transmit and receive -------------------------------------------------------- */
		_DBG_("\r\nPress '2' to Transmit, then repeat start and receive...");
		while (1)
		{
			buffer = _DG;
			if (buffer == '2')
				break;
		}

		/* Initialize buffer */
		Buffer_Init(Master_Buf, 0);

		transferMCfg.sl_addr7bit = M24C02_ADDR;
		transferMCfg.tx_data = &regAddr;
		transferMCfg.tx_length = 1;
		transferMCfg.rx_data = Master_Buf;
		transferMCfg.rx_length = 5;
		transferMCfg.tx_count = 0;
		transferMCfg.rx_count = 0;
		I2C_MasterTransferData(I2C0, &transferMCfg, I2C_TRANSFER_POLLING);
		__NOP();
	}
}

/***********************************************************************
 * @brief		I2C master application using interrupt
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void App_I2C_Interrupt_Master(void)
{
	I2C_M_SETUP_Type transferMCfg;
	uint8_t buffer = 0xFF, regAddr = 0;
	uint8_t Master_Buf[BUFFER_SIZE];

	while (1) {
		_DBG_("\r\n\r\nI2C Interrupt Master is running...\r\n");
		_DBG_(menu1);
		_DBG_(menu3);

		while (1)
		{
			buffer = _DG;
			if (buffer == 'x' || buffer == 'X')
			{
				return;
			}
			else if (buffer == 'c' || buffer == 'C')
			{
				break;
			}
		}
		/* Transmit -------------------------------------------------------- */
		_DBG_("\r\nPress '1' to transmit");
		while (1)
		{
			buffer = _DG;
			if (buffer == '1')
				break;
		}
		_DBG_("\r\nTransmitting...");

		/* Initialize buffer */
		complete = FALSE;
		Buffer_Init(Master_Buf, 0);
		Master_Buf[0]=0x00; //address
		Master_Buf[1]=0x2A; //data	
		Master_Buf[2]=0x4A; //data	
		Master_Buf[3]=0x6A; //data	
		Master_Buf[4]=0x8A; //data		
		Master_Buf[5]=0xA5; //data		
		transferMCfg.sl_addr7bit = M24C02_ADDR;
		transferMCfg.tx_data = Master_Buf;
		transferMCfg.tx_length = 6;
		I2C_MasterTransmitData(I2C0, &transferMCfg, I2C_TRANSFER_INTERRUPT);
		while(!complete);

		/* Transmit and receive -------------------------------------------------------- */
		_DBG_("\r\nPress '2' to Transmit, then repeat start and receive...");
		while (1)
		{
			buffer = _DG;
			if (buffer == '2')
				break;
		}

		/* Initialize buffer */
		complete = FALSE;
		Buffer_Init(Master_Buf, 0);
		transferMCfg.sl_addr7bit = M24C02_ADDR;
		transferMCfg.tx_data = &regAddr;
		transferMCfg.tx_length = 1;
		transferMCfg.rx_data = Master_Buf;
		transferMCfg.rx_length = 5;
		transferMCfg.tx_count = 0;
		transferMCfg.rx_count = 0;
		
		I2C_MasterTransferData(I2C0, &transferMCfg, I2C_TRANSFER_INTERRUPT);
		while(!complete);
		__NOP();
	}
}

/**********************************************************************
 * @brief		I2C slave application in polling mode
 * @param[in]	None
 * @return 		None
 **********************************************************************/
uint8_t dispSlave_Buf[BUFFER_SIZE];
void App_I2C_Polling_Slave (void)
{
	I2C_S_SETUP_Type transferSCfg;
	uint8_t buffer = 0xFF,i=0,j;
	uint8_t Slave_Buf[BUFFER_SIZE], S_buff[2];

	while (1) {
		_DBG_("\r\n\r\nI2C Polling Slave is running...\r\n");
		_DBG_(menu1);
		_DBG_(menu3);

		while (1)
		{
			buffer = _DG;
			if (buffer == 'x' || buffer == 'X')
			{
				return;
			}
			else if (buffer == 'c' || buffer == 'C')
			{
				break;
			}
		}
		/* Receive -------------------------------------------------------- */
		_DBG_("\r\nPress '1' to Start");

		while (1)
		{
			buffer = _DG;
			if (buffer == '1')
				break;
		}
		_DBG_("\r\nReceiving...");

		/* Initialize buffer */
		Buffer_Init(Slave_Buf, 0);

		transferSCfg.rx_data = Slave_Buf;
		transferSCfg.rx_length = 3;
		I2C_SlaveReceiveData(I2C0, &transferSCfg, I2C_TRANSFER_POLLING);
		for (j=0;j<BUFFER_SIZE;j++)
		{
			dispSlave_Buf[j]=Slave_Buf[j];
		}
		__NOP();

		/* Receive and transmit -------------------------------------------------------- */
		_DBG_("\r\nStart Receive, wait for repeat start and transmit...");

		/* Initialize buffer */
		Buffer_Init(Slave_Buf, 1);

		transferSCfg.rx_data = S_buff;
		transferSCfg.rx_length = 2;
		Slave_Buf[0]=((i++)&0xff);//0x8A; //data	
		Slave_Buf[1]=0xAA; //data		
		Slave_Buf[2]=0xA5; //data	
		transferSCfg.tx_data = Slave_Buf;
		transferSCfg.tx_length = 3;
		I2C_SlaveTransferData(I2C0, &transferSCfg, I2C_TRANSFER_POLLING);
		__NOP();		
	}
}

/*********************************************************************//**
 * @brief		I2C slave application using interrupt
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void App_I2C_Interrupt_Slave (void)
{
	I2C_S_SETUP_Type transferSCfg;
	uint8_t buffer = 0xFF;
	uint8_t Slave_Buf[BUFFER_SIZE], S_buff[2];

	while (1) {
		_DBG_("\r\n\r\nI2C Interrupt Slave is running...\r\n");
		_DBG_(menu1);
		_DBG_(menu3);

		while (1)
		{
			buffer = _DG;
			if (buffer == 'x' || buffer == 'X')
			{
				return;
			}
			else if (buffer == 'c' || buffer == 'C')
			{
				break;
			}
		}
		/* Receive -------------------------------------------------------- */
		_DBG_("\r\nPress '1' to Start");

		while (1)
		{
			buffer = _DG;
			if (buffer == '1')
				break;
		}
		_DBG_("\r\nReceiving...");

		/* Initialize buffer */
		Buffer_Init(Slave_Buf, 0);

		complete = FALSE;
		transferSCfg.rx_data = Slave_Buf;
		transferSCfg.rx_length = 3;
		I2C_SlaveReceiveData(I2C0, &transferSCfg, I2C_TRANSFER_INTERRUPT);
		 while(!complete);
		__NOP();
		
		/* Receive and transmit -------------------------------------------------------- */
		_DBG_("\r\nStart Receive, wait for repeat start and transmit...");

		/* Initialize buffer */
		Buffer_Init(Slave_Buf, 0);

		complete = FALSE;
		
		transferSCfg.rx_data = S_buff;
		transferSCfg.rx_length = 2;
		Slave_Buf[0]=0x6A; //data	
		Slave_Buf[1]=0x8A; //data		
		Slave_Buf[2]=0xA5; //data		
		transferSCfg.tx_data = Slave_Buf;
		transferSCfg.tx_length = 3;
		I2C_SlaveTransferData(I2C0, &transferSCfg, I2C_TRANSFER_INTERRUPT);
		 while(!complete);
		__NOP();
	}
}


void mainloop(void)
{
	uint8_t buffer = 0xFF;
	
	// Test Pin setting PE0
	PCU_ConfigureDirection(PE, 0, PCU_MODE_PUSH_PULL);
	PCU_ConfigurePullupdown(PE, 0, PCU_PUPD_DISABLE);	
	GPIO_ClearValue(PE, _BIT(0));
	
	// I2C0 PD0:SCL0, PD1:SDA0	
//	PCU_ConfigureDirection(PD, 0, PCU_MODE_ALT_FUNC);
//	PCU_ConfigureFunction(PD, 0, PCU_ALT_FUNCTION_1);
//	PCU_ConfigureDirection(PD, 1, PCU_MODE_ALT_FUNC);
//	PCU_ConfigureFunction(PD, 1, PCU_ALT_FUNCTION_1);
//	PCU_ConfigureDebounce(PD, 0, ENABLE);
//	PCU_ConfigureDebounce(PD, 1, ENABLE);	
	// I2C0 PF6:SCL0, PF7:SDA0
	PCU_ConfigureDirection((PORT_Type*)PF, 6, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction((PORT_Type*)PF, 6, PCU_ALT_FUNCTION_2);
	PCU_ConfigureDirection((PORT_Type*)PF, 7, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction((PORT_Type*)PF, 7, PCU_ALT_FUNCTION_2);
//	PCU_ConfigureDebounce((PORT_Type*)PF, 6, ENABLE);
//	PCU_ConfigureDebounce((PORT_Type*)PF, 7, ENABLE);		
	
	// I2C1 PA1:SCL1,	PA0:SDA1
//	PCU_ConfigureDirection(PA, 0, PCU_MODE_ALT_FUNC);
//	PCU_ConfigureFunction(PA, 0, PCU_ALT_FUNCTION_1);	
//	PCU_ConfigureDirection(PA, 1, PCU_MODE_ALT_FUNC);
//	PCU_ConfigureFunction(PA, 1, PCU_ALT_FUNCTION_1);	
//	PCU_ConfigureDebounce(PA, 1, ENABLE);
//	PCU_ConfigureDebounce(PA, 0, ENABLE);	
	
	// I2C2 PC6:SCL2,	PC5:SDA2
//	PCU_ConfigureDirection(PC, 5, PCU_MODE_ALT_FUNC);
//	PCU_ConfigureFunction(PC, 5, PCU_ALT_FUNCTION_1);	
//	PCU_ConfigureDirection(PC, 6, PCU_MODE_ALT_FUNC);
//	PCU_ConfigureFunction(PC, 6, PCU_ALT_FUNCTION_1);	
//	PCU_ConfigureDebounce(PC, 6, ENABLE);
//	PCU_ConfigureDebounce(PC, 5, ENABLE);	
	
	/* I2C block ------------------------------------------------------------------- */
	// Initialize Slave I2C peripheral
	I2C_Init(I2C0, 100000);

	/* Set  Own slave address for I2C device */
	I2C_SetOwnSlaveAddr0(I2C0, I2CDEV_S_ADDR, DISABLE);
	I2C_SetOwnSlaveAddr1(I2C0, I2CDEV_S_ADDR, DISABLE);
	
	__enable_irq();	
	
	print_menu();
	
	while(1){
		buffer = _DG;
		
		switch (buffer)
		{
			case '1': //Master Polling Mode
				App_I2C_Polling_Master();
				print_menu();
				break;
			case '2': //Master Interrupt Mode
				isMasterMode = TRUE;
				App_I2C_Interrupt_Master();
				print_menu();
				break;
			case '3': //Slave Polling Mode
				App_I2C_Polling_Slave();
				print_menu();
				break;
			case '4': //Slave Interrupt Mode
				isMasterMode = FALSE;
				App_I2C_Interrupt_Slave();
				print_menu();
				break;
			default:
				break;
		}
	}
}

int main(void)
{
	__disable_irq();
	SystemInit();
	PCU_Init(); // all port control setting  !!You have to set PORT function!!
	SCU_ClockInit(); 

	DEBUG_Init(DEBUG_INTERFACE_PERI);
	__enable_irq();
	
	mainloop();
	
	return (0);
}

/**********************************************************************
 * @brief		Initialize default clock for A31G22x
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SCU_ClockInit(void)
{
	uint32_t i;

//	CLKO function setting. check PORT setting (PF4).

#ifdef USED_CLKO
	SCU_SetCOR(4,ENABLE); //    /10
#else
	SCU_SetCOR(4,DISABLE);
#endif

	SCU->CMR&=~(1<<7); //mclk monitoring disable
	
#ifdef USED_LSI			//500khz
	SCU_SetLSI(LSI_EN); //LSI_EN_DIV2, LSI_EN_DIV4
	SystemCoreClock=500000; //500khz
	SystemPeriClock=500000; //500khz	
	
	for (i=0;i<10;i++);	
	
	SCU_ChangeSysClk(SCCR_LSI);
#endif 
	
#ifdef USED_SOSC  //32.768khz
	SCU_SetLSE(LSE_EN);
	SystemCoreClock=32768; //32.768khz
	SystemPeriClock=32768; //32.768khz	

	for (i=0;i<100;i++);	

// wait for SOSC stable	
	SCU_WaitForLSEStartUp();
	SCU_ChangeSysClk(SCCR_LSE);
#endif 	

#ifdef USED_HSI		//48MHz
	SCU_SetHSI(HSI_EN);
	SystemCoreClock=48000000; //48MHz
	SystemPeriClock=48000000; //48MHz	
	
	for (i=0;i<10;i++);	

	SCU_ChangeSysClk(SCCR_HSI);
#endif 	

#ifdef USED_MOSC	//xMHz
	SCU_SetHSE(HSE_EN);
	SystemCoreClock=8000000; //xMHz
	SystemPeriClock=8000000; //xMHz	

	for (i=0;i<100;i++);	

	SCU_WaitForHSEStartUp();
	SCU_ChangeSysClk(SCCR_HSE);
#endif


#ifdef USED_MOSCPLL
// PLL setting 
//    FIN=PLLINCLK/(R+1)                                             ; R: Pre Divider   
//    FOUT=(FIN*(N1+1)*(D+1))  / ((N2+1)*(P+1))          ; N1: Post Divider1, N2:Post Divider2, P:Output Divider,      
//             = FVCO *(D+1)                                             ; D:Frequency Doubler
//
//ex)    FIN=PLLINCLK/(R+1) = 8M/(3+1) = 2M                               ; R:3, PLLINCLK:8MHz(MOSC)
//         FOUT=(2M*(47+1)*(0+1)) / ((1+1)*(0+1) = 48MHz              ; N1:47, D:0, N2:1, P:0
//
	if (SCU_SetPLLandWaitForPLLStartUp(ENABLE,  
		PLLCON_BYPASS_PLL,    //PLLCON_BYPASS_FIN:0, PLLCON_BYPASS_PLL:1
		0,                                    //0:FOUT==VCO, 1:FOUT==2xVCO,  D=0
//		3,                                    //PREDIV, R=3
		1,                                    //PREDIV, R=1
		47,                                  //POSTDIV1, N1=47  
		1,                                    //POSTDIV2, N2=1
		0)==ERROR)                    //OUTDIV P=0
	{
		while(1);
	}

//	EOSC -->  EOSCPLL
	SCU_ChangeSysClk(SCCR_HSE_PLL);
	
	SystemCoreClock=48000000; 
	SystemPeriClock=48000000; 
#endif

	
#ifdef USED_HSIPLL
// PLL setting 
//    FIN=PLLINCLK/(R+1)                                             ; R: Pre Divider   
//    FOUT=(FIN*(N1+1)*(D+1))  / ((N2+1)*(P+1))          ; N1: Post Divider1, N2:Post Divider2, P:Output Divider,      
//             = FVCO *(D+1)                                             ; D:Frequency Doubler
//
//ex)    FIN=PLLINCLK/(R+1) = 8M/(3+1) = 2M                               ; R:3, PLLINCLK:8MHz(MOSC)
//         FOUT=(2M*(47+1)*(0+1)) / ((1+1)*(0+1) = 48MHz              ; N1:47, D:0, N2:1, P:0
//
	if (SCU_SetPLLandWaitForPLLStartUp(ENABLE,  
		PLLCON_BYPASS_PLL,    //PLLCON_BYPASS_FIN:0, PLLCON_BYPASS_PLL:1
		0,                                    //0:FOUT==VCO, 1:FOUT==2xVCO,  D=0
//		3,                                    //PREDIV, R=3
		1,                                    //PREDIV, R=1
		47,                                  //POSTDIV1, N1=47  
		1,                                    //POSTDIV2, N2=1
		0)==ERROR)                    //OUTDIV P=0
	{
		while(1);
	}

//	EOSC -->  EOSCPLL
	SCU_ChangeSysClk(SCCR_HSI_PLL);
	
	SystemCoreClock=48000000; 
	SystemPeriClock=48000000; 
#endif

	SCU->CMR|=(1<<7); //mclk monitoring enable

// wait setting order 1. default wait setting -> 2. clock change -> 3. adjust wait setting
//// flash memory controller
	CFMC->MR = 0x81;       // after changing 0x81 -> 0x28 in MR reg, flash access timing will be able to be set.
	CFMC->MR = 0x28;       // enter flash access timing changing mode
//	CFMC->CFG = (0x7858<<16) | (0<<8);  //flash access cycles 	20
//	CFMC->CFG = (0x7858<<16) | (1<<8);  //flash access cycles 	40
	CFMC->CFG = (0x7858<<16) | (2<<8);  //flash access cycles 	60		
//	CFMC->CFG = (0x7858<<16) | (3<<8);  //flash access cycles 		
	                              // flash access time cannot overflow 20MHz.
	                              // ex) if MCLK=48MHz, 
	                              //       48/1 = 48 (can't set no wait)
	                              //       48/2 = 24 (1 wait is not ok)
	                              //       48/3 = 16 (2 wait is ok)								  
	                              // so, 2 wait is possible.
	CFMC->MR = 0;	      // exit flash access timing --> normal mode				
}


void PCU_Init(void)
{
//Peripheral Enable Register 1  0:Disable, 1:Enable	
	SCU->PER1=SCU->PER1  
	| (1<<13)   // GPIOF
	| (1<<12)   // GPIOE
	| (1<<11)   // GPIOD
	| (1<<10)   // GPIOC
	| (1<<9)   // GPIOB
	| (1<<8)   // GPIOA
		; 		
//Peripheral Clock Enable Register 1 0:Disable, 1:Enable	
	SCU->PCER1=SCU->PCER1
	| (1<<13)   // GPIOF
	| (1<<12)   // GPIOE
	| (1<<11)   // GPIOD
	| (1<<10)   // GPIOC
	| (1<<9)   // GPIOB
	| (1<<8)   // GPIOA
		; 	

 	PORT_ACCESS_EN();  // enable writing permittion of ALL PCU register

	//--------------------------------------------------------------
	//	PORT INIT
	//		PA, PB, PC, PD, PE, PF
	//--------------------------------------------------------------
	// PORT - A
	PA->MOD = 0					// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9

	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PA->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9

	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PA->AFSR1 = 0
	| (0x00<<28)						// P7		0 : SEG 42,	1 : 			, 2 :				, 3 : AN7/CREF0/DAVREF	, 4 : ISEG2	
	| (0x00<<24)						// P6		0 : SEG 43,	1 : 			, 2 :				, 3 : AN6/CREF1/DAO		, 4 : ISEG3	
	| (0x00<<20)						// P5		0 :			,	1 : T12O	, 2 : T12C 	, 3 : AN5/CP1A/DAO			, 4 : ISEG4
	| (0x00<<16)						// P4		0 :			,	1 : 			, 2 :				, 3 : AN4/CP1B/CS7			, 4 : ISEG5
	| (0x00<<12)						// P3		0 : 			,	1 : 			, 2 :				, 3 : AN3/CP1C/CS6			, 4 : ISEG6
	| (0x00<<8)						// P2		0 : 			,	1 : EC12	, 2 :				, 3 : AN2/AVREF/CP0/CS5, 4 : ISEG7
	| (0x00<<4)						// P1		0 : 			,	1 : SCL1	, 2 :				, 3 : AN1/USBDP/CS4		, 4 : ISEG8
	| (0x00<<0)						// P0		0 : 			,	1 : SDA1	, 2 :				, 3 : AN0/USBDM/CS3		, 4 : ISEG9
	;

	PA->AFSR2 = 0
	| (0x00<<12)						// P11		0 : 			,	1 : 			, 2 :				, 3 : AN14/CS2					, 4 : 	
	| (0x00<<8)						// P10		0 : 			,	1 : 			, 2 :				, 3 : AN13/CS1					, 4 : 	 
	| (0x00<<4)						// P9		0 : 			,	1 : 			, 2 :				, 3 : AN12/CS0					, 4 : 	 
	;
	
	PA->PUPD = 0				// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9

	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PA->OUTDR = 0				// 0 : Output Low,	1 : Output High
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9

	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;

//////////////////////////////////////////////////////////////////////////////////////////////////////
	// PORT - B
	PB->MOD = 0					// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<30)						// P15
	| (0x01<<28)						// P14
	| (0x01<<26)						// P13
	| (0x01<<24)						// P12
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9
	| (0x01<<16)						// P8
	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x02<<10)						// P5   SWDIO
	| (0x02<<8)						// P4   SWCLK
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PB->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<15)						// P15
	| (0x00<<14)						// P14
	| (0x00<<13)						// P13
	| (0x00<<12)						// P12
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PB->AFSR1 = 0
	| (0x01<<28)					// P7		0 : SEG 34,	1 : RXD1	, 2 : 				, 3 :						, 4 : ICOM19 
	| (0x01<<24)					// P6		0 : SEG 35,	1 : TXD1	, 2 :				, 3 :						, 4 : ICOM20
	| (0x02<<20)					// P5		0 : SEG 36,	1 : RXD0	, 2 : SWDIO	, 3 :						, 4 : ICOM21 
	| (0x02<<16)					// P4		0 : SEG 37,	1 : TXD0	, 2 : SWCLK	, 3 :						, 4 : ICOM22 
	| (0x01<<12)					// P3		0 : SEG 38,	1 : BOOT	, 2 : SS10		, 3 :						, 4 : ICOM23
	| (0x00<<8)					// P2		0 : SEG 39,	1 : 			, 2 : SCK10	, 3 : AN10/CS10	, 4 : ICOM24 
	| (0x00<<4)					// P1		0 : SEG 40,	1 : RXD10	, 2 : MISO10	, 3 : AN9/CS9		, 4 : ICOM25/ISEG0
	| (0x00<<0)					// P0		0 : SEG 41,	1 : TXD10	, 2 : MOSI10	, 3 : AN8/CS8		, 4 : ICOM26/ISEG1
	;

	PB->AFSR2 = 0
	| (0x0<<28)					// P15		0 : SEG 26,	1 :			, 2 :				, 3 : CS18			, 4 :			 
	| (0x0<<24)					// P14		0 : SEG 27,	1 : 			, 2 :				, 3 : CS17			, 4 :			 
	| (0x0<<20)					// P13		0 : SEG 28,	1 :			, 2 :				, 3 : CS16			, 4 :			 
	| (0x0<<16)					// P12		0 : SEG 29,	1 :			, 2 :				, 3 : CS15			, 4 :			 
	| (0x0<<12)					// P11		0 : SEG 30,	1 : T15C	, 2 : EC16		, 3 : CS14			, 4 : T15O	 
	| (0x0<<8)					// P10		0 : SEG 31,	1 : T16C	, 2 : EC15		, 3 : CS13   		, 4 : T16O	 
	| (0x0<<4)					// P9		0 : SEG 32,	1 : T16O	, 2 : T16C		, 3 : CS12    		, 4 : EC15	 
	| (0x0<<0)					// P8		0 : SEG 33,	1 : T15O	, 2 : T15C		, 3 : CS11    		, 4 : EC16	 
	;
	
	PB->PUPD = 0			// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<30)						// P15
	| (0x0<<28)						// P14
	| (0x0<<26)						// P13
	| (0x0<<24)						// P12
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9
	| (0x0<<16)						// P8
	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PB->OUTDR = 0			// 0 : Output Low,	1 : Output High
	| (0x00<<15)						// P15
	| (0x00<<14)						// P14
	| (0x00<<13)						// P13
	| (0x00<<12)						// P12
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8	
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;

//////////////////////////////////////////////////////////////////////////////////////////////////////
	// PORT - C
	PC->MOD = 0				// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<24)						// P12
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9
	| (0x01<<16)						// P8
	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PC->TYP = 0				// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<12)						// P12
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PC->AFSR1 = 0
	| (0x0<<28)					// P7		0 : SEG 18	, 1 :				, 2 :				, 3 :          	, 4 :				
	| (0x0<<24)					// P6		0 : SEG 19	, 1 : SCL2		, 2 :				, 3 :          	, 4 :				
	| (0x0<<20)					// P5		0 : SEG 20	, 1 : SDA2		, 2 :				, 3 :          	, 4 :				
	| (0x0<<16)					// P4		0 : SEG 21	, 1 :				, 2 :				, 3 : CS23 	, 4 : ICOM14  
	| (0x0<<12)					// P3		0 : SEG 22	, 1 : EC21		, 2 :				, 3 : CS22 	, 4 : ICOM15  
	| (0x0<<8)					// P2		0 : SEG 23	, 1 : EC20		, 2 :				, 3 : CS21 	, 4 : ICOM16  
	| (0x0<<4)					// P1		0 : SEG 24	, 1 : T21O		, 2 : T21C		, 3 : CS20 	, 4 : ICOM17  
	| (0x0<<0)					// P0		0 : SEG 25	, 1 : T20O		, 2 : T20C		, 3 : CS19 	, 4 : ICOM18  
	;
	PC->AFSR2 = 0
	| (0x0<<16)					// P12		0 : SEG 13	, 1 : EC11		, 2 :				, 3 : 				, 4 :		
	| (0x0<<12)					// P11		0 : SEG 14	, 1 : EC10		, 2 :				, 3 : 				, 4 :		
	| (0x0<<8)					// P10		0 : SEG 15	, 1 :				, 2 :				, 3 : 				, 4 :		
	| (0x0<<4)					// P9		0 : SEG 16	, 1 :				, 2 :				, 3 : 				, 4 :		
	| (0x0<<0)					// P8		0 : SEG 17	, 1 :				, 2 				, 3 : 				, 4 :		
	;
	
	PC->PUPD = 0				// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<24)						// P12
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9
	| (0x0<<16)						// P8
	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PC->OUTDR = 0			// 0 : Output Low,	1 : Output High
	| (0x0<<12)						// P12
	| (0x0<<11)						// P11
	| (0x0<<10)						// P10
	| (0x0<<9)						// P9
	| (0x0<<8)						// P8	
	| (0x0<<7)						// P7
	| (0x0<<6)						// P6
	| (0x0<<5)						// P5
	| (0x0<<4)						// P4
	| (0x0<<3)						// P3
	| (0x0<<2)						// P2
	| (0x0<<1)						// P1
	| (0x0<<0)						// P0
	;

//////////////////////////////////////////////////////////////////////////////////////////////////////
	// PORT - D
	PD->MOD = 0			// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PD->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PD->AFSR1 = 0
	| (0x00<<20)					// P5		0 : SEG 7	,	1 : 			, 2 : SS11			, 3 : 		, 4 : ICOM8				
	| (0x00<<16)					// P4		0 : SEG 8	,	1 :			, 2 : SCK11		, 3 : 		, 4 : ICOM9
	| (0x00<<12)					// P3		0 : SEG 9	,	1 : RXD11	, 2 : MISO11		, 3 : 		, 4 : ICOM10
	| (0x00<<8)					// P2		0 : SEG 10,	1 : TXD11	, 2 : MOSI11		, 3 : 		, 4 : ICOM11			
	| (0x00<<4)					// P1		0 : SEG 11,	1 : SDA0	, 2 :					, 3 : 		, 4 : ICOM12			 
	| (0x00<<0)					// P0		0 : SEG 12,	1 : SCL0	, 2 :					, 3 : 		, 4 : ICOM13				 
	;
	
	PD->PUPD = 0			// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PD->OUTDR = 0			// 0 : Output Low,	1 : Output High
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;

//////////////////////////////////////////////////////////////////////////////////////////////////////
	// PORT - E
	PE->MOD = 0			// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<30)						// P15
	| (0x01<<28)						// P14
	| (0x01<<26)						// P13
	| (0x01<<24)						// P12
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9
	| (0x01<<16)						// P8
	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x01<<10)						// P5
	| (0x01<<8)						// P4
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PE->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x00<<15)						// P15
	| (0x00<<14)						// P14
	| (0x00<<13)						// P13
	| (0x00<<12)						// P12
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
	PE->AFSR1 = 0
	| (0x00<<28)					// P7		0 : COM7/SEG4	, 1 : T11O			, 2 : T11C	, 3 :		, 4 : ICOM7	
	| (0x00<<24)					// P6		0 : COM6/SEG3	, 1 : T10O			, 2 : T10C	, 3 :		, 4 : ICOM6	
	| (0x00<<20)					// P5		0 : COM5/SEG2	, 1 : PWM30CB	, 2 :			, 3 :		, 4 : ICOM5	
	| (0x00<<16)					// P4		0 : COM4/SEG1	, 1 : PWM30CA	, 2 :			, 3 :		, 4 : ICOM4	
	| (0x00<<12)					// P3		0 : COM3/SEG0	, 1 : PWM30BB	, 2 :			, 3 :		, 4 : ICOM3	
	| (0x00<<8)					// P2		0 : COM2				, 1 : PWM30BA	, 2 :			, 3 :		, 4 : ICOM2	
	| (0x00<<4)					// P1		0 : COM1				, 1 : PWM30AB	, 2 :			, 3 :		, 4 : ICOM1	
	| (0x00<<0)					// P0		0 : COM0				, 1 : PWM30AA	, 2 :			, 3 :		, 4 : ICOM0	
	;
	PE->AFSR2 = 0
	| (0x00<<28)					// P15		0 : SEG6	, 1 :					, 2 : SS12			, 3 :				, 4 :		
	| (0x00<<24)					// P14		0 : SEG5	, 1 :					, 2 : SCK12		, 3 :				, 4 :		
	| (0x00<<20)					// P13		0 :			, 1 : RXD12		, 2 : MISO12		, 3 :				, 4 :		
	| (0x00<<16)					// P12		0 :			, 1 : TXD12		, 2 : MOSI12		, 3 :				, 4 :		
	| (0x00<<12)					// P11		0 :			, 1 :					, 2 : SS13			, 3 : VLC3		, 4 :		
	| (0x00<<8)					// P10		0 :			, 1 :					, 2 : SCK13		, 3 : VLC2		, 4 :		
	| (0x00<<4)					// P9		0 :			, 1 : RXD13		, 2 : MISO13		, 3 : VLC1		, 4 :		
	| (0x00<<0)					// P8		0 :			, 1 : TXD13		, 2 : MOSI13		, 3 : VLC0		, 4 :		
	;
	
	PE->PUPD = 0			// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<30)						// P15
	| (0x0<<28)						// P14
	| (0x0<<26)						// P13
	| (0x0<<24)						// P12
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9
	| (0x0<<16)						// P8
	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PE->OUTDR = 0				// 0 : Output Low,	1 : Output High
	| (0x0<<15)						// P15
	| (0x0<<14)						// P14
	| (0x0<<13)						// P13
	| (0x0<<12)						// P12
	| (0x0<<11)						// P11
	| (0x0<<10)						// P10
	| (0x0<<9)						// P9
	| (0x0<<8)						// P8
	| (0x0<<7)						// P7
	| (0x0<<6)						// P6
	| (0x0<<5)						// P5
	| (0x0<<4)						// P4
	| (0x0<<3)						// P3
	| (0x0<<2)						// P2
	| (0x0<<1)						// P1
	| (0x0<<0)						// P0
	;
	
//////////////////////////////////////////////////////////////////////////////////////////////////////	
	// PORT - F
	PF->MOD = 0					// 0 : Input Mode,	1 : Output Mode,	2 : Alternative function mode
	| (0x01<<22)						// P11
	| (0x01<<20)						// P10
	| (0x01<<18)						// P9
	| (0x01<<16)						// P8
	| (0x01<<14)						// P7
	| (0x01<<12)						// P6
	| (0x01<<10)						// P5
	| (0x02<<8)						// P4     clko
	| (0x01<<6)						// P3
	| (0x01<<4)						// P2
	| (0x01<<2)						// P1
	| (0x01<<0)						// P0
	;
	
	PF->TYP = 0					// 0 : Push-pull Output,	1 : Open-drain Output
	| (0x0<<11)						// P11
	| (0x0<<10)						// P10
	| (0x0<<9)						// P9
	| (0x0<<8)						// P8
	| (0x0<<7)						// P7
	| (0x0<<6)						// P6
	| (0x0<<5)						// P5
	| (0x0<<4)						// P4
	| (0x0<<3)						// P3
	| (0x0<<2)						// P2
	| (0x0<<1)						// P1
	| (0x0<<0)						// P0
	;
	
	PF->AFSR1 = 0
	| (0x0<<28)					// P7		0 :		, 1 : T30C		, 2 : SDA0		, 3 :				, 4 :				 
	| (0x0<<24)					// P6		0 :		, 1 : EC30		, 2 : SCL0		, 3 :				, 4 :				 
	| (0x0<<20)					// P5		0 :		, 1 : BLNK		, 2 : INT		, 3 :				, 4 :				 
	| (0x1<<16)					// P4		0 :		, 1 : CLKO		, 2 :				, 3 : R-SET	, 4 :				 
	| (0x0<<12)					// P3		0 :		, 1 : RXD1		, 2 :				, 3 : SXOUT	, 4 :				 
	| (0x0<<8)					// P2		0 :		, 1 : TXD1		, 2 :				, 3 : SXIN		, 4 :				 
	| (0x0<<4)					// P1		0 :		, 1 : SDA1		, 2 :				, 3 : XIN		, 4 :				 
	| (0x0<<0)					// P0		0 :		, 1 : SCL1		, 2 :				, 3 : XOUT	, 4 : ISEG10	 
	;
	PF->AFSR2 = 0
	| (0x0<<12)					// P11		0 : SXOUT, 1 : T14O		, 2 : T14C		, 3 : 			, 4 :				 
	| (0x0<<8)					// P10		0 : SXIN	, 1 : T13O		, 2 : T13C		, 3 : 			, 4 :				 
	| (0x0<<4)					// P9		0 : XIN		, 1 : EC14		, 2 :				, 3 : 			, 4 :				 
	| (0x0<<0)					// P8		0 : XOUT	, 1 : EC13		, 2 :				, 3 : 			, 4 :				
	;
	
	PF->PUPD = 0			// 0 : Disable Pull-up/down,	1 : Enable Pull-up,	2 : Enable Pull-down
	| (0x0<<22)						// P11
	| (0x0<<20)						// P10
	| (0x0<<18)						// P9
	| (0x0<<16)						// P8
	| (0x0<<14)						// P7
	| (0x0<<12)						// P6
	| (0x0<<10)						// P5
	| (0x0<<8)						// P4
	| (0x0<<6)						// P3
	| (0x0<<4)						// P2
	| (0x0<<2)						// P1
	| (0x0<<0)						// P0
	;
	
	PF->OUTDR = 0			// 0 : Output Low,	1 : Output High
	| (0x00<<11)						// P11
	| (0x00<<10)						// P10
	| (0x00<<9)						// P9
	| (0x00<<8)						// P8	
	| (0x00<<7)						// P7
	| (0x00<<6)						// P6
	| (0x00<<5)						// P5
	| (0x00<<4)						// P4
	| (0x00<<3)						// P3
	| (0x00<<2)						// P2
	| (0x00<<1)						// P1
	| (0x00<<0)						// P0
	;
	
//	PORT_ACCESS_DIS();  // disable writing permittion of ALL PCU register 	
}

/* --------------------------------- End Of File ------------------------------ */
