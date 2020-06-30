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


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define M24C02_ADDR    (0x40>>1)
/** Own Slave address in Slave I2C device */
#define I2CDEV_S_ADDR	(0xC0>>1)

/** Max buffer length */
#define BUFFER_SIZE			10

/* Private function prototypes -----------------------------------------------*/
void I2C0_IRQHandler_IT(void);
void I2C1_IRQHandler_IT(void);
void print_menu(void);
void GPIO_Configure(void);
void I2C_Configure( void);
void Buffer_Init(uint8_t* buffer, uint8_t type);
void Buffer_Init(uint8_t* buffer, uint8_t type);
void App_I2C_Interrupt_Master(void);
void App_I2C_Polling_Slave (void);
void I2C_MasterSlaveRun (void);
void mainloop(void);
int main (void);
	
/* Private variables ---------------------------------------------------------*/
Bool complete_m;
Bool complete_s;
Bool i2c0isMasterMode = TRUE;
Bool i2c1isMasterMode = FALSE;

I2C_M_SETUP_Type transferMCfg;
I2C_S_SETUP_Type transferSCfg;

uint8_t regAddr[2];
uint8_t Master_Buf[10];
uint8_t Slave_Buf[10];
uint8_t Receive_Buf[10];
uint8_t buffer;

const uint8_t menu[] =
"\n\r************************************************\n\r"
"I2C demo \n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"\n\rPress 1-4 to select I2C running mode:\n\r\n\r"
" 1 - I2C Polling   Master (I2C0 : Master, I2C1 : Slave)\n\r"
" 2 - I2C Polling   Slave (I2C0 : Master, I2C1 : Slave) \n\r"
" 3 - I2C Interrupt MasterSlave (I2C0 : Matser, I2C1 : Slave)\n\r"
"************************************************\n\r";
const uint8_t  menu1[] = "\n\r \t - Press x to exit this mode!\n\r";
const uint8_t  menu3[] = "\t - Press c to continue...\n\r";


/**********************************************************************
 * @brief		I2C0_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void I2C0_IRQHandler_IT(void)
{
	if (i2c0isMasterMode)// Master Mode
	{
		HAL_I2C_Master_IRQHandler_IT(I2C0);
		if (HAL_I2C_Master_GetState(I2C0)){
			complete_m = TRUE;
		}
	}
	else// Slave Mode
	{
		HAL_I2C_Slave_IRQHandler_IT(I2C0);
		if (HAL_I2C_Slave_GetState(I2C0))
		{
			complete_s = TRUE;
		}
	}	
}

/**********************************************************************
 * @brief		I2C1_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void I2C1_IRQHandler_IT(void)
{
	if (i2c1isMasterMode)// Master Mode
	{
		HAL_I2C_Master_IRQHandler_IT(I2C1);
		if (HAL_I2C_Master_GetState(I2C1)){
			complete_m = TRUE;
		}
	}
	else// Slave Mode
	{
		HAL_I2C_Slave_IRQHandler_IT(I2C1);
		if (HAL_I2C_Slave_GetState(I2C1))
		{
			complete_s = TRUE;
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


/**********************************************************************
 * @brief		PCU_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void GPIO_Configure(void)
{
 // Test Pin setting PB0
	HAL_GPIO_ConfigureFunction(PB, 0, PB0_MUX_PB0);
	HAL_GPIO_ConfigOutput(PB, 0, PUSH_PULL_OUTPUT);
	HAL_GPIO_ConfigPullup(PB, 0, PULL_UP_DOWN_DISABLE);	
	HAL_GPIO_SetPin(PB, _BIT(0));
	
	// I2C0 PC7:SCL0, PC8:SDA0
	HAL_GPIO_ConfigureFunction(PC, 7, PC7_MUX_SCL0);
	HAL_GPIO_ConfigOutput(PC, 7, OPEN_DRAIN_OUTPUT);
	HAL_GPIO_ConfigPullup(PC, 7, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PC, 8, PC8_MUX_SDA0);
	HAL_GPIO_ConfigOutput(PC, 8, OPEN_DRAIN_OUTPUT);
	HAL_GPIO_ConfigPullup(PC, 8, PULL_UP_ENABLE);

	// I2C1 PD4:SCL1, PD5:SDA1
	HAL_GPIO_ConfigureFunction(PD, 4, PD4_MUX_SCL1);
	HAL_GPIO_ConfigOutput(PD, 4, OPEN_DRAIN_OUTPUT);
	HAL_GPIO_ConfigPullup(PD, 4, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PD, 5, PD5_MUX_SDA1);
	HAL_GPIO_ConfigOutput(PD, 5, OPEN_DRAIN_OUTPUT);
	HAL_GPIO_ConfigPullup(PD, 5, PULL_UP_ENABLE);
}

/**********************************************************************
 * @brief		ADC_Configure
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void I2C_Configure(void)
{	
	// Initialize Slave I2C peripheral
	HAL_I2C_Init(I2C0, 100000);
	HAL_I2C_Init(I2C1, 100000);
	
	/* Set  Own slave address for I2C device */
	HAL_I2C_Slave_SetAddress(I2C0, I2CDEV_S_ADDR, DISABLE);
	HAL_I2C_Slave_SetAddress(I2C1, I2CDEV_S_ADDR, ENABLE);

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

	if (type)
	{
		for (i = 0; i < BUFFER_SIZE; i++) {
			buffer[i] = i;
		}
	}
	else
	{
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
	uint8_t regAddr = 0x10;
	uint8_t i;
	
	while (1) {
		_DBG_("\n\r\n\rI2C Polling Master is running...\n\r");
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
		_DBG_("\n\rPress '1' to transmit");
		while (1)
		{
			buffer = _DG;
			if (buffer == '1')
				break;
		}
		
		_DBG_("\n\r\n\rI2C1 Interrupt Slave is running...\n\r");
		_DBG_("\n\rReceiving...");

		/* Receive -------------------------------------------------------- */
		/* Initialize buffer */
		Buffer_Init(Receive_Buf, 0);

		complete_s = FALSE;
		transferSCfg.rx_data = Receive_Buf;
		transferSCfg.rx_length = 7;
		HAL_I2C_Slave_Receive(I2C1, &transferSCfg, I2C_TRANSFER_INTERRUPT);
		
		_DBG_("\n\r\n\rI2C0 Polling Master is running...\n\r");
		_DBG_("\n\rTransmitting...");
		
		/* Transmit -------------------------------------------------------- */
		/* Initialize buffer */
		Buffer_Init(Master_Buf, 0);
		/* Device test */
		Master_Buf[0]=0x00; //address
		Master_Buf[1]=0xa2; //data	
		Master_Buf[2]=0xa4; //data	
		Master_Buf[3]=0xa6; //data	
		Master_Buf[4]=0xa8; //data		
		Master_Buf[5]=0xaA; //data	
		
		transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;		
		transferMCfg.tx_data = Master_Buf;
		transferMCfg.tx_length = 6;
		HAL_I2C_Master_Transmit(I2C0, &transferMCfg, I2C_TRANSFER_POLLING);

		while(!complete_s);
		
		_DBG("\r\n*************I2C1 Data !! ***************\r\n");
			for(i=0;i<7;i++){
				_DBG("I2C_Receive_Buf[");
				_DBD(i);
				_DBG("]= 0x");
				_DBH16(Receive_Buf[i]);
				_DBG("\r\n");
			}
		
		/* Transmit and receive -------------------------------------------------------- */
		_DBG_("\n\rPress '2' to Transmit, then repeat start and receive...");
		while (1)
		{
			buffer = _DG;
			if (buffer == '2')
				break;
		}
		
		/* Receive -------------------------------------------------------- */
		/* Initialize buffer */
		Buffer_Init(Receive_Buf, 0);
		
		complete_s = FALSE;
		
		transferSCfg.rx_data = Receive_Buf;
		transferSCfg.rx_length = 2;
		Slave_Buf[0]=0x6A; //data	
		Slave_Buf[1]=0x8A; //data		
		Slave_Buf[2]=0xA5; //data		
		transferSCfg.tx_data = Slave_Buf;
		transferSCfg.tx_length = 3;
		HAL_I2C_SlaveTransferData(I2C1, &transferSCfg, I2C_TRANSFER_INTERRUPT);
			
		/* Transmit -------------------------------------------------------- */		
		/* Initialize buffer */
		Buffer_Init(Master_Buf, 0);

		transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;	
		transferMCfg.tx_data = &regAddr;
		transferMCfg.tx_length = 1;
		transferMCfg.rx_data = Master_Buf;
		transferMCfg.rx_length = 3;
		transferMCfg.tx_count = 0;
		transferMCfg.rx_count = 0;
		HAL_I2C_MasterTransferData(I2C0, &transferMCfg, I2C_TRANSFER_POLLING);
	
		while(!complete_s);

		_DBG("\r\n*************I2C0 Data !! ***************\r\n");
		for(i=0;i<2;i++){
			_DBG("I2C_Receive_Buf[");
			_DBD(i);
			_DBG("]= 0x");
			_DBH16(Receive_Buf[i]);
			_DBG("\r\n");
		}
		_DBG("\r\n*************I2C1 Data !! ***************\r\n");
		for(i=0;i<3;i++){
			_DBG("I2C_Master_Buf[");
			_DBD(i);
			_DBG("]= 0x");
			_DBH16(Master_Buf[i]);
			_DBG("\r\n");
		}
	}
}

/***********************************************************************
 * @brief		I2C master application using interrupt
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void App_I2C_Interrupt_Master(void)
{
		uint8_t i;
		
		while (1) 
		{
			
		_DBG_("\n\r\n\rI2C Interrupt Master is running...\n\r");
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
		

		_DBG_("\n\r\n\rI2C1 Interrupt Slave is running...\n\r");
		_DBG_("\n\rReceiving...");

		/* Receive -------------------------------------------------------- */
		/* Initialize buffer */
		Buffer_Init(Receive_Buf, 0);
		
		complete_s = FALSE;
		transferSCfg.rx_data = Receive_Buf;
		transferSCfg.rx_length = 7;
		HAL_I2C_Slave_Receive(I2C1, &transferSCfg, I2C_TRANSFER_INTERRUPT);
					
		/* Transmit -------------------------------------------------------- */
		_DBG_("\n\rPress '1' to transmit");
		while (1)
		{
			buffer = _DG;
			if (buffer == '1')
				break;
		}
		
		_DBG_("\n\r\n\rI2C0 Interrupt Master is running...\n\r");
		_DBG_("\n\rTransmitting...");
		/* Initialize buffer */
		complete_m = FALSE;
		Buffer_Init(Master_Buf, 0);
		Master_Buf[0]=0x06; //address
		Master_Buf[1]=0x55; //data	
		Master_Buf[2]=0xAA; //data	
		Master_Buf[3]=0xAA; //data	
		Master_Buf[4]=0x55; //data	
		Master_Buf[5]=0x77; //data	
		transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
		transferMCfg.tx_data = Master_Buf;
		transferMCfg.tx_length = 6;
		HAL_I2C_Master_Transmit(I2C0, &transferMCfg, I2C_TRANSFER_INTERRUPT);
	
		while(!complete_m);
		
		while(!complete_s);
				
		_DBG("\r\n*************I2C1 Data !! ***************\r\n");
	
		for(i=0;i<7;i++){
			_DBG("I2C_Receive_Buf[");
			_DBD(i);
			_DBG("]= 0x");
			_DBH16(Receive_Buf[i]);
			_DBG("\r\n");
		}
		
		/* Transmit and receive -------------------------------------------------------- */
		_DBG_("\n\rPress '2' to Transmit, then repeat start and receive...");
		
		while (1)
		{
			buffer = _DG;
			if (buffer == '2')
				break;
		}
		
		/* Initialize buffer */
		Buffer_Init(Receive_Buf, 0);

		complete_s = FALSE;
		
		transferSCfg.rx_data = Receive_Buf;
		transferSCfg.rx_length = 3;
		Slave_Buf[0]=0x6A; //data	
		Slave_Buf[1]=0x8A; //data		
		Slave_Buf[2]=0xA5; //data		
		transferSCfg.tx_data = Slave_Buf;
		transferSCfg.tx_length = 3;
		HAL_I2C_SlaveTransferData(I2C1, &transferSCfg, I2C_TRANSFER_INTERRUPT);
		
		/* Initialize buffer */
		complete_m = FALSE;
		
		Buffer_Init(Master_Buf, 0);
		
		Buffer_Init(regAddr, 0);
		regAddr[0] = 0x02;
		regAddr[1] = 0x04;
		transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
		transferMCfg.tx_data = regAddr;
		transferMCfg.tx_length = 2;
		transferMCfg.rx_data = Master_Buf;
		transferMCfg.rx_length = 3;
		transferMCfg.tx_count = 0;
		transferMCfg.rx_count = 0;
		
		HAL_I2C_MasterTransferData(I2C0, &transferMCfg, I2C_TRANSFER_INTERRUPT);
		
		while(!complete_m);
		
		while(!complete_s);
		
		_DBG("\r\n*************I2C0 Data !! ***************\r\n");
		for(i=0;i<3;i++){
			_DBG("I2C_Receive_Buf[");
			_DBD(i);
			_DBG("]= 0x");
			_DBH16(Receive_Buf[i]);
			_DBG("\r\n");
		}
		_DBG("\r\n*************I2C1 Data !! ***************\r\n");
		for(i=0;i<3;i++){
			_DBG("I2C_Master_Buf[");
			_DBD(i);
			_DBG("]= 0x");
			_DBH16(Master_Buf[i]);
			_DBG("\r\n");
		}
		__NOP();
	}
}

/**********************************************************************
 * @brief		I2C slave application in polling mode
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void App_I2C_Polling_Slave (void)
{
	uint8_t i;
	
	while (1) {
		_DBG_("\n\r\n\rI2C Polling Slave is running...\n\r");
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
		_DBG_("\n\rPress '1' to Start");

		while (1)
		{
			buffer = _DG;
			if (buffer == '1')
				break;
		}
		
		_DBG_("\n\r\n\rI2C1 Polling Slave is running...\n\r");
		_DBG_("\n\rReceiving...");



		/* Transmit -------------------------------------------------------- */
		/* Initialize buffer */
		complete_m = FALSE;
		Buffer_Init(Master_Buf, 0);
		Master_Buf[0]=0x06; //address
		Master_Buf[1]=0x55; //data	
		Master_Buf[2]=0xAA; //data	
		Master_Buf[3]=0xAA; //data	
		Master_Buf[4]=0x55; //data	
		Master_Buf[5]=0x00; //data	
		transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
		transferMCfg.tx_data = Master_Buf;
		transferMCfg.tx_length = 6;
		HAL_I2C_Master_Transmit(I2C0, &transferMCfg, I2C_TRANSFER_INTERRUPT);
		
		/* Receive -------------------------------------------------------- */
		/* Initialize buffer */
		Buffer_Init(Receive_Buf, 0);
		transferSCfg.rx_data = Receive_Buf;
		transferSCfg.rx_length = 6;
		HAL_I2C_Slave_Receive(I2C1, &transferSCfg, I2C_TRANSFER_POLLING);
		__NOP();
		
		while(!complete_m);
				
		_DBG("\r\n*************I2C1 Data !! ***************\r\n");
	
		for(i=0;i<7;i++){
			_DBG("I2C_Receive_Buf[");
			_DBD(i);
			_DBG("]= 0x");
			_DBH16(Receive_Buf[i]);
			_DBG("\r\n");
		}
		
		/* Receive and transmit -------------------------------------------------------- */
		_DBG_("\n\rPress '2' Start Receive, wait for repeat start and transmit...");
		while (1)
		{
			buffer = _DG;
			if (buffer == '2')
				break;
		}

		/* Transmit -------------------------------------------------------- */
		/* Initialize buffer */
		complete_m = FALSE;
		Buffer_Init(Master_Buf, 0);	
		Buffer_Init(regAddr, 0);
		regAddr[0] = 0x02;
		regAddr[1] = 0x04;
		transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
		transferMCfg.tx_data = regAddr;
		transferMCfg.tx_length = 2;
		transferMCfg.rx_data = Master_Buf;
		transferMCfg.rx_length = 3;
		transferMCfg.tx_count = 0;
		transferMCfg.rx_count = 0;
		HAL_I2C_MasterTransferData(I2C0, &transferMCfg, I2C_TRANSFER_INTERRUPT);
			
		/* Receive -------------------------------------------------------- */
		/* Initialize buffer */
		Buffer_Init(Receive_Buf, 0);
		transferSCfg.rx_data = Receive_Buf;
		transferSCfg.rx_length = 3;
		Slave_Buf[0]=0x8A; //data	
		Slave_Buf[1]=0xAA; //data		
		Slave_Buf[2]=0xA5; //data	
		transferSCfg.tx_data = Slave_Buf;
		transferSCfg.tx_length = 3;
		HAL_I2C_SlaveTransferData(I2C1, &transferSCfg, I2C_TRANSFER_POLLING);
		__NOP();		
		
		while(!complete_m);
		
		_DBG("\r\n*************I2C0 Data !! ***************\r\n");
		for(i=0;i<3;i++){
			_DBG("I2C_Receive_Buf[");
			_DBD(i);
			_DBG("]= 0x");
			_DBH16(Receive_Buf[i]);
			_DBG("\r\n");
		}
		_DBG("\r\n*************I2C1 Data !! ***************\r\n");
		for(i=0;i<3;i++){
			_DBG("I2C_Master_Buf[");
			_DBD(i);
			_DBG("]= 0x");
			_DBH16(Master_Buf[i]);
			_DBG("\r\n");
		}
	}
}

/**********************************************************************
 * @brief		I2C_MasterSlaveRun
 * @param[in]	None
 * @return	None
 **********************************************************************/
void I2C_MasterSlaveRun (void)
{
  buffer = _DG;
		
	switch (buffer)
	{
		case '1': //Master Polling Mode
			App_I2C_Polling_Master();
			print_menu();
			break;
		case '2': //Slave Polling Mode
			App_I2C_Polling_Slave();
			print_menu();
			break;
		case '3': //Matser/Slave Interrupt Mode
			i2c0isMasterMode = TRUE;
			App_I2C_Interrupt_Master();
			print_menu();
			break;
		default:
			break;
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
	print_menu();
	
	/*Configure port peripheral*/
	GPIO_Configure();
	
	/*I2C Configure*/
	I2C_Configure();
	
	/* Enable IRQ Interrupts */
	__enable_irq();
	
   /* Infinite loop */
   while(1)
	{
		I2C_MasterSlaveRun();
	}
}

/**********************************************************************
 * @brief		Main program
 * @param[in]	None
 * @return	None
 **********************************************************************/
int main (void)
{
	/* Configure the system clock to HSI 48 MHz */
	SystemClock_Config();
	
	/* Initialize all port */
	Port_Init(); 
	
	/* Initialize Debug frame work through initializing USART port  */
	debug_frmwrk_init();
	
	/* Infinite loop */
	mainloop();  
	

	return (0);
}

