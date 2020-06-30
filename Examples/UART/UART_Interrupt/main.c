/***************************************************************************//**
* @file     main.c
* @brief    An example of UART using interrupt mode on A31G22x
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

/*******************************************************************************
* Included File
*******************************************************************************/
#include "A31G22x_pcu.h"
#include "A31G22x_scu.h"
#include "A31G22x_uartn.h"

#include "debug_frmwrk.h"


/*******************************************************************************
* Pre-processor Definition & Macro
*******************************************************************************/
//-------------------------
#define USED_CLKO
//-------------------------
#define USED_HSI
//#define USED_LSI
//#define USED_MOSC
//#define USED_SOSC
//#define USED_MOSCPLL
//#define USED_HSIPLL

#define UART_RING_BUFSIZE 				(32) // buffer size definition
#define BUF_MASK 						(UART_RING_BUFSIZE - 1) // Buf mask
#define BUF_IS_FULL(Head, Tail)			(((Tail & BUF_MASK) == ((Head + 1) & BUF_MASK)) ? TRUE : FALSE)
#define BUF_IS_EMPTY(Head, Tail)		(((Tail & BUF_MASK) == (Head & BUF_MASK)) ? TRUE : FALSE)
#define BUF_INCREASE(Index)				(Index = (Index + 1) & BUF_MASK)


/*******************************************************************************
* Private Typedef
*******************************************************************************/
typedef struct
{
	uint32_t Tx_Head;
	uint32_t Tx_Tail;
	uint32_t Rx_Head;
	uint32_t Rx_Tail;
	uint8_t  Tx_Buffer[UART_RING_BUFSIZE];
	uint8_t  Rx_Buffer[UART_RING_BUFSIZE];
} UART_RING_BUFFER_Type;


/*******************************************************************************
* Private Variable
*******************************************************************************/
static const uint8_t TEST_MENU[] =
"************************************************\r\n"
" UART demo\r\n"
"\t - MCU: A31G22x\r\n"
"\t - Core: ARM Cortex-M0+\r\n"
"\t - Communicate via: USART10 - 38400 bps\r\n"
"\t - Using UART1 interrupt mode\r\n"
"************************************************\r\n";

volatile FlagStatus TxStatus;
UART_RING_BUFFER_Type RingBuffer; // UART Ring buffer


/*******************************************************************************
* Private Function Prototype
*******************************************************************************/
void Main_RxIRQHandler(UART_Type *pUARTx);
void Main_TxIRQHandler(UART_Type *pUARTx);
void Main_HandleLSRError(uint32_t Error);
uint32_t Main_SendUART(UART_Type *pUARTx, uint8_t *pTxBuffer, uint32_t BufferLength);
uint32_t Main_ReceiveUART(UART_Type *pUARTx, uint8_t *pRxBuffer, uint32_t BufferLength);
static void Main_PrintMenu(void);
static void Main_MainLoop(void);
static void Main_InitializeClock(void);
static void Main_InitializePCU(void);


/*******************************************************************************
* Public Function
*******************************************************************************/

/*******************************************************************************
* @brief      UART1 interrupt handler
* @param      None
* @return     None
*******************************************************************************/
void UART1_IRQHandler(void)
{
	volatile uint32_t Reg32_IIR;
	volatile uint32_t Reg32_LSR;

	/* Determine the interrupt source */
	Reg32_IIR = UART1->IIR & UART_IIR_IID_Msk;

	if (Reg32_IIR == UART_IIR_IID_RLS) {
		// Check line status
		Reg32_LSR = UART1->LSR;
		// Mask out the Receive Ready and Transmit Holding empty status
		Reg32_LSR &= (UART_LSR_BI_Msk | UART_LSR_FE_Msk | UART_LSR_PE_Msk | UART_LSR_OE_Msk);
		// If any error exist
		if (Reg32_LSR) {
			Main_HandleLSRError(Reg32_LSR);
		}
	} else if (Reg32_IIR == UART_IIR_IID_RDA) {
		// Receive Data Available or Character time-out
		Main_RxIRQHandler(UART1);
	} else if (Reg32_IIR == UART_IIR_IID_THRE) {
		// Transmit Holding Empty
		Main_TxIRQHandler(UART1);
	}
}

/*******************************************************************************
* @brief      Main function
* @param      None
* @return     None
*******************************************************************************/
int main(void)
{
	SystemInit();
	Main_InitializePCU();
	Main_InitializeClock();

	DEBUG_Init(DEBUG_INTERFACE_PERI);

	Main_MainLoop();

	return (0);
}

/*******************************************************************************
* Private Function
*******************************************************************************/

/*******************************************************************************
* @brief      Rx interrupt handler of UART (ring buffer used)
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @return     None
*******************************************************************************/
static void Main_RxIRQHandler(UART_Type *pUARTx)
{
	uint8_t Data;

	while(1) {
		if (UART_Receive(pUARTx, &Data, 1, NONE_BLOCKING) != 0) {
			if (BUF_IS_FULL(RingBuffer.Rx_Head, RingBuffer.Rx_Tail) == FALSE) {
				RingBuffer.Rx_Buffer[RingBuffer.Rx_Head] = Data;
				BUF_INCREASE(RingBuffer.Rx_Head);
			}
		} else {
			break;
		}
	}
}

/*******************************************************************************
* @brief      Tx interrupt handler of UART (ring buffer used)
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @return     None
*******************************************************************************/
static void Main_TxIRQHandler(UART_Type *pUARTx)
{
	// Disable THRE interrupt
	UART_ConfigureInterrupt(pUARTx, UART_INTERRUPT_THREIE, DISABLE);

	// Wait until Transmit register empty
	while (UART_CheckBusy(pUARTx) == SET);

	while (BUF_IS_EMPTY(RingBuffer.Tx_Head, RingBuffer.Tx_Tail) == FALSE) {
		// Move a piece of data into the transmit FIFO
		if (UART_Send(pUARTx, &(RingBuffer.Tx_Buffer[RingBuffer.Tx_Tail]), 1, NONE_BLOCKING)) {
			// Update transmit ring FIFO tail pointer
			BUF_INCREASE(RingBuffer.Tx_Tail);
		} else {
			break;
		}
	}

	/* If there is no more data to send, disable the transmit
	interrupt - else enable it or keep it enabled */
	if (BUF_IS_EMPTY(RingBuffer.Tx_Head, RingBuffer.Tx_Tail) == TRUE) {
		UART_ConfigureInterrupt(pUARTx, UART_INTERRUPT_THREIE, DISABLE);
		TxStatus = RESET; // Reset Tx Interrupt state
	} else {
		TxStatus = SET; // Set Tx Interrupt state
		UART_ConfigureInterrupt(pUARTx, UART_INTERRUPT_THREIE, ENABLE);
	}
}

/*******************************************************************************
* @brief      UART Line Status Error
* @param      Error : UART Line Status Error Type
* @return     None
*******************************************************************************/
static void Main_HandleLSRError(uint32_t Error)
{
	volatile uint32_t Test;

	// Loop forever
	while (1) {
		Test = Error; // For testing purpose
	}
}

/*******************************************************************************
* @brief      UART send function for interrupt mode (using ring buffers)
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      pTxBuffer : Pointer to Tx buffer
* @param      BufferLength : Length of Tx buffer
* @return     Number of bytes actually sent to the ring buffer
*******************************************************************************/
static uint32_t Main_SendUART(UART_Type *pUARTx, uint8_t *pTxBuffer, uint32_t BufferLength)
{
	uint32_t ByteCount;

	/* Temporarily lock out UART transmit interrupts during this
	   read so the UART transmit interrupt won't cause problems
	   with the index values */
	UART_ConfigureInterrupt(pUARTx, UART_INTERRUPT_THREIE, DISABLE);

	// Loop until transmit run buffer is full or until n_bytes expires
	ByteCount = 0;
	while ((BufferLength != 0) && (BUF_IS_FULL(RingBuffer.Tx_Head, RingBuffer.Tx_Tail) == FALSE)) {
		// Write data from buffer into ring buffer
		RingBuffer.Tx_Buffer[RingBuffer.Tx_Head] = *pTxBuffer++;

		// Increment head pointer
		BUF_INCREASE(RingBuffer.Tx_Head);

		ByteCount++;
		BufferLength--;
	}

	/* Check if current Tx interrupt enable is reset,
	   that means the Tx interrupt must be re-enabled
	   due to call Main_TxIRQHandler() function to trigger
	   this interrupt type */
	if (TxStatus == RESET) {
		Main_TxIRQHandler(pUARTx);
	}
	else {
		// Otherwise, re-enables Tx Interrupt
		UART_ConfigureInterrupt(pUARTx, UART_INTERRUPT_THREIE, ENABLE);
	}

	return ByteCount;
}


/*******************************************************************************
* @brief      UART receive function for interrupt mode (using ring buffers)
* @param      pUARTx : UART peripheral selected, should be
*              - UART0 : UART0 peripheral
*              - UART1 : UART1 peripheral
* @param      pRxBuffer : Pointer to Rx buffer
* @param      BufferLength : Length of Rx buffer
* @return     Number of bytes actually read from the ring buffer
*******************************************************************************/
static uint32_t Main_ReceiveUART(UART_Type *pUARTx, uint8_t *pRxBuffer, uint32_t BufferLength)
{
	uint32_t ByteCount;

	/* Temporarily lock out UART receive interrupts during this
	   read so the UART receive interrupt won't cause problems
	   with the index values */
	UART_ConfigureInterrupt(pUARTx, UART_INTERRUPT_DRIE, DISABLE);

	// Loop until receive buffer ring is empty or until max_bytes expires
	ByteCount = 0;
	while ((BufferLength != 0) && ((BUF_IS_EMPTY(RingBuffer.Rx_Head, RingBuffer.Rx_Tail)) == FALSE)) {
		/* Read data from ring buffer into user buffer */
		*pRxBuffer++ = RingBuffer.Rx_Buffer[RingBuffer.Rx_Tail];

		/* Update tail pointer */
		BUF_INCREASE(RingBuffer.Rx_Tail);

		/* Increment data count and decrement buffer size count */
		ByteCount++;
		BufferLength--;
	}

	/* Re-enable UART interrupts */
	UART_ConfigureInterrupt(pUARTx, UART_INTERRUPT_DRIE, ENABLE);

	return ByteCount;
}

/*******************************************************************************
* @brief      Print test menu
* @param      None
* @return     None
*******************************************************************************/
static void Main_PrintMenu(void)
{
	_DBG(TEST_MENU);
}

/*******************************************************************************
* @brief      Main loop
* @param      None
* @return     None
*******************************************************************************/
static void Main_MainLoop(void)
{
	uint32_t Index;
	uint32_t Length;
	uint8_t Buffer[10];
	UART_CFG_Type UART_Config;

	Main_PrintMenu();

	// Initialize UART1 port connect
	PCU_ConfigureDirection(PB, 7, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(PB, 7, PCU_ALT_FUNCTION_1);
	PCU_ConfigurePullupdown(PB, 7, PCU_PUPD_PULL_UP);
	
	PCU_ConfigureDirection(PB, 6, PCU_MODE_ALT_FUNC);
	PCU_ConfigureFunction(PB, 6, PCU_ALT_FUNCTION_1);

	// Default Config : 38400-8-N-1
	UART_GetDefaultConfig(&UART_Config);
	UART_Init(UART1, &UART_Config);

	// Enable UART Rx interrupt
	UART_ConfigureInterrupt(UART1, UART_INTERRUPT_DRIE, ENABLE);
	// Enable UART line status interrupt
	UART_ConfigureInterrupt(UART1, UART_INTERRUPT_RLSIE, ENABLE);
	
	TxStatus = RESET;
	
	// Reset ring buf head and tail idx
	RingBuffer.Rx_Head = 0;
	RingBuffer.Rx_Tail = 0;
	RingBuffer.Tx_Head = 0;
	RingBuffer.Tx_Tail = 0;

	NVIC_SetPriority(UART1_IRQn, 3);
	NVIC_EnableIRQ(UART1_IRQn);

	__enable_irq();

	Main_SendUART(UART1, "Interrupt test \n\r", 17);

	while(1) {
		Length = 0;

		while (Length == 0) {
			Length = Main_ReceiveUART(UART1, Buffer, sizeof(Buffer));
		}

		/* Got some data */
		Index = 0;
		while (Index < Length) {
			if (Buffer[Index] == 0x0d) {
				Main_SendUART(UART1, "\r\n 123456789012334567890\r\n", (2+22+2));
			} else {
				/* Echo it back */
				Main_SendUART(UART1, &Buffer[Index], 1);
				Main_SendUART(UART1, "[1234567890123345678]\r\n", (2+22));
			}
			Index++;
	}
	}
}

/*******************************************************************************
* @brief      Initialize default clock
* @param      None
* @return     None
*******************************************************************************/
static void Main_InitializeClock(void)
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

#ifdef USED_HSI // 32MHz
	SCU_SetHSI(HSI_EN);
	SystemCoreClock = 32000000; // 32MHz
	SystemPeriClock = 32000000; // 32MHz

	for (i = 0; i < 10; i++);

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

/*******************************************************************************
* @brief      Initialize PCU(IO Port)
* @param      None
* @return     None
*******************************************************************************/
static void Main_InitializePCU(void)
{
	// Peripheral Enable Register 1  0:Disable, 1:Enable	
	SCU->PER1 |= 0x00UL 
			| (0x01UL << SCU_PER1_GPIOF_Pos) // GPIO F
			| (0x01UL << SCU_PER1_GPIOE_Pos) // GPIO E
			| (0x01UL << SCU_PER1_GPIOD_Pos) // GPIO D
			| (0x01UL << SCU_PER1_GPIOC_Pos) // GPIO C
			| (0x01UL << SCU_PER1_GPIOB_Pos) // GPIO B
			| (0x01UL << SCU_PER1_GPIOA_Pos) // GPIO A
			;
	// Peripheral Clock Enable Register 1 0:Disable, 1:Enable	
	SCU->PCER1 |= 0x00UL
			| (0x01UL << SCU_PCER1_GPIOF_Pos) // GPIO F
			| (0x01UL << SCU_PCER1_GPIOE_Pos) // GPIO E
			| (0x01UL << SCU_PCER1_GPIOD_Pos) // GPIO D
			| (0x01UL << SCU_PCER1_GPIOC_Pos) // GPIO C
			| (0x01UL << SCU_PCER1_GPIOB_Pos) // GPIO B
			| (0x01UL << SCU_PCER1_GPIOA_Pos) // GPIO A
			;

	PORT_ACCESS_EN();  // enable writing permittion of ALL PCU register

	//--------------------------------------------------------------
	//	PORT INIT
	//		PA, PB, PC, PD, PE, PF
	//--------------------------------------------------------------
	// PORT - A
	PA->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PA->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PA->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 :               , 2 :               , 3 : AN7/CREF0     , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : T11O          , 2 : T11C          , 3 : AN6/CREF1/DAO , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : T12O          , 2 : T12C          , 3 : AN5/CP1A      , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 :               , 2 :               , 3 : AN4/CP1B      , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 :               , 2 :               , 3 : AN3/CP1C      , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : EC12          , 2 :               , 3 : AN2/CP0       , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : SCL1          , 2 :               , 3 : AN1           , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : SDA1          , 2 :               , 3 : AN0           , 4 :               
			;

	PA->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 :               , 2 :               , 3 : AN14          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 :               , 2 :               , 3 : AN13          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 :               , 2 :               , 3 : AN12          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 :               , 2 :               , 3 : AN11          , 4 :               
			;

	PA->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PA->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - B
	PB->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE15_Pos) // P15
			| (0x01UL << PORT_MOD_MODE14_Pos) // P14
			| (0x01UL << PORT_MOD_MODE13_Pos) // P13
			| (0x01UL << PORT_MOD_MODE12_Pos) // P12
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x02UL << PORT_MOD_MODE5_Pos)  // P5  - Alternative function mode (SWDIO)
			| (0x02UL << PORT_MOD_MODE4_Pos)  // P4  - Alternative function mode (SWCLK)
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PB->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP15_Pos) // P15
			| (0x00UL << PORT_TYP_TYP14_Pos) // P14
			| (0x00UL << PORT_TYP_TYP13_Pos) // P13
			| (0x00UL << PORT_TYP_TYP12_Pos) // P12
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PB->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 : RXD1          , 2 :               , 3 : AN16          , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : TXD1          , 2 : EC11          , 3 : AN15          , 4 :               
			| (0x02UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : RXD0          , 2 : SWDIO         , 3 :               , 4 :               
			| (0x02UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 : TXD0          , 2 : SWCLK         , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : BOOT          , 2 : SS10/SS20     , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 :               , 2 : SCK10/SCK20   , 3 : AN10          , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : RXD10         , 2 : MISO10/MISO20 , 3 : AN9           , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : TXD10         , 2 : MOSI10/MOSI20 , 3 : AN8           , 4 :               
			;

	PB->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR15_Pos) // P15 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR14_Pos) // P14 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR13_Pos) // P13 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR12_Pos) // P12 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 : T15C          , 2 : EC16          , 3 : T15O          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 : T16C          , 2 : EC15          , 3 : T16O          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 : T16O          , 2 : T16C          , 3 : EC15          , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 : T15O          , 2 : T15C          , 3 : EC16          , 4 :               
			;

	PB->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD15_Pos) // P15
			| (0x00UL << PORT_PUPD_PUPD14_Pos) // P14
			| (0x00UL << PORT_PUPD_PUPD13_Pos) // P13
			| (0x00UL << PORT_PUPD_PUPD12_Pos) // P12
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PB->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR15_Pos) // P15
			| (0x00UL << PORT_OUTDR_OUTDR14_Pos) // P14
			| (0x00UL << PORT_OUTDR_OUTDR13_Pos) // P13
			| (0x00UL << PORT_OUTDR_OUTDR12_Pos) // P12
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - C
	PC->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE12_Pos) // P12
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PC->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP12_Pos) // P12
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PC->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : SCL2          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : SDA2          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 :               , 2 : SCK20         , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : EC21          , 2 : MISO20        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : EC20          , 2 : MOSI20        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : T21O          , 2 : T21C          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : T20O          , 2 : T20C          , 3 : AN17          , 4 :               
			;

	PC->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR12_Pos) // P12 - 0 :               , 1 : EC11          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 : EC10          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 :               , 2 :               , 3 :               , 4 :               
			;

	PC->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD12_Pos) // P12
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PC->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR12_Pos) // P12
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - D
	PD->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PD->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PD->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 :               , 2 : SS11/SS21     , 3 :               , 4 : ICOM6         
			| (0x00UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 : BLNK          , 2 : SCK11/SCK21   , 3 :               , 4 : ICOM7         
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : RXD11         , 2 : MISO11/MISO21 , 3 :               , 4 : ICOM8         
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : TXD11         , 2 : MOSI11/MOSI21 , 3 :               , 4 : ICOM9         
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : SDA0          , 2 : EC10          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : SCL0          , 2 : SS20          , 3 :               , 4 :               
			;

	PD->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PD->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - E
	PE->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE15_Pos) // P15
			| (0x01UL << PORT_MOD_MODE14_Pos) // P14
			| (0x01UL << PORT_MOD_MODE13_Pos) // P13
			| (0x01UL << PORT_MOD_MODE12_Pos) // P12
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PE->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP15_Pos) // P15
			| (0x00UL << PORT_TYP_TYP14_Pos) // P14
			| (0x00UL << PORT_TYP_TYP13_Pos) // P13
			| (0x00UL << PORT_TYP_TYP12_Pos) // P12
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PE->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 : T11O          , 2 : T11C          , 3 :               , 4 : ICOM5         
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : T10O          , 2 : T10C          , 3 :               , 4 : ICOM4         
			| (0x02UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : PWM30CB       , 2 : MOSI21        , 3 :               , 4 : ICOM3         
			| (0x02UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 : PWM30CA       , 2 : MISO21        , 3 :               , 4 : ICOM2         
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : PWM30BB       , 2 : SCK21         , 3 :               , 4 : ICOM1         
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : PWM30BA       , 2 : SS21          , 3 :               , 4 : ICOM0         
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : PWM30AB       , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : PWM30AA       , 2 : SS11          , 3 :               , 4 :               
			;

	PE->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR15_Pos) // P15 - 0 :               , 1 :               , 2 : SS12          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR14_Pos) // P14 - 0 :               , 1 :               , 2 : SCK12         , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR13_Pos) // P13 - 0 :               , 1 : RXD12         , 2 : MISO12        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR12_Pos) // P12 - 0 :               , 1 : TXD12         , 2 : MOSI12        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 :               , 2 : SS13          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 :               , 2 : SCK13         , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 : RXD13         , 2 : MISO13        , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 : TXD13         , 2 : MOSI13        , 3 :               , 4 :               
			;

	PE->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD15_Pos) // P15
			| (0x00UL << PORT_PUPD_PUPD14_Pos) // P14
			| (0x00UL << PORT_PUPD_PUPD13_Pos) // P13
			| (0x00UL << PORT_PUPD_PUPD12_Pos) // P12
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PE->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR15_Pos) // P15
			| (0x00UL << PORT_OUTDR_OUTDR14_Pos) // P14
			| (0x00UL << PORT_OUTDR_OUTDR13_Pos) // P13
			| (0x00UL << PORT_OUTDR_OUTDR12_Pos) // P12
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	// PORT - F
	PF->MOD = 0x00UL // 0 : Input Mode, 1 : Output Mode, 2 : Alternative function mode
			| (0x01UL << PORT_MOD_MODE11_Pos) // P11
			| (0x01UL << PORT_MOD_MODE10_Pos) // P10
			| (0x01UL << PORT_MOD_MODE9_Pos)  // P9
			| (0x01UL << PORT_MOD_MODE8_Pos)  // P8
			| (0x01UL << PORT_MOD_MODE7_Pos)  // P7
			| (0x01UL << PORT_MOD_MODE6_Pos)  // P6
			| (0x01UL << PORT_MOD_MODE5_Pos)  // P5
			| (0x01UL << PORT_MOD_MODE4_Pos)  // P4
			| (0x01UL << PORT_MOD_MODE3_Pos)  // P3
			| (0x01UL << PORT_MOD_MODE2_Pos)  // P2
			| (0x01UL << PORT_MOD_MODE1_Pos)  // P1
			| (0x01UL << PORT_MOD_MODE0_Pos)  // P0
			;

	PF->TYP = 0x00UL // 0 : Push-pull Output, 1 : Open-drain Output
			| (0x00UL << PORT_TYP_TYP11_Pos) // P11
			| (0x00UL << PORT_TYP_TYP10_Pos) // P10
			| (0x00UL << PORT_TYP_TYP9_Pos)  // P9
			| (0x00UL << PORT_TYP_TYP8_Pos)  // P8
			| (0x00UL << PORT_TYP_TYP7_Pos)  // P7
			| (0x00UL << PORT_TYP_TYP6_Pos)  // P6
			| (0x00UL << PORT_TYP_TYP5_Pos)  // P5
			| (0x00UL << PORT_TYP_TYP4_Pos)  // P4
			| (0x00UL << PORT_TYP_TYP3_Pos)  // P3
			| (0x00UL << PORT_TYP_TYP2_Pos)  // P2
			| (0x00UL << PORT_TYP_TYP1_Pos)  // P1
			| (0x00UL << PORT_TYP_TYP0_Pos)  // P0
			;

	PF->AFSR1 = 0x00UL
			| (0x00UL << PORT_AFSR1_AFSR7_Pos)  // P7  - 0 :               , 1 : T30C          , 2 : SDA0          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR6_Pos)  // P6  - 0 :               , 1 : EC30          , 2 : SCL0          , 3 :               , 4 :               
			| (0x02UL << PORT_AFSR1_AFSR5_Pos)  // P5  - 0 :               , 1 : BLNK          , 2 :               , 3 :               , 4 :               
			| (0x02UL << PORT_AFSR1_AFSR4_Pos)  // P4  - 0 :               , 1 : CLKO          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR3_Pos)  // P3  - 0 :               , 1 : RXD1          , 2 :               , 3 : SXOUT         , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR2_Pos)  // P2  - 0 :               , 1 : TXD1          , 2 :               , 3 : SXIN          , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR1_Pos)  // P1  - 0 :               , 1 : SDA1          , 2 :               , 3 : XIN           , 4 :               
			| (0x00UL << PORT_AFSR1_AFSR0_Pos)  // P0  - 0 :               , 1 : SCL1          , 2 :               , 3 : XOUT          , 4 :               
			;

	PF->AFSR2 = 0x00UL
			| (0x00UL << PORT_AFSR2_AFSR11_Pos) // P11 - 0 :               , 1 : T14O          , 2 : T14C          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR10_Pos) // P10 - 0 :               , 1 : T13O          , 2 : T13C          , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR9_Pos)  // P9  - 0 :               , 1 : EC14          , 2 :               , 3 :               , 4 :               
			| (0x00UL << PORT_AFSR2_AFSR8_Pos)  // P8  - 0 :               , 1 : EC13          , 2 :               , 3 :               , 4 :               
			;

	PF->PUPD = 0x00UL // 0 : Disable Pull-up/down, 1 : Enable Pull-up, 2 : Enable Pull-down
			| (0x00UL << PORT_PUPD_PUPD11_Pos) // P11
			| (0x00UL << PORT_PUPD_PUPD10_Pos) // P10
			| (0x00UL << PORT_PUPD_PUPD9_Pos)  // P9
			| (0x00UL << PORT_PUPD_PUPD8_Pos)  // P8
			| (0x00UL << PORT_PUPD_PUPD7_Pos)  // P7
			| (0x00UL << PORT_PUPD_PUPD6_Pos)  // P6
			| (0x00UL << PORT_PUPD_PUPD5_Pos)  // P5
			| (0x00UL << PORT_PUPD_PUPD4_Pos)  // P4
			| (0x00UL << PORT_PUPD_PUPD3_Pos)  // P3
			| (0x00UL << PORT_PUPD_PUPD2_Pos)  // P2
			| (0x00UL << PORT_PUPD_PUPD1_Pos)  // P1
			| (0x00UL << PORT_PUPD_PUPD0_Pos)  // P0
			;

	PF->OUTDR = 0x00UL // 0 : Output Low, 1 : Output High
			| (0x00UL << PORT_OUTDR_OUTDR11_Pos) // P11
			| (0x00UL << PORT_OUTDR_OUTDR10_Pos) // P10
			| (0x00UL << PORT_OUTDR_OUTDR9_Pos)  // P9
			| (0x00UL << PORT_OUTDR_OUTDR8_Pos)  // P8
			| (0x00UL << PORT_OUTDR_OUTDR7_Pos)  // P7
			| (0x00UL << PORT_OUTDR_OUTDR6_Pos)  // P6
			| (0x00UL << PORT_OUTDR_OUTDR5_Pos)  // P5
			| (0x00UL << PORT_OUTDR_OUTDR4_Pos)  // P4
			| (0x00UL << PORT_OUTDR_OUTDR3_Pos)  // P3
			| (0x00UL << PORT_OUTDR_OUTDR2_Pos)  // P2
			| (0x00UL << PORT_OUTDR_OUTDR1_Pos)  // P1
			| (0x00UL << PORT_OUTDR_OUTDR0_Pos)  // P0
			;

	PORT_ACCESS_DIS(); // disable writing permittion of ALL PCU register
}

/* --------------------------------- End Of File ------------------------------ */
