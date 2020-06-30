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
#define UART_RING_BUFSIZE 256	/* buffer size definition */
#define __BUF_MASK (UART_RING_BUFSIZE-1)		/* Buf mask */
#define __BUF_IS_FULL(head, tail) ((tail&__BUF_MASK)==((head+1)&__BUF_MASK))		/* Check buf is full or not */
#define __BUF_WILL_FULL(head, tail) ((tail&__BUF_MASK)==((head+2)&__BUF_MASK))		/* Check buf will be full in next receiving or not */
#define __BUF_IS_EMPTY(head, tail) ((head&__BUF_MASK)==(tail&__BUF_MASK))		/* Check buf is empty */
#define __BUF_INCR(bufidx)	(bufidx=(bufidx+1)&__BUF_MASK)		/* Reset buf */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void DEBUG_MenuPrint(void);
void DEBUG_Init(void);
void GPIO_Configure(void);
void UART_IntReceive(void);
void UART_IntTransmit(void);
void UART_IntErr(uint8_t bLSErrType);
uint32_t UARTSend(UART_Type *UARTx, uint8_t *txbuf, uint8_t buflen);
uint32_t UARTReceive(UART_Type *UARTx, uint8_t *rxbuf, uint8_t buflen);
void UART_InterruptRun(void);
void mainloop(void);
int main (void);
void Error_Handler(void);
	
/* Private variables ---------------------------------------------------------*/
const uint8_t menu[] =
"************************************************\n\r"
"\t - MCU: A34M41x \n\r"
"\t - Core: ARM Cortex-M4F \n\r"
"\t - Communicate via: UART0 - 38400 bps \n\r"
"************************************************\n\r";
void UART_IntErr(uint8_t bLSErrType);
void UART_IntTransmit(void);
void UART_IntReceive(void);

typedef struct
{
    __IO uint32_t tx_head;                /*!< UART Tx ring buffer head index */
    __IO uint32_t tx_tail;                /*!< UART Tx ring buffer tail index */
    __IO uint32_t rx_head;                /*!< UART Rx ring buffer head index */
    __IO uint32_t rx_tail;                /*!< UART Rx ring buffer tail index */
    __IO uint8_t  tx[UART_RING_BUFSIZE];  /*!< UART Tx data ring buffer */
    __IO uint8_t  rx[UART_RING_BUFSIZE];  /*!< UART Rx data ring buffer */
} UART_RING_BUFFER_T;

// Current Tx Interrupt Enable Status
__IO FlagStatus		TxIntStat;

// UART Ring buffer
UART_RING_BUFFER_T rb;


/**********************************************************************
 * @brief		UART0_IRQHandler_IT
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void UART0_IRQHandler_IT(void)
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART0->IIR;
	tmp = intsrc & UART_IIR_INTID_MASK;

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART0->LSR;
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI);
		// If any er ror exist
		if (tmp1) {
				UART_IntErr(tmp1);
		}
	}

	// Receive Data Available or Character time-out
	if (tmp == UART_IIR_INTID_RDA){
			UART_IntReceive();
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
			UART_IntTransmit();
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

}


/********************************************************************//**
 * @brief 		UART receive function (ring buffer used)
 * @param[in]	None
 * @return 		None
 *********************************************************************/
void UART_IntReceive(void)
{
	uint8_t tmpc;
	uint32_t rLen, tmp_val;

	while(1){
		// Call UART read function in UART driver
		rLen = HAL_UART_Receive(UART0, &tmpc, 1, NONE_BLOCKING);
		// If data received
		if (rLen){
			/* Check if buffer is more space
			 * If no more space, remaining character will be trimmed out
			 */
			tmp_val = rb.rx_head;
			if (!__BUF_IS_FULL(tmp_val,rb.rx_tail)){
				rb.rx[rb.rx_head] = tmpc;
				__BUF_INCR(rb.rx_head);
			}
		}
		// no more data
		else {
			break;
		}
	}
}

/********************************************************************//**
 * @brief 		UART transmit function (ring buffer used)
 * @param[in]	None
 * @return 		None
 *********************************************************************/
void UART_IntTransmit(void)
{
	uint32_t	tmp_val;
	
    // Disable THRE interrupt
   HAL_UART_ConfigInterrupt(UART0, UART_INTCFG_THRE, DISABLE);

	/* Wait until THR empty */
    while (HAL_UART_CheckBusy(UART0) == SET);

	tmp_val = rb.tx_head;
	while (!__BUF_IS_EMPTY(tmp_val,rb.tx_tail))
    {
        /* Move a piece of data into the transmit FIFO */
    	if (HAL_UART_Transmit(UART0, (uint8_t *)&rb.tx[rb.tx_tail], 1, NONE_BLOCKING)){
        /* Update transmit ring FIFO tail pointer */
        __BUF_INCR(rb.tx_tail);
    	} else {
    		break;
    	}
    }

    /* If there is no more data to send, disable the transmit
       interrupt - else enable it or keep it enabled */
		tmp_val = rb.tx_head;
	if (__BUF_IS_EMPTY(tmp_val, rb.tx_tail)) {
    	HAL_UART_ConfigInterrupt(UART0, UART_INTCFG_THRE, DISABLE);
    	// Reset Tx Interrupt state
    	TxIntStat = RESET;
    }
    else{
      	// Set Tx Interrupt state
		TxIntStat = SET;
    	HAL_UART_ConfigInterrupt(UART0, UART_INTCFG_THRE, ENABLE);
    }
}


/*********************************************************************//**
 * @brief		UART Line Status Error
 * @param[in]	bLSErrType	UART Line Status Error Type
 * @return		None
 **********************************************************************/
void UART_IntErr(uint8_t bLSErrType)
{
	uint8_t test;
	// Loop forever
	while (1){
		// For testing purpose
		test = bLSErrType;
		test = test;
	}
}



/*********************************************************************//**
 * @brief		UART transmit function for interrupt mode (using ring buffers)
 * @param[in]	UARTPort	Selected UART peripheral used to send data,
 * 				should be UART0
 * @param[out]	txbuf Pointer to Transmit buffer
 * @param[in]	buflen Length of Transmit buffer
 * @return 		Number of bytes actually sent to the ring buffer
 **********************************************************************/
uint32_t UARTSend(UART_Type *UARTx, uint8_t *txbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) txbuf;
    uint32_t bytes = 0;
		uint32_t	tmp_val;

	/* Temporarily lock out UART transmit interrupts during this
	   read so the UART transmit interrupt won't cause problems
	   with the index values */
    HAL_UART_ConfigInterrupt(UARTx, UART_INTCFG_THRE, DISABLE);

	/* Loop until transmit run buffer is full or until n_bytes
	   expires */
	tmp_val = rb.tx_head;
	while ((buflen > 0) && (!__BUF_IS_FULL(tmp_val, rb.tx_tail)))
	{
		/* Write data from buffer into ring buffer */
		rb.tx[rb.tx_head] = *data;
		data++;

		/* Increment head pointer */
		__BUF_INCR(rb.tx_head);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/*
	 * Check if current Tx interrupt enable is reset,
	 * that means the Tx interrupt must be re-enabled
	 * due to call UART_IntTransmit() function to trigger
	 * this interrupt type
	 */
	if (TxIntStat == RESET) {
		UART_IntTransmit();
	}
	/*
	 * Otherwise, re-enables Tx Interrupt
	 */
	else {
		HAL_UART_ConfigInterrupt(UARTx, UART_INTCFG_THRE, ENABLE);
	}

    return bytes;
}


/*********************************************************************//**
 * @brief		UART read function for interrupt mode (using ring buffers)
 * @param[in]	UARTPort	Selected UART peripheral used to send data,
 * 				should be UART0
 * @param[out]	rxbuf Pointer to Received buffer
 * @param[in]	buflen Length of Received buffer
 * @return 		Number of bytes actually read from the ring buffer
 **********************************************************************/
uint32_t UARTReceive(UART_Type *UARTx, uint8_t *rxbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) rxbuf;
    uint32_t bytes = 0;
		uint32_t	tmp_val;

	/* Temporarily lock out UART receive interrupts during this
	   read so the UART receive interrupt won't cause problems
	   with the index values */
	HAL_UART_ConfigInterrupt(UARTx, UART_INTCFG_RBR, DISABLE);

	/* Loop until receive buffer ring is empty or
		until max_bytes expires */
	tmp_val = rb.rx_head;
	while ((buflen > 0) && (!(__BUF_IS_EMPTY(tmp_val, rb.rx_tail))))
	{
		/* Read data from ring buffer into user buffer */
		*data = rb.rx[rb.rx_tail];
		data++;

		/* Update tail pointer */
		__BUF_INCR(rb.rx_tail);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/* Re-enable UART interrupts */
	HAL_UART_ConfigInterrupt(UARTx, UART_INTCFG_RBR, ENABLE);

    return bytes;
}


/**********************************************************************
 * @brief		UART_InterruptRun
 * @param[in]	None
 * @return	None
 **********************************************************************/
void UART_InterruptRun(void)
{
	uint32_t idx, len;
	uint8_t buffer[10];
	UART_CFG_Type UARTConfigStruct;
		
	// Initialize UART0 pin connect
	HAL_GPIO_ConfigureFunction(PC, 14, PC14_MUX_RXD0);
	HAL_GPIO_ConfigOutput(PC, 14, INPUT);
	HAL_GPIO_ConfigPullup(PC, 14, PULL_UP_ENABLE);
	
	HAL_GPIO_ConfigureFunction(PC, 15, PC15_MUX_TXD0);
	HAL_GPIO_ConfigOutput(PC, 15, PUSH_PULL_OUTPUT);	
	HAL_GPIO_ConfigPullup(PC, 15, PULL_UP_ENABLE);	

	// default : 38400-8-N-1
	HAL_UART_ConfigStructInit(&UARTConfigStruct);
	
	// Initialize peripheral with given to corresponding parameter
	if(HAL_UART_Init(UART0, &UARTConfigStruct) != HAL_OK)
	{
		Error_Handler();
	}
	
	// UART Rx Interrupt Enable
	HAL_UART_ConfigInterrupt(UART0, UART_INTCFG_RBR, ENABLE);
	
	// UART Line Status Interrupt
	HAL_UART_ConfigInterrupt(UART0, UART_INTCFG_RLS, ENABLE);
	
	TxIntStat = RESET;
	
	// Reset ring buf head and tail idx
	rb.rx_head=0;
	rb.rx_tail=0;
	rb.tx_head=0;
	rb.tx_tail=0;
	
	NVIC_SetPriority(UART0_IRQn, 3);
	NVIC_EnableIRQ(UART0_IRQn);

	__enable_irq();
	
	UARTSend(UART0,(uint8_t *)"Interrupt test \n\r",17);	
	UARTSend(UART0,(uint8_t *)"- Press any button !! \n\r",26);	
	while(1){
		len = 0;
		while (len == 0)
		{
		    len = UARTReceive(UART0, buffer, sizeof(buffer));
		}
		/* Got some data */
		idx = 0;
		while (idx < len)
		{
		    if (buffer[idx] == 0x0d)
		    {
						UARTSend((UART_Type *)UART0, (uint8_t *)"\n\r", 2);
		    }
		    else
		    {
						/* Echo it back */
						UARTSend((UART_Type *)UART0, &buffer[idx], 1);	
		    }
		    idx++;
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

	/* Enable Uart */
	UART_InterruptRun();
	
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

