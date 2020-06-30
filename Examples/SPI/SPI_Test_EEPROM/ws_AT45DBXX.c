/*********************************************************************************************************
*
* File                : ws_AT45DBXX.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

#include "A33G52x.h"
#include <stdint.h>

#include "A33G52x_spi.h"
#include "A33G52x_pcu.h"
#include "ws_AT45DBXX.h"

#include "console.h"
#include "slib.h"

void AT45DBXX_Init(void) {

	SPI_Configuration();
    
	cprintf("SPI is ready!\r\n");
    
	AT45DBXX_Disable;
    
}

static void AT45DBXX_BUSY(void)
{
	uint8_t AT45DBXXStruct;
	AT45DBXX_Enable;
	SPI0_Send_byte(READ_STATE_REGISTER);
	do
	{AT45DBXXStruct = SPI0_Receive_byte();}
	while(!(AT45DBXXStruct & 0x80));
	AT45DBXX_Disable;
}

void AT45DBXX_Read_ID(uint8_t *IData)
{
	uint8_t i;
	AT45DBXX_BUSY();
	AT45DBXX_Enable;
	SPI0_Send_byte(Read_ID); // Manufactured and Device ID Read
  	for(i=0;i<4;i++)
  	{
  		// ----- Receive Data -----
			// Byte 1 - Manufacture ID , 1FH 
			// Byte 2 - Device ID(Part1), 24H
			// Byte 3 - Device ID(Part2), 00H
			// Byte 4 - Extended Device Information String Length, 00H
			IData[i] = SPI0_Receive_byte();
			
  	}
  	AT45DBXX_Disable;
}

void flash_erase_page(uint16_t	page_id)	//page erase - 264byte
{

	AT45DBXX_Enable;
	
	//----- PAGE BLOCK -----
	// Write command
	SPI0_Send_byte(0x81);	
	// 0 0 0 0 PA10 PA9 PA8 PA7
	SPI0_Send_byte((uint8_t)page_id>>4);
	// PA6 PA5 PA4 PA3 PA2 PA1 PA0 0
	SPI0_Send_byte((uint8_t)(page_id)<<1);
	// 0 0 0 0 0 0 0 0 0 
	SPI0_Send_byte(0x00);
	
	AT45DBXX_Disable;
}

void flash_write_page(uint32_t address, uint8_t *buffer, uint8_t erase_first)	//write page - 264byte
{
	uint8_t		count;
	
	AT45DBXX_BUSY();
	AT45DBXX_Enable;	
	//---------------------------
	//----- DO BUFFER WRITE -----
	//---------------------------
	//Write command
	AT45DBXX_Enable;
	SPI0_Send_byte(BUF1_WRITE);

	// Address
	SPI0_Send_byte(0x00);
	SPI0_Send_byte(0x00);
	SPI0_Send_byte(0x00);

	//Data
	count=0;
	while(1)
	{
		SPI0_Send_byte(*buffer++);
		count++;
		
		if(count==0) break;
	}
	AT45DBXX_Disable;
	DO_CS_HIGH_DELAY;
	AT45DBXX_Enable; 
	
	//---------------------------------------------------
	//----- DO BUFFER 1 TO MAIN MEMORY PAGE PROGRAM -----
	//---------------------------------------------------
	//Write command
	if(erase_first)
	{
		SPI0_Send_byte(BBUF1_TO_MM_PAGE_PROG_WITH_ERASE);		//0x83 = write with built in erase
	}
	else
	{
		SPI0_Send_byte(0x88);	//0x88 = write without built in erase
	}
	
	//Address
	SPI0_Send_byte((uint8_t)((address>>15) & 0x000000ff));	// 0 0 0 0 PA10 PA9 PA8 PA7
	SPI0_Send_byte((uint8_t)((address>>7) & 0x000000fe));	// PA6 PA5 PA4 PA3 PA2 PA1 PA0 0 (offset for DataFlash Page Size = 264Bytes)
	SPI0_Send_byte(0x00);	// 0 0 0 0 0 0 0 0 
	
	AT45DBXX_Disable;	
	
	
}

void flash_read_page(uint32_t address, uint8_t *buffer)	//read page - 256byte
{
	uint8_t		count;
	
	AT45DBXX_Enable;
	
	//----- DO MAIN MEMORY PAGE READ -----
	//Write command
	SPI0_Send_byte(0xd2);		// Main Memory Page Read
	
	//Address (Page+Byte)
	SPI0_Send_byte((uint8_t)((address>>15)&0x000000ff));	// 0 0 0 0 PA10 PA9 PA8 PA7
	SPI0_Send_byte((uint8_t)((address>>7)&0x000000fe));	// PA6 PA5 PA4 PA3 PA2 PA1 PA0 0 (offset for DataFlash Page Size = 264 Bytes)
	SPI0_Send_byte(0x00);	// 0 0 0 0 0 0 0 0 
	
	//32 don't care bits
	SPI0_Send_byte(0x00);
	SPI0_Send_byte(0x00);
	SPI0_Send_byte(0x00);
	SPI0_Send_byte(0x00);
	
	//Read page
	count=0;
	
	while(1)
	{
		*buffer++ = SPI0_Receive_byte();
		count++;
		
		if (count==0)
			break;
	}
	
	AT45DBXX_Disable;	
	
}


void write_buffer(uint16_t BufferOffset,uint8_t Data)
{			
	AT45DBXX_BUSY();
	AT45DBXX_Enable;
	SPI0_Send_byte(BUF1_WRITE);			
	SPI0_Send_byte(0xff);
	SPI0_Send_byte((uint8_t)BufferOffset>>8);
	SPI0_Send_byte((uint8_t)BufferOffset);		
	SPI0_Send_byte(Data);
	AT45DBXX_Disable;
}

uint8_t read_buffer(uint16_t BufferOffset)
{		
	uint8_t temp=0;
	AT45DBXX_BUSY();
	AT45DBXX_Enable;
 	SPI0_Send_byte(0xd4);
	SPI0_Send_byte(0xff);
	SPI0_Send_byte((uint8_t)BufferOffset>>8);
	SPI0_Send_byte((uint8_t)BufferOffset);
	
	// additional Don't Care Bytes
	SPI0_Send_byte(0x00);
	
	temp=SPI0_Receive_byte();
	
	AT45DBXX_Disable;
	return temp;			
}

void SPI_Configuration(void) {
	SPI_CFG_Type SPI_InitStruct;
    
	SPI_InitStruct.Databit = SPI_DS_8BITS;
	SPI_InitStruct.CPHA = SPI_CPHA_HI;
	SPI_InitStruct.CPOL = SPI_CPOL_HI;
	SPI_InitStruct.DataDir = SPI_MSB_FIRST;
	SPI_InitStruct.Mode = SPI_MASTER_MODE;
	SPI_InitStruct.BaudRate = 74; // PCLK / (79+1) 
	SPI_Init(SPI0, &SPI_InitStruct);
	SPI_SSOutputCmd(SPI0, SS_MANUAL);
	GPIO_Configuration();

	SPI_Cmd(SPI0, ENABLE);
    
	AT45DBXX_Disable;
}

void SPI0_Send_byte(uint16_t data)
{
	while((SPI_GetStatus(SPI0)&SPI_STAT_TXBUF_EMPTY) != SPI_STAT_TXBUF_EMPTY);
	SPI_SendData(SPI0, data);

	while((SPI_GetStatus(SPI0)&SPI_STAT_RXBUF_READY) != SPI_STAT_RXBUF_READY);
	SPI_ReceiveData(SPI0);
}

uint16_t SPI0_Receive_byte(void)
{
	while((SPI_GetStatus(SPI0)&SPI_STAT_TXBUF_EMPTY) != SPI_STAT_TXBUF_EMPTY);
	SPI_SendData(SPI0, 0x00);

	while((SPI_GetStatus(SPI0)&SPI_STAT_RXBUF_READY) != SPI_STAT_RXBUF_READY);
	return SPI_ReceiveData(SPI0);
}


static void GPIO_Configuration(void) {
    // Devices SPI GPIO Setting
    PCU_ConfigureFunction (PCB, PIN_10, PB10_MUX_SS0);
    PCU_Set_Direction_Type (PCB, PIN_10, PnCR_OUTPUT_PUSH_PULL); 
    PCU_ConfigurePullup_Pulldown (PCB, PIN_10, 0, PnPCR_PULLUPDOWN_DISABLE); 
    
    PCU_ConfigureFunction (PCB, PIN_11, PB11_MUX_SCK0);
    PCU_Set_Direction_Type (PCB, PIN_11, PnCR_OUTPUT_PUSH_PULL); 
    PCU_ConfigurePullup_Pulldown (PCB, PIN_11, 0, PnPCR_PULLUPDOWN_DISABLE);     
    
    PCU_ConfigureFunction (PCB, PIN_12, PB12_MUX_MOSI0);
    PCU_Set_Direction_Type (PCB, PIN_12, PnCR_OUTPUT_PUSH_PULL); 
    PCU_ConfigurePullup_Pulldown (PCB, PIN_12, 0, PnPCR_PULLUPDOWN_DISABLE);     
    
    PCU_ConfigureFunction (PCB, PIN_13, PB13_MUX_MISO0);
    PCU_Set_Direction_Type (PCB, PIN_13, PnCR_INPUT_LOGIC); 
    PCU_ConfigurePullup_Pulldown (PCB, PIN_13, 0, PnPCR_PULLUPDOWN_UP);
}
