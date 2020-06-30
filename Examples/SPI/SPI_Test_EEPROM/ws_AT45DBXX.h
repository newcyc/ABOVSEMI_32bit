/*********************************************************************************************************
*
* File                : ws_AT45DBXX.h
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

#ifndef _AT45DBXX_H
#define _AT45DBXX_H

#include <stdint.h>

#define FLASH_END_ADDRESS	0x7FFFF
#define FLASH_BLOCK_SIZE		0x800 // A block is 8 pages
#define BUF1_WRITE 0x84
#define BUF2_WRITE 0x87
#define BUF1_READ 0xD4
#define BUF2_READ 0xD6
#define BBUF1_TO_MM_PAGE_PROG_WITH_ERASE 0x83
#define BBUF2_TO_MM_PAGE_PROG_WITH_ERASE 0x86
#define MM_PAGE_TO_B1_XFER 0x53
#define MM_PAGE_TO_B2_XFER 0x55
#define PAGE_ERASE 0x81   // 512/528 bytes per page
#define SECTOR_ERASE 0x7C // 128k bytes per sector
#define READ_STATE_REGISTER 0xD7
#define Read_ID 0x9F
#define DO_CS_HIGH_DELAY __NOP(); __NOP();


//#define AT45DBXX_Enable GPIO_ClearValue(PA, _BIT(12));
#define AT45DBXX_Enable SPI_SSOutput(SPI0, SS_OUT_LO);
//#define AT45DBXX_Enable GPIO_SetValue(PA, _BIT(12));
#define AT45DBXX_Disable SPI_SSOutput(SPI0, SS_OUT_HI);

void AT45DBXX_Init(void);
static void AT45DBXX_BUSY(void);
void AT45DBXX_Read_ID(uint8_t *Data);
void write_buffer(uint16_t BufferOffset,uint8_t Data);
uint8_t read_buffer(uint16_t BufferOffset);
void flash_erase_page(uint16_t	block_id);
void flash_write_page(uint32_t address, uint8_t *buffer, uint8_t erase_first);
void flash_read_page(uint32_t address, uint8_t *buffer);

void SPI_Configuration(void);
void SPI0_Send_byte(uint16_t data);
uint16_t SPI0_Receive_byte(void);

static void GPIO_Configuration(void);

#endif /*_AT45DBXX_H*/

