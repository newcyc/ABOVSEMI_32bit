
//////////////////////////////////////////////////////////////////////////
// This confidential and proprietary software may be used only as       //
// authorized by a licensing agreement from Abov Semiconductor.         //
// In the event of publication, the following notice is applicable:     //
//                                                                      //
// (C)COPYRIGHT 2016 Abov Semiconductor                                 //
// ALL RIGHTS RESERVED                                                  //
//                                                                      //
// The entire notice above must be reproduced on all authorized copies. //
//                                                                      //
// File : A34M41x_flash_acc_lib.h                                               //
// Author: byunghee an                                                  //
// Date: 2017.01.14                                                     //
// Version: 0.0                                                         //
// Abstract: flash acceleration controller                              //
//////////////////////////////////////////////////////////////////////////
#ifndef _FLASH_ACC_H_
#define _FLASH_ACC_H_

#include "stdint.h"


extern uint8_t flash_acc_write_done;
extern uint8_t flash_acc_chksum_done;
extern uint32_t var32;

 #define FLASH_ACC_CONF_PRFTEN  (1UL << 8)
 #define FLASH_ACC_CONF_ICEN    (1UL << 9)
    #define FLASH_ACC_CONF_DCEN    (1UL << 10)
    #define FLASH_ACC_CONF_PRFTRST (1UL << 16)
    #define FLASH_ACC_CONF_ICRST   (1UL << 17)
    #define FLASH_ACC_CONF_DCRST   (1UL << 18)
    #define FLASH_ACC_CONF_BBLOCK  (1UL << 24)

    #define FLASH_ACC_CTRL_PGM     (1UL << 0)
    #define FLASH_ACC_CTRL_PERS    (1UL << 1)
    #define FLASH_ACC_CTRL_S1KERS  (1UL << 2)
    #define FLASH_ACC_CTRL_S4KERS  (1UL << 3)
    #define FLASH_ACC_CTRL_CERS    (1UL << 4)
    #define FLASH_ACC_CTRL_WDIEN   (1UL << 8)
    #define FLASH_ACC_CTRL_OLOCK   (1UL << 30)
    #define FLASH_ACC_CTRL_FLOCK   (1UL << 31)

    #define FLASH_ACC_STAT_WBUSY   (1UL << 0)
    #define FLASH_ACC_STAT_CBUSY   (1UL << 1)
    #define FLASH_ACC_STAT_WDONE   (1UL << 8)
    #define FLASH_ACC_STAT_CDONE   (1UL << 9)
    #define FLASH_ACC_STAT_FLERR   (1UL << 16)
    #define FLASH_ACC_STAT_OLERR   (1UL << 17)
    #define FLASH_ACC_STAT_FPERR   (1UL << 18)
    #define FLASH_ACC_STAT_OPERR   (1UL << 19)
    #define FLASH_ACC_STAT_WSERR   (1UL << 20)
    #define FLASH_ACC_STAT_TLERR   (1UL << 24)
    #define FLASH_ACC_STAT_TPERR   (1UL << 25)

    #define FLASH_ACC_CHKCTRL_BGEN   (1UL << 0)
    #define FLASH_ACC_CHKCTRL_BSEN   (1UL << 1)
    #define FLASH_ACC_CHKCTRL_CDIEN  (1UL << 8)
    #define FLASH_ACC_CHKCTRL_CDRST  (1UL << 16)

    #define FLASH_ACC_SPECCTRL_BOOTROM    (1UL << 0)
    #define FLASH_ACC_SPECCTRL_NONFLSBOOT (1UL << 8)

    #define FLASH_ACC_TESTCTRL_CSBM   (1UL << 0)
    #define FLASH_ACC_TESTCTRL_SBWM   (1UL << 1)
    #define FLASH_ACC_TESTCTRL_FRST   (1UL << 16)
    #define FLASH_ACC_TESTCTRL_TSMODE (1UL << 17)
    #define FLASH_ACC_TESTCTRL_FUSERD (1UL << 18)
    #define FLASH_ACC_TESTCTRL_TLOCK  (1UL << 31)

    #define FLASH_KEY1_PARAM  0x01234567
    #define FLASH_KEY2_PARAM  0x12345678
    #define FLASH_KEY3_PARAM  0x23456789

    #define OTP_KEY1_PARAM  0x3456789A
    #define OTP_KEY2_PARAM  0x456789AB
    #define OTP_KEY3_PARAM  0x56789ABC

    #define TRIM_KEY1_PARAM  0x6789ABCD
    #define TRIM_KEY2_PARAM  0x789ABCDE
    #define TRIM_KEY3_PARAM  0x89ABCDEF

    #define FLASH_ACC_FLSPROT_FPBY16_0  (1UL << 0 )
    #define FLASH_ACC_FLSPROT_FPBY16_1  (1UL << 1 )
    #define FLASH_ACC_FLSPROT_FPBY16_2  (1UL << 2 )
    #define FLASH_ACC_FLSPROT_FPBY16_3  (1UL << 3 )
    #define FLASH_ACC_FLSPROT_FPBY16_4  (1UL << 4 )
    #define FLASH_ACC_FLSPROT_FPBY16_5  (1UL << 5 )
    #define FLASH_ACC_FLSPROT_FPBY16_6  (1UL << 6 )
    #define FLASH_ACC_FLSPROT_FPBY16_7  (1UL << 7 )
    #define FLASH_ACC_FLSPROT_FPBY16_8  (1UL << 8 )
    #define FLASH_ACC_FLSPROT_FPBY16_9  (1UL << 9 )
    #define FLASH_ACC_FLSPROT_FPBY16_10 (1UL << 10)
    #define FLASH_ACC_FLSPROT_FPBY16_11 (1UL << 11)
    #define FLASH_ACC_FLSPROT_FPBY16_12 (1UL << 12)
    #define FLASH_ACC_FLSPROT_FPBY16_13 (1UL << 13)
    #define FLASH_ACC_FLSPROT_FPBY16_14 (1UL << 14)
    #define FLASH_ACC_FLSPROT_FPBY16_15 (1UL << 15)

    #define FLASH_ACC_FLSPROT_FUP512B_0 (1UL << 24 )
    #define FLASH_ACC_FLSPROT_FUP512B_1 (1UL << 25 )
    #define FLASH_ACC_FLSPROT_FUP512B_2 (1UL << 26 )
    #define FLASH_ACC_FLSPROT_FUP512B_3 (1UL << 27 )
    #define FLASH_ACC_FLSPROT_FUP512B_4 (1UL << 28 )
    #define FLASH_ACC_FLSPROT_FUP512B_5 (1UL << 29 )
    #define FLASH_ACC_FLSPROT_FUP512B_6 (1UL << 30 )
    #define FLASH_ACC_FLSPROT_FUP512B_7 (1UL << 31 )

    #define FLASH_ACC_OTPPROT_OP0 (1UL << 0)
    #define FLASH_ACC_OTPPROT_OP1 (1UL << 1)
    #define FLASH_ACC_OTPPROT_OP2 (1UL << 2)

    #define FLASH_ACC_TRIMPROT_PROT0 (1UL << 0)
    #define FLASH_ACC_TRIMPROT_PROT1 (1UL << 1)
    #define FLASH_ACC_TRIMPROT_PROT2 (1UL << 2)
    #define FLASH_ACC_TRIMPROT_PROT3 (1UL << 3)
    #define FLASH_ACC_TRIMPROT_PROT4 (1UL << 4)
    #define FLASH_ACC_TRIMPROT_PROT5 (1UL << 5)
    #define FLASH_ACC_TRIMPROT_PROT6 (1UL << 6)
    #define FLASH_ACC_TRIMPROT_PROT7 (1UL << 7)
    #define FLASH_ACC_TRIMPROT_PROT8 (1UL << 8)
    #define FLASH_ACC_TRIMPROT_RED   (1UL << 9)
    #define FLASH_ACC_TRIMPROT_LDT   (1UL << 10)
    #define FLASH_ACC_TRIMPROT_DCT   (1UL << 11)
    #define FLASH_ACC_TRIMPROT_TEST  (1UL << 12)

    #define WRITE_BUSY_POLLING  	while(CFMC->STAT & FLASH_ACC_STAT_WBUSY)
    #define CHKSUM_BUSY_POLLING 	while(CFMC->STAT & FLASH_ACC_STAT_CBUSY)
		#define MIO32(addr)  					(*(volatile unsigned int *)(addr))

		
		
//--------------------------------------------------------------
//		Public Function
//--------------------------------------------------------------
void flash_acc_latency(uint8_t latency);
void flash_acc_prefetch_en(void);
void flash_acc_inst_cache_en(void);
void flash_acc_data_cache_en(void);
void flash_acc_prefetch_dis(void);
void flash_acc_inst_cache_dis(void);
void flash_acc_data_cache_dis(void);
void flash_acc_prefetch_reset(void);
void flash_acc_inst_cache_reset(void);
void flash_acc_data_cache_reset(void);
void all_cache_restart(void);
void data_cache_restart(void);
void flash_acc_boot_lock(void);
void flash_acc_otp_lock(void);
void flash_acc_flash_lock(void);
void flash_acc_trim_lock(void);
void flash_acc_otp_unlock(void);
void flash_acc_flash_unlock(void);
void flash_acc_trim_unlock(void);
void flash_acc_write_done_int_en(void);
void flash_acc_chksum_done_int_en(void);
void flash_acc_reset(void);
void flash_acc_latch(uint32_t addr, uint32_t data);
void flash_acc_program(uint32_t addr, uint32_t data);
void flash_acc_page_erase(uint32_t addr);
void flash_acc_sect1k_erase(uint32_t addr);
void flash_acc_sect4k_erase(uint32_t addr);
void flash_acc_chip_erase(void);
void flash_acc_checksum_back_ground(uint32_t start_addr, uint32_t end_addr);
void flash_acc_checksum_burst(uint32_t start_addr, uint32_t end_addr);
void flash_acc_staus_check(void);


		
		
#endif
