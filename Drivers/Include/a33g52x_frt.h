/**
**********************(C) Copyright 2017 ABOV Semiconductor Co., Ltd *******************
* @ File : a33g52x_frt.h
* 
* @ Author : Application Team, ABOV Semiconductor Co., Ltd 
*
* @ Version : V1.0
*
* @ Date : Sep, 2017
*
* @ Description 
*   ABOV Semiconductor is supplying this software for use with A33G52x
*   processor. This software contains the confidential and proprietary information
*   of ABOV Semiconductor Co., Ltd ("Confidential Information").
*
*
**************************************************************************************
* DISCLAIMER 
*
* 	THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS  
* 	WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE  
* 	TIME. AS A RESULT, ABOV SEMICONDUCTOR DISCLAIMS ALL LIABILITIES FROM ANY
* 	DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING  
* 	FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE  
* 	CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.  
*
**************************************************************************************
*/


#include <stdint.h>
#include "A33G52x.h"


//==========================================================================
// 	FRTPRD
//		
//				@ address = 0x4000_0500
//
//==========================================================================





//==========================================================================
// 	FRTCNT
//		
//				@ address = 0x4000_0504
//
//==========================================================================





//==========================================================================
// 	FRTCON
//		
//				@ address = 0x4000_0508
//
//==========================================================================
#define FRTCON_FMF							(0x0001<<9)
#define FRTCON_FOF							(0x0001<<8)

#define FRTCON_FEC							(0x0001<<7)
#define FRTCON_FEC_PCLK						(0x0000<<7)
#define FRTCON_FEC_PMUPCSR					(0x0001<<7)

#define FRTCON_FPRS_FRTCLKIN_DIV_BY_1		(0x0000<<4)
#define FRTCON_FPRS_FRTCLKIN_DIV_BY_4		(0x0001<<4)
#define FRTCON_FPRS_FRTCLKIN_DIV_BY_8		(0x0002<<4)
#define FRTCON_FPRS_FRTCLKIN_DIV_BY_16		(0x0003<<4)
#define FRTCON_FPRS_FRTCLKIN_DIV_BY_32		(0x0004<<4)
#define FRTCON_FPRS_FRTCLKIN_DIV_BY_64		(0x0005<<4)
#define FRTCON_FPRS_FRTCLKIN_DIV_BY_128		(0x0006<<4)
#define FRTCON_FPRS_FRTCLKIN_DIV_BY_256		(0x0007<<4)
#define FRTCON_FPRS_MASK					(0x0007<<4)

#define FRTCON_FMIE							(0x0001<<3)
#define FRTCON_FOIE							(0x0001<<2)

#define FRTCON_FAC							(0x0001<<1)
#define FRTCON_FAC_FREERUN					(0x0000<<1)
#define FRTCON_FAC_PERIODIC					(0x0001<<1)



#define FRTCON_FEN							(0x0001<<0)








//==========================================================================
// 
//		F U N C T I O N    D E C L A R A T I O N S 
//
//==========================================================================
void FRT_ConfigureInterrupt (FRT_Type  * const frt, uint32_t intr_mask, uint32_t enable); 






