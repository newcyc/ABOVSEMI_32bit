/**********************************************************************
* @file		A34M41x_aes128.c
* @brief	Contains all functions support for aes128 firmware library
* 			on A34M41x
* @version	1.0
* @date		
* @author	ABOV Application2 team
*
* Copyright(C) 2017, ABOV Semiconductor
* All rights reserved.
*
**********************************************************************/

/* Includes ------------------------------------------------------------------- */
#include "A34M41x_hal_aes128.h"
#include "A34M41x_hal_libcfg.h"


/* Public Functions ----------------------------------------------------------- */
/*********************************************************************
 * @brief		Initializes the AES128 peripheral 
 * @param[in]	None
 * @return 		HAL Status
 *********************************************************************/
void HAL_AES128_Init(void)
{
	//System config enable
	SCU->SYSTEN=0x57; SCU->SYSTEN=0x75;
	
	SCU->PER2 &= ~(1UL<<30);
	SCU->PCER2 &= ~(1UL<<30);
	
	SCU->PER2 |= (1UL<<30);
	SCU->PCER2 |= (1UL<<30);

	//System config disable
	SCU->SYSTEN=0x00;
}

/**********************************************************************
 * @brief		Set AES128 Text Input
 * @param[in]	numOfText
 *				the number of Text Input Register (0~3)
 * @param[in]	data
 *				Text Input Data (32-bit)
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_AES128_SetAESTextIn(uint32_t *in)
{
	uint32_t i;
	
	FLUSH_IN_FIFO();
	FLUSH_OUT_FIFO();
	
	for(i=0;i<4;i++){
		AES128->INFIFO = in[i];
	}

}

/**********************************************************************
 * @brief		Get AES128 Text Input
 * @param[in]	numOfText
 *				the number of Text Input Register (0~3)
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_AES128_GetAESTextIn(uint32_t *in)
{
	in[0] = AES128->TEXTIN0;
	in[1] = AES128->TEXTIN1;
	in[2] = AES128->TEXTIN2;
	in[3] = AES128->TEXTIN3;
}

/**********************************************************************
 * @brief		Get AES128 Text Output
 * @param[in]	numOfText
 *				the number of Text Output Register (0~3)
 * @return		Text Output Data (32-bit)
 *
 **********************************************************************/
void HAL_AES128_GetAESTextOut(uint32_t *out)
{
	out[0] = AES128->TEXTOUT0;
	out[1] = AES128->TEXTOUT1;
	out[2] = AES128->TEXTOUT2;
	out[3] = AES128->TEXTOUT3;
}

/**********************************************************************
 * @brief		Set AES128 Key Input
 * @param[in]	numOfText
 *				the number of Key Input Register (0~3)
 * @param[in]	data
 *				Key Input Data (32-bit)
 * @return		HAL Status
 *
 **********************************************************************/
void HAL_AES128_SetAESKeyIn(uint32_t *key)
{
	AES128->KEYIN0 = key[0];
	AES128->KEYIN1 = key[1];
	AES128->KEYIN2 = key[2];
	AES128->KEYIN3 = key[3];
}

/**********************************************************************
 * @brief		Get AES128 Key input
 * @param[in]	numOfText
 *				the number of Key Input Register (0~3)
 * @return		Key Input Data (32-bit)
 *
 **********************************************************************/
void HAL_AES128_GetAESKeyIn(uint32_t *key)
{
	key[0] = AES128->KEYIN0;
	key[1] = AES128->KEYIN1;
	key[2] = AES128->KEYIN2;
	key[3] = AES128->KEYIN3;
}

/**********************************************************************
 * @brief		AES128 Mode Selection
 * @param[in]	mode
 * 					- Cipher Mode
 * 					- Inverse Cipher Mode
 * @return 		HAL Status
 **********************************************************************/
void HAL_AES128_ModSel(eAES128_mode mode)
{
	AES128->CTRL &= ~(0x3);
	AES128->CTRL |= (mode&0x03);
}

/**********************************************************************
 * @brief		AES128 Word Order Selection
 * @param[in]	order
 * 					- Least Significant Word
 * 					- Most Significant Word
 * @return 		HAL Status
 **********************************************************************/
void HAL_AES128_OrderSel(eAES128_order In_order, eAES128_order Out_order)
{
	AES128->CTRL &= ~(1<<2);
	AES128->CTRL |= (((Out_order&0x7)<<5)|((In_order&0x7)<<2));
}



/***********************************************************************
 * @brief		Enable or disable specified interrupt type in AES128 peripheral
 * @param[in]	IntType	Interrupt type in SPI peripheral, should be:
 * 					- AES128_CTRL_CIPHERIE		:Cipher mode done interrupt
 * 					- AES128_CTRL_INVCIPHERIE	:Inverse Cipher mode done interrupt
 * 					- AES128_CTRL_DMAOUTIE		:DMA out complete interrupt
 * 					- AES128_CTRL_DMAINIE		:DMA in complete interrupt
 * @param[in]	NewState New State of specified interrupt type, should be:
 * 					- ENABLE	:Enable this interrupt type
 * 					- DISABLE	:Disable this interrupt type
 * @return		HAL Status
 **********************************************************************/
void HAL_AES128_ConfigInterrupt(uint32_t IntType, FunctionalState NewState)
{
	if((IntType & 0xF0000) != 0){
		if (NewState == ENABLE)
		{
			AES128->CTRL |= IntType;
		}
		else
		{
			AES128->CTRL &= (~IntType);
		}
	}
}

/**********************************************************************
 * @brief		Checks whether the specified AES128 status flag is set or not
 * @return		AES128 status flag
 **********************************************************************/
uint32_t HAL_AES128_GetStatus(void){
	return AES128->STAT;
}

/**********************************************************************
 * @brief		Clear status flag
 * @param[in]	status
 * @return		HAL Status
 **********************************************************************/
void HAL_AES128_ClearStatus(uint32_t status)
{
	AES128->STAT = status;
}

