/**********************************************************************
* @file		A34M41x_adc.c
* @brief	Contains all functions support for ADC firmware library
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
#include "A34M41x_hal_adc.h"
#include "A34M41x_hal_scu.h"
#include "A34M41x_hal_libcfg.h"


void HAL_ISTOD_EX(void)
{
	ADC_CFG_Type AD_config;
	char var;

	// ADC Configuration
	AD_config.Mode = ADC_SINGLE_MODE;
	AD_config.SamplingTime = 0xa;
	AD_config.SeqCnt = 1;
	AD_config.RestartEn = 0;
	AD_config.TrgSel = ADC_TRIGGER_DISABLE;
	
	AD_config.UseClk = ADC_EXTERNAL_CLK;
	AD_config.InClkDiv = 1;	
	HAL_ADC_Init(ADC0, &AD_config);
	HAL_ADC_Init(ADC1, &AD_config);
	HAL_ADC_Init(ADC2, &AD_config);
	
	//	----------------------------------------- Source Code ----------------------------------------------
	for (var=0; var<2; var++)
	{
		 ADC0->CCR = 0x0000;	// ADC control : 1/1 clk, PD off, sample 2 cycle
		 ADC1->CCR = 0x0000;	// ADC control : 1/1 clk, PD off, sample 2 cycle
		 ADC2->CCR = 0x0000;	// ADC control : 1/1 clk, PD off, sample 2 cycle
		 ADC0->MR = 0x010080;	// ADC mode reg : ADC enable, single
		 ADC1->MR = 0x010080;	// ADC mode reg : ADC enable, single
		 ADC2->MR = 0x010080;	// ADC mode reg : ADC enable, single
		 ADC0->SR = 0x1;
		 ADC1->SR = 0x1;
		 ADC2->SR = 0x1;
		 ADC0->CR = 0x000001;  // ADC start
		 ADC1->CR = 0x000001;  // ADC start
		 ADC2->CR = 0x000001;  // ADC start
		 while((ADC0->SR & 0x01) == 0)
		 {
				 ; /* do nothing*/
		 }
		 while((ADC1->SR & 0x01) == 0)
		 {
			 ; /* do nothing*/
		 }
		 while((ADC2->SR & 0x01) == 0)
		 {
			 ; /* do nothing*/
		 }
	}

	ADC0->CCR = 0x0080;
	ADC1->CCR = 0x0080;
	ADC2->CCR = 0x0080;
	ADC0->MR  = 0x0000;
	ADC1->MR  = 0x0000;
	ADC2->MR  = 0x0000;
	//	------------------------------------------------------------------------------------------------------

	HAL_ADC_Stop(ADC0);
	HAL_ADC_Stop(ADC1);
	HAL_ADC_Stop(ADC2);
}
/* Public Functions ----------------------------------------------------------- */
/**********************************************************************
 * @brief 		Initial for ADC
 * @param[in]	ADCx pointer to ADC_Type, should be: ADCx
 * @param[in]	ADC_ConfigStruct
 * @return 		HAL Status
 **********************************************************************/
HAL_Status_Type HAL_ADC_Init(ADC_Type *ADCx, ADC_CFG_Type *ADC_ConfigStruct)
{
	uint32_t tempreg;


	 /* Check ADC handle */
         if(ADCx == NULL)
        {
            return HAL_ERROR;
         }
		 
	//System config enable
	SYST_ACCESS_EN();
	
	// Turn on power and clock
	if(ADCx == ADC0)
	{
		SCU->PER2&=~(1<<20);
		SCU->PCER2&=~(1<<20);
		/* Set up peripheral clock for ADC0 module */
		SCU->PER2|=(1<<20);
		SCU->PCER2|=(1<<20);
	}
	else if(ADCx == ADC1)
	{
		SCU->PER2&=~(1<<21);
		SCU->PCER2&=~(1<<21);
		/* Set up peripheral clock for ADC1 module */
		SCU->PER2|=(1<<21);
		SCU->PCER2|=(1<<21);
	}
	else if(ADCx == ADC2)
	{
		SCU->PER2&=~(1<<22);
		SCU->PCER2&=~(1<<22);
		/* Set up peripheral clock for ADC2 module */
		SCU->PER2|=(1<<22);
		SCU->PCER2|=(1<<22);
	}
	
	//System config disable
	SYST_ACCESS_DIS();
	
		tempreg=0;
	switch(ADC_ConfigStruct->Mode){
		case ADC_SINGLE_MODE:
			tempreg = 0
				| (1<<17) // DMAEN
//				| ((ADC_ConfigStruct->DmaOpt & 1) << 17) // DMAEN   Modified (2019.11.20)
				| ((ADC_ConfigStruct->SamplingTime & 0x1f)<<12)  // STSEL
				| (((ADC_ConfigStruct->SeqCnt-1) & 7)<<8) 		// SEQCNT
				| (1<<7)  // ADCEN
				| (((ADC_ConfigStruct->RestartEn) & 1)<<6) 		// ARST
				| (0<<4)  // ADCMOD
				| (((ADC_ConfigStruct->TrgSel) & 3)<<0)  		// TRGSRC
				;
			break;
		case ADC_BURST_MODE:
			tempreg = 0
				| (1<<17) // DMAEN
//				| ((ADC_ConfigStruct->DmaOpt & 1) << 17) // DMAEN   Modified (2019.11.20)        
				| ((ADC_ConfigStruct->SamplingTime & 0x1f)<<12)  // STSEL //
				| (((ADC_ConfigStruct->SeqCnt-1) & 7)<<8)  		// SEQCNT//	| ((ADC_ConfigStruct->SeqCnt+3)<<8)  // SEQCNT
				| (1<<7)  // ADCEN
				| (((ADC_ConfigStruct->RestartEn) & 1)<<6)  	// ARST
				| (1<<4)  // ADCMOD
				| (((ADC_ConfigStruct->TrgSel) & 3)<<0)  		// TRGSRC
				;
			break;
		case ADC_MULTI_MODE:
			tempreg = 0
				| (1<<17)	// DMAEN
//				| ((ADC_ConfigStruct->DmaOpt & 1) << 17) // DMAEN   Modified (2019.11.20)        
				| ((ADC_ConfigStruct->SamplingTime & 0x1F)<<12)	// STSEL
				| (((ADC_ConfigStruct->SeqCnt-1) & 7)<<8)  		// SEQCNT//	| ((ADC_ConfigStruct->SeqCnt+3)<<8)  // SEQCNT
				| (1<<7)  // ADCEN
				| (((ADC_ConfigStruct->RestartEn) & 1)<<6)  	// ARST
				| (2<<4)  // ADCMOD
				| (((ADC_ConfigStruct->TrgSel) & 3)<<0)  		// TRGSRC
				;
			break;
		
		default:
			break;
	}
	ADCx->MR = tempreg;


	tempreg = 0
		| (0<<8)  // CKDIV  1.5Mhz * 15 = 22.5Mhz
		| (0<<7)  // ADCPD
		| (ADC_ConfigStruct->UseClk<<6)  // CLKSEL
		| (0<<5)  // CLKINVT
		;
	if (ADC_ConfigStruct->UseClk == ADC_INTERNAL_CLK) {
		tempreg |= ((ADC_ConfigStruct->InClkDiv & 0x7f)<<8);	// CKDIV  1.5Mhz * 15 =22.5Mhz,  PCLK / CLKDIV =48Mhz / 4 =  12Mhz

	}
//	else if (ADC_ConfigStruct->UseClk == ADC_EXTERNAL_CLK) {
//		SCU_SetMCCRx(4, ADCx_TYPE, SCU_MCCR_CSEL_MCLK, 2);	// ACLK = MCLK/2
//	}
	
	ADCx->CCR = tempreg;
	ADCx->CSCR = 0;

	HAL_ADC_Stop(ADCx);
	return HAL_OK;
}


/**********************************************************************
* @brief 		Close ADC
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @return 		HAL Status
**********************************************************************/
HAL_Status_Type HAL_ADC_DeInit(ADC_Type *ADCx)
{
	/* Check ADC handle */
	if(ADCx == NULL)
	{
		 return HAL_ERROR;
	}

	ADCx->CR = (1<<7); // stop
	ADCx->CCR = 0;
	ADCx->IER = 0;
	ADCx->MR = 0;
	return HAL_OK;
}

HAL_Status_Type CSP_ADC_SetMR(ADC_Type *ADCx, uint32_t mode)
{
	/* Check ADC handle */
	if(ADCx == NULL)
	{
		 return HAL_ERROR;
	}
	ADCx->MR = mode;
	return HAL_OK;
}
/**********************************************************************
* @brief 		Get Result conversion from A/D data register
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @return 		Result of conversion
**********************************************************************/
uint16_t HAL_ADC_GetData(ADC_Type *ADCx)
{
	uint16_t adc_value;


	adc_value = (ADCx->DDR>>4);
	return ADC_DR_RESULT(adc_value);
}

/**********************************************************************
* @brief 		Set start mode for ADC
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @return 		HAL Status
*********************************************************************/
HAL_Status_Type HAL_ADC_Start(ADC_Type *ADCx)
{
      	/* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	
	ADCx->CR =ADC_CR_START_MASK;
	
	HAL_ADC_GetData(ADCx); // dummy data read
	return HAL_OK;
}

/**********************************************************************
* @brief 		Set stop mode for ADC
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @return 		HAL Status
*********************************************************************/
HAL_Status_Type HAL_ADC_Stop(ADC_Type *ADCx)
{
      	/* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	ADCx->CR =ADC_CR_STOP_MASK;
	return HAL_OK;
}

/**********************************************************************
* @brief 		Set AD conversion in power mode
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @param[in]	NewState
* 					- DISABLE : AD converter is normal mode
* 					- ENABLE: AD Converter is in power down mode
* @return 		HAL Status
**********************************************************************/
HAL_Status_Type HAL_ADC_EnterPowerdownMode(ADC_Type *ADCx, FunctionalState NewState)
{
       	/* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	ADCx->CCR &= ~ADC_CCR_PDN;
	if (NewState){
		ADCx->CCR |= ADC_CCR_PDN;
	}
	return HAL_OK;
}

/**********************************************************************
* @brief 		ADC interrupt configuration
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @param[in]	IntType: type of interrupt, should be:
* 					- ADC_INTEN_DMA
* 					- ADC_INTEN_TRIGGER
* 					- ADC_INTEN_SEQ
* 					- ADC_INTEN_SINGLE
* @param[in]	NewState:
* 					- SET : enable ADC interrupt
* 					- RESET: disable ADC interrupt
* @return 		HAL Status
**********************************************************************/
HAL_Status_Type HAL_ADC_ConfigInterrupt(ADC_Type *ADCx, uint8_t IntType, FunctionalState NewState)
{
       	/* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	ADCx->IER &= ~IntType;
	if (NewState){
		ADCx->IER |= IntType;
	}
	return HAL_OK;
}

/**********************************************************************
* @brief 		Select ADC channel number
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @param[in]	Channel channel number
* @return 		HAL Status
**********************************************************************/
HAL_Status_Type HAL_ADC_ChannelSel(ADC_Type *ADCx, uint32_t Channel)
{
        /* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	ADCx->SCSR1 = Channel;
	return HAL_OK;
}

/**********************************************************************
* @brief 		Select ADC channel number
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @param[in]	Channel channel number
* @return 		HAL Status
**********************************************************************/
HAL_Status_Type HAL_ADC_ChannelSel_1(ADC_Type *ADCx, uint32_t Channel)
{
       	/* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	ADCx->SCSR1 = Channel;
	return HAL_OK;
}

/**********************************************************************
* @brief 		Select ADC channel number
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @param[in]	Channel channel number
* @return 		HAL Status
**********************************************************************/
HAL_Status_Type HAL_ADC_ChannelSel_2(ADC_Type *ADCx, uint32_t Channel)
{
     	/* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	ADCx->SCSR2 = Channel;
	return HAL_OK;
}

/**********************************************************************
* @brief 		Select Burst ADC channel number
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @param[in]	Channel channel number
* @return 		HAL Status
**********************************************************************/
HAL_Status_Type HAL_ADC_TriggerlSel(ADC_Type *ADCx, uint32_t Trigger)
{
      	/* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	ADCx->TRG = Trigger;
	return HAL_OK;
}

/**********************************************************************
* @brief 		Get ADC channel status
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @return 		ADC status register
**********************************************************************/
uint32_t HAL_ADC_GetStatus(ADC_Type *ADCx)
{

	return (ADCx->SR);
}

/**********************************************************************
* @brief 		Clear ADC Chanel status
* @param[in]	ADCx pointer to ADC_Type, should be: ADCx
* @param[in]	status  channel number
* @return 		HAL Status
**********************************************************************/
HAL_Status_Type HAL_ADC_ClearStatus(ADC_Type *ADCx, uint32_t status)
{
      	/* Check ADC handle */
	if(ADCx == NULL)
	{
		return HAL_ERROR;
	}
	ADCx->SR=status;
	return HAL_OK;
}


