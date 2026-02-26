/*
 * custom_ADC.h
 *
 *  Created on: Feb 24, 2026
 *      Author: Debasish Das
 */

#ifndef INC_CUSTOM_ADC_H_
#define INC_CUSTOM_ADC_H_

#include <stm32f769xx.h>
#include "custom.h"


/*************** ADC1 function Prototypes *******************/
void ADC1_Init();

void ADC1_StartConversion();
void ADC1_StopConversion();

void ADC1_Interrupt_Initialization();
void ADC1_DMA_Initialization(uint16_t *pMemoryDst, uint16_t wDataLen);

void ADC_Callback(uint16_t dataADC);

#endif /* INC_CUSTOM_ADC_H_ */
