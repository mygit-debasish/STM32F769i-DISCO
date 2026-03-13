/*
 * custom_timer.h
 *
 *  Created on: Mar 6, 2026
 *      Author: Debasish Das
 */

#ifndef INC_CUSTOM_TIMER_H_
#define INC_CUSTOM_TIMER_H_

#include "custom.h"

#define CH6 6U
#define CH1 1U
#define PATT_SIZE 2U

/* Private function Prototypes */

void STM32F769_USER2_LED_Init();
void STM32F769_TIM1_Init(uint8_t timeBase_Hz);
void STM32F769_TIM2_Init(uint8_t timeBase_Sec);

void STM32F769_DMA2_Stream5_Init(uint32_t *pMemSrc, uint32_t bPattSize);
void TIM1_IRQHandler();

void STM32F769_DMA2_Stream1_Init(uint8_t *pMemSrc, uint8_t *pMemDst,
		uint32_t buffSize);

#endif /* INC_CUSTOM_TIMER_H_ */
