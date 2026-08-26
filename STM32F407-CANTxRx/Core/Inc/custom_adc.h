/*
 * custom_adc.h
 *
 *  Created on: Aug 26, 2026
 *      Author: Debasish Das
 */

#ifndef INC_CUSTOM_ADC_H_
#define INC_CUSTOM_ADC_H_
#include "stm32f4xx.h"

/* Function Prototypes */
void DD_CAN_AddTemperatureMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, float coreTemperature);


#endif /* INC_CUSTOM_ADC_H_ */
