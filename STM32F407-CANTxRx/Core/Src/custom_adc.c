/*
 * custom_adc.c
 *
 *  Created on: Aug 26, 2026
 *      Author: Debasish Das
 */

#include "custom_adc.h"
#include "stm32f4xx.h"
#include "custom.h"
#include "core_temp.h"

extern CAN_TxHeaderTypeDef TxHeaderTemp;
extern CAN_TxHeaderTypeDef TxHeaderPres;
extern CAN_TxHeaderTypeDef TxHeaderBaro;
extern uint32_t TxMailbox;
extern CAN_HandleTypeDef hcan1;


extern UART_HandleTypeDef huart3;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	float coreTemp = 0.0f;

	if (hadc->Instance == ADC1)
	{
		coreTemp = STM32_ReadTemperature();
		DD_CAN_AddTemperatureMessage(&hcan1, &TxHeaderTemp, coreTemp);
	}
}

void DD_CAN_AddTemperatureMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, float coreTemperature)
{
	uint8_t Core_Temperature[2];
	uint16_t parse_Temperature = 0;

	if(coreTemperature <= 0.0f )
	{
		return;
	}

	parse_Temperature = (uint16_t)(coreTemperature * 100.0f);
	Core_Temperature[0] = (uint8_t)(parse_Temperature >> 8U);
	Core_Temperature[1] = (uint8_t)(parse_Temperature &  0xFFU);

	TxHeader->DLC = 2U;

	/* Add Temperature Message to CAN1 */
	uint32_t TxMailbox;

	if (HAL_CAN_AddTxMessage(hcan, TxHeader, Core_Temperature, &TxMailbox)
			== HAL_OK)
	{
		writeFormatData(&huart3, "[%s] [TEMPERATURE MSG ADDED]\r\n", __func__);
	}
}

