/*
 * custom_F407_CAN.c
 *
 *  Created on: Aug 20, 2026
 *      Author: Debasish Das
 */

#include "custom_F407_CAN.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "customITM.h"
#include  "custom.h"
#include  "core_temp.h"



/** Extern declaration **/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern CAN_TxHeaderTypeDef TxHeaderTemp;
extern CAN_TxHeaderTypeDef TxHeaderPres;
extern CAN_TxHeaderTypeDef TxHeaderBaro;
extern uint32_t TxMailbox;
extern ADC_HandleTypeDef hadc1;

/* CAN receive variables */
extern CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];


/* ============================================================
 * CAN1 TX mailbox0CompleteCallback
 * ============================================================ */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	if (hcan->Instance == CAN1)
    {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

/* ============================================================
 * CAN2 RX RxFifoMsgPendingCallbackCallback
 * ============================================================ */

char recvMsg[100];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN2)
	{
		CAN_RxHeaderTypeDef RxHeader;
		if (HAL_CAN_GetRxMessage(hcan,
		CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		{
			if (RxHeader.StdId == MSG_ID_TEMP)
			{
				writetoSerial(&huart3, "[RECEIVED TEMP] \r\n");
				writeHextoSerial(&huart3, "[CoreTemperature]: ", RxData, 2U);
				writeFormatData(&huart3, "[RECEIVED TEMP] : %.2f Celsius \r\n",
						DD_CAN_ConvertTemperatureMessage(RxData, RxHeader.DLC));
			}

			if (RxHeader.StdId == MSG_ID_PRES)
			{
				writetoSerial(&huart3, "[RECEIVED PRES] \r\n");
			}
		}
	}
}

/* ============================================================
 * TIM3 PERIOD Elasped Callback
 * ============================================================ */

uint8_t TxDataTEMP[8] = {0xDE, 0xBA, 0x51, 0x56, 0xDE, 0xAD, 0xBE, 0xA7};
uint8_t TxDataPRES[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		/* Start ADC1 for Reporting MCU core temperature */
		HAL_ADC_Start_IT(&hadc1);

		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeaderPres, TxDataPRES, &TxMailbox)
				== HAL_OK)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		}
		else
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		}
	}
}

float DD_CAN_ConvertTemperatureMessage(uint8_t *pRxData, uint8_t DataLen)
{
	float TemperatureEU = 0.0f;

	if((pRxData == NULL) || (DataLen < 2U))
	{
		return 0.0f;
	}

	uint16_t TemperatureU16 = 	((uint16_t)pRxData[0] << 8U) |
								((uint16_t)pRxData[1] & 0xFF);

	TemperatureEU = (float)TemperatureU16 / 100.0f;
	return TemperatureEU;
}
