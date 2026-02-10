/*
 * custom.c
 *
 *  Created on: Dec 31, 2025
 *  Author: Debasish Das
 */
#include "stm32f769xx.h"
#include "custom.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

void _gettimeofday()
{
}

void convrtEngUnit(uint8_t *pRxData, volatile float *xg, volatile float *yg,
		volatile float *zg)
{
	int16_t signed_x, signed_y, signed_z;

	signed_x = (int16_t) ((pRxData[1] << 8) | pRxData[0]);
	signed_y = (int16_t) ((pRxData[3] << 8) | pRxData[2]);
	signed_z = (int16_t) ((pRxData[5] << 8) | pRxData[4]);

	signed_x = signed_x >> 4;
	signed_y = signed_y >> 4;
	signed_z = signed_z >> 4;

	*xg = signed_x * 0.06f / 100.0f;
	*yg = signed_y * 0.06f / 100.0f;
	*zg = signed_z * 0.06f / 100.0f;
}

/* Code to Reset the UDSART2 */
void ResetUSART(UART_HandleTypeDef *huart)
{
	HAL_UART_Abort(huart);
	HAL_UART_DeInit(huart);
	HAL_UART_Init(huart);
}

void writeHexBuffer(uint8_t *pBuffer, uint16_t wLen)
{
	uint16_t wCount = 0;

	for (wCount = 0; wCount < wLen; wCount++)
	{
		*(pBuffer++) = wCount + 48;
	}
}

void writetoSerial(UART_HandleTypeDef *huart, char *pMsg)
{
	HAL_UART_Transmit(huart, (uint8_t*) pMsg, strlen(pMsg), 100);
}

void formattoSerial(UART_HandleTypeDef *huart, char *pMsg, int msgLenIn,
		int *msgLenOut, uint32_t dwData, const char *pCustMsg)
{
	*msgLenOut = snprintf(pMsg, msgLenIn, "%s : 0x%08lX\r\n", pCustMsg, dwData);

	if (!*msgLenOut)
	{
		writetoSerial(huart, "Format message to UART failed..\n\r");
	}
}

void writeHextoSerial(UART_HandleTypeDef *huart, const uint8_t *pArray,
		int wArrSize)
{
	HAL_UART_Transmit(huart, (uint8_t*) pArray, wArrSize, HAL_MAX_DELAY);
}

/**
 * @brief Sending formatted hex array to UART terminal
 * @retval None
 */
void writeASCIItoSerial(UART_HandleTypeDef *huart, uint8_t bFormat,
		const uint8_t *pArray, int wArrSize, const char *pHeader)
{
	int bLen = 0;
	char outBuff[MAX_ARR_SIZE];
	uint8_t bPos = 0;

	if (!huart || !pArray || wArrSize == 0)
	{
		writetoSerial(huart, "Invalid parameters !\r\n");
		return;
	}

	char *format;
	format = (bFormat == ASCII) ? "%c" : "%02X ";

	if (MAX_ARR_SIZE < wArrSize * 3 + 2)
	{
		writetoSerial(huart, "Insufficient output buffer size !\r\n");
		return;
	}

	if (pHeader)
	{
		bPos += snprintf(&outBuff[bPos], sizeof(outBuff), "%-15.15s: ",
				pHeader);
	}
	else
		bPos += snprintf(&outBuff[bPos], sizeof(outBuff), "%-15.15s: ", " ");

	for (bLen = 0; bLen < wArrSize; bLen++)
	{

		bPos += snprintf(&outBuff[bPos], (sizeof(outBuff) - bPos), format,
				pArray[bLen]);
	}
	outBuff[bPos++] = '\r';
	outBuff[bPos++] = '\n';

	HAL_UART_Transmit(huart, (uint8_t*) outBuff, bPos, HAL_MAX_DELAY);
}

void binaryToASCII(const uint8_t *pHex, uint16_t wHexLen, char *pAscii)
{
	uint16_t count = 0;
	for (count = 0; count < wHexLen; count++)
	{
		if (pHex[count] >= 0x20 && pHex[count] <= 0x7E)
		{
			pAscii[count] = (char) pHex[count];
		}
		else
			pAscii[count] = '.';
	}
	pAscii[count] = '\0';
}

void getCpuId()
{
	char aCpuID[20];
	volatile uint32_t dwCpuID = SCB->CPUID;
	snprintf(aCpuID, sizeof(aCpuID), "0x%" PRIX32, dwCpuID);;

	writeASCIItoSerial(&huart1, ASCII, aCpuID, strlen(aCpuID), "CPU ID");
}
