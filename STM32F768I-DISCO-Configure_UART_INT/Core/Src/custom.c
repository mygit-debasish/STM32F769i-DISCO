/*
 * custom.c
 *
 *  Created on: Dec 31, 2025
 *  Author: Debasish Das
 */

#include "custom.h"

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
	snprintf(aCpuID, sizeof(aCpuID), "0x%" PRIX32, dwCpuID);
	;

	writeASCIItoSerial(&huart1, ASCII, aCpuID, strlen(aCpuID), "CPU ID");
}

HAL_StatusTypeDef returnError()
{
	return HAL_ERROR;
}
HAL_StatusTypeDef returnSuccess()
{
	return HAL_OK;
}

void LOG_Error_Status(int bError, const char *pFile, uint8_t bLineNum,
		const char *pFunName, const char *pMsg)
{
	int len = 0;
	char aMsgBanner[MAX_ARR_SIZE];

	len = snprintf(aMsgBanner, sizeof(aMsgBanner), "Error Code: %d ", bError);
	len += snprintf(&aMsgBanner[len], sizeof(aMsgBanner) - len,
			"File: %s : Line: %d : Function: %s : %s\r\n", pFile, bLineNum, pFunName, pMsg);

	if(len <0 || len > sizeof(aMsgBanner))
		return;

	writetoSerial(&huart1, aMsgBanner);
}

const char* ErrorString(ERROR_t bError)
{
	switch(bError)
	{
	case FUN_NOERROR: 		return "No Error";
	case FUN_ERROR: 		return "Function Error";
	case FUN_ERROR_INVALID: return "Invalid function";
	case FUN_ERROR_EMPTY: 	return "Empty Function";
	}

	return SUCCESS;
}

void writeFormatData(UART_HandleTypeDef *huart, const char *format, ...)
{
	char txBuffer[SERIAL_TX_BUFF_SIZE];
	va_list args;
	int txBufferLen = 0;

	/* Initialize variable argument list */
	va_start(args, format);

	/* Format the string */
	txBufferLen = vsnprintf(txBuffer, sizeof(txBuffer), format, args);

	/*  End variale argument passing */
	va_end(args);

	if( txBufferLen < 0 )
	{
		return;
	}

	if(txBufferLen >= SERIAL_TX_BUFF_SIZE)
		txBufferLen = sizeof(txBuffer);

	HAL_UART_Transmit(huart, txBuffer, txBufferLen, HAL_MAX_DELAY);
}

void UART1_TxRx_init()
{
	/* PA9 -> Transmit
	 * Pa10 -> Received */

	/******************  GPIO setup *****************************/

	/* Enabling the clock for GPIOA*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	/* Enabling the CLK for UART1 module */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	/*Selecting alternate function Mode : AF7 */

	/*  UART1 Transmiter MODER setting */
	GPIOA->MODER &= ~(3U << (9 * 2));
	GPIOA->MODER |= GPIO_MODER_MODER9_1;

	/*  UART1 Receiver  MODER setting */
	GPIOA->MODER &= ~(3U << (10 * 2));
	GPIOA->MODER |= GPIO_MODER_MODER10_1;


	/* UART1 AF7 setting for transmitter */
	GPIOA->AFR[1] &= ~(0x0F << 4U);
	GPIOA->AFR[1] |= GPIO_AF7_USART1 << 4U;

	GPIOA->AFR[1] &= ~(0x0F << 8U);
	GPIOA->AFR[1] |= GPIO_AF7_USART1 << 8U;

	/* Pull up  register - Receive */
	GPIOA->PUPDR &= ~(0x0F << 18);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;

	/******************  USART setup *****************************/
	/* Disable the UART1 and make sure transmission has been completed */
	USART1->CR1 &= ~USART_CR1_UE;

	/* Set Baud rate */
	USART1->BRR = (USART1_CLK + (BAUD_RATE/2U)) / BAUD_RATE;

	/* Enable transmit and receive */
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

	/* Start-Bit, Stop-Bit & Parity-Bit are default configuration of USART_CR1
	 * Register */

	/*Enable UART1 */
	USART1->CR1 |= USART_CR1_UE;
}

/*Writing complete character to a USART */
void UART1_Tx_Inter_write_char(char ch)
{
	/* Wait for Transfer Register is Empty */
	while(!(USART1->ISR & USART_ISR_TXE));

	/* Writing the data to Transmit Data Register */
	USART1->TDR = (ch & 0xFF);
}

void UART1_Tx_Inter_write_str(const char *pData, size_t wDataLen)
{
	while (*pData)
	{
		while (!(USART1->ISR & USART_ISR_TXE));
		USART1->TDR = (uint8_t) *pData++;
	}

	while (!(USART1->ISR & USART_ISR_TXE));
	UART1_Tx_Inter_write_char('\r');

	while (!(USART1->ISR & USART_ISR_TXE));
	UART1_Tx_Inter_write_char('\n');

	while (!(USART1->ISR & USART_ISR_TC));
}


 uint8_t UART1_Rx_Inter_read_char()
{
	uint8_t CharData;
	/* Wait for Transfer Register is Empty */

	while (!(USART1->ISR & USART_ISR_RXNE));
	CharData = (uint8_t) USART1->RDR & 0xFF;

	return CharData;
}

size_t UART1_Rx_Inter_read_string(uint8_t *pOutData, size_t wDataLen)
{
	size_t wLen = 0;
	uint8_t ch;

	if (!pOutData || wDataLen == 0)
		return ERROR;

	for (; wLen < wDataLen - 1; wLen++)
	{
		ch = UART1_Rx_Inter_read_char();

		if (ch == '\r' || ch == '\n')
			break;

		pOutData[wLen] = ch;
	}
	pOutData[wLen] = '\0';

	return wLen;
}



