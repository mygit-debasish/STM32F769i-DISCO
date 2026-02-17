/*
 * cutom.h
 *
 *  Created on: Dec 31, 2025
 *      Author: debasish
 */

#ifndef INC_CUSTOM_H_
#define INC_CUSTOM_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stm32f769xx.h>

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

#define BAUD_RATE 		(115200U)
#define USART1_CLK 		(108000000U)

typedef enum
{
	FUN_NOERROR = 0, FUN_ERROR, FUN_ERROR_INVALID, FUN_ERROR_EMPTY
} ERROR_t;

#define SERIAL_TX_BUFF_SIZE 80

/* MACRO definition */

#define CHECK_ERROR_STATUS(x) do {						\
							ERROR_t retErr = (x);		\
							if(retErr != FUN_NOERROR)	\
							{							\
								LOG_Error_Status(retErr,	\
								__FILE__,			\
								__LINE__,			\
								__func__,			\
								ErrorString(retErr));	\
								return;				\
			}\
}while(0);

#define SPI_READ 	0x80
#define SPI_WRITE 	0x00

#define READ_MULTI 	0x40
#define WRITE_SINGLE 0x3F

#define ACCL_CTRL_REG 0x24U
#define ENB_DRDY 	0x80U
#define ENB_INTR1 	0x08U

/* System Clock Source */
typedef enum
{
	CLK_HSI = 0x00, CLK_HSE = 0x01, CLK_PLL = 0x02, CLK_NA = 0x03
} CLK_SRC;

#define ACCL_SCALE_2    (0U << 3)
#define ACCL_SCALE_4 	(1U << 3)
#define ACCL_SCALE_6 	(2U << 3)
#define ACCL_SCALE_8 	(3U << 3)
#define ACCL_SCALE_16 	(4U << 3)
#define ACC_ST_DISABLE 	(3U << 1)

#define MSP_ADDR 0x08000000U
#define GET_DATAU16(addr)	(*((volatile uint32_t*)(addr)))

#define BUFF_LEN 		128U
#define MAX_ARR_SIZE 	512U

void convrtEngUnit(uint8_t *pRxData, volatile float *xg, volatile float *yg,
		volatile float *zg);
void printSerialAcc(float xg, float yg, float zg);
bool ACCL_IsDataDataReady(void);
void ACCL_CS_ON();
void ACCL_CS_OFF();
void ACCL_Set_SetScale_g(uint8_t bScale);
void ResetUSART(UART_HandleTypeDef *huart);
void writeHexBuffer(uint8_t *pBuffer, uint16_t wLen);
void getSysClkSource();
void writetoSerial(UART_HandleTypeDef *huart, char *pMsg);
void formattoSerial(UART_HandleTypeDef *huart, char *pMsg, int msgLenIn,

int *msgLenOut, uint32_t dwData, const char *pCustMsg);

void writeHextoSerial(UART_HandleTypeDef *huart, const uint8_t *pArray,
		int wArrSize);

void writeASCIItoSerial(UART_HandleTypeDef *huart, uint8_t bFormat,
		const uint8_t *pArray, int wArrSize, const char *pHeader);

void binaryToASCII(const uint8_t *pHex, uint16_t wHexLen, char *pAscii);

void writeFormatData(UART_HandleTypeDef *huart, const char *format, ...);

void getCpuId();

void LOG_Error_Status(int bError, const char *pFile, uint8_t bLineNum,
		const char *pFunName, const char *pMsg);

typedef enum
{
	Device_ID = 0x0F,
	PEAK_VALUE1 = 0x19,
	PEAK_VALUE2 = 0x1A,
	CTRL_REG1 = 0x21,
	CTRL_REG2 = 0x22,
	CTRL_REG3 = 0x23,
	CTRL_REG4 = 0x20,
	CTRL_REG5 = 0x24,
	CTRL_REG6 = 0x25,
	STATUS = 0x27,
	OUT_XL = 0x28,
	OUT_XH = 0x29,
	OUT_YL = 0x28,
	OUT_YH = 0x29,
	OUT_ZL = 0x28,
	OUT_ZH = 0x29,
	OUT,
} REG_ADDR;

typedef enum
{
	ASCII, HEX,
} printformat_t;

#endif /* INC_CUSTOM_H_ */

HAL_StatusTypeDef returnError();
HAL_StatusTypeDef returnSuccess();

const char* ErrorString(ERROR_t bError);

/* Practice UART code starts */
void UART1_Tx_Inter_write_char(char ch);
void UART1_Tx_Inter_write_str(const char *pData, size_t wDataLen);
void UART1_Rx_Inter_read_str(char *pData, size_t wDataLen);
uint8_t UART1_Rx_Inter_read_char();
size_t UART1_Rx_Inter_read_string(uint8_t *pOutData, size_t wDataLen);

void GreenLED_init();
void TurnGreenLED_ON();
void TurnGreenLED_OFF();

/* Practice UART code ends */
