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
#include "stm32f769xx.h"
#include "main.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

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

void writeASCIItoSerial(UART_HandleTypeDef *huart, uint8_t bFormat, const uint8_t *pArray,
		int wArrSize, char *pHeader);

void binaryToASCII(const uint8_t *pHex, uint16_t wHexLen, char *pAscii);

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
	ASCII,
	HEX,
} printformat_t;

#endif /* INC_CUSTOM_H_ */
