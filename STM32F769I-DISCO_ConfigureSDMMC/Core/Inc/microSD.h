/*
 * microSD.h
 *
 *  Created on: Feb 3, 2026
 *      Author: debasish
 */

#ifndef INC_MICROSD_H_
#define INC_MICROSD_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f769xx.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"


typedef enum
{
	BINARY_FILE,
	ASCII_FILE,
	OTHER_FILE
}FiletypeDef_t;

bool SD_IsDetected(void);
void showSDcardInfo(SD_HandleTypeDef *hsd);
uint8_t readFileSDCard(char *pBuffer, uint16_t wSize, int *bByteRead);
void SD_Read_Test(void);
void readSector0();

FRESULT SDMMC2_mount();
FRESULT SDMMC2_ReadFile(const char *fname, uint8_t bType, uint8_t *aBuffer,
		size_t *wByeCount);

#endif /* INC_MICROSD_H_ */
