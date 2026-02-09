/*
 * microSD.c
 *
 *  Created on: Feb 3, 2026
 *      Author: Debasish Das
 */

#include "microSD.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"
#include "custom.h"

extern FATFS SDFatFS; /* Make it extern  */

extern SD_HandleTypeDef hsd2;
extern UART_HandleTypeDef huart1;

void showSDcardInfo(SD_HandleTypeDef *hsd)
{
	HAL_SD_CardInfoTypeDef cardInfo;

	if (HAL_SD_GetCardInfo(&hsd2, &cardInfo) != HAL_OK)
	{
		writetoSerial(&huart1, "SD Card information failed \r\n");
	}
	else
	{
		writetoSerial(&huart1, "SD Card information succeed \r\n");
	}
}

FIL file1;

__attribute__((aligned(32)))
    static uint8_t buffer[512];

void SD_Read_Test(void)
{
	FRESULT res;
	UINT bytesRead;

	res = f_mount(&SDFatFS, "0:", 1);
	if (res != FR_OK)
	{
		writetoSerial(&huart1, "File mount Failed \r\n");
		return;
	}
	else
	{
		writetoSerial(&huart1, "File mount SUCCEED. Really :) \r\n");
	}

	writetoSerial(&huart1, "Waiting for reading the file ..\r\n");

	res = f_open(&file1, "Readme.txt", FA_READ);
	//res = f_open(&file1, "STM32Key.bin", FA_READ);

	if (res != FR_OK)
	{
		writetoSerial(&huart1, "Error reading binary file !\r\n");
		return;
	}

	res = f_read(&file1, buffer, sizeof(buffer), &bytesRead);

	writeASCIItoSerial(&huart1, ASCII, buffer, bytesRead, "Received Key");

	f_close(&file1);
	f_mount(NULL, "", 0);
}

void readSector0()
{
	uint8_t sector0[512];
	DRESULT dres;
	UNUSED(dres);
	uint8_t bs[512] __attribute__((aligned(32)));

	dres = disk_read(0, sector0, 0, 1);

	DWORD start_lba = sector0[0x1BE + 8] | (sector0[0x1BE + 9] << 8)
			| (sector0[0x1BE + 10] << 16) | (sector0[0x1BE + 11] << 24);

	disk_read(0, bs, start_lba, 1);

}

FRESULT SDMMC2_mount()
{
	FRESULT res;

	res = f_mount(&SDFatFS, "0:", 1);

	if (res != FR_OK)
	{
		writetoSerial(&huart1, "SD Card mounting failed \r\n");
		return res;
	}
	else
		writetoSerial(&huart1, "SD Card mounting succeed \r\n");

	return res;
}

FRESULT SDMMC2_ReadFile(const char *fname, uint8_t bType, uint8_t *aBuffer,
		size_t *wByeCount)
{

	FIL fp;
	FRESULT status;
	UINT bytesRead;

	status = f_open(&fp, fname, FA_READ);

	if (status != FR_OK)
	{
		writetoSerial(&huart1, "Error reading binary file !\r\n");
		return status;
	}

	/* Reading from File */
	status = f_read(&fp, aBuffer, sizeof(buffer), &bytesRead);

	if (status != FR_OK)
	{
		writetoSerial(&huart1, "Error reading from file !\r\n");
		return status;
	}

	writetoSerial(&huart1, "Key read from file !\r\n");
	*wByeCount = bytesRead;


//	switch (bType)
//		{
//
//	case ASCII_FILE:
//		status = f_read(&file1, buffer, sizeof(buffer), &bytesRead);
//
//		}
//	status = f_read(&file1, buffer, sizeof(buffer), &bytesRead);
//	writeASCIItoSerial(&huart1, ASCII, buffer, bytesRead, "Received Key");

	return status;

}

