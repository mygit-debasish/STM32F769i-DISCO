/*
 * myCrypto.c
 *
 *  Created on: Jul 28, 2026
 *      Author: Debasish Das
 */

#include "myCrypto.h"
#include "microSD.h"
#include "ff.h"

/* CA Self signed certificate */
const uint8_t CA_certificate_der[] =
#if 1
{
  0x30, 0x82, 0x02, 0x27, 0x30, 0x82, 0x01, 0xcd, 0xa0, 0x03, 0x02, 0x01,
  0x02, 0x02, 0x14, 0x05, 0x85, 0x6d, 0x5b, 0xe6, 0x5f, 0x92, 0x82, 0x40,
  0x4a, 0x1c, 0x81, 0x1d, 0x62, 0xc6, 0x35, 0x9b, 0x42, 0x80, 0x2a, 0x30,
  0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02, 0x30,
  0x69, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02,
  0x41, 0x55, 0x31, 0x0c, 0x30, 0x0a, 0x06, 0x03, 0x55, 0x04, 0x08, 0x0c,
  0x03, 0x4b, 0x41, 0x52, 0x31, 0x0d, 0x30, 0x0b, 0x06, 0x03, 0x55, 0x04,
  0x07, 0x0c, 0x04, 0x42, 0x4e, 0x47, 0x4c, 0x31, 0x1a, 0x30, 0x18, 0x06,
  0x03, 0x55, 0x04, 0x0a, 0x0c, 0x11, 0x44, 0x65, 0x62, 0x61, 0x73, 0x69,
  0x73, 0x68, 0x20, 0x54, 0x65, 0x63, 0x68, 0x20, 0x4c, 0x74, 0x64, 0x31,
  0x21, 0x30, 0x1f, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01,
  0x09, 0x01, 0x16, 0x12, 0x64, 0x65, 0x62, 0x61, 0x73, 0x69, 0x73, 0x68,
  0x40, 0x67, 0x6d, 0x61, 0x69, 0x6c, 0x2e, 0x63, 0x6f, 0x6d, 0x30, 0x1e,
  0x17, 0x0d, 0x32, 0x36, 0x30, 0x37, 0x32, 0x36, 0x31, 0x33, 0x31, 0x32,
  0x32, 0x32, 0x5a, 0x17, 0x0d, 0x33, 0x36, 0x30, 0x37, 0x32, 0x33, 0x31,
  0x33, 0x31, 0x32, 0x32, 0x32, 0x5a, 0x30, 0x69, 0x31, 0x0b, 0x30, 0x09,
  0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x41, 0x55, 0x31, 0x0c, 0x30,
  0x0a, 0x06, 0x03, 0x55, 0x04, 0x08, 0x0c, 0x03, 0x4b, 0x41, 0x52, 0x31,
  0x0d, 0x30, 0x0b, 0x06, 0x03, 0x55, 0x04, 0x07, 0x0c, 0x04, 0x42, 0x4e,
  0x47, 0x4c, 0x31, 0x1a, 0x30, 0x18, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c,
  0x11, 0x44, 0x65, 0x62, 0x61, 0x73, 0x69, 0x73, 0x68, 0x20, 0x54, 0x65,
  0x63, 0x68, 0x20, 0x4c, 0x74, 0x64, 0x31, 0x21, 0x30, 0x1f, 0x06, 0x09,
  0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x09, 0x01, 0x16, 0x12, 0x64,
  0x65, 0x62, 0x61, 0x73, 0x69, 0x73, 0x68, 0x40, 0x67, 0x6d, 0x61, 0x69,
  0x6c, 0x2e, 0x63, 0x6f, 0x6d, 0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a,
  0x86, 0x48, 0xce, 0x3d, 0x02, 0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce,
  0x3d, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04, 0xbe, 0xb0, 0xa3, 0x5b,
  0xa5, 0xbe, 0xd6, 0x74, 0xdc, 0x3b, 0xee, 0x43, 0x43, 0x7e, 0xe0, 0x28,
  0x67, 0x27, 0x7c, 0xa4, 0xf3, 0x0d, 0xe5, 0x6f, 0x30, 0x9f, 0x14, 0xf9,
  0x7b, 0x7b, 0xf8, 0xf6, 0x70, 0xc2, 0x65, 0xaa, 0xad, 0xd8, 0x0c, 0x0e,
  0xcb, 0x71, 0xe6, 0x00, 0x98, 0x1d, 0x00, 0x2a, 0xf0, 0x5f, 0x1b, 0xb5,
  0x38, 0x97, 0x79, 0x1c, 0x53, 0xa9, 0xbb, 0x05, 0xa5, 0x3a, 0xad, 0xa8,
  0xa3, 0x53, 0x30, 0x51, 0x30, 0x1d, 0x06, 0x03, 0x55, 0x1d, 0x0e, 0x04,
  0x16, 0x04, 0x14, 0xac, 0xd7, 0x14, 0x24, 0xd0, 0x46, 0x52, 0x4f, 0x20,
  0x0b, 0x7e, 0xf5, 0x1c, 0x05, 0x5f, 0x8b, 0x68, 0x33, 0x39, 0xf3, 0x30,
  0x1f, 0x06, 0x03, 0x55, 0x1d, 0x23, 0x04, 0x18, 0x30, 0x16, 0x80, 0x14,
  0xac, 0xd7, 0x14, 0x24, 0xd0, 0x46, 0x52, 0x4f, 0x20, 0x0b, 0x7e, 0xf5,
  0x1c, 0x05, 0x5f, 0x8b, 0x68, 0x33, 0x39, 0xf3, 0x30, 0x0f, 0x06, 0x03,
  0x55, 0x1d, 0x13, 0x01, 0x01, 0xff, 0x04, 0x05, 0x30, 0x03, 0x01, 0x01,
  0xff, 0x30, 0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03,
  0x02, 0x03, 0x48, 0x00, 0x30, 0x45, 0x02, 0x20, 0x0a, 0x40, 0x3a, 0x7c,
  0x89, 0xc2, 0x9c, 0x45, 0x83, 0x90, 0xc4, 0xbe, 0x67, 0x46, 0x80, 0x2d,
  0x28, 0x25, 0x47, 0x12, 0x1b, 0xf5, 0xf2, 0xd2, 0x51, 0x8e, 0x00, 0x68,
  0x02, 0xcd, 0x3d, 0x36, 0x02, 0x21, 0x00, 0xf7, 0x52, 0x7e, 0xb3, 0x28,
  0x8e, 0xaa, 0x8e, 0x41, 0x75, 0x5f, 0x38, 0xd5, 0x06, 0xc6, 0x96, 0x81,
  0xcc, 0x15, 0x1e, 0x43, 0x4d, 0xf2, 0x53, 0x6e, 0xa0, 0xc2, 0xb6, 0x77,
  0xb3, 0x12, 0x90
};
#endif


  /* Crypto Files in SD card */
  const char *pDevSignatureFile = "Firmware.sig";
  const char *pFirmwareFile = "Firmware.bin";
  const char *pDevCertificateFile = "Developer_cert.der";

  size_t byteRead;
  FIL pFile;

  FRESULT CryptoData_ReadFromSdCard(uint8_t *pDevSignBuff ,size_t *pDevSignLen,
  								uint8_t *pdevCertBuff, size_t *pDevCertLen,
  								uint8_t *pFirmwareBuff, size_t *pFirmLen,
								uint8_t *pFirmwareHashBuff)
{
	FRESULT status = FR_OK;

	SDMMC2_mount();

	  /* Reading Developer Signature */
	  status = SDMMC2_ReadFile(pDevSignatureFile,
					  0,
					  pDevSignBuff,
					  &byteRead);
	  ASSERT_FRESULT_OK(status);
	  *pDevSignLen = byteRead;

	  /* Reading Developer Certificate signed by CA */
	  status = SDMMC2_ReadFile(pDevCertificateFile,
					  0,
					  pdevCertBuff,
					  &byteRead);
	  ASSERT_FRESULT_OK(status);
	  *pDevCertLen = byteRead;

	  /* Reading Firmware*/
	  status = SDMMC2_ReadFile(pFirmwareFile,
					  0,
					  pFirmwareBuff,
					  &byteRead);
	  ASSERT_FRESULT_OK(status);
	  *pFirmLen = byteRead;

	  /* Compute the Firmware Hash. TODO to calcalted upto 512 Bytes */
	  Calculate_SHA256(pFirmwareBuff,
			 *pFirmLen,
			 pFirmwareHashBuff);

	  SDMMC2_Unmount();
	  return status;
}

#define SHA256_LEN 32U

void Calculate_SHA256(const uint8_t *data,
                      size_t data_len,
                      uint8_t hash[SHA256_LEN])
{
    mbedtls_sha256(data, data_len, hash, 0);
}

/* Taken from Eclipse Project */
int Verify_DeveloperCertificate (const uint8_t *pCACertBuff,
						size_t CACertBuffLen,
						const uint8_t *pDevCertBuff,
						size_t DevCertBuffLen,
						uint32_t *ErrCode,
						mbedtls_x509_crt *devCert)
{
	/* Initialize the Certificate Context */
	mbedtls_x509_crt CACert;
	mbedtls_x509_crt_init (&CACert);

	int ret = mbedtls_x509_crt_parse (
			&CACert,
			pCACertBuff,
			CACertBuffLen);

	if (ret != 0)
		{
			writetoSerial(&huart1,"[❌] CA Certificate parse failed \r\n");
			return ret;
		}
	writetoSerial(&huart1,"[✔] CA Certificate parse succeed \r\n");

	ret = mbedtls_x509_crt_parse (
			devCert,
			pDevCertBuff,
			DevCertBuffLen);

	if (ret != 0)
		{
			writetoSerial(&huart1,"[❌] Developer Certificate parse failed \r\n");
			return ret;
		}
	writetoSerial(&huart1,"[✔] Developer Certificate parse succeed \r\n");

	uint32_t veriFlag = 0;
	char veri_info[200];
	int status = mbedtls_x509_crt_verify(
										devCert,
										&CACert,
										NULL,
										NULL,
										&veriFlag,
										Verify_Callback,
										//NULL,
										NULL);
	if (status != 0)
	{
		mbedtls_x509_crt_verify_info(
									veri_info,
									sizeof(veri_info),
									"Error: ",
									veriFlag);

		*ErrCode = veriFlag;
		writetoSerial(&huart1,"[❌] Developer Certificate Verification Failed \r\n");
		writetoSerial(&huart1, veri_info);
		return status;
	}

	return status;
}

void Read_File_Signature(const char *pSigFilePath, uint8_t *SigBuff, size_t *SigLen)
{
	FILE *fp = fopen(pSigFilePath, "rb");

	*SigLen = fread(
					SigBuff,
					1,
					MAX_SIG_LEN,
					fp);
	fclose(fp);
}

void Read_File_Hash(const char *pHashFilePath, uint8_t *HashBuff, size_t *HashLen)
{
	FILE *fp = fopen(pHashFilePath, "rb");

	*HashLen = fread(
			HashBuff,
	        1,
			MAX_HASH_LEN,
	        fp);

	fclose(fp);
}

#define BUFFER_SIZE 1024U
int Calculate_File_SHA256(const char *filename,
                          uint8_t hash[32])
{
    FILE *fp;
    uint8_t buffer[BUFFER_SIZE];
    size_t bytes_read;

    mbedtls_sha256_context ctx;

    fp = fopen(filename, "rb");

    if (fp == NULL)
    {
        printf("Cannot open file\r\n");
        return -1;
    }

    mbedtls_sha256_init(&ctx);

    /* Start SHA-256 calculation */
    mbedtls_sha256_starts(&ctx, 0);  // 0 = SHA-256, 1 = SHA-224

    while ((bytes_read = fread(buffer,
                               1U,
                               BUFFER_SIZE,
                               fp)) > 0U)
    {
        mbedtls_sha256_update(&ctx,
                              buffer,
                              bytes_read);
    }

    fclose(fp);

    /* Get final hash */
    mbedtls_sha256_finish(&ctx, hash);
    mbedtls_sha256_free(&ctx);

    return 0;
}

/* Taken from Eclipse Project END */

void WritetoFlash(uint8_t *pAddr, uint8_t *pData, size_t dataLen)
{

	if((pAddr == NULL) || (pData == NULL) || (dataLen == 0))
	{
		return;
	}

	HAL_FLASH_Unlock();
	for (size_t i = 0; i < dataLen; i++)
	{
		HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
													(uint32_t)(pAddr + i),
													pData[i]);

		if(status != HAL_OK)
		{
		    uint32_t err = HAL_FLASH_GetError();
		    writeFormatData(&huart1, "Flash error: %d\r\n", err);
			HAL_FLASH_Lock();
			return;
		}
	}

	writeFormatData(&huart1, "Flash written: %d Bytes \r\n", dataLen);

	int status = memcmp((const void *)pAddr, (const void *)pData, dataLen);
	if(status != 0)
	{
	    writeFormatData(&huart1, "Data Mismatched: %d\r\n", status);
		HAL_FLASH_Lock();
		return;
	}

	HAL_FLASH_Lock();
}

uint8_t EraseFlash(uint32_t SectorNum)
{
	FLASH_EraseInitTypeDef eraseInit = {0x00};
	uint32_t eraseError = 0U;

	eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInit.Banks = FLASH_BANK_1;
	eraseInit.NbSectors = 1U;
	eraseInit.Sector = SectorNum;
	eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();

	HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &eraseError);
	if (status != HAL_OK)
	{
		writeFormatData(&huart1,
				"Error in Erasing Flash \r\n",
				eraseError);

		HAL_FLASH_Lock();
		return status;
	}

	HAL_FLASH_Lock();

	return status;
}

void Write_Certificate_Flash(uint8_t *pAddr, uint8_t *pData, size_t dataLen, bool checkByte)
{
	uint32_t FlashSector = 0;

	/* Find the sector CA Certificate located */
	FlashSector = DD_GetFlashSectorNumber((uint32_t)pAddr);
	writeFormatData(&huart1, "CA Certificate Flash Sector:	%d \r\n", FlashSector);
			;
	if((pAddr == NULL) || (pData == NULL) || (dataLen == 0))
	{
		return;
	}

	/* Check of Certificate header. If first byte == 0x30, Return */
	if ((checkByte == true) && pAddr[0] == 0x30U)
	{
		writetoSerial(&huart1, "[✔] CA Public key already in flash\r\n");
		return;
	}

	/* Erasing Flash Sector_6 */
	writetoSerial(&huart1, "Erasing Flash ...\r\n");
	if(EraseFlash(FlashSector) != HAL_OK)
	{
		writetoSerial(&huart1, "Flash Erase Failed\r\n");
		return;
	}

	/* Writing to the FLash */
	writetoSerial(&huart1, "Storing CA Certificate in Flash ..\r\n");
	WritetoFlash(pAddr, pData, dataLen);
}


static int Verify_Callback(void *ctx,
                           mbedtls_x509_crt *crt,
                           int depth,
                           uint32_t *flags)
{
    (void)ctx;
    (void)crt;
    (void)depth;

    if ((*flags & MBEDTLS_X509_BADCERT_FUTURE) != 0U)
    {
        /* Ignore certificate start date check */
        *flags &= ~MBEDTLS_X509_BADCERT_FUTURE;
    }

    return 0;
}

/* Initialize NTP Recieve Buffer State machine */
void Init_NTPByte_Receive(NtpBuffRx_t *rx)
{
	memset(rx, 0, sizeof(NtpBuffRx_t));
	rx->state = WAIT_LI_VM_MODE;
	rx->isComplete = false;
	rx->flag = 0;
}

/* NTP Recieve Buffer State machine */
void NTP_ReceiveStateMachine(NtpBuffRx_t *rx, uint8_t data)
{
	switch (rx->state )
	{
	case WAIT_LI_VM_MODE:
		rx->flag = (rx->flag << 3U) | 0x01;
		if (data == LI_VM_MODE)
		{
			rx->NTPBuff[rx->index++] = (uint8_t) data;
			rx->state = WAIT_STRATUM;
		}
		break;

	case WAIT_STRATUM:
		rx->flag = (rx->flag << 3U) | 0x02;
		if (data == STRATUM)
		{
			rx->NTPBuff[rx->index++] = (uint8_t) data;
			rx->state = WAIT_POLL_INT;
		}
		else
		{
			rx->state = WAIT_LI_VM_MODE;
		}
		break;

	case WAIT_POLL_INT:
		rx->flag = (rx->flag << 3U) | 0x03;
		if (data == (uint8_t) POLL_INT)
		{
			rx->NTPBuff[rx->index++] = (uint8_t) data;
			rx->state = WAIT_PRECISION;
		}
		else
		{
			rx->state = RX_ERROR;
		}
		break;

	case WAIT_PRECISION:
		rx->flag = (rx->flag << 3U) | 0x04;
		if (data == (uint8_t) PRECION )
		{
			rx->NTPBuff[rx->index++] = (uint8_t) data;
			rx->state = WAIT_DATA;
		}
		else if(data == (uint8_t) STM32_RST )
		{
			rx->NTPBuff[rx->index++] = (uint8_t) data;
			rx->state = WAIT_END_BYTE;
		}

		else if(data == (uint8_t) STM32_SIG )
		{
			rx->NTPBuff[rx->index++] = (uint8_t) data;
			rx->state = WAIT_END_BYTE;
		}
		else
		{
			rx->state = RX_ERROR;
		}
		break;

	case WAIT_DATA:
		rx->flag = (rx->flag << 3U) | 0x05;
		rx->NTPBuff[rx->index++] = (uint8_t) data;
		if (rx->index == (NTP_BUFF_SIZE -1U) )
		{
			rx->state = WAIT_END_BYTE;
		}
		break;

	case WAIT_END_BYTE:
		rx->flag = (rx->flag << 3U) | 0x06;
		if (data == END_BYTE)
		{
			rx->NTPBuff[rx->index++] = (uint8_t) data;
			rx->isComplete = true;
			rx->index = 0;
			rx->state = WAIT_LI_VM_MODE;
		}
		break;

	case RX_ERROR:
		rx->flag = 0;
		Init_NTPByte_Receive(rx);
		break;

	default:
		rx->flag = 0;
		Init_NTPByte_Receive(rx);
		break;
	}
}


typedef void (*JumpToApplication)(void);
void JumpToCryptoApplication()
{

	uint32_t AppStack = *(volatile uint32_t*)APPLICATION_ADDR;
	uint32_t AppReset = *(volatile uint32_t*)(APPLICATION_ADDR + 4);

	JumpToApplication CryptoApplication;

	/* Disable Interrupt and Deinit RCC  ✅ */
	__disable_irq();
	HAL_RCC_DeInit();
	HAL_DeInit();


	/* Systick Clock */
	SysTick->CTRL = 0;

	/* Vector Relocation Table */
	SCB->VTOR = APPLICATION_ADDR;

	/* Set MSP */
	__set_MSP(AppStack);

	/* Initializing Function pointer */
	CryptoApplication = (JumpToApplication)AppReset;
	CryptoApplication();
}

uint32_t DD_GetFlashSectorNumber(uint32_t flashAddress)
{
	uint32_t sectorNum;

	if ((flashAddress >= 0x08000000) && (flashAddress <= 0x08007FFF))
	{
		sectorNum = FLASH_SECTOR_0;
	}
	else if ((flashAddress >= 0x08008000) && (flashAddress <= 0x0800FFFF))
	{
		sectorNum = FLASH_SECTOR_1;
	}
	else if ((flashAddress >= 0x08010000) && (flashAddress <= 0x08017FFF))
	{
		sectorNum = FLASH_SECTOR_2;
	}
	else if ((flashAddress >= 0x08018000) && (flashAddress <= 0x0801FFFF))
	{
		sectorNum = FLASH_SECTOR_3;
	}
	else if ((flashAddress >= 0x08020000) && (flashAddress <= 0x0803FFFF))
	{
		sectorNum = FLASH_SECTOR_4;
	}
	else if ((flashAddress >= 0x08040000) && (flashAddress <= 0x0807FFFF))
	{
		sectorNum = FLASH_SECTOR_5;
	}
	else if ((flashAddress >= 0x08080000) && (flashAddress <= 0x080BFFFF))
	{
		sectorNum = FLASH_SECTOR_6;
	}
	else if ((flashAddress >= 0x080C0000) && (flashAddress <= 0x080FFFFF))
	{
		sectorNum = FLASH_SECTOR_7;
	}
	else if ((flashAddress >= 0x08100000) && (flashAddress <= 0x0813FFFF))
	{
		sectorNum = FLASH_SECTOR_8;
	}
	else if ((flashAddress >= 0x08140000) && (flashAddress <= 0x080BFFFF))
	{
		sectorNum = FLASH_SECTOR_9;
	}
	else if ((flashAddress >= 0x08180000) && (flashAddress <= 0x081BFFFF))
	{
		sectorNum = FLASH_SECTOR_10;
	}
	else if ((flashAddress >= 0x081C0000) && (flashAddress <= 0x081FFFFF))
	{
		sectorNum = FLASH_SECTOR_11;
	}
	else
	{
		sectorNum = 0xFF;
	}
	return sectorNum;
}



