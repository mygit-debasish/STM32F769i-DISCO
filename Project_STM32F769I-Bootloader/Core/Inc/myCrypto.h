/*
 * myCrypto.h
 *
 *  Created on: Jul 28, 2026
 *      Author: debasish
 */

#ifndef INC_MYCRYPTO_H_
#define INC_MYCRYPTO_H_

#include "microSD.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"
#include "custom.h"
#include <stdio.h>


#include <mbedtls/certs.h>
#include <mbedtls/x509.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/pk.h>
#include <mbedtls/sha256.h>

#define CA_CERT_ADDR  		0x08180000UL
#define SHA256_LEN 			32U
#define NTP_FRAME_LEN 		9U
#define APPLICATION_ADDR 	0x08080000UL
#define FWHEADER_ADDR 		0x08180000UL

#define HASH_LEN 		32U
#define SIGN_LEN		100U
#define COMP_FW_LEN 	256U


#define FW_HEADER_ADDR  0x081C0000UL	/* Firmware Header Location in FLASH */

#define HASH_LEN 32U
#define CERT_LEN 800U
#define COMP_FW_LEN 256U		/*Finite Firmware byte over which Hash calculated */

/* NTP Data Receive State machine */
typedef enum
{
	WAIT_LI_VM_MODE,
	WAIT_STRATUM,
	WAIT_POLL_INT,
	WAIT_PRECISION,
	WAIT_DATA,
	WAIT_END_BYTE,
	RX_PACKET_READY,
	RX_ERROR,
	WAIT_COMPLETE
} NTPState;

/* Taken from Eclispe Project */
#define MAX_SIG_LEN 100
#define MAX_HASH_LEN 32

#define NTP_BUFF_SIZE 		9U
#define MAX_ERR_MSG_LEN		80U

typedef struct
{
	NTPState state;
	size_t index;
	uint8_t NTPBuff[NTP_BUFF_SIZE];
	uint8_t errMsg[MAX_ERR_MSG_LEN];
	bool isComplete;
	uint32_t flag;
} NtpBuffRx_t;

typedef struct
{
	uint32_t magic;
	uint8_t hashVal[MAX_HASH_LEN];
	uint16_t Revision;
} SignHeader_t;


	typedef enum
	{
	SUCK_OK = 0,
	SUCK_ERROR,
	SUCK_CURSED,
	SUCK_DIE
	} SuckStatus_t;




/* Bootloader Header Definition */
	typedef struct
		{
			uint32_t magic;
			uint32_t firmwareSize;
			uint32_t caCertLen;
			uint32_t devCertLen;
			uint8_t hash[HASH_LEN];
			uint8_t signature[SIGN_LEN];
			uint8_t devCert[CERT_LEN];
			uint8_t signLen;
			uint8_t firmwareRevision;
			uint8_t firmwareVersion;
		} FirmwareHeader_t;



#define LI_VM_MODE	 	0x1C
#define STRATUM  		0x01
#define POLL_INT		0x0D
#define PRECION			0xE3
#define STM32_RST		0xEE
#define STM32_SIG		0xDE
#define END_BYTE		0xFF

#define ASSERT_FRESULT_OK(status)	\
							do {	\
								if ( (status) != FR_OK)	\
								 {						\
								writetoSerial(&huart1, "Error Reading File !\r\n");\
								return (status);	\
								 }					\
							}while(0);


FRESULT CryptoData_ReadFromSdCard(uint8_t *pDevSignBuff ,size_t *pDevSignLen,
								uint8_t *pdevCertBuff, size_t *pDevCertLen,
								uint8_t *pFirmwareBuff, size_t *pFirmLen,
								uint8_t *pFirmwareHashBuff);
/* Function Prototypes */
int Verify_DeveloperCertificate (const uint8_t *pCACertBuff,
						size_t CACertBuffLen,
						const uint8_t *pDevCertBuff,
						size_t DevCertBuffLen,
						 uint32_t *ErrCode,
						 mbedtls_x509_crt *devCert);

void Read_File_Signature(const char *pSigFilePath, uint8_t *SigBuff, size_t *SigLen);
void Read_File_Hash(const char *pHashFilePath, uint8_t *HashBuff, size_t *HashLen);

int Calculate_File_SHA256(const char *filename,
                          uint8_t hash[32]);

void WritetoFlash(uint8_t *pAddr, uint8_t *pData, size_t dataLen);
uint8_t EraseFlash(uint32_t SectorNum);
void Write_Certificate_Flash(uint8_t *pAddr, uint8_t *pData, size_t dataLen, bool checkByte);

static int Verify_Callback(void *ctx,
                           mbedtls_x509_crt *crt,
                           int depth,
                           uint32_t *flags);

void Calculate_SHA256(const uint8_t *data,
                      size_t data_len,
                      uint8_t hash[SHA256_LEN]);

void Init_NTPByte_Receive(NtpBuffRx_t *rx);
void NTP_ReceiveStateMachine(NtpBuffRx_t *rx, uint8_t data);
void JumpToCryptoApplication();


#endif /* INC_MYCRYPTO_H_ */
