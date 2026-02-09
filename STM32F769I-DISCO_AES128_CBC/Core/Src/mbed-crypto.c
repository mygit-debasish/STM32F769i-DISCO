/*\
 * mbed-tlscrypto.c
 *
 *  Created on: Feb 1, 2026
 *      Author: Debasish Das
 */
#include <mbed-crypto.h>
#include <mbed-crypto.h>
#include <mbedtls/sha256.h>
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "custom.h"

extern RNG_HandleTypeDef hrng;

void generateRndNumber(uint32_t *pRnd, uint8_t *bRndLen)
{

	if (HAL_RNG_GenerateRandomNumber(&hrng, pRnd) != HAL_OK)
	{
		writetoSerial(&huart1, "Random number generator fails. Exiting ..\n\r");
		return;
	}
	*bRndLen = 32;
}

void generateHash256(const unsigned char *pData, uint8_t bDataLen,
		uint8_t *pHashVal)
{
	mbedtls_sha256(pData, bDataLen, pHashVal, 0);
}

void readDeviceID(uint32_t *pID)
{
	volatile uint32_t *pID_BASE = (volatile uint32_t*) DEVICE_ID_BASE;

	if (!pID)
	{
		return;
	}
	/* Copy device ID into output buffer */
	memcpy(pID, (const void*) pID_BASE, DEVICE_ID_LEN_WORD * sizeof(uint32_t));
}

void generateIvfromUID(uint8_t *pIVector)
{
	uint8_t aHash[HASH_LEN];
	uint32_t aUID[DEVICE_ID_LEN_WORD];

	/* Reading device UID */
	readDeviceID(aUID);

	mbedtls_sha256_context ctx;
	mbedtls_sha256_init(&ctx);

	mbedtls_sha256_starts(&ctx, 0);
	mbedtls_sha256_update(&ctx, (const unsigned char*) aUID,
	DEVICE_ID_LEN_WORD * sizeof(uint32_t));
	mbedtls_sha256_finish_ret(&ctx, aHash);
	mbedtls_sha256_free(&ctx);

	/*Copy UID hash as IVector */
	memcpy(pIVector, aHash, IVECTOR_LEN_AES128);
}

/**
 * @brief Encrypt the data using AES-128
 * @retval AES128_ERROR_t
 */

uint8_t enryptDataAES_128(uint8_t *pData, uint16_t wDataLen, uint8_t *pKey,
		uint8_t wKeyLen, uint8_t *pIVector, uint8_t *pDataEnc, uint16_t *wDataEncLen)
{
	uint8_t aDataPadded[MAX_DATALEN_AES128];
	uint8_t bPad;
	mbedtls_aes_context ctx;

	if (!pData || !pKey)
		return AES128_NULL_ERROR;

	if (wKeyLen != AES128_KEY_LEN)
		return AES128_LEN_ERROR;

	/* Apply padding even if it is block aligned */
	bPad = padDataEnc(pData, wDataLen, aDataPadded, MAX_DATALEN_AES128);
	writeASCIItoSerial(&huart1, BYTE, aDataPadded, bPad, "PaddedData");

	mbedtls_aes_init(&ctx);
	if(mbedtls_aes_setkey_enc(&ctx, pKey, 128) != 0)
	{
		return AES128_GEN_ERROR;
	}

	if(mbedtls_aes_crypt_cbc(&ctx,
			MBEDTLS_AES_ENCRYPT,
			bPad,
			pIVector,
			aDataPadded,
			aDataPadded) != 0)
		{
			mbedtls_aes_free(&ctx);
			return AES128_GEN_ERROR;
		}

	/* Free the context */
	mbedtls_aes_free(&ctx);

	/* Copy encrypted data into output Buffer */
	memcpy(pDataEnc, aDataPadded, bPad);
	*wDataEncLen = bPad;

	return AES128_SUCCESS;
}


uint8_t deryptDataAES_128(uint8_t *pDataEnc, uint16_t wDataEncLen, uint8_t *pKey,
		uint8_t wKeyLen, uint8_t *pIVector, uint8_t *pDataDec, uint16_t *wDataDecLen)
{
	mbedtls_aes_context ctx;

	if (!pDataEnc || !pKey)
		return AES128_NULL_ERROR;

	if (wKeyLen != AES128_KEY_LEN)
		return AES128_LEN_ERROR;

	mbedtls_aes_init(&ctx);
	if(mbedtls_aes_setkey_dec(&ctx, pKey, 128) != 0)
	{
		return AES128_GEN_ERROR;
	}

	if(mbedtls_aes_crypt_cbc(&ctx,
			MBEDTLS_AES_DECRYPT,
			wDataEncLen,
			pIVector,
			pDataEnc,
			pDataDec) != 0)
		{
			mbedtls_aes_free(&ctx);
			return AES128_GEN_ERROR;
		}

	/* Free the context */
	mbedtls_aes_free(&ctx);

	return AES128_SUCCESS;
}

uint8_t padDataEnc(uint8_t *pInData, uint16_t wInDataInLen, uint8_t *pOutData,
		uint16_t wOutDataLen)
{
	uint8_t bPadLen;

	if (!pInData || !pOutData)
		return AES128_NULL_ERROR;

	bPadLen = AES128_BLOCK_SIZE - (wInDataInLen % AES128_BLOCK_SIZE);

	if ((wInDataInLen + bPadLen) > wOutDataLen)
		return AES128_LEN_ERROR;

	memcpy(pOutData, pInData, wInDataInLen);
	memset(&pOutData[wInDataInLen], bPadLen, bPadLen);

	return wInDataInLen + bPadLen;
}

