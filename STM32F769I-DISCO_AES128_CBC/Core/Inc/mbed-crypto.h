/*
 * mbed-tlscrypto.h
 *
 *  Created on: Feb 1, 2026
 *      Author: Debasish Das
 */

#ifndef INC_MBED_CRYPTO_H_
#define INC_MBED_CRYPTO_H_

#include "custom.h"
#define HASH_LEN 32U
#define DEVICE_ID_BASE	(0x1FF0F420UL)
#define DEVICE_ID_LEN_WORD	(0x3U)
#define IVECTOR_LEN			(0x10)

#define MAX_DATALEN_AES128	512U

/************ AES-128 specific definition ********************/
#define AES128_KEY_LEN 		16U
#define AES128_BLOCK_SIZE	16U
#define HASH_LEN_AES128  16U
#define AES_MAXDATA_LEN 512U
#define IVECTOR_LEN_AES128			(0x10)
/*************** AES-128 ENDS *********************************/

typedef enum
{
	AES128_SUCCESS,
	AES128_ERROR,
	AES128_LEN_ERROR,
	AES128_NULL_ERROR,
	AES128_GEN_ERROR
} AES128_ERROR_t;

/* AES-128 Key */

void generateRndNumber(uint32_t *pRnd, uint8_t *bRndLen);
void generateHash256(const unsigned char *pData, uint8_t bDataLen, uint8_t *pHashVal);
void readDeviceID(uint32_t *pID);
void generateIvfromUID(uint8_t *pIVector);
uint8_t padDataEnc(uint8_t *pInData, uint16_t wInDataInLen, uint8_t *pOutData,
		uint16_t wOutDataLen);
uint8_t enryptDataAES_128(uint8_t *pData, uint16_t wDataLen, uint8_t *pKey,
		uint8_t wKeyLen, uint8_t *pIVector, uint8_t *pDataEnc, uint16_t *wDataEncLen);

uint8_t deryptDataAES_128(uint8_t *pDataEnc, uint16_t wDataEncLen, uint8_t *pKey,
		uint8_t wKeyLen, uint8_t *pIVector, uint8_t *pDataDec, uint16_t *wDataDecLen);

#endif /* INC_MBED_CRYPTO_H_ */
