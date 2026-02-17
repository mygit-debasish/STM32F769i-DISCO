/*
 * custom_Intr.h
 *
 *  Created on: Feb 14, 2026
 *      Author: debasish
 */

#ifndef INC_CUSTOM_INTR_H_
#define INC_CUSTOM_INTR_H_

#include "custom.h"
#include "stm32f769xx.h"


extern volatile uint8_t flagLED;
extern volatile uint8_t flagLED_DMA_RX;
extern volatile uint8_t flagLED_DMA_TX;


extern volatile uint8_t flagLED_IT_IDLE;
extern uint8_t indxPos;
extern uint8_t headPos;

#define CIRC_BUFFER_LEN 64U
#define DMA_BUF_SIZE	16U
extern uint8_t aDmaRxBuf[DMA_BUF_SIZE];
extern uint8_t aCirBuff[CIRC_BUFFER_LEN];

#define DATA_LEN 10U


/* DMA Buffer structure*/
typedef struct
{
	volatile size_t usart_dma_buff_head;
	volatile size_t cir_buff_head;
	volatile size_t cir_buff_tail;
	volatile uint8_t *cirBufDMA;
} DmaCirularBuff_t;


/************* Initialization function prototype starts ************/
void Initialize_USART1_DMA_transfer();
void Configure_DMA_Transmit_USART1(uint8_t *pScr, uint8_t *pDst, size_t wDataLen);
void Configure_DMA_Receive_USART1(uint8_t *pPeripheralSrc);
void DmaCircularBuffer_ReadReceiveData_Init(DmaCirularBuff_t *pUsartDMA);
/************* Initialization function prototype ends ************/

/************* USART IDLE Interrupt callback ****************/
void DmaCircularBuffer_ReadReceiveData(DmaCirularBuff_t *pDmaUsart);

/*************  IRQ Handlers prototype starts  **************/
void DMA2_Stream7_IRQHandler();
void DMA2_Stream7_Callback();
void DMA2_Stream2_IRQHandler();
void DMA2_Stream2_Callback();
void USART1_Idle_Intr_Callback();
/*************  IRQ Handlers prototype ends  **************/


/************** Circular buffer helper functions prototype starts  **************/
void DMA_CircBuff_init(DmaCirularBuff_t *pUsartDMA);
size_t getCirBufHead(DmaCirularBuff_t *pUsartDMA);
size_t getCirBufTail(DmaCirularBuff_t *pUsartDMA);
size_t getCirBufFreeSpace(DmaCirularBuff_t *pUsartDMA);
bool isCircBuffFull(DmaCirularBuff_t *pUsartDMA);
bool isCircBufEmpty(DmaCirularBuff_t *pUsartDMA);
void circualrBufferRead(uint8_t *pCirBuffer, size_t bcirBuffLen, char *pData,
		size_t bDataLen);
/************** Circular buffer helper functions prototype ends  **************/

#endif /* INC_CUSTOM_INTR_H_ */
