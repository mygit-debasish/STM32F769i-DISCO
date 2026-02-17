/*
 * custom_Intr.c
 *
 *  Created on: Feb 14, 2026
 *      Author: Debasish Das
 */

#include "custom_Intr.h"

void USART1_IRQHandler(void)
{
	/* USART1 IDLE interrupt detection */
	if (USART1->ISR & USART_ISR_IDLE)
	{
		/* Clear the interrupt */
		USART1->ICR = USART_ICR_IDLECF;
		USART1_Idle_Intr_Callback();
	}
}

uint8_t NDTR_value;
extern DmaCirularBuff_t DmaUsartCircularBuffer;

void USART1_Idle_Intr_Callback()
{
	DmaCircularBuffer_ReadReceiveData(&DmaUsartCircularBuffer);
}

void UART1_RXNE_Callback()
{
	/* RXNE flag is cleared only by reading RDR */
	uint8_t dummyRead;
	/* Clearing RXNE interrupt flag */
	dummyRead = (uint8_t) USART1->RDR;
	flagLED = 1;
}

/* Initialize USART1 for DMA transfer */
void Initialize_USART1_DMA_transfer()
{
	/* PA9 -> Transmit
	 * Pa10 -> Received */

	/******************  GPIO setup *****************************/

	/* Enabling the clock for GPIOA*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	/* Enabling the CLK for UART1 module */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	/*Selecting alternate function Mode : AF7 */

	/*  UART1 Transmitter MODER setting */
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
	USART1->BRR = (USART1_CLK + (BAUD_RATE / 2U)) / BAUD_RATE;

	/* Enable transmit and receive */
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

	/* Start-Bit, Stop-Bit & Parity-Bit are default configuration of USART_CR1
	 * Register */

	/* Enabling  RXNEIE: Receiver Not Empty ❗ ( Nor needed. USART_CR3_DMAR handle DMA Rx)
	 * TXEIE: 	Transmitter Empty interrup ❗ (Not same time with RXNEIE)
	 * IDLEIE:	IDLE Interrupt Enable ✍*/
	USART1->CR1 |= USART_CR1_IDLEIE;

	/* Clear pending flags */
	volatile uint32_t tmp;
	tmp = USART1->ISR;
	tmp = USART1->RDR;
	(void) tmp;

	/*Enable DMA2 for USART1 */
	USART1->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
	USART1->CR3 |= USART_CR3_DMAR;

	/* Setting Interrupt priority */
	NVIC_SetPriority(USART1_IRQn, 5);

	/*Enable above interrupt in NVIC*/
	NVIC_EnableIRQ(USART1_IRQn);

	/*Enable UART1 */
	USART1->CR1 |= USART_CR1_UE;
}

void GreenLED_init()
{
	/* User LED is GPIOI1 (LD1)*/

	/* CLK for GPIOI */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

	/* Set Mode as output mode */
	GPIOJ->MODER &= ~(GPIO_MODER_MODER5);
	GPIOJ->MODER |= GPIO_MODER_MODER5_0;
}

void TurnGreenLED_ON()
{
	GPIOJ->ODR |= (1U << 5);
	//GPIOJ->BSRR = (1U << 5);
}

void TurnGreenLED_OFF()
{
	GPIOJ->ODR &= ~(1U << 5);
	//GPIOJ->BSRR = (1U << (5 + 16));
}

void Configure_DMA_Transmit_USART1(uint8_t *pScr, uint8_t *pDst,
		size_t wDataLen)
{
	/*	USART1_TX is alloted to DMA2. Stream7, Channel:4
	 * 	USART1_RX is alloted to DMA2. Stream2, Channel:4
	 * 	DMA2 is located in AHB Bus*/

	/* Enable clock for DMA1*/
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/*Disable DMA2 steam7 */
	DMA2_Stream7->CR &= ~(DMA_SxCR_EN);

	/*Disable all interrupt  is the stream. Not necessary ✅ */
	//DMA2_Stream7->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
	/* clear all DMA2 interrupt flags. HIFCR: DMA high interrupt flag clear register is
	 * meant for updating all stream of DMA2  */
	DMA2->HIFCR = (DMA_HIFCR_CDMEIF7 |
	DMA_HIFCR_CTEIF7 |
	DMA_HIFCR_CHTIF7 |
	DMA_HIFCR_CTCIF7 |
	DMA_HIFCR_CFEIF7);

	/* Enabling Channel 4 of stream 7 */
	/* Clearing all channels + Enable CH4*/
	DMA2_Stream7->CR &= ~DMA_SxCR_CHSEL_Msk;
	DMA2_Stream7->CR |= DMA_SxCR_CHSEL_2;

	/* Enabling transfer direction. Here Data need to be transfer
	 * from Memory to Peripheral */
	DMA2_Stream7->CR |= DMA_SxCR_DIR_0;

	/*Configure source : Memory */
	DMA2_Stream7->M0AR = (uint32_t) pScr;

	/*Configure destination : Peripheral. This is TDR for USART1 */
	DMA2_Stream7->PAR = (uint32_t) pDst;

	/*Configure Data size */
	DMA2_Stream7->NDTR = (uint16_t) wDataLen;

	/* Configure Peripheral and Memory size.
	 * 00 :  8 bits default
	 * 01 : 16 bits
	 * 10 : 32 bits
	 * */
	DMA2_Stream7->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);

	/*Configure Peripheral address pointer increment */
	DMA2_Stream7->CR &= ~DMA_SxCR_PINC;

	/*Configure Memory address pointer increment */
	DMA2_Stream7->CR |= DMA_SxCR_MINC;

	/* Enable direct Mode and Disable FIFO.
	 * DMA stream FIFO control Register (FCR) */
	DMA2_Stream7->FCR &= ~DMA_SxFCR_DMDIS;

	/*Enable transfer control interrupt.We have already cleared Interrupt flags,
	 * we can enable atleast one Interrupt */
	DMA2_Stream7->CR |= DMA_SxCR_TCIE;

	/*Enable DMA2 steam2 */
	DMA2_Stream7->CR |= (DMA_SxCR_EN);

	/* Enable DMA2 in NVIC. This will enable interrupt of Steam:7.
	 * Stream:7 will have more than one channel, all having same priority */

	/* Set priority of DMA2 steam:7 CH:4 in NVIC */
	NVIC_SetPriority(DMA2_Stream7_IRQn, 5);

	/* Enable DMA2_stream7_IRQn */
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void Configure_DMA_Receive_USART1(uint8_t *pPeripheralSrc)
{
	/*
	 * 	USART1_RX is alloted to DMA2. Stream2, Channel:4
	 * 	DMA2 is located in AHB Bus
	 * 	*/

	/* Enable clock for DMA1*/
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/*Disable DMA2 steam7 and wait untill is has been disabled */
	DMA2_Stream2->CR &= ~(DMA_SxCR_EN);
	while (DMA2_Stream2->CR & DMA_SxCR_EN);

	/*Disable all interrupt  is the stream. Not necessary ✅ */
	//DMA2_Stream7->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
	/* clear all DMA2 interrupt flags. HIFCR: DMA high interrupt flag clear register is
	 * meant for updating all stream of DMA2  */
	DMA2->LIFCR = (DMA_LIFCR_CDMEIF2 |
	DMA_LIFCR_CTEIF2 |
	DMA_LIFCR_CHTIF2 |
	DMA_LIFCR_CTCIF2 |
	DMA_LIFCR_CFEIF2);

	/* Enabling Channel 4 of stream 2 */
	/* Clearing all channels + Enable CH4*/
	DMA2_Stream2->CR &= ~DMA_SxCR_CHSEL_Msk;
	DMA2_Stream2->CR |= DMA_SxCR_CHSEL_2;

	/* Enabling transfer direction. Here Data need to be transfer
	 * from Peripheral to Memory */
	DMA2_Stream2->CR &= ~DMA_SxCR_DIR;

	/*Configure source : Peripheral */
	DMA2_Stream2->PAR = (uint32_t) pPeripheralSrc;

	/*Configure destination : Memory */
	DMA2_Stream2->M0AR = (uint32_t) aDmaRxBuf;

	/*Configure Data size */
	DMA2_Stream2->NDTR = (uint16_t) DMA_BUF_SIZE;

	/* Configure Peripheral and Memory size.
	 * 00 :  8 bits default
	 * 01 : 16 bits
	 * 10 : 32 bits
	 * */
	DMA2_Stream2->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);

	/*Configure Peripheral address pointer increment */
	DMA2_Stream2->CR &= ~DMA_SxCR_PINC;

	/*Configure Memory address pointer increment */
	DMA2_Stream2->CR |= DMA_SxCR_MINC;

	/* Enable Circular Mode for continuous reception from GTKTerm ✅ ✍ */
	DMA2_Stream2->CR |= DMA_SxCR_CIRC;

	/* Enable direct Mode and Disable FIFO.
	 * DMA stream FIFO control Register (FCR) */
	DMA2_Stream2->FCR &= ~DMA_SxFCR_DMDIS;

	/*Enable transfer control interrupt.We have already cleared Interrupt flags,
	 * Enable Transfer Complete and Transfer Error Interrupt */
	DMA2_Stream2->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE;

	/*Enable DMA2 steam2 */
	DMA2_Stream2->CR |= (DMA_SxCR_EN);

	/* Enable DMA2 in NVIC. This will enable interrupt of Steam:2.
	 * Stream:2 will have more than one channel, all having same priority */

	/* Set priority of DMA2 steam:2 CH:4 in NVIC */
	NVIC_SetPriority(DMA2_Stream2_IRQn, 5);

	/* Enable DMA2_stream2_IRQn */
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/* DMA2_Stream7_IRQHandler() handle USART1 Transmit */
void DMA2_Stream7_IRQHandler()
{
	if (DMA2->HISR & DMA_HISR_TCIF7)
	{
		/* Clearing Transmission Complete Interrupt status register */
		DMA2->HIFCR = DMA_HIFCR_CTCIF7;
		DMA2_Stream7_Callback();
	}
}

void DMA2_Stream7_Callback()
{
	flagLED_DMA_TX = 1;
}

/* DMA2_Stream2_IRQHandler() handle USART1 Receive */
void DMA2_Stream2_IRQHandler()
{
	if (DMA2->LISR & DMA_LISR_TCIF2)
	{
		/* Clearing Transmission Complete Interrupt status register */
		DMA2->LIFCR = DMA_LIFCR_CTCIF2;
		DMA2_Stream2_Callback();
	}

	/* Handle if Transfer Error Interrupt Flag (TEIF) is set ✍ */
	if (DMA2->LISR & DMA_LISR_TEIF2)
	{
		DMA2->LIFCR = DMA_LIFCR_CTEIF2; /* Clear Transmit Error Interrupt Flag */
	}
}
void DMA2_Stream2_Callback()
{
	flagLED_DMA_RX = 1;
}

/* Implementation:
 * DataLen > CirBuffLen */
void circualrBufferRead(uint8_t *pCirBuffer, size_t bcirBuffLen, char *pData,
		size_t bDataLen)
{
	uint8_t cPos = 0;

	while (cPos < bDataLen)
	{
		pCirBuffer[cPos % CIRC_BUFFER_LEN] = pData[cPos];
		cPos++;
	}
}

/* Implementation:
 *  CirBuffLen > bDataLen */
void circualrBufferRead_Process(uint8_t *pCirBuffer, size_t bcirBuffLen,
		char *pData, size_t bDataLen)
{
	uint8_t curPos = 0;

	if (!pCirBuffer || bcirBuffLen == 0)
		return;

	if (!pData || bDataLen == 0)
		return;

	while (1)
	{
		for (curPos = 0; curPos < bDataLen; curPos++)
		{
			pCirBuffer[(headPos + curPos) % bcirBuffLen] = DMA2_Stream2->M0AR;
		}
		headPos = (headPos + bDataLen) % bcirBuffLen;
	}
}

void DmaCircularBuffer_ReadReceiveData_Init(DmaCirularBuff_t *pUsartDMA)
{
	pUsartDMA->cir_buff_head = 0;
	pUsartDMA->cir_buff_tail = 0;
	pUsartDMA->cirBufDMA = aCirBuff;
	memset((void*) pUsartDMA->cirBufDMA, 0, CIRC_BUFFER_LEN);
}

size_t getCirBufHead(DmaCirularBuff_t *pUsartDMA)
{
	return pUsartDMA->cir_buff_head;
}

size_t getCirBufTead(DmaCirularBuff_t *pUsartDMA)
{
	return pUsartDMA->cir_buff_head;
}

size_t getCirBufFreeSpace(DmaCirularBuff_t *pUsartDMA)
{

	if (pUsartDMA->cir_buff_head - pUsartDMA->cir_buff_tail)
	{
		return (CIRC_BUFFER_LEN
				- (pUsartDMA->cir_buff_head - pUsartDMA->cir_buff_tail) - 1);
	}

	else
	{
		return (pUsartDMA->cir_buff_head - pUsartDMA->cir_buff_head - 1);
	}
}

bool isCircBuffFull(DmaCirularBuff_t *pUsartDMA)
{
	return (pUsartDMA->cir_buff_head + 1 == pUsartDMA->cir_buff_tail);
}

bool isCircBufEmpty(DmaCirularBuff_t *pUsartDMA)
{
	return (pUsartDMA->cir_buff_head == pUsartDMA->cir_buff_tail);
}

uint8_t currDmaBuffIndex = 0;
uint8_t prevDmaBuffIndex = 0;
uint8_t srcIndex = 0;


/** Below in clearer version
void DmaCircularBuffer_ReadReceiveData(DmaCirularBuff_t *pDmaUsart)
{
	size_t byteCount = 0;

	currDmaBuffIndex = (DMA_BUF_SIZE - DMA2_Stream2->NDTR);
	byteCount = ( currDmaBuffIndex - prevDmaBuffIndex ) & (DMA_BUF_SIZE -1);

	for(size_t i=0; i<byteCount; i++ )
	{
		srcIndex = (prevDmaBuffIndex + i) & (DMA_BUF_SIZE -1);

		pDmaUsart->cirBufDMA[pDmaUsart->cir_buff_head] = aDmaRxBuf[srcIndex];

		pDmaUsart->cir_buff_head = (pDmaUsart->cir_buff_head +1) & (CIRC_BUFFER_LEN - 1);
		//pDmaUsart->usart_dma_buff_head = (pDmaUsart->usart_dma_buff_head +1) & (DMA_BUF_SIZE - 1);
	}

	prevDmaBuffIndex = currDmaBuffIndex;
}
**/

void DmaCircularBuffer_ReadReceiveData(DmaCirularBuff_t *pDmaUsart)
{
	size_t byteCount = 0;

	currDmaBuffIndex = (DMA_BUF_SIZE - DMA2_Stream2->NDTR);
	byteCount = (currDmaBuffIndex - prevDmaBuffIndex) & (DMA_BUF_SIZE - 1);

	for (size_t i = 0; i < byteCount; i++)
	{
		pDmaUsart->cirBufDMA[pDmaUsart->cir_buff_head] =
				aDmaRxBuf[pDmaUsart->usart_dma_buff_head];

		/* Updating buffer head for Circular buffer and DMA buffer */
		pDmaUsart->cir_buff_head = (pDmaUsart->cir_buff_head + 1)
				& (CIRC_BUFFER_LEN - 1);
		pDmaUsart->usart_dma_buff_head = (pDmaUsart->usart_dma_buff_head + 1)
				& (DMA_BUF_SIZE - 1);
	}

	prevDmaBuffIndex = currDmaBuffIndex;
}

