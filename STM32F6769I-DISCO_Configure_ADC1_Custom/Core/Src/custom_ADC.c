/*
 * custom_ADC.c
 *
 *  Created on: Feb 24, 2026
 *      Author: Debasish Das
 */

#include "custom_ADC.h"

void ADC1_Init()
{
	/************************ Configuring GPIO *********************/
	/*Enable clock for ADC1*/
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	/* GPIO setting for ADC1_IN6 : 	PA6 (CN14-A0). Configured here */
	/* GPIO setting for ADC1_IN4 : 	PA4 (CN14-A1) */
	/* GPIO setting for ADC1_IN12 : PC2 (CN14-A2) */

	/* Enable clock for PA6 */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	/* Turn OFF  ADC1 */
	ADC1->CR2 &= ~ADC_CR2_ADON;

	/* PA6 mode : Analog Mode */
	GPIOA->MODER &= ~GPIO_MODER_MODER6_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER6_1;

	/* Pull Up/Down register setting. No Pull Up/Down */
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR6;

	/************************ Configuring ADC1 *********************/
	/* Setting Pre-scaler clock. ADC1 is derieved from PCLK2/Prescaler
	 * 108MHz/2 = 54MHz */
	ADC->CCR &= ~ADC_CCR_ADCPRE_Msk;

	/* Selecting continuous mode */
	ADC1->CR2 |= ADC_CR2_CONT;

	/* Enabnling DMA for ADC1 ✍ */
	ADC1->CR2 |= ADC_CR2_DMA;

	/* Selecting continuous DMA request mode ✍ */
	ADC1->CR2 |= ADC_CR2_DDS;

	/* Set resolution
	 * Set to 00b for 12-bit resolution*/
	ADC1->CR1 &= ~ADC_CR1_RES_Msk;

	/* Set Sampling time for ADC1_IN6
	 * Setting b111 for 480 Cycle */
	ADC1->SMPR2 |= ADC_SMPR2_SMP6_0 | ADC_SMPR2_SMP6_1 | ADC_SMPR2_SMP6_2;

	/* Set ADC Sequence Register
	 *Total number of concersion in ADC1_IN6
	 *Seting b000 = 1 converstion */

	/* Setting Sequence length
	 * We have only one channle. Seting b0000  for 1 */
	ADC1->SQR1 &= ~ADC_SQR1_L_Msk;

	/* CH6 is first channel in the Sequence */
	ADC1->SQR3 |= ADC_SQR3_SQ1_1 | ADC_SQR3_SQ1_2;

}

void ADC1_StartConversion()
{
	/* Turing ON ADC1 */
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC1_StopConversion()
{
	ADC1->CR2 &= ~ADC_CR2_ADON;
}


void ADC1_Interrupt_Initialization()
{
	/********************* Interrupt based setting ***********************/
	/* Enable EOC )End of Conversion Inerrrupt Enable
	 * Other interrupts:
	 * End of conversion of an injected group 	: JEOC
	 * Analog watchdog status bit is set		: AWD
	 * Overrun									: Overrun
	 * */

	/* Disable Interrupt for ADC peripheral */
	NVIC_DisableIRQ(ADC_IRQn);

	/* Clear pending flag*/
	ADC1->SR = 0;

	/* Clear Pending Flag */
	NVIC_ClearPendingIRQ(ADC_IRQn);

	/* Setting EOC for each chanel */
	ADC1->CR2 |= ADC_CR2_EOCS;

	/* Disable Interrupt flag  */
	ADC1->CR1 &= ~ADC_CR1_EOCIE;

	/* Enabling EOCIE Interrupt Enable */
	ADC1->CR1 |= ADC_CR1_EOCIE;

	/* Enable  ADC1 Interrupt in NVIC*/
	NVIC_EnableIRQ(ADC_IRQn);
}

void ADC1_DMA_Initialization(uint16_t *pMemoryDst, uint16_t wDataLen)
{
	/* ADC1 is configured in DMA2 as either as Stream_0 : Channel_0
	 * or  Stream_4 : Channel_0*/

	/* Enable CLK for DMA2 */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/* Disable the DMA2_Stream0. Wait till DMA2_Stream0 is Disable */
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream0->CR & DMA_SxCR_EN);

	/* Clear all DMA2 Flags */
	DMA2->LIFCR = (DMA_LIFCR_CFEIF0 |
	DMA_LIFCR_CDMEIF0 |
	DMA_LIFCR_CTEIF0 |
	DMA_LIFCR_CHTIF0 |
	DMA_LIFCR_CTCIF0);

	/* Enable CH_0 in DMA2 Stream_0*/
	DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL;

	/* Configure Transfer Direction
	 * Peripharal to Memory, b00 */
	DMA2_Stream0->CR &= ~DMA_SxCR_DIR;

	/* Configure Memory Byte size.
	 * 16-bit. b01 */
	DMA2_Stream0->CR &= ~DMA_SxCR_MSIZE;
	DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;

	/* Configure Peripheal size
	 * 16-bits. 01 */
	DMA2_Stream0->CR &= ~DMA_SxCR_PSIZE;
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;

	/* Configure Peripharal (Source) */
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;

	/* Configure Destination (Memory location) */
	DMA2_Stream0->M0AR = (uint32_t) pMemoryDst;

	/* Enabling Circular buffer. Otherwise, the DMA will stop
	 * when NDTR becomes zero */
	DMA2_Stream0->CR |= DMA_SxCR_CIRC;

	/* Configure Peripheral address pointer increment.
	 * DONT need to increase Peripheral pointer register */
	DMA2_Stream0->CR &= ~DMA_SxCR_PINC;

	/* Configure Memory address pointer increment */
	DMA2_Stream0->CR |= DMA_SxCR_MINC;

	/*Configure Number of Data Register */
	DMA2_Stream0->NDTR = wDataLen;

	/* Configure Direct Mode. Enable Direct Mode */
	DMA2_Stream0->FCR &= ~DMA_SxFCR_DMDIS;

	/* Configure DMA2 internal priority */
	DMA2_Stream0->CR |= DMA_SxCR_PL_1;

	/* Enable Transfer Complete Interrupt Enable (TCIE) and
	 * Transfer Error Interrupt Enable (TEIE) */
	DMA2_Stream0->CR |= DMA_SxCR_TEIE |
	DMA_SxCR_TCIE;

	/* Set  DMA2_Stream0  priority in NVIC*/
	NVIC_SetPriority(DMA2_Stream0_IRQn, 5);

	/* Enable DMA2_Steram0 in NVIC */
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	/* Enable DMA2 Stream_0 */
	DMA2_Stream0->CR |= DMA_SxCR_EN;
}


void ADC_Callback(uint16_t dataADC)
{

}

extern uint8_t EOCFlagON;
extern uint16_t AdcCount;

/* ADC Interrupt handler. Handles End of Conversion-EOC interrrpt */
void ADC_IRQHandler()
{
	if (ADC1->SR & ADC_SR_EOC)
	{
		/* EOC flag set */
		EOCFlagON = 1;

		/* Reading ADC count and clearing ADC's EOC  */
		AdcCount = ADC1->DR;
	}
}

extern uint8_t DTCFlagON;

void DMA2_Stream0_IRQHandler()
{
	if(DMA2->LISR & DMA_LISR_TCIF0)
	{
		DTCFlagON = 1;

		/* CLearing TCIF */
		DMA2->LIFCR = DMA_LIFCR_CTCIF0;
	}
}
