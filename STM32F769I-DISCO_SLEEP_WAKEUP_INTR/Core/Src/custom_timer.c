#include "custom_timer.h"

#define TIM1_INTR_ENB 		0U
#define TIM1_DMA_ENB		1U
#define SLEEP_MODE			0U

#define INTR_ENB			1U
#define BUFF_SIZE 			32U

/*Initializing TIM2 for 1 seconf timebase and DMA request.
 * TIM2 located in APB1 bus with 108 MHz CLK */

volatile uint8_t INTR_FLAG_ON = 0;

void STM32F769_USER2_LED_Init()
{
	/* Enable GPIOJ clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;

	/* PJ5 Output mode
	 * PJ13 Output mode */
	GPIOJ->MODER &= ~GPIO_MODER_MODER5 | ~GPIO_MODER_MODER13 ;
	GPIOJ->MODER |= GPIO_MODER_MODER5_0 | GPIO_MODER_MODER13_0;
}

/* TIM1 is driven by APB2 Bus to which DMA2 has direct interface
 * TIM1 is located in High speed bus (108MHz) */
void STM32F769_TIM1_Init(uint8_t timeBase_Hz)
{
	uint32_t TIM1_CLKFREQ;

	/*Timer configuration*/
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	/* Get APB2 PCLK2 Frequency */
	uint32_t PCLK2 = HAL_RCC_GetPCLK2Freq();

	/* Get Frequency scaling before TIM1 */
	if ((RCC->CFGR & RCC_CFGR_PPRE2) == RCC_CFGR_PPRE2_DIV1)

		TIM1_CLKFREQ = PCLK2;
	else
		TIM1_CLKFREQ = 2 * PCLK2;

	/* Setting PRS and ARR for required timeBase_Hz */
	TIM1->PSC = (TIM1_CLKFREQ / 10000) - 1;
	TIM1->ARR = (10000 / timeBase_Hz) - 1;

#if TIM1_INTR_ENB
	/* Enable Update interrupt for TIM1*/
	TIM1->DIER |= TIM_DIER_UIE;

	/* Enable TIM1 UP interrupt in NVIC */
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
#endif

#if TIM1_DMA_ENB
	/* Enable DMA Event for TIM1*/
	TIM1->DIER |= TIM_DIER_UDE;
#endif
}


/* TIM2 is driven by APB1 Bus
 * TIM2 is located in Low speed bus (54MHz) */
void STM32F769_TIM2_Init(uint8_t timeBase_Sec)
{
	uint32_t TIM2_CLKFREQ;

	/*Timer configuration*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Get APB2 PCLK1 Frequency */
	uint32_t PCLK1 = HAL_RCC_GetPCLK1Freq();

    /* Timer clock doubling when APB prescaler > 1 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1_Msk) == RCC_CFGR_PPRE1_DIV1)
        TIM2_CLKFREQ = PCLK1;
    else
        TIM2_CLKFREQ = 2 * PCLK1;;

	/* Setting PRS and ARR for required timeBase_Hz */
	TIM2->PSC = (TIM2_CLKFREQ / 10000) - 1;
	TIM2->ARR = (10000 * timeBase_Sec) - 1;

	/* Enable Update interrupt for TIM2 */
	//TIM2->DIER |= TIM_DIER_UIE;

	/* Enable TIM2 UP interrupt in NVIC */
	//NVIC_EnableIRQ(TIM2_IRQn);
}

/* Configuring DMA2 request for TIM1_UP
 * Stream_5, Channle_6*/

void STM32F769_DMA2_Stream5_Init(uint32_t *pMemSrc, uint32_t bPattSize)
{
	/* DMA2 Configuration for TIM1 UP */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	DMA2_Stream5->CR &= ~DMA_SxCR_EN;
	while ((DMA2_Stream5->CR) & DMA_SxCR_EN);

	DMA2_Stream5->CR = (CH6 << DMA_SxCR_CHSEL_Pos) |
	DMA_SxCR_MSIZE |
	DMA_SxCR_PSIZE |
	DMA_SxCR_MINC |
	DMA_SxCR_CIRC |
	DMA_SxCR_DIR_0;

	DMA2_Stream5->NDTR = (uint32_t) bPattSize;
	DMA2_Stream5->PAR = (uint32_t) (&GPIOJ->BSRR);
	DMA2_Stream5->M0AR = (uint32_t) pMemSrc;

	/* Clear corresponding DMA event flags */
	DMA2->HIFCR |= DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5;

#if SLEEP_MODE
	/* Enable DMA TCIE and TEIE for DMA2 */
	DMA2_Stream5->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE;

	/* Enable DMA Interrupt in NVIC */
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);

#endif

	/* Enabling DMA2 stream_5*/
	DMA2_Stream5->CR |= DMA_SxCR_EN;
}

/* Configuring DMA2 request for Memory to Memory Transfer
 * Stream_1, Channle_1 (Any channel. Channel Selection is irrelevent) */

void STM32F769_DMA2_Stream1_Init(uint8_t *pMemSrc, uint8_t *pMemDst,
		uint32_t buffSize)
{
	/* DMA2 Configuration for memory to memory transfer */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	DMA2_Stream1->CR &= ~DMA_SxCR_EN;
	while ((DMA2_Stream1->CR) & DMA_SxCR_EN);

	DMA2_Stream1->CR = (CH1 << DMA_SxCR_CHSEL_Pos) |
	DMA_SxCR_PINC |
	DMA_SxCR_MINC |
	DMA_SxCR_DIR_1;

	/* Set the PSIZE and MSIZE : 8 Bits */
	DMA2_Stream1->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);

	DMA2_Stream1->NDTR = (uint32_t) buffSize;
	DMA2_Stream1->PAR = (uint32_t) pMemSrc;
	DMA2_Stream1->M0AR = (uint32_t) pMemDst;




	/* Clear corresponding DMA event flags */
	DMA2->LIFCR |= DMA_LIFCR_CTCIF1 | DMA_LIFCR_CTEIF1;

#if INTR_ENB
	/* Enable DMA TCIE and TEIE for DMA2 */
	DMA2_Stream1->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE;

	/* Enable DMA Interrupt in NVIC */
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);

#endif

	/* Enabling DMA2 stream_5*/
	DMA2_Stream1->CR |= DMA_SxCR_EN;
}


/* TIM1 Interrupt Handler */
void TIM1_UP_TIM10_IRQHandler(void)
{
	if (TIM1->SR & TIM_SR_UIF)
	{
		/* Reset the Flag */
		TIM1->SR &= ~TIM_SR_UIF;

		/* Update Flag */
		INTR_FLAG_ON = 1;
	}
}

/* TIM2 Interrupt Handler */
void TIM2_IRQHandler()
{
	if(TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;
		GPIOJ->ODR ^= GPIO_ODR_OD13;
	}
}

/* Interrupt handler for Memory to Memory Transfer */

volatile uint8_t M2M_TC_ON = 0;
void DMA2_Stream1_IRQHandler()
{
	if(DMA2->LISR & DMA_LISR_TCIF1)
	{
		/* Clear using Interrupt Flag Clear Register */
		DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
		M2M_TC_ON = 1;
	}
}

