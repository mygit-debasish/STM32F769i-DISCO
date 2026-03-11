#include "custom_timer.h"

/*Initializing TIM2 for 1 seconf timebase and DMA request.
 * TIM2 located in APB1 bus with 108 MHz CLK */

volatile uint8_t INTR_FLAG_ON = 0;


void STM32F769_USER2_LED_Init()
{
    /* Enable GPIOJ clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;

    /* PJ5 Output mode */
    GPIOJ->MODER &= ~GPIO_MODER_MODER5;
    GPIOJ->MODER |=  GPIO_MODER_MODER5_0;
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

	/* Enable Update interrupt for TIM1 */
	TIM1->DIER |= TIM_DIER_UIE;

	/* Enable DMA Event for TIM1*/
	TIM1->DIER = TIM_DIER_UDE;
}


/* Configuring DMA2 request for TIM1_UP
 * Stream_5, Channle_6*/

void STM32F769_DMA2_Stream5_Init(uint32_t *pMemSrc, uint32_t bPattSize )
{
  /* DMA2 Configuration for TIM1 UP */
	RCC->AHB1ENR|=RCC_AHB1ENR_DMA2EN;

	DMA2_Stream5->CR&=DMA_SxCR_EN;
	while((DMA2_Stream5->CR) & DMA_SxCR_EN);

	DMA2_Stream5->CR =(CH6<<DMA_SxCR_CHSEL_Pos)|
	    DMA_SxCR_MSIZE|
	    DMA_SxCR_PSIZE|
	    DMA_SxCR_MINC|
	    DMA_SxCR_CIRC|
	    DMA_SxCR_DIR_0;

	DMA2_Stream5->NDTR=(uint32_t)bPattSize;
	DMA2_Stream5->PAR=(uint32_t)(&GPIOJ->BSRR);
	DMA2_Stream5->M0AR=(uint32_t)pMemSrc;

	/* Clear corresponding DMA event flags */
	DMA2->HIFCR |= DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5;

	/* Enable DMA TCIE and TEIE for DMA2 */
	DMA2_Stream5->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE;

	/* Enable Interrupt in NVIC */
	//NVIC_EnableIRQ(DMA2_Stream5_IRQn);

	/* Enabling DMA2 stream_5*/
	DMA2_Stream5->CR |= DMA_SxCR_EN;
}


/* TIM2 Interrupt Handler */
void TIM1_IRQHandler()
{
	if (TIM1->SR & TIM_SR_UIF)
	{
		/* Reset the Flag */
		TIM1->SR = ~TIM_SR_UIF;

		/* Update Flag */
		INTR_FLAG_ON = 1;
	}
}
