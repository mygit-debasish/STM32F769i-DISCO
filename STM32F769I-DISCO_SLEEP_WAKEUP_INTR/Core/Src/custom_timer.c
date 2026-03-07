#include "custom_timer.h"

/*Initializing TIM2 for 1 seconf timebase and DMA request.
 * TIM2 located in APB1 bus with 108 MHz CLK */

void Custom_TIM2_Initiazation()
{
	/* CLK enable for TIM2 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/*Disable/STOP TIM2 for configuation to be cmpleted.
	 * Wiat till TIM2 has been disabled */
	TIM2->CR1 &= ~TIM_CR1_CEN;
	while (TIM2->CR1 & TIM_CR1_CEN);

	/* Configure Prescale and AAR for 1 second timebase */
	TIM2->PSC = 1080 - 1;
	TIM2->ARR = 100000 - 1;

	/* Force an update event after PSC and ARR to take immediate effect */
	TIM2->EGR = TIM_EGR_UG;

	/* Event will be generated for TIM2 counter overflow/underflow.
	 * Update Disable is 0 on Reset,so not requied explicitely.
	 * But Setting it to 1 will diable interrupt generated uisng TIM_DIER_UIE */
	TIM2->CR1 &= ~TIM_CR1_UDIS;

	/* Enable Update interrupt */
	TIM2->DIER |= TIM_DIER_UIE;

	/* Enable Update DMA Request */
	TIM2->DIER |= TIM_DIER_UDE;

	/* Enable TIM2_IRQHandler in NVIC❗ ✍*/
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Start TIM2 and check if couner is upadted */
	TIM2->CR1 |= TIM_CR1_CEN;
}

volatile uint8_t INTR_FLAG_ON = 0;

/* TIM2 Interrupt Handler */
void TIM2_IRQHandler()
{
	if (TIM2->SR & TIM_SR_UIF)
	{
		/* Reset the Flag */
		TIM2->SR = ~TIM_SR_UIF;

		/* Update Flag */
		INTR_FLAG_ON = 1;
	}
}
