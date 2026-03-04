/*
 * custom_rtc.c
 *
 *  Created on: Mar 3, 2026
 *      Author: Debasish Das
 */

#include "custom_rtc.h"
#include "stm32f769xx.h"

void RTC_Disable_Write_Protection()
{
	/* After System Reset, RTC registers are protected against parasitic write access by clearing
	 * DBP (Disable Backup-Domain Protection) bit of CR1. To enable Write to RTC registers */

	/* Key Setting for Remove Write protection. After DISABLING "Write-Protection"
	  modification of RTC_TR, RTC_DR, RTC_PRER, RTC_CR etc are possible */

	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
}

void RTC_Enable_Write_Protection()
{
	/* This will ENFORCE "Write-Protection */
	RTC->WPR = 0xFF;
}

void Enter_INIT_Mode()
{
	/* During INIT mode:
	 * Calender counter frozen
	 * Time (Seconds, Minutes and Hours) stop incrementing
	 * Date Stop updating */

	RTC->ISR |= RTC_ISR_INIT;
	while(!(RTC->ISR & RTC_ISR_INITF));
}


void Exit_INIT_Mode()
{
	RTC->ISR &= ~RTC_ISR_INIT;
	/* Wait for INITF reset */
	while((RTC->ISR & RTC_ISR_INITF));
}


#define USE_HAL 0

uint32_t RTC_Get_Clock_Src()
{
#if USE_HAL
	RCC_PeriphCLKInitTypeDef periClk;
	HAL_RCCEx_GetPeriphCLKConfig(&periClk);
	return periClk.RTCClockSelection;
#else

	uint8_t RTC_CLK_SEL = (RCC->BDCR & RCC_BDCR_RTCSEL) >> RCC_BDCR_RTCSEL_Pos;

	switch (RTC_CLK_SEL)
		{
	case 0x00:
		return 0x00;

	case 0x01:
		return RCC_RTCCLKSOURCE_LSE;

	case 0x02:
		return RCC_RTCCLKSOURCE_LSI;

	case 0x03:
		return RCC_RTCCLKSOURCE_HSE_DIV31;

	default:
		return 0;
		}
#endif
}


void RTC_Time_Date_Init()
{
	/* Setting Time: 10:45:32 */
	RTC->TR = (1 << RTC_TR_HT_Pos) |
			(0 << RTC_TR_HU_Pos)  |
			(4 << RTC_TR_MNT_Pos) |
			(5 << RTC_TR_MNU_Pos) |
			(3 << RTC_TR_ST_Pos)  |
			(2 << RTC_TR_SU_Pos);

	/* Setting AM/PM */
	//RTC->TR |= RTC_TR_PM;

	/* Setting Date: 15:12:1979 */
	RTC->DR =
			(7 << RTC_DR_YT_Pos) |
			(9 << RTC_DR_YU_Pos) |
			(6 << RTC_DR_WDU_Pos)|
			(1 << RTC_DR_MT_Pos) |
			(2 << RTC_DR_MU_Pos) |
			(1 << RTC_DR_DT_Pos) |
			(5 << RTC_DR_DU_Pos);
}

void RTC_AlarmA_Init()
{
	/* Disable Alarm-A*/
	RTC->CR &= ~RTC_CR_ALRAE;

	/* Wait for RTC_ALR-A is writable */
	while(!(RTC->ISR & RTC_ISR_ALRAWF));

	/* Clear pending Flags */
	RTC->ISR &= ~RTC_ISR_ALRAF;

	/* Setting alarm interrupt for second = 15 */
	RTC->ALRMAR |= (1 << RTC_ALRMAR_ST_Pos) | (5 << RTC_ALRMAR_SU_Pos);

	/* Clear all Masks */
	RTC->ALRMAR &= ~RTC_ALARMMASK_ALL;

	/* Mask all other Alarms */
	RTC->ALRMAR |= RTC_ALRMAR_MSK2;
	RTC->ALRMAR |= RTC_ALRMAR_MSK3;
	RTC->ALRMAR |= RTC_ALRMAR_MSK4;

	/*Enable Alarm-A interrupt Enable */
	RTC->CR |= RTC_CR_ALRAIE;

	/* Enable Alarm-A*/
	RTC->CR |= RTC_CR_ALRAE;

	/* Enable EXTI Line17 and Rising Edge Trigger */
	EXTI->IMR  |= EXTI_IMR_IM17;
	EXTI->RTSR |= EXTI_RTSR_TR17;

	/* Enable NVIC*/
	NVIC_EnableIRQ(RTC_Alarm_IRQn);
}

void Generate_RTC_1Hz_Clock()
{
	/* Configuring 1Hz timebase */
	/* For HSEFreq = 32.768 KHz, HSEFreq/((PREDIV_A+1) * (PREDIV_S+1)), PREDIV_S = 255, PREDIV_A = 127 */
	/* For HSIFreq = 32.00 KHz, HSEFreq/((PREDIV_A+1) * (PREDIV_S+1)), PREDIV_S = 251, PREDIV_A = 127 */

	/* Setting Prescaler value based on RTC clock source */
	if (RTC_Get_Clock_Src() == RCC_RTCCLKSOURCE_LSE)
	{
		/* Setting RTC Prescaler (PREDVI_A) Register = 255 */
		RTC->PRER = 255 | 127 << 16;
	}
	else
	{
		/* Setting RTC Prescaler (PREDVI_A) Register = 255 */
		RTC->PRER = 251 | 127 << 16;
	}
}

void RTC_Calender_Init()
{
	/* This bit is write protected. Write must be enabled */
	RTC_Disable_Write_Protection();

	/* Enter the initialization mode. ISR: Initialization and Status Register */
	RTC->ISR |= RTC_ISR_INIT;

	/* Poll for Initialization has been Successful */
	while(!(RTC->ISR & RTC_ISR_INITF));

	Generate_RTC_1Hz_Clock();
}

void Enable_RTC()
{
	/* Enable RTC . This provides CLK to to RTC peripharal
	 * Configuration must be done after RTC has been clocked */
	RCC->BDCR |= RCC_BDCR_RTCEN;

}

void RTC_Disable_BackupDomain_Protection()
{
	/* Access to RTC and RTC Backup registers and backup SRAM Enabled */
	PWR->CR1 |= PWR_CR1_DBP;
}

void RTC_Enable_BackupDomain_Protection()
{
	/* Access to RTC and RTC Backup registers and backup SRAM Disable */
	PWR->CR1 &= ~PWR_CR1_DBP;
}

void RTC_Calander_Config()
{
	/* Enable CLK for PWR Regiser located in APB1 */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/*Disable Backup-Domain Protection for Write-Protect
	 * RTC Register
	 * RTC clock source selection
	 * Backup Data Registers
	 * Backup SRAM
	 * LSE Oscillator Configuration
	 *  */
	RTC_Disable_BackupDomain_Protection();

	/* Enable LSE as Oscillator */
	RCC->BDCR |= RCC_BDCR_LSEON;
	while(!(RCC->BDCR & RCC_BDCR_LSERDY));


	/* Enable LSE as CLK source for RTC
	 * Clear and  Assign as below:
	 * 01 : LSE
	 * 10: LSI
	 * 11: HSE oscillator clock divided by a programmable prescaler */
	RCC->BDCR &= ~RCC_BDCR_RTCSEL;
	RCC->BDCR |= RCC_BDCR_RTCSEL_0;

	/* Enable RTC */
	Enable_RTC();

	/* Disable Write-Protection for RTC reister */
	RTC_Disable_Write_Protection();

	/* Entering INIT Mode */
	Enter_INIT_Mode();

	/* Update Prescaler for generating 1 Hz timebase
	 * 32.768 kHz -> 1 Hz */
	Generate_RTC_1Hz_Clock();

	/* Initializing Calender */
	RTC_Time_Date_Init();

	/* Initializing Alarm-A */
	RTC_AlarmA_Init();

	/* Entering INIT Mode. Time, Date and Calender counter starts increamenting */
	Exit_INIT_Mode();

	/* Enable Write Protection of RTC registers */
	RTC_Enable_Write_Protection();
}


void RTC_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec)
{
	/* Get Updated time from Time Register
	 * TR must be read followed by DR (shadow register unlocking) */
	uint32_t CurrDate = RTC->DR;
	uint32_t CurrTime = RTC->TR;

	/* Prevent throwing warning */
	(void)CurrDate;

	uint8_t hour_T = (CurrTime >> 20) & 0x03;
	uint8_t hour_U = (CurrTime >> 16) & 0x0F;

	uint8_t min_T = (CurrTime >> 12) & 0x07;
	uint8_t min_U = (CurrTime >> 8) & 0x0F;

	uint8_t sec_T = (CurrTime >> 4) & 0x07;
	uint8_t sec_U = (CurrTime >> 0) & 0x0F;

	/* Converting BCD to Decimal */
	*hour = hour_T * 10 + hour_U;
	*min = min_T * 10 + min_U;
	*sec = sec_T * 10 + sec_U;
}

void RTC_GetDate(uint8_t *year, uint8_t *month, uint8_t *day)
{
	/* Get Updated time from Time Register
	 * TR must be read followed by DR (shadow register unlocking) */
	uint32_t CurrDate = RTC->DR;
	uint32_t CurrTime = RTC->TR;

	/* Prevent throwing warning */
	(void)CurrTime;

	uint8_t year_T = (CurrDate >> 20) & 0x0F;
	uint8_t year_U = (CurrDate >> 16) & 0x0F;

	uint8_t month_T = (CurrDate >> 12) & 0x01;
	uint8_t month_U = (CurrDate >> 8) & 0x0F;

	uint8_t date_T = (CurrDate >> 4) & 0x03;
	uint8_t date_U = (CurrDate >> 0) & 0x0F;

	/* Converting BCD to Decimal */
	*year = year_T * 10 + year_U;
	*month = month_T * 10 + month_U;
	*day = date_T * 10 + date_U;
}



/* Interrupt Servive Routine for RTC Alarm Handler */
void RTC_Alarm_IRQHandler(void)
{
	if (RTC->ISR & RTC_ISR_ALRAF)
	{
		/* Clear the Alarm Flag */
		RTC->ISR &= ~RTC_ISR_ALRAF;

		/* Clear EXTI Flag */
		EXTI->PR = EXTI_PR_PR17;

		/* RTC Alarm Interrupt triggered Flag */
		RTC_INTR_ON = 1;
	}
}


