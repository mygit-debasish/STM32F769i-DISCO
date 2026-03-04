/*
 * custom_rtc.h
 *
 *  Created on: Mar 3, 2026
 *      Author: Debasish Das
 */

#ifndef INC_CUSTOM_RTC_H_
#define INC_CUSTOM_RTC_H_

#include "custom.h"

/* External variables */
extern uint8_t RTC_INTR_ON;


/* Private Function Prototypes starts */
void RTC_Disable_Write_Protection();
void RTC_Enable_Write_Protection();
void Enter_INIT_Mode();
void Exit_INIT_Mode();
uint32_t RTC_Get_Clock_Src();
void RTC_Time_Date_Init();
void Generate_RTC_1Hz_Clock();
void RTC_Calender_Init();
void RTC_Disable_BackupDomain_Protection();
void RTC_Enable_BackupDomain_Protection();

void RTC_Calander_Config();
void RTC_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec);
void RTC_GetDate(uint8_t *year, uint8_t *month, uint8_t *day);

void RTC_AlarmA_Init();


/* Private Function Prototypes ends */

#endif /* INC_CUSTOM_RTC_H_ */
