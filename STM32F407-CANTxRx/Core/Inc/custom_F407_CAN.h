/*
 * custom_F407_CAN.h
 *
 *  Created on: Aug 20, 2026
 *      Author: debasish
 */

#ifndef INC_CUSTOM_F407_CAN_H_
#define INC_CUSTOM_F407_CAN_H_

#include "stm32f407xx.h"
#include "stm32f4xx.h"

/** Define CAN message IDs **/
#define MSG_ID_TEMP	0x100
#define MSG_ID_PRES	0x102
#define MSG_ID_HUMD	0x150
#define MSG_ID_BARO	0x160
#define MSG_ID_RPM	0x180

void CAN1_Transmit(void);
void CAN2_Receive(void);
float DD_CAN_ConvertTemperatureMessage(uint8_t *pRxData, uint8_t DataLen);

#endif /* INC_CUSTOM_F407_CAN_H_ */
