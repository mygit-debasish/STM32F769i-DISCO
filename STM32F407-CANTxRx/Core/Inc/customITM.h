/*
 * customITM.h
 *
 *  Created on: Aug 23, 2026
 *      Author: debasish
 */

#ifndef INC_CUSTOMITM_H_
#define INC_CUSTOMITM_H_

void SWO_Pin_Init(void);
void ITM_TPIU_Init();
void sendtoITM(char ch);
void ITM_sendStr(const char *str);


#endif /* INC_CUSTOMITM_H_ */
