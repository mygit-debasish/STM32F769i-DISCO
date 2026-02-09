/*
 * customLwIP.h
 *
 *  Created on: Feb 7, 2026
 *      Author: Debasish Das
 */

#ifndef INC_CUSTOMLWIP_H_
#define INC_CUSTOMLWIP_H_

#include "lwip/netif.h"

extern UART_HandleTypeDef huart1;

int ethernet_link_up(void);
void getMacAddr(struct netif *pnetif);
void getGateway(struct netif *pnetif);
void getIPAddress(struct netif *pnetif);
void getSubnetMask(struct netif *pnetif);



#endif /* INC_CUSTOMLWIP_H_ */
