/*
 * customLwIP.c
 *
 *  Created on: Feb 7, 2026
 *      Author: Debasish Das
 */

#include "customLwIP.h"
#include "custom.h"
#include "lan8742.h"

extern lan8742_Object_t LAN8742;
char aMsgBuff[MAX_ARR_SIZE];

int ethernet_link_up(void)
{
	int32_t status;
	status = LAN8742_GetLinkState(&LAN8742);
	return (status != LAN8742_STATUS_LINK_DOWN);
}

void getMacAddr(struct netif *pnetif)
{
	char MACStr[50];
	int wMsgLen = 0;

	wMsgLen = snprintf(MACStr, MAX_ARR_SIZE,
			"%02X:%02X:%02X:%02X:%02X:%02X", pnetif->hwaddr[0],
			pnetif->hwaddr[1], pnetif->hwaddr[2], pnetif->hwaddr[3],
			pnetif->hwaddr[4], pnetif->hwaddr[5]);

	writeASCIItoSerial(&huart1, ASCII, MACStr, strlen(MACStr), "MAC address");
}

void getGateway(struct netif *pnetif)
{
	char GWStr[50];
	u32_t GW_u32 = pnetif->gw.addr;
	ip4addr_ntoa_r((const ip4_addr_t*) &GW_u32, GWStr, sizeof(GWStr));

	writeASCIItoSerial(&huart1, ASCII, GWStr, strlen(GWStr), "Gateway");
}

void getSubnetMask(struct netif *pnetif)
{
	char SNetStr[50];
	u32_t SNet_u32 = pnetif->netmask.addr;
	ip4addr_ntoa_r((const ip4_addr_t*) &SNet_u32, SNetStr, sizeof(SNetStr));

	writeASCIItoSerial(&huart1, ASCII, SNetStr, strlen(SNetStr), "Subnet Mask");
}

void getIPAddress(struct netif *pnetif)
{
	char IPStr[50];
	u32_t IP_u32 = pnetif->ip_addr.addr;
	ip4addr_ntoa_r((const ip4_addr_t*) &IP_u32, IPStr, sizeof(IPStr));

	writeASCIItoSerial(&huart1, ASCII, IPStr, strlen(IPStr), "IP address");
}
