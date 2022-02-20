/**
  ******************************************************************************
  * @file    LwIP/LwIP_UDP_Echo_Server/Inc/ethernetif.h 
  * @author  MCD Application Team
  * @brief   Header for ethernetif.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ETH_RX_BUFFER_SIZE                     (1024UL)
#define ETH_TX_BUFFER_SIZE                     (1024UL)

/* UDP local connection port */
#define UDP_SERVER_PORT    7
/* UDP remote connection port */
#define UDP_CLIENT_PORT    7

/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   (uint8_t) 10
#define IP_ADDR1   (uint8_t) 0
#define IP_ADDR2   (uint8_t) 0
#define IP_ADDR3   (uint8_t) 101

/*NETMASK*/
#define NETMASK_ADDR0   (uint8_t) 255
#define NETMASK_ADDR1   (uint8_t) 255
#define NETMASK_ADDR2   (uint8_t) 255
#define NETMASK_ADDR3   (uint8_t) 0

/*Gateway Address*/
#define GW_ADDR0   (uint8_t) 192
#define GW_ADDR1   (uint8_t) 168
#define GW_ADDR2   (uint8_t) 0
#define GW_ADDR3   (uint8_t) 1

/* Exported types ------------------------------------------------------------*/
err_t ethernetif_init(struct netif *netif);
void ethernetif_input(struct netif *netif);
void ethernet_link_check_state(struct netif *netif);
void EthInit(void);
void EthTick(void);
uint8_t* EthTxAlloc(uint16_t len);
uint8_t EthSend();
void EthTxEnd();

#ifdef __cplusplus
}
#endif

#endif
