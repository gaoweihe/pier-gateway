/*
 * collector.h
 *
 * Created: 5/23/2021 10:08:26 PM
 *  Author: Gao, Weihe
 */ 

#include <xc.h>

#ifndef COLLECTOR_H_
#define COLLECTOR_H_

#define COLLECTOR_COUNT		4
#define USART_SIZE_TXBUFF	128
#define USART_SIZE_RXBUFF	128

typedef struct CollectorData_struct {
	bool isTxC;
	bool isRxC;
	bool mutex;
	bool isCollectReq;
	uint8_t rxBuff[USART_SIZE_RXBUFF];
	uint8_t fwdBuff[USART_SIZE_RXBUFF];
	uint16_t rxPtr;
	uint16_t rxLen;
	uint16_t fwdLen;
	uint8_t txBuff[USART_SIZE_TXBUFF];
	uint16_t txPtr;
	uint16_t txLen;
} CollectorData_t;

void RetransmitToEthernet(uint8_t sock_num, const uint8_t const* data, uint8_t size);
void Read(uint8_t collector_num);
void CollectorTxDispatch();
void CollectorRxDispatch();
void CollectorTxCommSwitch(uint8_t collector_num, bool isTurnOn);
void CollectorRxCommSwitch(uint8_t collector_num, bool isTurnOn);

void CollectorOnReceive(uint8_t COLLECTOR_CURSOR);
void CollectorOnTransmit(uint8_t COLLECTOR_CURSOR);

#endif /* COLLECTOR_H_ */