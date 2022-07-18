/*
 * prototype.h
 *
 * Created: 5/20/2021 1:43:48 PM
 *  Author: Gao, Weihe
 */ 

#include <xc.h>
#include "dependency.h"
#include "nic.h"
#include "collector.h"

#ifndef PROTOTYPE_H_
#define PROTOTYPE_H_

#ifdef USART_BAUD_RATE
#undef USART_BAUD_RATE
#endif // USART1_BAUD_RATE
#define USART_BAUD_RATE(BAUD_RATE)		((double)(64 * F_CPU / (16 * (double)BAUD_RATE)) + 0.5)

// RS485A
#define USART1_TX_ENABLE()				(PORTC.OUTSET = PIN2_bm)
#define USART1_TX_DISABLE()				(PORTC.OUTCLR = PIN2_bm)
#define USART1_RX_ENABLE()				(PORTC.OUTCLR = PIN3_bm)
#define USART1_RX_DISABLE()				(PORTC.OUTSET = PIN3_bm)

// RS485B
#define USART3_TX_ENABLE()				(PORTB.OUTSET = PIN2_bm)
#define USART3_TX_DISABLE()				(PORTB.OUTCLR = PIN2_bm)
#define USART3_RX_ENABLE()				(PORTB.OUTCLR = PIN3_bm)
#define USART3_RX_DISABLE()				(PORTB.OUTSET = PIN3_bm)

// RS485C
#define USART4_TX_ENABLE()				(PORTE.OUTSET = PIN2_bm)
#define USART4_TX_DISABLE()				(PORTE.OUTCLR = PIN2_bm)
#define USART4_RX_ENABLE()				(PORTE.OUTCLR = PIN3_bm)
#define USART4_RX_DISABLE()				(PORTE.OUTSET = PIN3_bm)

// RS485D
#define USART5_TX_ENABLE()				(PORTG.OUTSET = PIN2_bm)
#define USART5_TX_DISABLE()				(PORTG.OUTCLR = PIN2_bm)
#define USART5_RX_ENABLE()				(PORTG.OUTCLR = PIN3_bm)
#define USART5_RX_DISABLE()				(PORTG.OUTSET = PIN3_bm)

#define SPI0_SLAVE_SELECT()				(PORTA.OUTCLR = PIN7_bm)
#define SPI0_SLAVE_DESELECT()			(PORTA.OUTSET = PIN7_bm)

typedef struct RuntimeFlags_struct {
	bool isNICInit;
	bool isChkNICStatusReq;
	bool isNICRdPtrRdy;
	bool isNICWrPtrRdy;
} RuntimeFlags_t;

typedef struct RuntimeConfig_struct {
	char guid[16];
	uint8_t sid; // segment id 
	uint8_t did; // device id
	NIC_SIP_t sip; // src ip
	NIC_DIP_t dip; // dest ip
	NIC_SMAC_t smac; // src mac
	NIC_DPORT_t dport; // dest port
	NIC_GATEWAY_t gateway;
	NIC_SUBNETMASK_t submask; // subnet mask
	NIC_SPORT_t sport; // src port
} RuntimeConfig_t;

#endif /* PROTOTYPE_H_ */