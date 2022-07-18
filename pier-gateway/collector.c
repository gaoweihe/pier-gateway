/*
 * collector.c
 *
 * Created: 5/23/2021 10:08:15 PM
 *  Author: Gao, Weihe
 */ 

#include <xc.h>
#include "dependency.h"
#include "prototype.h"
#include "collector.h"
#include "nic.h"
#include "util.h"

volatile extern NICData_t NICData;
volatile CollectorData_t CollectorData[COLLECTOR_COUNT];
volatile extern RuntimeConfig_t RuntimeConfig;

uint8_t* txEnt[COLLECTOR_COUNT] = {
	&(USART1.TXDATAL),
	&(USART3.TXDATAL),
	&(USART4.TXDATAL),
	&(USART5.TXDATAL)
};
uint8_t* rxEnt[COLLECTOR_COUNT] = {
	&(USART1.RXDATAL),
	&(USART3.RXDATAL),
	&(USART4.RXDATAL),
	&(USART5.RXDATAL)
};

void RetransmitToEthernet(uint8_t sock_num, const uint8_t const* data, uint8_t size) {
	NIC_PACKET_t packet;
	NICSockData_t* sock_data = &(NICData.sock_data[sock_num]);
	// if (sock_data->isTxPtrRdy == false) return;
	uint8_t txwr[2];
	txwr[0] = sock_data->txwr[0];
	txwr[1] = sock_data->txwr[1];
	packet.payload.addr.bytes[0] = sock_data->txwr[0];
	packet.payload.addr.bytes[1] = sock_data->txwr[1];
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_TXBUFFER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	memcpy(packet.payload.data, data, size);
	packet.meta.dp_length = size;
	packet.meta.packet_length = size + 3;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_TXBUFF;
	InsertTxPacket(&packet);
	
	uint16_t txwr_word = GetWordFromBytes(txwr[0], txwr[1]);
	txwr_word += size;
	uint8_t txwr_upd[2];
	txwr_upd[0] = GetBytesFromWord(txwr_word, 0);
	txwr_upd[1] = GetBytesFromWord(txwr_word, 1);
	packet = RenderUpdTxPtrPacket(sock_num, txwr_upd);
	if (InsertTxPacket(&packet) != NULL) {
		sock_data->txwr[0] = txwr_upd[0];
		sock_data->txwr[1] = txwr_upd[1];
	}
	
	packet = RenderSocketCmdPacket(sock_num, NIC_SOCKET_CMD_SEND);
	InsertTxPacket(&packet);
}

void CollectorTxDispatch() {
	for (uint8_t i = 0; i < COLLECTOR_COUNT; i++)
	{
		if (CollectorData[i].isCollectReq == true) {
			Read(i);
			CollectorData[i].isCollectReq = false;
		}
	}
}

void CollectorRxDispatch() {
	for (uint8_t i = 0; i < COLLECTOR_COUNT; i++)
	{
		if (CollectorData[i].isRxC == true) {
			CollectorData[i].fwdLen = CollectorData[i].rxLen;
			memcpy(
				CollectorData[i].fwdBuff, 
				CollectorData[i].rxBuff, 
				CollectorData[i].fwdLen);
			CollectorData[i].rxLen = 0;
			CollectorData[i].rxPtr = 0;
			CollectorData[i].isRxC = false;
			if (CheckChecksum(CollectorData[i].fwdBuff, CollectorData[i].fwdLen)) {
				RetransmitToEthernet(
					i, 
					CollectorData[i].fwdBuff, 
					CollectorData[i].fwdLen);
			}
		}
	}
}

void CollectorTxCommSwitch(uint8_t collector_num, bool isTurnOn) {
	if (isTurnOn == true) goto on;
	if (isTurnOn == false) goto off;

on:	
	switch (collector_num) {
	case 0: // RS485A
		USART1_TX_ENABLE();
	break;
	case 1: // RS485B
		USART3_TX_ENABLE();
	break;
	case 2: // RS485C
		USART4_TX_ENABLE();
	break;
	case 3: // RS485D
		USART5_TX_ENABLE();
	break;	
	}
	return;

off: 
	switch (collector_num) {
	case 0: // RS485A
		USART1_TX_DISABLE();
	break;
	case 1: // RS485B
		USART3_TX_DISABLE();
	break;
	case 2: // RS485C
		USART4_TX_DISABLE();
	break;
	case 3: // RS485D
		USART5_TX_DISABLE();
	break;
	}
	return;
}

void CollectorRxCommSwitch(uint8_t collector_num, bool isTurnOn) {
	if (isTurnOn == true) goto on;
	if (isTurnOn == false) goto off;

on:
	switch (collector_num) {
	case 0: // RS485A
		USART1_RX_ENABLE();
	break;
	case 1: // RS485B
		USART3_RX_ENABLE();
	break;
	case 2: // RS485C
		USART4_RX_ENABLE();
	break;
	case 3: // RS485D
		USART5_RX_ENABLE();
	break;
	}
	return;

off:
	switch (collector_num) {
	case 0: // RS485A
		USART1_RX_DISABLE();
	break;
	case 1: // RS485B
		USART3_RX_DISABLE();
	break;
	case 2: // RS485C
		USART4_RX_DISABLE();
	break;
	case 3: // RS485D
		USART5_RX_DISABLE();
	break;
	}
	return;
}

void Read(uint8_t collector_num) {
	// if (CollectorData[collector_num].mutex == true) return;
	uint8_t* txBuffer = CollectorData[collector_num].txBuff;
	txBuffer[0] = 0x80 | collector_num;
	txBuffer[1] = 0;
	txBuffer[2] = 6;
	txBuffer[3] = RuntimeConfig.did;
	txBuffer[4] = 'R';
	txBuffer[5] = GetChecksum(txBuffer, 5);
	CollectorData[collector_num].txLen = 6;
	CollectorData[collector_num].txPtr = 0;
	
	CollectorRxCommSwitch(collector_num, false);
	CollectorTxCommSwitch(collector_num, true);
	*(txEnt[collector_num]) = txBuffer[0];
	CollectorData[collector_num].txPtr++;
}

void CollectorOnReceive(uint8_t COLLECTOR_CURSOR) {
	CollectorData_t* CurCltData = &(CollectorData[COLLECTOR_CURSOR]);
	
	uint8_t rxByte = 0;
	rxByte = *(rxEnt[COLLECTOR_CURSOR]);
	
	if (CurCltData->isRxC == true) reti(); // if prev pack not buffed
	
	CurCltData->mutex = true;
	if ((rxByte & 0x80) != 0) { // synchronization head
		rxByte &= 0x7F; // eliminate synchronization head
		CurCltData->rxPtr = 0;
	}	
	CurCltData->rxBuff[CurCltData->rxPtr] = rxByte;
	CurCltData->rxPtr++;
	if (CurCltData->rxPtr == 2) {
		CurCltData->rxLen = rxByte; // length
	}
	else if (CurCltData->rxPtr == 3) {
		CurCltData->rxLen = GetWordFrom14Bits(CurCltData->rxLen, rxByte);
	}
	else if (CurCltData->rxLen == CurCltData->rxPtr) {
		CurCltData->isRxC = true;
		CurCltData->mutex = false;
	}
}

void CollectorOnTransmit(uint8_t COLLECTOR_CURSOR) {
	CollectorData_t* CurCltData = &(CollectorData[COLLECTOR_CURSOR]);
	if (CurCltData->txPtr >= CurCltData->txLen) {
		CurCltData->isTxC = true;
		CollectorTxCommSwitch(COLLECTOR_CURSOR, false);
		CollectorRxCommSwitch(COLLECTOR_CURSOR, true);
		CurCltData->txPtr = 0;
		CurCltData->txLen = 0;
	}
	else {
		*(txEnt[COLLECTOR_CURSOR]) = CurCltData->txBuff[CurCltData->txPtr];
		CurCltData->txPtr++;
	}
}

ISR(USART0_RXC_vect) {
}

ISR(USART0_TXC_vect) {
	USART0.STATUS = USART_TXCIF_bm; 
}

ISR(USART1_RXC_vect) {
	const uint8_t COLLECTOR_CURSOR = 0;
	CollectorOnReceive(COLLECTOR_CURSOR);
}

ISR(USART1_TXC_vect) {
	const uint8_t COLLECTOR_CURSOR = 0;
	CollectorOnTransmit(COLLECTOR_CURSOR);
	USART1.STATUS = USART_TXCIF_bm; // clear txc flag
}

ISR(USART3_RXC_vect) {
	const uint8_t COLLECTOR_CURSOR = 1;
	CollectorOnReceive(COLLECTOR_CURSOR);
}

ISR(USART3_TXC_vect) {
	const uint8_t COLLECTOR_CURSOR = 1;
	CollectorOnTransmit(COLLECTOR_CURSOR);
	USART3.STATUS = USART_TXCIF_bm; // clear txc flag
}

ISR(USART4_RXC_vect) {
	const uint8_t COLLECTOR_CURSOR = 2;
	CollectorOnReceive(COLLECTOR_CURSOR);
}

ISR(USART4_TXC_vect) {
	const uint8_t COLLECTOR_CURSOR = 2;
	CollectorOnTransmit(COLLECTOR_CURSOR);
	USART4.STATUS = USART_TXCIF_bm; // clear txc flag
}

ISR(USART5_RXC_vect) {
	const uint8_t COLLECTOR_CURSOR = 3;
	CollectorOnReceive(COLLECTOR_CURSOR);
}

ISR(USART5_TXC_vect) {
	const uint8_t COLLECTOR_CURSOR = 3;
	CollectorOnTransmit(COLLECTOR_CURSOR);
	USART5.STATUS = USART_TXCIF_bm; // clear txc flag
}