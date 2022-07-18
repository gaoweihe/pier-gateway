/*
 * nic.c
 *
 * Created: 5/20/2021 8:26:38 AM
 *  Author: Gao, Weihe
 */ 
#include <xc.h>
#include "dependency.h"
#include "prototype.h"
#include "nic.h"

volatile NICData_t NICData;
volatile extern RuntimeFlags_t RuntimeFlags;
volatile extern RuntimeConfig_t RuntimgConfig;

void nic_var_init(void) {
	memset((void *)&NICData, 0, sizeof(NICData_t));
	NICData.isRxC = true;
	NICData.isTxC = true;
}

void nic_sock_init(void) {
	while (!RuntimeFlags.isNICInit) {} ; // Wait for NIC to initialize
	
	const uint8_t socket_num = 1;
	NIC_PACKET_t packet;
	
	NIC_SOCKET_MR_t socket_mode;
	socket_mode.bits.Protocol = 1; // TCP
	packet = RenderUpdateSocketModePacket(socket_num, socket_mode);
	InsertTxPacket(&packet);
	
	packet = RenderSocketCmdPacket(socket_num, NIC_SOCKET_CMD_OPEN);
	InsertTxPacket(&packet);
	
	NICData.sock_status[socket_num] = NIC_SOCKET_STATUS_OPENPEND;
}

void nic_init(void) {
	// NIC Int init
	PORTG.OUTCLR = PIN7_bm;
	PORTG.DIRSET = PIN7_bm;
	PORTG.PIN7CTRL |= PORT_ISC_FALLING_gc;
	
	nic_var_init();
	
	NIC_SIP_t ipAddr;
	NIC_SMAC_t macAddr;
	NIC_GATEWAY_t gatewayAddr;
	NIC_SUBNETMASK_t subnetMask;
	const uint8_t ip[] = { 192, 168, 3, 127 };
	const uint8_t mac[] = { 0x00, 0x08, 0xDC, 0x01, 0x02, 0x03 };
	const uint8_t gateway[] = { 192, 168, 3, 100 };
	const uint8_t submask[] = { 255, 255, 252, 0 };
		
	memcpy(&(macAddr.bytes), &mac, sizeof(NIC_SMAC_t));
	memcpy(&(ipAddr.bytes), &ip, sizeof(NIC_SIP_t));
	memcpy(&(gatewayAddr.bytes), &gateway, sizeof(NIC_GATEWAY_t));
	memcpy(&(subnetMask.bytes), &submask, sizeof(NIC_SUBNETMASK_t));
	
	NIC_PACKET_t packet;
	
	// Initialize mac address
	packet = RenderUpdateMACPacket(macAddr);
	InsertTxPacket(&packet);
	
	// Initialize ip address	
	packet = RenderUpdateIpAddrPacket(ipAddr);
	InsertTxPacket(&packet);
	
	// Initialize gateway
	packet = RenderUpdateGatewayPacket(gatewayAddr);
	InsertTxPacket(&packet);
	
	// Initialize subnet mask
	packet = RenderUpdateSubnetMaskPacket(subnetMask);
	InsertTxPacket(&packet);
	
	RuntimeFlags.isNICInit = true;
	
	nic_sock_init();
}

void Open(uint8_t sock_num) {
	if (sock_num >= NIC_SOCKET_COUNT) {
		return;
	}
	
	NIC_SOCKET_CR_t cmd = NIC_SOCKET_CMD_OPEN;
	NIC_PACKET_t packet = RenderSocketCmdPacket(sock_num, cmd);
	InsertTxPacket(&packet);
}

void Close(uint8_t sock_num) {
	if (sock_num >= NIC_SOCKET_COUNT) {
		return;
	}
	
	NIC_SOCKET_CR_t cmd = NIC_SOCKET_CMD_CLOSE;
	NIC_PACKET_t packet = RenderSocketCmdPacket(sock_num, cmd);
	InsertTxPacket(&packet);
}

void Connect(uint8_t sock_num) {
	if (sock_num >= NIC_SOCKET_COUNT) {
		return;
	}
	
	NIC_PACKET_t packet;
	
	NIC_DPORT_t destPort;
	NIC_DIP_t destIp;
	const uint8_t dip[] = { 192, 168, 3, 109 };
	const uint8_t dport[] = { 0x1A, 0x0D }; // 6669
		
	memcpy(&(destIp.bytes), &dip, sizeof(NIC_DIP_t));
	memcpy(&(destPort.bytes), &dport, sizeof(NIC_DPORT_t));
	
	packet = RenderUpdateSocketDPortPacket(1, destPort);
	InsertTxPacket(&packet);
	
	packet = RenderUpdateSocketDIPPacket(1, destIp);
	InsertTxPacket(&packet);
	
	packet = RenderSocketCmdPacket(1, NIC_SOCKET_CMD_CONNECT);
	InsertTxPacket(&packet);
	
	NICData.sock_status[sock_num] = NIC_SOCKET_STATUS_CONNPEND;
}

// @ Deprecated
uint16_t Send(uint8_t sock_num, uint8_t* buffer, uint16_t length) {
	if (sock_num >= NIC_SOCKET_COUNT) {
		return 0xFFFF;
	}
	return 0;
}

uint16_t SendEx(uint8_t sock_num, NIC_PACKET_t packet) {
	// if (NICData.isRxC == false) return 0;
	if (NICData.isTxC == false) return 0;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return 0xFFFF;
	}
	
	NICData.isTxC = false;
	NICData.isRxC = false;
	
	memcpy((void *)NICData.txBuff, (void *)&packet.payload, packet.meta.packet_length);
	NICData.txPtr = 0;
	NICData.txLen = packet.meta.packet_length;
	NICData.rxPtr = 0;
	NICData.rxLen = NICData.txLen;
	SPI0_SLAVE_SELECT();
	SPI0.DATA = NICData.txBuff[0];
	
	return 0;
}

uint16_t Receive(uint8_t sock_num, uint8_t* buffer, uint16_t length) {
	if (sock_num >= NIC_SOCKET_COUNT) {
		return 0xFFFF;
	}
	for (uint16_t i = 0; i < length; i++)
	{
		buffer[i] = SPI0.DATA;
	}
	return length;
}

bool isNICIdle() {
	if (NICData.isRxC == false) return false;
	if (NICData.isTxC == false) return false;
	return true;
}

void NICTxDispatch() {
	if (NICData.isTxC == false) return;
	for (uint8_t i = 0; i < NIC_SIZE_TXPACKBUFF; i++)
	{
		if (NICData.txPacketBuff[i].meta.type != NIC_PACKET_TYPE_IDLE) {
			NICData.curTxPacket = &(NICData.txPacketBuff[i]);
			SendEx(0, NICData.txPacketBuff[i]);
			break;
		}
	}
}

void NICRxDispatch() {
	for (uint8_t i = 0; i < NIC_SIZE_RXPACKBUFF; i++)
	{
		if (NICData.rxPacketBuff[i].meta.type != NIC_PACKET_TYPE_IDLE) {
			NICData.curRxPacket = &(NICData.rxPacketBuff[i]);
			NICRxDigestPacket(NICData.curRxPacket);
			NICData.curRxPacket->meta.type = NIC_PACKET_TYPE_IDLE;
		}
	}
}

volatile NIC_PACKET_t* FindIdleTxPacket() {
	for (uint8_t i = 0; i < NIC_SIZE_TXPACKBUFF; i++)
	{
		if (NICData.txPacketBuff[i].meta.type == NIC_PACKET_TYPE_IDLE) {
			volatile NIC_PACKET_t* packet = &(NICData.txPacketBuff[i]);
			packet->meta.serialNum = NICData.serialCounter;
			NICData.serialCounter++;
			return packet;
		}
	}
	return NULL;
}

volatile NIC_PACKET_t* FindIdleRxPacket() {
	for (uint8_t i = 0; i < NIC_SIZE_RXPACKBUFF; i++)
	{
		if (NICData.rxPacketBuff[i].meta.type == NIC_PACKET_TYPE_IDLE) {
			volatile NIC_PACKET_t* packet = &(NICData.rxPacketBuff[i]);
			packet->meta.serialNum = NICData.serialCounter;
			NICData.serialCounter++;
			return packet;
		}
	}
	return NULL;
}

NIC_PACKET_t RenderReadChipVersionPacket() {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x39;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_READ_bm;
	packet.payload.data[0] = 0;
	packet.meta.dp_length = 1;
	packet.meta.packet_length = 4;
	packet.meta.type = NIC_PACKET_TYPE_READ_VERSION;
	return packet;
}

volatile NIC_PACKET_t* InitializePacket(volatile NIC_PACKET_t* packet) {
	packet->meta.dp_length = 0;
	packet->meta.packet_length = 0;
	packet->meta.serialNum = 0;
	packet->meta.type = NIC_PACKET_TYPE_IDLE;
	memset((void *)packet->payload.data, 0, NIC_SIZE_TXBUFFER);
	packet->payload.addr.word = 0;
	packet->payload.ctrl.byte = 0;
	return packet;
}

NIC_PACKET_t RenderUpdateIpAddrPacket(NIC_SIP_t ipAddr) {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x0F;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = ipAddr.bytewise.SIPR0;
	packet.payload.data[1] = ipAddr.bytewise.SIPR1;
	packet.payload.data[2] = ipAddr.bytewise.SIPR2;
	packet.payload.data[3] = ipAddr.bytewise.SIPR3;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_SIP;
	return packet;
}

NIC_PACKET_t RenderReadIpAddrPacket() {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x0F;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_READ_bm;
	packet.payload.data[0] = 0;
	packet.payload.data[1] = 0;
	packet.payload.data[2] = 0;
	packet.payload.data[3] = 0;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_READ_SIP;
	return packet;
}

NIC_PACKET_t RenderUpdateMACPacket(NIC_SMAC_t macAddr) {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x09;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = macAddr.bytewise.SHAR0;
	packet.payload.data[1] = macAddr.bytewise.SHAR1;
	packet.payload.data[2] = macAddr.bytewise.SHAR2;
	packet.payload.data[3] = macAddr.bytewise.SHAR3;
	packet.payload.data[4] = macAddr.bytewise.SHAR4;
	packet.payload.data[5] = macAddr.bytewise.SHAR5;
	packet.meta.dp_length = 6;
	packet.meta.packet_length = 9;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_SMAC;
	return packet;
}

NIC_PACKET_t RenderReadMACPacket() {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x09;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_READ_bm;
	packet.payload.data[0] = 0;
	packet.payload.data[1] = 0;
	packet.payload.data[2] = 0;
	packet.payload.data[3] = 0;
	packet.payload.data[4] = 0;
	packet.payload.data[5] = 0;
	packet.meta.dp_length = 6;
	packet.meta.packet_length = 9;
	packet.meta.type = NIC_PACKET_TYPE_READ_SMAC;
	return packet;
}

NIC_PACKET_t RenderUpdateGatewayPacket(NIC_GATEWAY_t gateway) {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x01;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = gateway.bytewise.GAR0;
	packet.payload.data[1] = gateway.bytewise.GAR1;
	packet.payload.data[2] = gateway.bytewise.GAR2;
	packet.payload.data[3] = gateway.bytewise.GAR3;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_GATEWAY;
	return packet;
}

NIC_PACKET_t RenderReadGatewayPacket() {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x01;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_READ_bm;
	packet.payload.data[0] = 0;
	packet.payload.data[1] = 0;
	packet.payload.data[2] = 0;
	packet.payload.data[3] = 0;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_READ_GATEWAY;
	return packet;
}

NIC_PACKET_t RenderUpdateSubnetMaskPacket(NIC_SUBNETMASK_t subnetMask) {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x05;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = subnetMask.bytewise.SUBR0;
	packet.payload.data[1] = subnetMask.bytewise.SUBR1;
	packet.payload.data[2] = subnetMask.bytewise.SUBR2;
	packet.payload.data[3] = subnetMask.bytewise.SUBR3;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_SUBMASK;
	return packet;
}

NIC_PACKET_t RenderReadSubnetMaskPacket() {
	NIC_PACKET_t packet;
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x05;
	packet.payload.ctrl.bits.BSB = 0b00000;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_READ_bm;
	packet.payload.data[0] = 0;
	packet.payload.data[1] = 0;
	packet.payload.data[2] = 0;
	packet.payload.data[3] = 0;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_READ_SUBMASK;
	return packet;
}

NIC_PACKET_t RenderUpdateSocketModePacket(uint8_t sock_num, NIC_SOCKET_MR_t mode) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
	
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x00;
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = mode.byte;
	packet.meta.dp_length = 1;
	packet.meta.packet_length = 4;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_MODE;
	return packet;
}

NIC_PACKET_t RenderSocketCmdPacket(uint8_t sock_num, NIC_SOCKET_CR_t cmd) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
		
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x01;
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = cmd;
	packet.meta.dp_length = 1;
	packet.meta.packet_length = 4;
	packet.meta.type = NIC_PACKET_TYPE_CMD;
	return packet;
}

NIC_PACKET_t RenderUpdateSocketSPortPacket(uint8_t sock_num, NIC_SPORT_t sock_sport) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
	
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x04;	
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = sock_sport.bytewise.SPORTR0;
	packet.payload.data[1] = sock_sport.bytewise.SPORTR1;	
	packet.meta.dp_length = 2;
	packet.meta.packet_length = 5;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_SPORT;
	return packet;
}

NIC_PACKET_t RenderUpdateSocketDPortPacket(uint8_t sock_num, NIC_DPORT_t sock_dport) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
	
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x10;
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = sock_dport.bytewise.DPORTR0;
	packet.payload.data[1] = sock_dport.bytewise.DPORTR1;
	packet.meta.dp_length = 2;
	packet.meta.packet_length = 5;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_DPORT;
	return packet;	
}

NIC_PACKET_t RenderUpdateSocketDIPPacket(uint8_t sock_num, NIC_DIP_t sock_dip) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
	
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x0C;
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = sock_dip.bytewise.DIPR0;
	packet.payload.data[1] = sock_dip.bytewise.DIPR1;
	packet.payload.data[2] = sock_dip.bytewise.DIPR2;
	packet.payload.data[3] = sock_dip.bytewise.DIPR3;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_DIP;
	return packet;
}

NIC_PACKET_t RenderReadRxPtrPacket(uint8_t sock_num) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x28;
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_READ_bm;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_READ_RXPTR;
	return packet;
}

NIC_PACKET_t RenderReadTxPtrPacket(uint8_t sock_num) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x22;
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_READ_bm;
	packet.meta.dp_length = 4;
	packet.meta.packet_length = 7;
	packet.meta.type = NIC_PACKET_TYPE_READ_TXPTR;
	return packet;
}

NIC_PACKET_t RenderUpdRxPtrPacket(uint8_t sock_num, uint8_t rxrd[2]) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x28;
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = rxrd[0];
	packet.payload.data[1] = rxrd[1];
	packet.meta.dp_length = 2;
	packet.meta.packet_length = 5;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_RXPTR;
	return packet;
}

NIC_PACKET_t RenderUpdTxPtrPacket(uint8_t sock_num, uint8_t txwr[2]) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) {
		return packet;
	}
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x24;
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_WRITE_bm;
	packet.payload.data[0] = txwr[0];
	packet.payload.data[1] = txwr[1];
	packet.meta.dp_length = 2;
	packet.meta.packet_length = 5;
	packet.meta.type = NIC_PACKET_TYPE_MODIFY_TXPTR;
	return packet;
}

volatile NIC_PACKET_t* InsertTxPacket(volatile NIC_PACKET_t *packet) {
	volatile NIC_PACKET_t* idlePacket;
	idlePacket = FindIdleTxPacket();
	if (idlePacket == NULL) {
		return NULL;
	}
	
	packet->meta.serialNum = idlePacket->meta.serialNum;
	memcpy((void *)idlePacket, (const void *)packet, sizeof(NIC_PACKET_t));
	
	return idlePacket;
}

volatile NIC_PACKET_t* InsertRxPacket(volatile NIC_PACKET_t *packet) {
	return NULL;
}

void NICRxDigestPacket(volatile NIC_PACKET_t* packet) {
	NIC_PACKET_TYPE_t packetType = packet->meta.type;
	NIC_SOCKET_SR_t sr;
	uint8_t *data, *txrd, *txwr, *rxrd, *rxwr;
	switch (packetType) {
	case NIC_PACKET_TYPE_READ_SOCKSTAT:
		sr = (NIC_SOCKET_SR_t)((packet->payload).data[3]);
		UpdateSockStat(1, sr);
		break;
	case NIC_PACKET_TYPE_READ_TXPTR:
		data = (packet->payload).data;
		txrd = NICData.sock_data[1].txrd;
		txwr = NICData.sock_data[1].txwr;
		txrd[0] = data[3];
		txrd[1] = data[4];
		txwr[0] = data[5];
		txwr[1] = data[6];
		NICData.sock_data[1].isTxPtrRdy = true;
		break;
	case NIC_PACKET_TYPE_READ_RXPTR:
		data = (packet->payload).data;
		rxrd = NICData.sock_data[1].rxrd;
		rxwr = NICData.sock_data[1].rxwr;
		rxrd[0] = data[3];
		rxrd[1] = data[4];
		rxwr[0] = data[5];
		rxwr[1] = data[6];
		NICData.sock_data[1].isRxPtrRdy = true;
		break;
	default: 
		break;
	}
	packet->meta.type = NIC_PACKET_TYPE_IDLE;
}

void UpdateSockStat(uint8_t sock_num, NIC_SOCKET_SR_t sr) {
	switch (sr) {
	case NIC_SOCK_INIT:
		NICData.sock_status[sock_num] = NIC_SOCKET_STATUS_OPEN;
		break;
	case NIC_SOCK_ESTABLISHED:
		NICData.sock_status[sock_num] = NIC_SOCKET_STATUS_ESTABLISHED;
		break;
	case NIC_SOCK_CLOSED:
		NICData.sock_status[sock_num] = NIC_SOCKET_STATUS_UNINIT;
		break;
	default:
		break;
	}
}

void ReadSockPtr(uint8_t sock_num) {
	NIC_PACKET_t packet;
	if (sock_num >= NIC_SOCKET_COUNT) return; 
	if (NICData.sock_data[sock_num].isRxPtrRdy == false) {
		packet = RenderReadRxPtrPacket(sock_num);
		InsertTxPacket(&packet);		
	}
	if (NICData.sock_data[sock_num].isTxPtrRdy == false) {
		packet = RenderReadTxPtrPacket(sock_num);
		InsertTxPacket(&packet);		
	}
}

void NICSchedule() {
	for (uint8_t i = 1 /* 0 */; i < 2 /* NIC_SOCKET_COUNT */; i++)
	{
		NIC_SOCKET_STATEx_t status = NICData.sock_status[i];
		switch (status) {
		case NIC_SOCKET_STATUS_UNINIT:
			nic_sock_init();
		break;
		case NIC_SOCKET_STATUS_OPENPEND: 
		break;
		case NIC_SOCKET_STATUS_OPEN:
			Connect(1);
		break;
		case NIC_SOCKET_STATUS_ESTABLISHED:
			ReadSockPtr(1);
		break;
		default:
		break;
		}
	}
}

void CheckNICStatus() {
	// NIC_PACKET_t packet;
	
	CheckSocketStatus(1);
}

void CheckSocketStatus(uint8_t sock_num) {
	if (sock_num >= NIC_SOCKET_COUNT) {
		return;
	}
	
	NIC_PACKET_t packet;
	
	packet.payload.addr.bytes[0] = 0x00;
	packet.payload.addr.bytes[1] = 0x03;	
	packet.payload.ctrl.bits.BSB = NIC_BS_SOCKET1_REGISTER;
	packet.payload.ctrl.bits.OM = NIC_CTRL_PHASE_OM_N_gc;
	packet.payload.ctrl.bits.RWB = NIC_CTRL_PHASE_RWB_READ_bm;
	packet.payload.data[0] = 0;
	packet.meta.dp_length = 1;
	packet.meta.packet_length = 4;
	packet.meta.type = NIC_PACKET_TYPE_READ_SOCKSTAT;
	
	InsertTxPacket(&packet);
}

ISR(SPI0_INT_vect) {
	if ((SPI0.CTRLB & SPI_BUFEN_bm) == SPI_BUFEN_bm) { // if buffer mode
		if ((SPI0.INTFLAGS & SPI_RXCIF_bm) ==  SPI_RXCIF_bm) { // receive complete
			NICData.rxBuff[NICData.rxPtr] = SPI0.DATA;
			NICData.rxPtr++;
			if (NICData.rxPtr >= NICData.rxLen) {
				volatile NIC_PACKET_t* packet = FindIdleRxPacket();
				if (packet == NULL) reti(); // Discard
				memcpy((void *)packet->payload.data, (void *)NICData.rxBuff, NICData.rxLen);
				packet->meta.dp_length = NICData.rxLen;
				packet->meta.type = NICData.curTxPacket->meta.type;
				NICData.rxLen = 0;
				NICData.rxPtr = 0;
				NICData.isRxC = true;
			}
		}
		if ((SPI0.INTFLAGS & SPI_TXCIF_bm) == SPI_TXCIF_bm) { // transmission complete
			SPI0.INTFLAGS = SPI_TXCIF_bm;
			NICData.txPtr++;
			if (NICData.txPtr < NICData.txLen) {
				SPI0.DATA = NICData.txBuff[NICData.txPtr];
			}
			else {
				InitializePacket(NICData.curTxPacket);
				NICData.curTxPacket = NULL;
				NICData.txLen = 0;
				NICData.txPtr = 0;
				NICData.isTxC = true;
				SPI0_SLAVE_DESELECT();
			}
		}
	}
	else if ((SPI0.CTRLB & SPI_BUFEN_bm) == 0) { // if not buffer mode
		volatile uint8_t intflags = SPI0.INTFLAGS;
		NICData.rxBuff[NICData.rxPtr] = SPI0.DATA;
		NICData.rxPtr++;
		if (NICData.rxPtr >= NICData.rxLen) {
			volatile NIC_PACKET_t* packet = FindIdleRxPacket();
			if (packet == NULL) // Discard
			{
				reti();
			}
			memcpy((void *)packet->payload.data, (void *)NICData.rxBuff, NICData.rxLen);
			packet->meta.dp_length = NICData.rxLen;
			packet->meta.type = NICData.curTxPacket->meta.type;
			NICData.rxLen = 0;
			NICData.rxPtr = 0;
			NICData.isRxC = true;
		}
		
		NICData.txPtr++;
		if (NICData.txPtr < NICData.txLen) {
			SPI0.DATA = NICData.txBuff[NICData.txPtr];
		}
		else {
			InitializePacket(NICData.curTxPacket);
			NICData.curTxPacket = NULL;
			NICData.txLen = 0;
			NICData.txPtr = 0;
			NICData.isTxC = true;
			SPI0_SLAVE_DESELECT();
		}
	}
}

ISR(PORTG_PORT_vect) {
	volatile uint8_t a = 0;
	if (PORTG.INTFLAGS & PIN7_bm) { // nic int
		a++;	
	}
}