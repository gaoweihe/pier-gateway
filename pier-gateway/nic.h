/*
 * nic.h
 *
 * Created: 5/20/2021 8:26:49 AM
 *  Author: Gao, Weihe
 */ 

#ifndef NIC_H_
#define NIC_H_

#ifdef NIC_VERSION
#undef NIC_VERSION
#warning "NIC_VERSION changed"
#endif
#define NIC_VERSION	"w5500v4"


/************************************************************************/
/* NIC Block Select Section                                             */
/************************************************************************/
typedef enum NIC_BS_enum {
	NIC_BS_COMMON_REGISTER = 0x00, // 0b00000
	NIC_BS_SOCKET0_REGISTER,
	NIC_BS_SOCKET0_TXBUFFER,
	NIC_BS_SOCKET0_RXBUFFER,
	NIC_BS_RESERVED0,
	NIC_BS_SOCKET1_REGISTER,
	NIC_BS_SOCKET1_TXBUFFER,
	NIC_BS_SOCKET1_RXBUFFER,
	NIC_BS_RESERVED1,
	NIC_BS_SOCKET2_REGISTER,
	NIC_BS_SOCKET2_TXBUFFER,
	NIC_BS_SOCKET2_RXBUFFER,
	NIC_BS_RESERVED2,
	NIC_BS_SOCKET3_REGISTER,
	NIC_BS_SOCKET3_TXBUFFER,
	NIC_BS_SOCKET3_RXBUFFER,
	NIC_BS_RESERVED3,
	NIC_BS_SOCKET4_REGISTER,
	NIC_BS_SOCKET4_TXBUFFER,
	NIC_BS_SOCKET4_RXBUFFER,
	NIC_BS_RESERVED4,
	NIC_BS_SOCKET5_REGISTER,
	NIC_BS_SOCKET5_TXBUFFER,
	NIC_BS_SOCKET5_RXBUFFER,
	NIC_BS_RESERVED5,
	NIC_BS_SOCKET6_REGISTER,
	NIC_BS_SOCKET6_TXBUFFER,
	NIC_BS_SOCKET6_RXBUFFER,
	NIC_BS_RESERVED6,
	NIC_BS_SOCKET7_REGISTER,
	NIC_BS_SOCKET7_TXBUFFER,
	NIC_BS_SOCKET7_RXBUFFER = 0x1F // 0b11111
} NIC_BS_t;



/************************************************************************/
/* NIC Status Section                                                   */
/************************************************************************/
typedef enum NIC_STATUS_enum {
	NIC_STATUS_IDLE = 0x00,
	NIC_STATUS_BUSY = 0x01
} NIC_STATUS_t;

typedef enum NIC_SOCKET_STATEx_enum {
	NIC_SOCKET_STATUS_UNINIT		= 0x00,
	NIC_SOCKET_STATUS_OPENPEND		= 0x01,
	NIC_SOCKET_STATUS_OPEN			= 0x02,
	NIC_SOCKET_STATUS_CONNPEND		= 0x03,
	NIC_SOCKET_STATUS_ESTABLISHED	= 0x04,
	NIC_SOCKET_STATUS_CLOSED		= 0x10
} NIC_SOCKET_STATEx_t;

typedef enum NIC_SOCKET_SR_enum {
	NIC_SOCK_CLOSED			= 0x00,
	NIC_SOCK_INIT			= 0x13,
	NIC_SOCK_LISTEN			= 0x14,
	NIC_SOCK_ESTABLISHED	= 0x17,
	NIC_SOCK_CLOSE_WAIT		= 0x1C,
	NIC_SOCK_UDP			= 0x22,
	NIC_SOCK_MACRAW			= 0x42,
	NIC_SOCK_SYNSENT		= 0x15,
	NIC_SOCK_SYNRECV		= 0x16,
	NIC_SOCK_FIN_WAIT		= 0x18,
	NIC_SOCK_CLOSING		= 0x1A,
	NIC_SOCK_TIME_WAIT		= 0x1B,
	NIC_SOCK_LAST_ACK		= 0x1D
} NIC_SOCKET_SR_t;


/************************************************************************/
/* NIC Register Data Structure Section                                  */
/************************************************************************/
typedef union NIC_ADDR_PHASE_union {
	uint8_t bytes[2];
	uint16_t word;
} NIC_ADDR_PHASE_t;

typedef enum NIC_CTRL_PHASE_RWB_enum {
	NIC_CTRL_PHASE_RWB_READ_bm		= 0b0,
	NIC_CTRL_PHASE_RWB_WRITE_bm		= 0b1
} NIC_CTRL_PHASE_RWB_t;

typedef enum NIC_CTRL_PHASE_OM_enum {
	NIC_CTRL_PHASE_OM_N_gc = 0b00,
	NIC_CTRL_PHASE_OM_1_gc = 0b01,
	NIC_CTRL_PHASE_OM_2_gc = 0b10,
	NIC_CTRL_PHASE_OM_4_gc = 0b11
} NIC_CTRL_PHASE_OM_t;

typedef union NIC_CTRL_PHASE_union {
	struct {
		uint8_t OM:2;
		uint8_t RWB:1;
		uint8_t BSB:5;
	} bits;
	uint8_t byte;
} NIC_CTRL_PHASE_t;

typedef struct NIC_PACKET_PAYLOAD_struct {
	NIC_ADDR_PHASE_t addr;
	NIC_CTRL_PHASE_t ctrl;
	uint8_t data[128];
} NIC_PACKET_PAYLOAD_t;

typedef enum NIC_PACKET_TYPE_enum {
	NIC_PACKET_TYPE_IDLE			= 0x00,
	NIC_PACKET_TYPE_READ_VERSION	= 0x01,
	NIC_PACKET_TYPE_READ_NICSTAT	= 0x02,
	NIC_PACKET_TYPE_READ_SOCKSTAT	= 0x03,
	NIC_PACKET_TYPE_CMD_OPEN		= 0x10,
	NIC_PACKET_TYPE_CMD_CLOSE		= 0x11,
	NIC_PACKET_TYPE_CMD				= 0x1D,
	NIC_PACKET_TYPE_MODIFY_MODE		= 0x1F,
	NIC_PACKET_TYPE_MODIFY_GATEWAY	= 0x20,
	NIC_PACKET_TYPE_MODIFY_SUBMASK	= 0x21,
	NIC_PACKET_TYPE_MODIFY_SMAC		= 0x22,
	NIC_PACKET_TYPE_MODIFY_SIP		= 0x23,
	NIC_PACKET_TYPE_READ_GATEWAY	= 0x30,
	NIC_PACKET_TYPE_READ_SUBMASK	= 0x31,
	NIC_PACKET_TYPE_READ_SMAC		= 0x32,
	NIC_PACKET_TYPE_READ_SIP		= 0x33,
	NIC_PACKET_TYPE_MODIFY_SPORT	= 0x40,
	NIC_PACKET_TYPE_MODIFY_DIP		= 0x41,
	NIC_PACKET_TYPE_MODIFY_DPORT	= 0x42,
	NIC_PACKET_TYPE_READ_RXPTR		= 0x50,
	NIC_PACKET_TYPE_READ_TXPTR		= 0x51,
	NIC_PACKET_TYPE_MODIFY_RXPTR	= 0x52,
	NIC_PACKET_TYPE_MODIFY_TXPTR	= 0x53,
	NIC_PACKET_TYPE_MODIFY_TXBUFF	= 0x54,
	NIC_PACKET_TYPE_MODIFY_RXBUFF	= 0x55
} NIC_PACKET_TYPE_t;

typedef struct NIC_PACKET_META_struct {
	uint8_t serialNum;
	NIC_PACKET_TYPE_t type;
	uint16_t packet_length;
	uint16_t dp_length; // data phase length
} NIC_PACKET_META_t;

typedef struct NIC_PACKET_struct {
	NIC_PACKET_META_t meta;
	NIC_PACKET_PAYLOAD_t payload;
} NIC_PACKET_t;

typedef union NIC_COMMON_MODE_enum {
	struct {
		uint8_t RST		:1;
		uint8_t rsvd0	:1;
		uint8_t WOL		:1;
		uint8_t PB		:1;
		uint8_t PPPoE	:1;
		uint8_t rsvd1	:1;
		uint8_t FARP	:1;
		uint8_t rsvd2	:1;
	} bits;
	uint8_t byte;
} NIC_COMMON_MODE_t;

#define OFFSET_NIC_GATEWAY	0x0001
typedef union NIC_GATEWAY_union {
	struct {
		uint8_t GAR0;
		uint8_t GAR1;
		uint8_t GAR2;
		uint8_t GAR3;
	} bytewise;
	uint8_t bytes[4];
} NIC_GATEWAY_t;

#define OFFSET_NIC_SUBMASK	0x0005
typedef union NIC_SUBNETMASK_union {
	struct {
		uint8_t SUBR0;
		uint8_t SUBR1;
		uint8_t SUBR2;
		uint8_t SUBR3;
	} bytewise;
	uint8_t bytes[4];
} NIC_SUBNETMASK_t;

#define OFFSET_NIC_SMAC		0x0009
typedef union NIC_SMAC_union {
	struct {
		uint8_t SHAR0;
		uint8_t SHAR1;
		uint8_t SHAR2;
		uint8_t SHAR3;
		uint8_t SHAR4;
		uint8_t SHAR5;
	} bytewise;
	uint8_t bytes[6];
} NIC_SMAC_t;

#define OFFSET_NIC_SIP		0x000F
typedef union NIC_SIP_union {
	struct {
		uint8_t SIPR0;
		uint8_t SIPR1;
		uint8_t SIPR2;
		uint8_t SIPR3;
	} bytewise;
	uint8_t bytes[4];
} NIC_SIP_t;

typedef union NIC_INTLEVEL_union {
	struct {
		uint8_t INTLEVEL0;
		uint8_t INTLEVEL1;
	} bytewise;
	uint8_t bytes[2];
} NIC_INTLEVEL_t;

typedef union NIC_RTR_union {
	struct {
		uint8_t RTR0;
		uint8_t RTR1;
	} bytewise;
	uint8_t bytes[2];
} NIC_RTR_t;

typedef union NIC_PPPDMAC_union {
	struct {
		uint8_t PHAR0;
		uint8_t PHAR1;
		uint8_t PHAR2;
		uint8_t PHAR3;
		uint8_t PHAR4;
		uint8_t PHAR5;
	} bytewise;
	uint8_t bytes[6];
} NIC_PPPDMAC_t;

typedef union NIC_PSID_union {
	struct {
		uint8_t PSID0;
		uint8_t PSID1;
	} bytewise;
	uint8_t bytes[2];
} NIC_PSID_t;

typedef union NIC_PMRU_union {
	struct {
		uint8_t PMRU0;
		uint8_t PMRU1;
	} bytewise;
	uint8_t bytes[2];
	uint16_t word;
} NIC_PMRU_t;

typedef union NIC_UIP_union {
	struct {
		uint8_t UIPR0;
		uint8_t UIPR1;
		uint8_t UIPR2;
		uint8_t UIPR3;
	} bytewise;
	uint8_t bytes[4];
} NIC_UIP_t;

typedef union NIC_UPORT_union {
	struct {
		uint8_t UPORTR0;
		uint8_t UPORTR1;
	} bytewise;
	uint8_t bytes[2];
	uint16_t word;
} NIC_UPORT_t;

typedef struct NIC_COMMON_REGISTER_struct {
	NIC_COMMON_MODE_t	MR;
	NIC_GATEWAY_t		GAR;
	NIC_SUBNETMASK_t	SUBR;
	NIC_SMAC_t			SHAR;
	NIC_SIP_t			SIPR;
	NIC_INTLEVEL_t		INTLEVEL;
	uint8_t				IR;
	uint8_t				IMR;
	uint8_t				SIR;
	uint8_t				SIMR;
	NIC_RTR_t			RTR;
	uint8_t				RCR;
	uint8_t				PTIMER;
	uint8_t				PMAGIC;
	NIC_PPPDMAC_t		PHAR;
	NIC_PSID_t			PSID;
	NIC_PMRU_t			PMRU;
	NIC_UIP_t			UIPR;
	NIC_UPORT_t			UPORTR;
	uint8_t				PHYCFGR;
	uint8_t				rsvd[0x0038 - 0x002F + 1];
	uint8_t				VERSIONR;
} NIC_COMMON_REGISTER_t;

/************************************************************************/
/* NIC Socket Data   Structure Section                                  */
/************************************************************************/
typedef union NIC_SOCKET_MR_union {
	struct {		
		uint8_t Protocol	:4;
		uint8_t UCASTB		:1;
		uint8_t ND			:1;
		uint8_t BCASTB		:1;
		uint8_t MULTI		:1;
	} bits;
	uint8_t byte;
} NIC_SOCKET_MR_t;

typedef union NIC_SPORT_union {
	struct {
		uint8_t SPORTR0;
		uint8_t SPORTR1;
	} bytewise;
	uint8_t bytes[2];
	uint16_t word;
} NIC_SPORT_t;

typedef union NIC_DIP_union {
	struct {
		uint8_t DIPR0;
		uint8_t DIPR1;
		uint8_t DIPR2;
		uint8_t DIPR3;
	} bytewise;
	uint8_t bytes[4];
} NIC_DIP_t;

typedef union NIC_DPORT_union {
	struct {
		uint8_t DPORTR0;
		uint8_t DPORTR1;
	} bytewise;
	uint8_t bytes[2];
	uint16_t word;
} NIC_DPORT_t;

typedef enum NIC_SOCKET_CR_enum {
	NIC_SOCKET_CMD_OPEN			= 0x01,
	NIC_SOCKET_CMD_LISTEN		= 0x02,
	NIC_SOCKET_CMD_CONNECT		= 0x04,
	NIC_SOCKET_CMD_DISCON		= 0x08,
	NIC_SOCKET_CMD_CLOSE		= 0x10,
	NIC_SOCKET_CMD_SEND			= 0x20,
	NIC_SOCKET_CMD_SEND_MAC		= 0x21,
	NIC_SOCKET_CMD_SEND_KEEP	= 0x22,
	NIC_SOCKET_CMD_RECV			= 0x40
} NIC_SOCKET_CR_t;

#define NIC_SOCKET_COUNT	8
#define NIC_SIZE_TXBUFFER	128
#define NIC_SIZE_RXBUFFER	128
#define NIC_SIZE_TXPACKBUFF	16
#define NIC_SIZE_RXPACKBUFF	16

typedef struct NICSockData_struct {
	bool isRxPtrRdy;
	bool isTxPtrRdy;
	uint8_t rxrd[2];
	uint8_t rxwr[2];
	uint8_t txrd[2];
	uint8_t txwr[2];
} NICSockData_t;

typedef struct NICData_struct {
	volatile NIC_SOCKET_STATEx_t sock_status[NIC_SOCKET_COUNT];
	volatile NICSockData_t sock_data[NIC_SOCKET_COUNT];
	volatile uint8_t serialCounter;
	volatile bool isRxC;
	volatile uint8_t rxBuff[NIC_SIZE_RXBUFFER];
	volatile uint16_t rxPtr;
	volatile uint16_t rxLen;
	volatile bool isTxC;
	volatile uint8_t txBuff[NIC_SIZE_TXBUFFER];
	volatile uint16_t txPtr;
	volatile uint16_t txLen;
	volatile NIC_STATUS_t NIC_STATUS;
	volatile NIC_PACKET_t txPacketBuff[NIC_SIZE_TXPACKBUFF];
	volatile NIC_PACKET_t rxPacketBuff[NIC_SIZE_RXPACKBUFF];
	volatile NIC_PACKET_t* curRxPacket; 
	volatile NIC_PACKET_t* curTxPacket;
} NICData_t;

void nic_var_init(void);
void nic_sock_init(void);
void nic_init(void);
void Open(uint8_t sock_num);
void Close(uint8_t sock_num);
void Connect(uint8_t sock_num);
uint16_t Send(uint8_t sock_num, uint8_t* buffer, uint16_t length); // @ Deprecated
uint16_t SendEx(uint8_t sock_num, NIC_PACKET_t packet);
uint16_t Receive(uint8_t sock_num, uint8_t* buffer, uint16_t length);
bool isNICIdle() __ATTR_PURE__;
volatile NIC_PACKET_t* FindIdleTxPacket();
volatile NIC_PACKET_t* FindIdleRxPacket();
volatile NIC_PACKET_t* InsertTxPacket(volatile NIC_PACKET_t *packet);
volatile NIC_PACKET_t* InsertRxPacket(volatile NIC_PACKET_t *packet);

void CheckNICStatus();
void CheckSocketStatus(uint8_t sock_num);
void NICSchedule();
void NICTxDispatch();
void NICRxDispatch();
void NICRxDigestPacket(volatile NIC_PACKET_t* packet);
void UpdateSockStat(uint8_t sock_num, NIC_SOCKET_SR_t sr);
void ReadSockPtr(uint8_t sock_num); 

volatile NIC_PACKET_t* InitializePacket(volatile NIC_PACKET_t* packet);

NIC_PACKET_t RenderReadChipVersionPacket() __ATTR_PURE__;

NIC_PACKET_t RenderUpdateIpAddrPacket(NIC_SIP_t ipAddr) __ATTR_PURE__;
NIC_PACKET_t RenderUpdateMACPacket(NIC_SMAC_t macAddr) __ATTR_PURE__;
NIC_PACKET_t RenderUpdateSubnetMaskPacket(NIC_SUBNETMASK_t subnetMask) __ATTR_PURE__;
NIC_PACKET_t RenderUpdateGatewayPacket(NIC_GATEWAY_t gateway) __ATTR_PURE__;

NIC_PACKET_t RenderReadIpAddrPacket() __ATTR_PURE__;
NIC_PACKET_t RenderReadMACPacket() __ATTR_PURE__;
NIC_PACKET_t RenderReadSubnetMaskPacket() __ATTR_PURE__;
NIC_PACKET_t RenderReadGatewayPacket() __ATTR_PURE__;

NIC_PACKET_t RenderUpdateSocketModePacket(uint8_t sock_num, NIC_SOCKET_MR_t sock_mode) __ATTR_PURE__;
NIC_PACKET_t RenderUpdateSocketSPortPacket(uint8_t sock_num, NIC_SPORT_t sock_sport) __ATTR_PURE__;
NIC_PACKET_t RenderUpdateSocketDIPPacket(uint8_t sock_num, NIC_DIP_t sock_dip) __ATTR_PURE__;
NIC_PACKET_t RenderUpdateSocketDPortPacket(uint8_t sock_num, NIC_DPORT_t sock_dport) __ATTR_PURE__;
NIC_PACKET_t RenderSocketCmdPacket(uint8_t sock_num, NIC_SOCKET_CR_t cmd) __ATTR_PURE__;
NIC_PACKET_t RenderCloseSocketPacket(uint8_t sock_num) __ATTR_PURE__;

NIC_PACKET_t RenderReadRxPtrPacket(uint8_t sock_num) __ATTR_PURE__;
NIC_PACKET_t RenderReadTxPtrPacket(uint8_t sock_num) __ATTR_PURE__;
NIC_PACKET_t RenderUpdRxPtrPacket(uint8_t sock_num, uint8_t rxrd[2]) __ATTR_PURE__;
NIC_PACKET_t RenderUpdTxPtrPacket(uint8_t sock_num, uint8_t txwr[2]) __ATTR_PURE__;

#endif /* NIC_H_ */