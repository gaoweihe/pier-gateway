/*
 * main.c
 *
 * Created: 5/20/2021 8:25:03 AM
 *  Author: Gao, Weihe
 */ 

#ifdef F_CPU
#undef F_CPU
#endif /* F_CPU */
#define F_CPU	(4000000UL) // default clock 4MHz

#include <xc.h>
#include "prototype.h"
#include "dependency.h"
#include "nic.h"
#include "collector.h"

void vars_init(void);
void eepromvars_init(void);
void usart_init(void);
void rtc_init(void);
void pwm_init(void);
void adc_init(void);
void clock_init(void);
void spi_init(void);

volatile extern NICData_t NICData;
volatile RuntimeFlags_t RuntimeFlags;
volatile EEMEM RuntimeConfig_t RuntimeConfig_eemem = {
	{ "1234567890123456" }, // guid
	0x01, // seg id
	0x0F, // dev id
	{{ 192, 168, 3, 127 }}, // sip
	{{ 192, 168, 3, 109 }}, // dip
	{{ 0x00, 0x08, 0xDC, 0x01, 0x02, 0x03 }}, // smac
	{{ 0x1A, 0x0D }}, // dport
	{{ 192, 168, 3, 100 }}, // gateway
	{{ 255, 255, 252, 0 }}, // submask	
	{{ 0x00, 0x00 }} // sport
};
volatile RuntimeConfig_t RuntimeConfig;
volatile extern CollectorData_t CollectorData[COLLECTOR_COUNT];

int main(void)
{
	di();
	vars_init();
    clock_init();
	spi_init();
	rtc_init();
	pwm_init();
	nic_init();
	usart_init();
	ei();
	while(1)
    {
        //TODO:: Please write your application code 
		if (RuntimeFlags.isNICInit) {
			NICTxDispatch();
			NICRxDispatch();
		}
		if (RuntimeFlags.isChkNICStatusReq) {
			NICSchedule();
			CheckNICStatus();
			RuntimeFlags.isChkNICStatusReq = false;
		}
		if (NICData.sock_status[1] == NIC_SOCKET_STATUS_ESTABLISHED) {
			if (NICData.sock_data[1].isRxPtrRdy == false || 
				NICData.sock_data[1].isTxPtrRdy == false) {
				ReadSockPtr(1);
			}	
		}
		
		CollectorTxDispatch();
		CollectorRxDispatch();
    }
}

void vars_init(void) {
	NICData.serialCounter = 0;
	eepromvars_init();
}

void eepromvars_init(void) {
	eeprom_busy_wait();
	for (uint16_t i = 0; i < sizeof(RuntimeConfig_t); i++)
	{
		*((uint8_t *)&RuntimeConfig + i) = eeprom_read_byte((uint8_t *)(i)); // read byte-wise
		eeprom_busy_wait();
	}
	eeprom_busy_wait();
}

void usart_init(void) {
	// USART0
	
	// USART1
	USART1_TX_DISABLE();
	USART1_RX_ENABLE();
	PORTC.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm; // txd, txen, rxen
	PORTC.DIRCLR = PIN1_bm; // rxd
	USART1.BAUD = (uint16_t)(USART_BAUD_RATE(19200UL));
	USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc // asynchronous mode
				 | USART_CHSIZE_8BIT_gc // 8-bit
				 | USART_PMODE_DISABLED_gc // parity disabled
				 | USART_SBMODE_1BIT_gc; // one stop bit
	USART1.CTRLB |= USART_RXEN_bm; // rxen
	USART1.CTRLB |= USART_TXEN_bm; // txen
	USART1.CTRLA = USART_RXCIE_bm
				 | USART_TXCIE_bm; // enable interrupts
	USART1.DBGCTRL |= USART_DBGRUN_bm; // continue on debug break
	
	// USART3
	USART3_TX_DISABLE();
	USART3_RX_ENABLE();
	PORTB.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm; // txd, txen, rxen
	PORTB.DIRCLR = PIN1_bm; // rxd
	USART3.BAUD = (uint16_t)(USART_BAUD_RATE(19200UL));
	USART3.CTRLC = USART_CMODE_ASYNCHRONOUS_gc // asynchronous mode
				 | USART_CHSIZE_8BIT_gc // 8-bit
				 | USART_PMODE_DISABLED_gc // parity disabled
				 | USART_SBMODE_1BIT_gc; // one stop bit
	USART3.CTRLB = USART_TXEN_bm // txen
				 | USART_RXEN_bm; // rxen
	USART3.CTRLA = USART_RXCIE_bm
				 | USART_TXCIE_bm; // enable interrupts
	USART3.DBGCTRL |= USART_DBGRUN_bm; // continue on debug break
	
	// USART4
	USART4_TX_DISABLE();
	USART4_RX_ENABLE();
	PORTE.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm; // txd, txen, rxen
	PORTE.DIRCLR = PIN1_bm; // rxd
	USART4.BAUD = (uint16_t)(USART_BAUD_RATE(19200UL));
	USART4.CTRLC = USART_CMODE_ASYNCHRONOUS_gc // asynchronous mode
				 | USART_CHSIZE_8BIT_gc // 8-bit
				 | USART_PMODE_DISABLED_gc // parity disabled
				 | USART_SBMODE_1BIT_gc; // one stop bit
	USART4.CTRLB = USART_TXEN_bm // txen
				 | USART_RXEN_bm; // rxen
	USART4.CTRLA = USART_RXCIE_bm
				 | USART_TXCIE_bm; // enable interrupts
	USART4.DBGCTRL |= USART_DBGRUN_bm; // continue on debug break
	
	// USART5
	USART5_TX_DISABLE();
	USART5_RX_ENABLE();
	PORTG.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm; // txd, txen, rxen
	PORTG.DIRCLR = PIN1_bm; // rxd
	USART5.BAUD = (uint16_t)(USART_BAUD_RATE(19200UL));
	USART5.CTRLC = USART_CMODE_ASYNCHRONOUS_gc // asynchronous mode
				 | USART_CHSIZE_8BIT_gc // 8-bit
				 | USART_PMODE_DISABLED_gc // parity disabled
				 | USART_SBMODE_1BIT_gc; // one stop bit
	USART5.CTRLB = USART_TXEN_bm // txen
				 | USART_RXEN_bm; // rxen
	USART5.CTRLA = USART_RXCIE_bm
				 | USART_TXCIE_bm; // enable interrupts
	USART5.DBGCTRL |= USART_DBGRUN_bm; // continue on debug break
}

void clock_init(void)
{
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKOUT_bm
					  | CLKCTRL_CLKSEL_OSCHF_gc;
	CLKCTRL.OSCHFCTRLA = CLKCTRL_FREQSEL_24M_gc;
}

void rtc_init(void)
{
	// PIT
	while (RTC.STATUS || RTC.PITSTATUS);
	RTC.CLKSEL = RTC_CLKSEL_OSC1K_gc; // select 1.024k clock
	while (RTC.STATUS || RTC.PITSTATUS);
	RTC.PITINTCTRL = RTC_PI_bm; // enable pit interrupt
	while (RTC.STATUS || RTC.PITSTATUS);
	RTC.PITCTRLA |= RTC_PERIOD_CYC128_gc; // 512 cycles
	while (RTC.STATUS || RTC.PITSTATUS);
	RTC.PITCTRLA |= RTC_PITEN_bm; // pit enable
	while (RTC.STATUS || RTC.PITSTATUS);
}

void pwm_init(void) {
	PORTD.OUTCLR = PIN0_bm | PIN1_bm;
	PORTD.DIRSET = PIN0_bm | PIN1_bm;
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTD_gc;
	
	// TCA0
	TCA0.SINGLE.CMP0 = 0;
	TCA0.SINGLE.CMP1 = 0;
	TCA0.SINGLE.CMP2 = 0;
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm 
					  | TCA_SINGLE_CMP1EN_bm 
					  | TCA_SINGLE_CMP2EN_bm 
					  | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
	TCA0.SINGLE.PER = 256;
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc 
					  | TCA_SINGLE_ENABLE_bm;
	TCA0.SINGLE.DBGCTRL = TCA_SINGLE_DBGRUN_bm;
}

void spi_init(void) {
	PORTA.OUTCLR = 0xFF;
	PORTA.DIRSET = PIN4_bm; // MOSI output
	PORTA.DIRCLR = PIN5_bm; // MISO input
	PORTA.DIRSET = PIN6_bm; // SCK output
	PORTA.DIRSET = PIN7_bm; // SCSn output
	
	PORTG.OUTSET = PIN6_bm;
	PORTG.DIRSET = PIN6_bm; // NIC ERst
	PORTG.OUTCLR = PIN6_bm; 
	_delay_ms(10);
	PORTG.OUTSET = PIN6_bm;
	
	PORTMUX.SPIROUTEA |= PORTMUX_SPI1_NONE_gc;
	PORTMUX.SPIROUTEA |= PORTMUX_SPI0_DEFAULT_gc;
	
	SPI0.CTRLA |= SPI_MASTER_bm; // master mode
	SPI0.CTRLA &= ~SPI_CLK2X_bm; // clock not doubled
	SPI0.CTRLA &= ~SPI_DORD_bm; // transmit MSB first
	SPI0.CTRLB &= ~SPI_BUFEN_bm; // buffer mode disable
	SPI0.CTRLB &= ~SPI_SSD_bm; // enable slave select
	SPI0.CTRLA |= SPI_ENABLE_bm; // spi enable

	SPI0_SLAVE_DESELECT();
	
	SPI0.CTRLA &= ~SPI_PRESC_DIV4_gc; // CLK_PER/4
	SPI0.CTRLB |= SPI_MODE_0_gc; // mode 0
	
	// SPI0.INTCTRL |= SPI_RXCIE_bm;
	// SPI0.INTCTRL |= SPI_TXCIE_bm;
	SPI0.INTCTRL |= SPI_IE_bm;
	
}

ISR(RTC_PIT_vect) {
	while (RTC.STATUS || RTC.PITSTATUS);
	RTC.PITINTFLAGS |= RTC_PI_bm;
	while (RTC.STATUS || RTC.PITSTATUS);
	
	RuntimeFlags.isChkNICStatusReq = true;
	// NICData.sock_data[1].isRxPtrRdy = false;
	NICData.sock_data[1].isTxPtrRdy = false;
	
	for (uint8_t i = 0; i < COLLECTOR_COUNT; i++)
	{
		CollectorData[i].isCollectReq = true;
	}
}