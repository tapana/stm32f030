#include "nrf24.h"
#include "diag/Trace.h"
#include "Timer.h"

//************ private parameter **************************
#define MAX_PAYLOAD_SIZE 32

uint8_t p_variant = 0;
uint8_t wide_band = 0;
uint8_t payload_size = 0;
uint64_t pipe0_reading_address = 0;
uint8_t dynamic_payloads_enabled = 0;
uint8_t ack_payload_available = 0;
uint8_t ack_payload_length = 0;

const uint8_t child_pipe[] = { RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3,
RX_ADDR_P4, RX_ADDR_P5 };
const uint8_t child_payload_size[] = { RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3,
RX_PW_P4, RX_PW_P5 };
const uint8_t child_pipe_enable[] = { ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4,
ERX_P5 };

char rf24_datarate_str[3][10] = { "1M", "2M", "250K" };
char rf24_model_str[2][10] = { "nRF24L01", "nRF24L01+" };
char rf24_crclength_str[3][10] = { "Disable", "8 bits", "16 bits" };
char rf24_pa_dbm_str[4][10] = { "PA_MIN", "PA_LOW", "PA_MED", "PA_HIGH" };

//************ private function **************************
#define _BV(bit) (1 << (bit))
#define CS_EN() GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define CS_DIS() GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define CE_DIS() GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define CE_EN() GPIO_SetBits(GPIOA, GPIO_Pin_11)

uint8_t spiReceiveByte();
uint8_t spiSendByte(uint8_t byte);
void io_init();

uint8_t read_reg(uint8_t reg, uint8_t *buffer, uint8_t len);
uint8_t write_reg(uint8_t reg, uint8_t *buffer, uint8_t len);
uint8_t write_1_reg(uint8_t address, uint8_t byte);
uint8_t read_1_reg(uint8_t address);

void delayMicroseconds(uint16_t delay) {
	timer_sleep(1);
	delay++;
}

//***********public function implement *******************

void nrf24_init() {
	//initialize spi and gpio
	io_init();

	CS_DIS();
	CE_DIS();

	nrf24_setRetries(15, 15);

	// Restore our default PA level
	nrf24_setPALevel(RF24_PA_MAX);

	// Determine if this is a p or non-p RF24 module and then
	// reset our data rate back to default value. This works
	// because a non-P variant won't allow the data rate to
	// be set to 250Kbps.
	if (nrf24_setDataRate(RF24_250KBPS)) {
		p_variant = 1;
	}

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
	nrf24_setDataRate(RF24_1MBPS);

	// Initialize CRC and request 2-byte (16bit) CRC
	nrf24_setCRCLength(RF24_CRC_16);

	// Disable dynamic payloads, to match dynamic_payloads_enabled setting
	write_1_reg(DYNPD, 0);

	// Reset current status
	// Notice reset and flush is the last thing we do
	write_1_reg(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	nrf24_setChannel(76);

	// Flush buffers
	nrf24_flush_rx();
	nrf24_flush_tx();
//*/
}

void nrf24_setRetries(uint8_t delay, uint8_t count) {
	write_1_reg(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

void nrf24_setPALevel(rf24_pa_dbm_e level) {
	uint8_t setup = read_1_reg(RF_SETUP);
	setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

	// switch uses RAM (evil!)
	if (level == RF24_PA_MAX) {
		setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
	} else if (level == RF24_PA_HIGH) {
		setup |= _BV(RF_PWR_HIGH);
	} else if (level == RF24_PA_LOW) {
		setup |= _BV(RF_PWR_LOW);
	} else if (level == RF24_PA_MIN) {
		// nothing
	} else if (level == RF24_PA_ERROR) {
		// On error, go to maximum PA
		setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
	}

	write_1_reg( RF_SETUP, setup);
}

rf24_pa_dbm_e nrf24_getPALevel(void) {
	rf24_pa_dbm_e result = RF24_PA_ERROR;
	uint8_t power = read_1_reg(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

	// switch uses RAM (evil!)
	if (power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) {
		result = RF24_PA_MAX;
	} else if (power == _BV(RF_PWR_HIGH)) {
		result = RF24_PA_HIGH;
	} else if (power == _BV(RF_PWR_LOW)) {
		result = RF24_PA_LOW;
	} else {
		result = RF24_PA_MIN;
	}

	return result;
}

uint8_t nrf24_setDataRate(rf24_datarate_e speed) {
	uint8_t result = 0;
	uint8_t setup = read_1_reg(RF_SETUP);

	// HIGH and LOW '00' is 1Mbs - our default
	wide_band = 0;

	setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
	if (speed == RF24_250KBPS) {
		// Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
		// Making it '10'.
		wide_band = 0;
		setup |= _BV(RF_DR_LOW);
	} else {
		// Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
		// Making it '01'
		if (speed == RF24_2MBPS) {
			wide_band = 1;
			setup |= _BV(RF_DR_HIGH);
		} else {
			// 1Mbs
			wide_band = 0;
		}
	}
	write_1_reg(RF_SETUP, setup);

	// Verify our result
	if (read_1_reg(RF_SETUP) == setup) {
		result = 1;
	} else {
		wide_band = 0;
	}

	return result;
}

void nrf24_setCRCLength(rf24_crclength_e length) {
	uint8_t config = read_1_reg((CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)));

	// switch uses RAM (evil!)
	if (length == RF24_CRC_DISABLED) {
		// Do nothing, we turned it off above.
	} else if (length == RF24_CRC_8) {
		config |= _BV(EN_CRC);
	} else {
		config |= _BV(EN_CRC);
		config |= _BV(CRCO);
	}
	write_1_reg( CONFIG, config);
}

rf24_crclength_e nrf24_getCRCLength(void) {
	rf24_crclength_e result = RF24_CRC_DISABLED;
	uint8_t config = read_1_reg(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC));

	if (config & _BV(EN_CRC)) {
		if (config & _BV(CRCO))
			result = RF24_CRC_16;
		else
			result = RF24_CRC_8;
	}
	return result;
}

void nrf24_setChannel(uint8_t channel) {
	// TODO: This method could take advantage of the 'wide_band' calculation
	// done in setChannel() to require certain channel spacing.

	if (channel > 127)
		channel = 127;

	write_1_reg(RF_CH, channel);
}

//**************** direction command function ***************

uint8_t nrf24_flush_rx(void) {
	uint8_t status;
	CS_EN();
	status = spiSendByte(FLUSH_RX);
	CS_DIS();
	return status;
}

uint8_t nrf24_flush_tx(void) {
	uint8_t status;
	CS_EN();
	status = spiSendByte(FLUSH_TX);
	CS_DIS();
	return status;
}

uint8_t nrf24_get_status(void) {
	uint8_t status;
	CS_EN();
	status = spiSendByte(NOP);
	CS_DIS();
	return status;
}

void nrf24_setPayloadSize(uint8_t size) {
	if (size > MAX_PAYLOAD_SIZE)
		size = MAX_PAYLOAD_SIZE;
	payload_size = size;
}

uint8_t nrf24_getPayloadSize(void) {
	return payload_size;
}

rf24_datarate_e nrf24_getDataRate(void) {
	rf24_datarate_e result;
	uint8_t dr = read_1_reg(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

	// switch uses RAM (evil!)
	// Order matters in our case below
	if (dr == _BV(RF_DR_LOW)) {
		// '10' = 250KBPS
		result = RF24_250KBPS;
	} else if (dr == _BV(RF_DR_HIGH)) {
		// '01' = 2MBPS
		result = RF24_2MBPS;
	} else {
		// '00' = 1MBPS
		result = RF24_1MBPS;
	}
	return result;
}

void nrf24_openReadingPipe(uint8_t child, uint64_t address) {
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if (child == 0)
		pipe0_reading_address = address;

	if (child <= 6) {
		// For pipes 2-5, only write the LSB

		if (child < 2) {
			//TODO: reverse byte send LSB should be first to send

			uint8_t *addr_byte = (uint8_t*) &address;
			write_reg(child_pipe[child], addr_byte, 5);

		} else {
			uint8_t addr = address & 0xff;
			write_1_reg(child_pipe[child], addr);
		}

		write_1_reg(child_payload_size[child], payload_size);

		//write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

		// Note it would be more efficient to set all of the bits for all open
		// pipes at once.  However, I thought it would make the calling code
		// more simple to do it this way.
		//write_register(EN_RXADDR,read_register(EN_RXADDR)| _BV(pgm_read_byte(&child_pipe_enable[child])));

		write_1_reg( EN_RXADDR,
				read_1_reg( EN_RXADDR) | _BV(child_pipe_enable[child]));

	}
}

void nrf24_openWritingPipe(uint64_t address) {
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.

	//TODO: reverse byte send LSB should be first to send

	uint8_t *addr_byte = (uint8_t*) &address;

	write_reg(RX_ADDR_P0, addr_byte, 5);
	write_reg(TX_ADDR, addr_byte, 5);

	write_1_reg( RX_PW_P0, payload_size);
}

void nrf24_startListening(void) {

	write_1_reg(CONFIG, read_1_reg(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
	write_1_reg(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	//TODO: reverse byte send LSB should be first to send
	if (pipe0_reading_address) {
		uint8_t *addr_byte = (uint8_t*) &pipe0_reading_address;
		write_reg(RX_ADDR_P0, addr_byte, 5);
	}

	// Flush buffers
	nrf24_flush_rx();
	nrf24_flush_tx();

	// Go!
	CE_EN();

	// wait for the radio to come up (130us actually only needed)
	delayMicroseconds(130);
}

void nrf24_stopListening(void) {
	CE_DIS();
	nrf24_flush_rx();
	nrf24_flush_tx();
}

void nrf24_powerDown(void) {
	write_1_reg( CONFIG, read_1_reg(CONFIG) & ~_BV(PWR_UP));
}

void nrf24_powerUp(void) {
	write_1_reg( CONFIG, read_1_reg(CONFIG) | _BV(PWR_UP));
}

uint8_t nrf24_write_payload(uint8_t* buf, uint8_t len) {
	uint8_t status;

	const uint8_t* current = buf;	//reinterpret_cast<const uint8_t*>(buf);

	if (len > payload_size)
		len = payload_size;
	uint8_t data_len = len;
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

	CS_EN();

	status = spiSendByte(W_TX_PAYLOAD);

	while (data_len--)
		spiSendByte(*current++);
	while (blank_len--)
		spiSendByte(0);

	CS_DIS();

	return status;
}

uint8_t nrf24_read_payload(uint8_t* buf, uint8_t len) {
	uint8_t status;
	uint8_t* current = (buf);

	if (len > payload_size)
		len = payload_size;
	uint8_t data_len = len;
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

	CS_EN();

	status = spiSendByte( R_RX_PAYLOAD);
	while (data_len--)
		*current++ = spiSendByte(0xff);
	while (blank_len--)
		spiSendByte(0xff);

	CS_DIS();
	return status;
}

void nrf24_startWrite(uint8_t* buf, uint8_t len) {
	// Transmitter power-up
	write_1_reg( CONFIG, (read_1_reg(CONFIG) | _BV(PWR_UP)) & ~_BV(PRIM_RX));

	delayMicroseconds(150);

	// Send the payload
	nrf24_write_payload(buf, len);

	// Allons!
	CE_EN();
	delayMicroseconds(15);
	CE_DIS();
}

uint8_t nrf24_getDynamicPayloadSize(void) {
	uint8_t result = 0;

	CS_EN();
	spiSendByte( R_RX_PL_WID);
	result = spiSendByte(0xff);
	CS_DIS();

	return result;
}

uint8_t nrf24_write(uint8_t* buf, uint8_t len) {

	uint8_t result = 0;

	// Begin the write
	nrf24_startWrite(buf, len);

	// ------------
	// At this point we could return from a non-blocking write, and then call
	// the rest after an interrupt

	// Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
	// or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
	// is flaky and we get neither.

	// IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
	// if I tighted up the retry logic.  (Default settings will be 1500us.
	// Monitor the send
	uint8_t observe_tx;
	uint8_t status;
	uint32_t sent_at = system_ticks;
	const uint32_t timeout = 500; //ms to wait for timeout
	do {
		status = read_reg(OBSERVE_TX, &observe_tx, 1);
	} while (!(status & ( _BV(TX_DS) | _BV(MAX_RT)))
			&& (system_ticks - sent_at < timeout));

	// The part above is what you could recreate with your own interrupt handler,
	// and then call this when you got an interrupt
	// ------------

	// Call this when you get an interrupt
	// The status tells us three things
	// * The send was successful (TX_DS)
	// * The send failed, too many retries (MAX_RT)
	// * There is an ack packet waiting (RX_DR)

	status = write_1_reg( STATUS , _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	// Report to the user what happened
	uint8_t tx_ok = status & _BV(TX_DS);
	//uint8_t tx_fail = status & _BV(MAX_RT);
	ack_payload_available = status & _BV(RX_DR);

	result = tx_ok;

	// Handle the ack packet
	if (ack_payload_available) {
		ack_payload_length = nrf24_getDynamicPayloadSize();
	}

	// Yay, we are done.

	// Power down
	nrf24_powerDown();

	// Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
	nrf24_flush_tx();

	return result;
}



uint8_t nrf24_available(uint8_t* pipe_num) {
	uint8_t status = nrf24_get_status();

// Too noisy, enable if you really want lots o data!!
//IF_SERIAL_DEBUG(print_status(status));

	uint8_t result = (status & _BV(RX_DR));

	if (result) {
		// If the caller wants the pipe number, include that
		if (pipe_num)
			*pipe_num = (status >> RX_P_NO) & 0b111;

		// Clear the status bit

		// ??? Should this REALLY be cleared now?  Or wait until we
		// actually READ the payload?

		write_1_reg(STATUS, _BV(RX_DR));

		// Handle ack payload receipt
		if (status & _BV(TX_DS)) {
			write_1_reg(STATUS, _BV(TX_DS));
		}
	}

	return result;
}

uint8_t nrf24_read(uint8_t* buf, uint8_t len) {
// Fetch the payload
	nrf24_read_payload(buf, len);
//read_payload(buf, len);

// was this the last of the data available?
	return read_1_reg(FIFO_STATUS) & _BV(RX_EMPTY);

}



//***************** debug print function ************************

void print_status(uint8_t status) {
	trace_printf(
			"STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
			status, (status & _BV(RX_DR)) ? 1 : 0,
			(status & _BV(TX_DS)) ? 1 : 0, (status & _BV(MAX_RT)) ? 1 : 0,
			((status >> RX_P_NO) & 0b111), (status & _BV(TX_FULL)) ? 1 : 0);
}

void print_byte_register(char* name, uint8_t reg, uint8_t n) {
	trace_printf(name);
	while (n--) {
		trace_printf(" 0x%02x", read_1_reg(reg));
		reg++;
	}
	trace_printf("\n");
}

void print_address_register(char* name, uint8_t reg, uint8_t n) {
	trace_printf(name);

	while (n--) {
		uint8_t buffer[5];
		read_reg(reg++, buffer, 5);
		int i;
		for (i = 0; i < 5; i++)
			trace_printf(" %02x", buffer[i]);
		trace_printf("\t");
	}
	trace_printf("\n");

}

void nrf24_printDetails(void) {
	print_status(nrf24_get_status());
	print_address_register("RX_ADDR_P0-1", RX_ADDR_P0, 2);
	print_byte_register("RX_ADDR_P2-5", RX_ADDR_P2, 4);
	print_address_register("TX_ADDR ", TX_ADDR, 1);

	print_byte_register("RX_PW_P0-6", RX_PW_P0, 6);
	print_byte_register("EN_AA", EN_AA, 1);
	print_byte_register("EN_RXADDR", EN_RXADDR, 1);
	print_byte_register("RF_CH", RF_CH, 1);
	print_byte_register("RF_SETUP", RF_SETUP, 1);
	print_byte_register("CONFIG", CONFIG, 1);
	print_byte_register("DYNPD/FEATURE", DYNPD, 2);

	trace_printf("Data Rate\t = %s\n", rf24_datarate_str[nrf24_getDataRate()]);
	trace_printf("Model\t\t = %s\n", rf24_model_str[p_variant]);
	trace_printf("CRC Length\t = %s\n",
			rf24_crclength_str[nrf24_getCRCLength()]);
	trace_printf("PA Power\t = %s\n", rf24_pa_dbm_str[nrf24_getPALevel()]);

}

//***************** register read-write function ***********************

// Read len bytes from a nRF24L register. 5 Bytes max
uint8_t read_reg(uint8_t reg, uint8_t *buffer, uint8_t len) {
	uint8_t status;
	int i;
	CS_EN();
//send first byte as register address
	status = spiSendByte( R_REGISTER | ( REGISTER_MASK & reg));

	for (i = 0; i < len; i++)
		buffer[i] = spiReceiveByte();

	CS_DIS();
	return status;
}

// Write len bytes a nRF24L register. 5 Bytes max
uint8_t write_reg(uint8_t reg, uint8_t *buffer, uint8_t len) {
	uint8_t status;
	int i;
	CS_EN();
	status = spiSendByte( W_REGISTER | ( REGISTER_MASK & reg));

	for (i = 0; i < len; i++)
		spiSendByte(buffer[i]);

	CS_DIS();
	return status;
}

/* Write only one byte (useful for most of the reg.) */
uint8_t write_1_reg(uint8_t address, uint8_t byte) {
	return write_reg(address, &byte, 1);
}

/* Read only one byte (useful for most of the reg.) */
uint8_t read_1_reg(uint8_t address) {
	uint8_t byte;
	read_reg(address, &byte, 1);
	return byte;
}

//****************** low level function *****************************
void io_init() {

	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

//setup spi pin sck, miso,mosi PA 5,6,7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);

	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

// Set interrupt on 8-bit return
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

// Enable the SPI
	SPI_Cmd(SPI1, ENABLE);

// CS & CE pin PA 4,11
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

uint8_t spiSendByte(uint8_t byte) {
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
		;

	/* Send byte through the SPI1 peripheral */
	SPI_SendData8(SPI1, byte);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
		;

	/* Return the byte read from the SPI bus */
	return SPI_ReceiveData8(SPI1);
}

uint8_t spiReceiveByte() {
	return spiSendByte(0xff);
}
