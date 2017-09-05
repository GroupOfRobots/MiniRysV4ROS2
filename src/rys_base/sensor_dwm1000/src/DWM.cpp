/**
 * Author:
 * MJBogusz <mjbogusz.email.address@domain.name.com>
 *
 * Based on "Single-sided two-way ranging (SS TWR) initiator example code" (example 6a)
 * by: Decawave
 * and on "Simple RX example code" (example 2a)
 * by: Decawave
 * as modified by: Anh Luong <luong@eng.utah.edu> (https://github.com/SPAN-UofU/dw1000_bbb)
 */

#include "DWM.hpp"

// For console output
#include <iostream>
// For file access (GPIO)
#include <fstream>
// For std::memcmp
#include <cstring>
// For open() and O_RDWR
#include <fcntl.h>
// For SPI_* defines
#include <linux/spi/spidev.h>
// For ioctl()
#include <sys/ioctl.h>

// Decawave api and platform wrappers
extern "C" {
	#include "deca_api/deca_device_api.h"
	#include "deca_api/deca_regs.h"
	#include "dwm_platform.h"
}

DWM::DWM(const int resetPin, const int irqPin) :
	txPollMessage{0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	rxResponseMessage{0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
{
	this->resetPin = resetPin;
	this->irqPin = irqPin;

	for (int i = 0; i < FRAME_LEN_MAX; ++i) {
		this->receiveBufferFrame[i] = 0;
	}
	for (int i = 0; i < FRAME_LEN_MAX; ++i) {
		this->receiveBufferRanging[i] = 0;
	}

	this->frameSequenceNumber = 0;
}

DWM::~DWM() {}

void DWM::initialize(bool ranging) {
	std::cout << "Initializing hardware\n";
	/* Start with board specific hardware init. */
	this->initializeHardware();

	spiMode = 0;
	spiBits = 8;
	spiSpeed = SPI_SPEED_SLOW;
	spiDelay = 0;

	std::cout << "Resetting DW1000\n";
	// Reset and initialize DW1000.
	// For initialisation, DW1000 clocks must be temporarily set to crystal speed.
	// After initialisation SPI rate can be increased for optimum performance.
	this->reset();

	std::cout << "Setting SPI rate LOW\n";
	this->spiSetRateLow();

	std::cout << "Initializing DW1000\n";
	if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
		throw(std::string("Error initializing DWT!\n"));
	}

	std::cout << "Setting SPI rate HIGH\n";
	this->spiSetRateHigh();

	/* Configure DW1000. */
	std::cout << "Configuring DW1000\n";
	dwt_config_t config;
	// Channel number
	config.chan = 2;
	// Pulse repetition frequency.
	config.prf = DWT_PRF_64M;
	// TX preamble code. Used in TX only.
	config.txCode = 9;
	// RX preamble code. Used in RX only.
	config.rxCode = 9;
	// PHY header mode.
	config.phrMode = DWT_PHRMODE_STD;
	if (ranging) {
		// Preamble length. Used in TX only.
		config.txPreambLength = DWT_PLEN_128;
		// Preamble acquisition chunk size. Used in RX only.
		config.rxPAC = DWT_PAC8;
		// 0 to use standard SFD, 1 to use non-standard SFD.
		config.nsSFD = 0;
		// Data rate.
		config.dataRate = DWT_BR_6M8;
		// SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
		config.sfdTO = (129 + 8 - 8);
	} else {
		// Preamble length. Used in TX only.
		config.txPreambLength = DWT_PLEN_1024;
		// Preamble acquisition chunk size. Used in RX only.
		config.rxPAC = DWT_PAC32;
		// 0 to use standard SFD, 1 to use non-standard SFD.
		config.nsSFD = 1;
		// Data rate.
		config.dataRate = DWT_BR_110K;
		// SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
		config.sfdTO = (1025 + 64 - 32);
	}
	dwt_configure(&config);

	// Apply default antenna delay values
	std::cout << "Configuring Antenna delay\n";
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	// Set expected response's delay and timeout.
	// As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all.
	std::cout << "Configuring timeouts\n";
	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
}

uint8_t * DWM::receiveFrame() {
	// Clear RX buffer to avoid having leftovers from previous receptions.
	for (int i = 0; i < FRAME_LEN_MAX; i++) {
		this->receiveBufferFrame[i] = 0;
	}

	// Activate immediate reception.
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

	// Poll until a frame is properly received or an error/timeout occurs.
	// STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register,
	// we can use this simplest API function to access it.
	std::cout << "Waiting for message...\n";
	uint32_t statusRegisterValue = dwt_read32bitreg(SYS_STATUS_ID);
	while (!(statusRegisterValue & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
		deca_sleep(1);
		statusRegisterValue = dwt_read32bitreg(SYS_STATUS_ID);
	}

	if (statusRegisterValue & SYS_STATUS_RXFCG) {
		// A frame has been received, copy it to our local buffer.
		uint16_t frameLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frameLength <= FRAME_LEN_MAX) {
			dwt_readrxdata(this->receiveBufferFrame, frameLength, 0);
		}

		// Clear good RX frame event in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		std::cout << "MSG Received! DATA: " << this->receiveBufferFrame << std::endl;
		// Copy received data and return it
		uint8_t * buffer = new uint8_t[FRAME_LEN_MAX];
		for (int i = 0; i < FRAME_LEN_MAX; ++i) {
			buffer[i] = this->receiveBufferFrame[i];
		}
		return buffer;
	} else {
		// Clear RX error events in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

		std::cout << "MSG Receive error!\n";
		throw(std::string("MSG Receive error"));
	}
}

double DWM::readRange() {
	// Clear RX buffer to avoid having leftovers from previous receptions.
	for (int i = 0; i < FRAME_LEN_MAX; i++) {
		this->receiveBufferRanging[i] = 0;
	}

	// Write frame data to DW1000 and prepare transmission.
	this->txPollMessage[ALL_MSG_SN_IDX] = this->frameSequenceNumber;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	// Zero offset in TX buffer.
	dwt_writetxdata(sizeof(this->txPollMessage), this->txPollMessage, 0);
	// Zero offset in TX buffer, ranging.
	dwt_writetxfctrl(sizeof(this->txPollMessage), 0, 1);

	// Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	// We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout.
	std::cout << "Waiting for message...\n";
	uint32_t statusRegisterValue = dwt_read32bitreg(SYS_STATUS_ID);
	while (!(statusRegisterValue & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
		deca_sleep(1);
		statusRegisterValue = dwt_read32bitreg(SYS_STATUS_ID);
	}

	// Increment frame sequence number after transmission of the poll message (modulo 256).
	this->frameSequenceNumber++;

	if (statusRegisterValue & SYS_STATUS_RXFCG) {
		// A frame has been received, read it into the local buffer.
		uint32_t frameLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frameLength <= RX_BUF_LEN) {
			dwt_readrxdata(this->receiveBufferRanging, frameLength, 0);
		}

		// Clear good RX frame event in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		// Check that the frame is the expected response from the companion "SS TWR responder" example.
		// As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame.
		this->receiveBufferRanging[ALL_MSG_SN_IDX] = 0;
		if (std::memcmp(this->receiveBufferRanging, this->rxResponseMessage, ALL_MSG_COMMON_LEN) != 0) {
			std::cout << "Received response for bad frame!\n";
			return 65536.0;
		}

		// Retrieve poll transmission and response reception timestamps.
		uint32_t pollTransmitTimestamp = dwt_readtxtimestamplo32();
		uint32_t responseReceiveTimestamp = dwt_readrxtimestamplo32();

		// Read carrier integrator value and calculate clock offset ratio.
		double clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6);

		// Get timestamps embedded in response message.
		uint32_t pollReceiveTimestamp = DWM::getTimestampFromMessage(&(this->receiveBufferRanging[RESP_MSG_POLL_RX_TS_IDX]));
		uint32_t responseTransmitTimestamp = DWM::getTimestampFromMessage(&(this->receiveBufferRanging[RESP_MSG_RESP_TX_TS_IDX]));

		// Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates.
		int32_t totalTime = responseReceiveTimestamp - pollTransmitTimestamp;
		int32_t responderProcessingTime = responseTransmitTimestamp - pollReceiveTimestamp;

		double tof = ((totalTime - responderProcessingTime * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
		return (tof * SPEED_OF_LIGHT);
	} else {
		// Clear RX error/timeout events in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

		// Reset RX to properly reinitialise LDE operation.
		dwt_rxreset();

		std::cout << "MSG Receive error!\n";
		throw(std::string("MSG Receive error"));
	}

	return 65536.0;
}

// ### Private methods ###

void DWM::spiSetRateLow() {
	spiSpeed = SPI_SPEED_SLOW;
	if (ioctl(spiFileDescriptor, SPI_IOC_WR_MAX_SPEED_HZ, &(spiSpeed)) == -1) {
		throw(std::string("SPI: Can't set max speed HZ"));
	}
	if (ioctl(spiFileDescriptor, SPI_IOC_RD_MAX_SPEED_HZ, &(spiSpeed)) == -1) {
		throw(std::string("SPI: Can't get max speed HZ."));
	}
}

void DWM::spiSetRateHigh() {
	spiSpeed = SPI_SPEED_FAST;
	if (ioctl(spiFileDescriptor, SPI_IOC_WR_MAX_SPEED_HZ, &(spiSpeed)) == -1) {
		throw(std::string("SPI: Can't set max speed HZ"));
	}
	if (ioctl(spiFileDescriptor, SPI_IOC_RD_MAX_SPEED_HZ, &(spiSpeed)) == -1) {
		throw(std::string("SPI: Can't get max speed HZ."));
	}
}

void DWM::initializeHardware() {
	std::ofstream file;
	std::string resetPinDirectionFilename = std::string("/sys/class/gpio/gpio") + std::to_string(this->resetPin) + std::string("/value");
	std::string irqPinDirectionFilename = std::string("/sys/class/gpio/gpio") + std::to_string(this->irqPin) + std::string("/value");
	this->resetPinFilename = std::string("/sys/class/gpio/gpio") + std::to_string(this->resetPin) + std::string("/value");
	this->irqPinFilename = std::string("/sys/class/gpio/gpio") + std::to_string(this->irqPin) + std::string("/value");

	file.open("/sys/class/gpio/export", std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: /sys/class/gpio/export"));
	}
	file << this->resetPin;
	file << this->irqPin;
	file.close();

	file.open(resetPinDirectionFilename.c_str(), std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: ") + resetPinDirectionFilename);
	}
	file << "out";
	file.close();

	file.open(irqPinDirectionFilename.c_str(), std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: ") + irqPinDirectionFilename);
	}
	file << "out";
	file.close();

	// The following calls set up the SPI bus properties
	if ((spiFileDescriptor = open(SPI_PATH, O_RDWR)) < 0) {
		throw(std::string("SPI Error: Can't open device."));
	}
	if (ioctl(spiFileDescriptor, SPI_IOC_WR_MODE, &(spiMode)) == -1) {
		throw(std::string("SPI: Can't set SPI mode."));
	}
	if (ioctl(spiFileDescriptor, SPI_IOC_RD_MODE, &(spiMode)) == -1) {
		throw(std::string("SPI: Can't get SPI mode."));
	}
	if (ioctl(spiFileDescriptor, SPI_IOC_WR_BITS_PER_WORD, &(spiBits)) == -1) {
		throw(std::string("SPI: Can't set bits per word."));
	}
	if (ioctl(spiFileDescriptor, SPI_IOC_RD_BITS_PER_WORD, &(spiBits)) == -1) {
		throw(std::string("SPI: Can't get bits per word."));
	}
	if (ioctl(spiFileDescriptor, SPI_IOC_WR_MAX_SPEED_HZ, &(spiSpeed)) == -1) {
		throw(std::string("SPI: Can't set max speed HZ"));
	}
	if (ioctl(spiFileDescriptor, SPI_IOC_RD_MAX_SPEED_HZ, &(spiSpeed)) == -1) {
		throw(std::string("SPI: Can't get max speed HZ."));
	}
}

void DWM::reset() {
	std::ofstream file;

	file.open(this->resetPinFilename.c_str(), std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: ") + this->resetPinFilename);
	}
	file << '0';
	file.close();

	deca_sleep(2);

	file.open(this->resetPinFilename.c_str(), std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: ") + this->resetPinFilename);
	}
	file << '1';
	file.close();

	deca_sleep(2);
}

uint32_t DWM::getTimestampFromMessage(uint8_t * timestampField) {
	uint32_t timestamp = 0;
	for (int i = 0; i < RESP_MSG_TS_LEN; i++) {
		timestamp += timestampField[i] << (i * 8);
	}
	return timestamp;
}
