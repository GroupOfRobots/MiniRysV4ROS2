/**
 * Author:
 * MJBogusz <mjbogusz.email.address@domain.name.com>
 *
 * Based on "Simple RX example code" by:
 * Decawave
 * as modified by:
 * Anh Luong <luong@eng.utah.edu>
 * (source: https://github.com/SPAN-UofU/dw1000_bbb)
 */

#include "DWM.hpp"

#include <iostream>
#include <fstream>

// For open() and O_RDWR
#include <fcntl.h>
// For SPI_* defines
#include <linux/spi/spidev.h>
// For ioctl()
#include <sys/ioctl.h>

DWM::DWM(const int resetPin, const int irqPin) {
	this->resetPin = resetPin;
	this->irqPin = irqPin;

	for (int i = 0; i < FRAME_LEN_MAX; ++i) {
		this->receiveBuffer[i] = 0;
	}
	this->statusRegisterValue = 0;
	this->frameLength = 0;
	// Default communication configuration. We use here EVK1000's default mode (mode 3).
	// Channel number
	this->config.chan = 2;
	// Pulse repetition frequency.
	this->config.prf = DWT_PRF_64M;
	// Preamble length. Used in TX only.
	this->config.txPreambLength = DWT_PLEN_1024;
	// Preamble acquisition chunk size. Used in RX only.
	this->config.rxPAC = DWT_PAC32;
	// TX preamble code. Used in TX only.
	this->config.txCode = 9;
	// RX preamble code. Used in RX only.
	this->config.rxCode = 9;
	// 0 to use standard SFD, 1 to use non-standard SFD.
	this->config.nsSFD = 1;
	// Data rate.
	this->config.dataRate = DWT_BR_110K;
	// PHY header mode.
	this->config.phrMode = DWT_PHRMODE_STD;
	// SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
	this->config.sfdTO = (1025 + 64 - 32);
}

DWM::~DWM() {}

void DWM::initialize() {
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
	dwt_configure(&config);
}

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

uint8_t * DWM::receiveFrame() {
	// Clear RX buffer to avoid having leftovers from previous receptions.
	for (int i = 0; i < FRAME_LEN_MAX; i++) {
		this->receiveBuffer[i] = 0;
	}

	// Activate immediate reception.
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

	// Poll until a frame is properly received or an error/timeout occurs.
	// STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register,
	// we can use this simplest API function to access it.
	std::cout << "Waiting for message...\n";
	statusRegisterValue = dwt_read32bitreg(SYS_STATUS_ID);
	while (!(statusRegisterValue & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
		deca_sleep(10);
		statusRegisterValue = dwt_read32bitreg(SYS_STATUS_ID);
	}

	if (statusRegisterValue & SYS_STATUS_RXFCG) {
		// A frame has been received, copy it to our local buffer.
		frameLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frameLength <= FRAME_LEN_MAX) {
			dwt_readrxdata(this->receiveBuffer, frameLength, 0);
		}

		// Clear good RX frame event in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		std::cout << "MSG Received! DATA: " << this->receiveBuffer << std::endl;
		// Copy received data and return it
		uint8_t * buffer = new uint8_t[FRAME_LEN_MAX];
		for (int i = 0; i < FRAME_LEN_MAX; ++i) {
			buffer[i] = this->receiveBuffer[i];
		}
		return buffer;
	} else {
		// Clear RX error events in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

		std::cout << "MSG Receive error!\n";
		throw(std::string("MSG Receive error"));
	}
}
