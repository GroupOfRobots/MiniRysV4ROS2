/**
 * Author:
 * MJBogusz <mjbogusz.email.address@domain.name.com>
 *
 * Based on platform.c
 * by:
 * Anh Luong <luong@eng.utah.edu>
 * Peter Hillyard <peterhillyard@gmail.com>
 * (source: https://github.com/SPAN-UofU/dw1000_bbb)
 */

#include "dwm_platform.h"

#include "deca_api/deca_regs.h"
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

int spiFileDescriptor;

uint32_t spiMode;
uint8_t spiBits;
uint32_t spiSpeed;
uint16_t spiDelay;

void deca_sleep(unsigned int time_ms) {
	usleep(time_ms * 1000);
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer) {
	int status;

	uint8_t txbuf[headerLength + bodyLength];
	uint8_t rxbuf[headerLength + bodyLength];

	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)txbuf,
		.rx_buf = (unsigned long)rxbuf,
		.len = headerLength + bodyLength,
		.delay_usecs = spiDelay,
		.speed_hz = spiSpeed,
		.bits_per_word = spiBits,
	};

	for (unsigned int i = 0; i < headerLength; i++) {
		txbuf[i] = headerBuffer[i];
	}

	for (unsigned int i = 0; i < bodyLength; i++) {
		txbuf[headerLength + i] = bodyBuffer[i];
	}

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(spiFileDescriptor, SPI_IOC_MESSAGE(1), &transfer);
	if (status < 1) {
		return DWT_ERROR;
	}

	return DWT_SUCCESS;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readLength, uint8 *readBuffer) {
	int status;
	uint8_t buf[readLength];

	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)headerBuffer,
		.rx_buf = (unsigned long)buf,
		.len = headerLength + readLength,
		.delay_usecs = spiDelay,
		.speed_hz = spiSpeed,
		.bits_per_word = spiBits,
	};

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(spiFileDescriptor, SPI_IOC_MESSAGE(1), &transfer);
	if (status < 1) {
		return DWT_ERROR;
	}

	for (unsigned int i = 0; i < readLength; i++) {
		readBuffer[i] = buf[i + headerLength];
	}

	return DWT_SUCCESS;
}

decaIrqStatus_t decamutexon() {
	decaIrqStatus_t s = 0;

	// no interrupt lines
	if (s) {}

	// return state before disable, value is used to re-enable in decamutexoff call
	return s;
}

void decamutexoff(decaIrqStatus_t s) {
	// put a function here that re-enables the interrupt at the end of the critical section

	// need to check the port state as we can't use level sensitive interrupt on the STM ARM
	if (s) {
		// no interrupt lines
	}
}

void dwt_readtx_sys_count(uint8 * timestamp) {
	// Read bytes directly into buffer
	dwt_readfromdevice(TX_TIME_ID, TX_TIME_TX_RAWST_OFFSET, TX_TIME_TX_STAMP_LEN, timestamp);
}

void dwt_readrx_sys_count(uint8 * timestamp) {
	// Read bytes directly into buffer
	dwt_readfromdevice(RX_TIME_ID, RX_TIME_FP_RAWST_OFFSET, RX_TIME_RX_STAMP_LEN, timestamp);
}
