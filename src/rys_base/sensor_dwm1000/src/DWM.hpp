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

#ifndef _DWM_HPP_
#define _DWM_HPP_

#include <cstdint>
#include <string>
extern "C" {
	#include "deca_api/deca_device_api.h"
	#include "deca_api/deca_regs.h"
	#include "dwm_platform.h"
}

// Maximum frame length, as per 802.15.4 UWB standard.
// DW1000 supports up to 1023.
#define FRAME_LEN_MAX 127

#define SPI_SPEED_SLOW (3000000)
#define SPI_SPEED_FAST (10000000)
#define SPI_PATH "/dev/spidev1.0"

class DWM {
	private:
		int resetPin;
		int irqPin;
		std::string resetPinFilename;
		std::string irqPinFilename;

		// Received frame buffer
		uint8_t receiveBuffer[FRAME_LEN_MAX];
		// Copy of register status value
		uint32_t statusRegisterValue;
		// Copy of length of received frame
		uint16_t frameLength;
		// Communication configuration
		dwt_config_t config;

		// Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
		void spiSetRateLow();
		// Set SPI rate as close to 20 MHz as possible for optimum performances.
		void spiSetRateHigh();
		// Initialise all peripherals at once.
		void initializeHardware();
		void reset();
	public:
		DWM(const int resetPin, const int irqPin);
		~DWM();
		void initialize();
		uint8_t * receiveFrame();
};

#endif
