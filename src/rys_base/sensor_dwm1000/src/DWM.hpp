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

#define SPI_SPEED_SLOW (3000000)
#define SPI_SPEED_FAST (10000000)
#define SPI_PATH "/dev/spidev1.0"

// Maximum frame length, as per 802.15.4 UWB standard (DW1000 supports up to 1023), for frame receiving.
#define FRAME_LEN_MAX 127
// Buffer to store received response message, for ranging.
#define RX_BUF_LEN 20

// Default antenna delay values for 64 MHz PRF.
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

// Length of the common part of the message
#define ALL_MSG_COMMON_LEN 10
// Indexes to access some of the fields in the frames (txPollMessage and rxResponseMessage)
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

// UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
// 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu
#define UUS_TO_DWT_TIME 65536

// Delay between frames, in UWB microseconds.
#define POLL_TX_TO_RESP_RX_DLY_UUS 140
// Receive response timeout.
#define RESP_RX_TIMEOUT_UUS 210

// Speed of light in air, in metres per second.
#define SPEED_OF_LIGHT 299702547

class DWM {
	private:
		int resetPin;
		int irqPin;
		std::string resetPinFilename;
		std::string irqPinFilename;

		// Received frame buffer
		uint8_t receiveBufferFrame[FRAME_LEN_MAX];
		uint8_t receiveBufferRanging[FRAME_LEN_MAX];
		// Frame sequence number, incremented after each transmission.
		uint8_t frameSequenceNumber;

		uint8_t txPollMessage[RX_BUF_LEN];
		uint8_t rxResponseMessage[RX_BUF_LEN];

		// Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
		void spiSetRateLow();
		// Set SPI rate as close to 20 MHz as possible for optimum performances.
		void spiSetRateHigh();
		// Initialise all peripherals at once.
		void initializeHardware();
		void reset();
		static uint32_t getTimestampFromMessage(uint8_t * message);
	public:
		DWM(const int resetPin, const int irqPin);
		~DWM();
		void initialize(bool ranging = true);
		uint8_t * receiveFrame();
		double readRange();
};

#endif
