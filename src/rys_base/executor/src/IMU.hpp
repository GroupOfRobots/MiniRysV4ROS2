#ifndef _IMU_HPP_
#define _IMU_HPP_

#include "MPU6050.hpp"
#define IMU_CALIBRATION_READINGS 5

class IMU {
	private:
		// The MPU object (from I2Cdev)
		MPU6050 *mpu;
		// expected DMP packet size (default is 42 bytes)
		uint16_t packetSize;
		// FIFO storage buffer
		uint8_t fifoBuffer[64];

		int fetchData(uint8_t * buffer);
	public:
		struct ImuData {
			double orientationQuaternion[4];
			double angularVelocity[3];
			double linearAcceleration[3];
		};

		IMU();
		~IMU();
		int getData(ImuData * data);
		/**
		* Set offsets to MPU6050 readings. Uses set*GyroOffset() and set*AccelOffset().
		* \param offsets - list of offsets to apply, in order: ax, ay, az, gx, gy, gz
		*/
		void setOffsets(const int offsets[6]);
};

#endif
