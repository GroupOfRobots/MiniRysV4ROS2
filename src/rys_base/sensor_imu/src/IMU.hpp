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
		// Calibration offsets
		float yawOffset;
		float pitchOffset;
		float rollOffset;

		int fetchData(uint8_t * buffer);
	public:
		struct ImuData {
			double orientationQuaternion[4];
			double angularVelocity[3];
			double linearAcceleration[3];
		};

		IMU();
		~IMU();
		void initialize();
		int getData(ImuData * data);
		void setOffsets(float yawOffset = 0.0f, float pitchOffset = 0.0f, float rollOffset = 0.0f, bool relative = true);
};

#endif
