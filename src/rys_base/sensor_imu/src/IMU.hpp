#ifndef _IMU_HPP_
#define _IMU_HPP_

#include "MPU6050.hpp"
#define IMU_CALIBRATION_READINGS 5

class IMU {
	private:
		// The MPU object (from I2Cdev)
		MPU6050 *mpu;
		// Whether DMP init was successful
		bool dmpReady;
		// Status of last operation (0 - success, not 0 - error)
		uint8_t devStatus;
		// expected DMP packet size (default is 42 bytes)
		uint16_t packetSize;
		// FIFO storage buffer
		uint8_t fifoBuffer[64];
		// Calibration offsets
		float yawOffset;
		float pitchOffset;
		float rollOffset;
		bool preHeatingExitFlag;
		// static void preHeatingThreadFn();
	public:
		IMU();
		~IMU();
		void initialize(int rate = 100);
		void readData();
		void getYawPitchRoll(float *, float *, float *);
		float getYaw();
		float getPitch();
		float getRoll();
		void getGyro(float * rotationX, float * rotationY, float * rotationZ, bool getNewData = false);
		void resetFIFO();
		void calibrate();
		bool getPreHeatingExitFlag();
		void setOffsets(float yawOffset = 0.0f, float pitchOffset = 0.0f, float rollOffset = 0.0f, bool relative = true);
};

#endif
