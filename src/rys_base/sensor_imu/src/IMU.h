#ifndef _IMU_H_
#define _IMU_H_

#include "MPU6050.h"
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
		// Number of all bytes currently in FIFO
		// uint16_t fifoCount;
		// FIFO storage buffer
		uint8_t fifoBuffer[64];
		// [w, x, y, z] - quaternion container
		Quaternion *quaternion;
		// [x, y, z] - gravity vector
		VectorFloat *gravity;
		// [yaw, pitch, roll] - yaw/pitch/roll container
		float yawPitchRoll[3];
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
		// void initializeNoDMP(int rate = 100);
		void readData();
		void getYawPitchRoll(float *, float *, float *);
		float getYaw();
		float getPitch();
		float getRoll();
		// float getRollNoDMP();
		void resetFIFO();
		// Static calibration - reads the sensor n times, averages it and sets as offset
		void calibrate();
		bool getPreHeatingExitFlag();
};

#endif
