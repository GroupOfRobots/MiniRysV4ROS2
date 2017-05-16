#include "IMU.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

IMU::IMU() {
	this->mpu = new MPU6050();
	this->dmpReady = false;
	this->devStatus = 0;
	this->packetSize = 0;
	for (int i = 0; i < 64; ++i) {
		this->fifoBuffer[i] = 0;
	}

	this->preHeatingExitFlag = true;
	this->yawOffset = 0;
	this->pitchOffset = 0;
	this->rollOffset = 0;
}

IMU::~IMU() {
	delete this->mpu;
}

void IMU::initialize(int rate) {
	// initialize device
	this->mpu->initialize();

	// verify connection
	if (!this->mpu->testConnection()) {
		throw(std::string("MPU6050 connection failed"));
	}

	// load and configure the DMP
	devStatus = this->mpu->dmpInitialize();
	// setRate accepts factor that works like this: 1kHz / (1 + x)
	int rateFactor = (1000/rate) - 1;
	this->mpu->setRate(rateFactor);

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		this->mpu->setDMPEnabled(true);

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = this->mpu->dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		throw(std::string("DMP Initialization failed (code %d)", devStatus));
	}
}

void IMU::readData() {
	// if programming failed, don't try to do anything
	if (!dmpReady) {
		return;
	}

	// Get current FIFO length
	uint16_t fifoCount = this->mpu->getFIFOCount();

	// Check for FIFO overflow
	if (fifoCount == 1024) {
		// reset so we can continue cleanly
		this->mpu->resetFIFO();
		fifoCount = 0;
	}

	// Check for DMP data ready interrupt (this should happen frequently)
	while (fifoCount < this->packetSize) {
		fifoCount = this->mpu->getFIFOCount();
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}

	// Read a packet from FIFO to buffer
	this->mpu->getFIFOBytes(this->fifoBuffer, this->packetSize);
}

void IMU::getYawPitchRoll(float * yaw, float * pitch, float * roll) {
	this->readData();

	Quaternion quaternion;
	VectorFloat gravity;

	// Parse data from FIFO to quaternion
	this->mpu->dmpGetQuaternion(&quaternion, this->fifoBuffer);
	// Get gravity vector based on quaternion (simple float multiplication and addition/subtraction)
	this->mpu->dmpGetGravity(&gravity, &quaternion);
	// Calculate Yaw/Pitch/Roll based on gravity vector
	float data[3];
	this->mpu->dmpGetYawPitchRoll(data, &quaternion, &gravity);

	*yaw = data[0] - this->yawOffset;
	*pitch = data[1] - this->pitchOffset;
	*roll = data[2] - this->rollOffset;
}

float IMU::getYaw() {
	this->readData();

	Quaternion quaternion;

	this->mpu->dmpGetQuaternion(&quaternion, this->fifoBuffer);
	// Return Yaw (based on MPU's dmpGetYawPitchRoll())
	float qx = quaternion.x;
	float qy = quaternion.y;
	float qz = quaternion.z;
	float qw = quaternion.w;
	return atan2(2*qx*qy - 2*qw*qz, 2*qw*qw + 2*qx*qx - 1) - this->yawOffset;
}

float IMU::getPitch() {
	this->readData();

	Quaternion quaternion;
	VectorFloat gravity;

	this->mpu->dmpGetQuaternion(&quaternion, this->fifoBuffer);
	this->mpu->dmpGetGravity(&gravity, &quaternion);
	float gx = gravity.x;
	float gy = gravity.y;
	float gz = gravity.z;
	return atan(gx / sqrt(gy*gy + gz*gz)) - this->pitchOffset;
}

float IMU::getRoll() {
	this->readData();

	Quaternion quaternion;
	VectorFloat gravity;

	this->mpu->dmpGetQuaternion(&quaternion, this->fifoBuffer);
	this->mpu->dmpGetGravity(&gravity, &quaternion);
	float gx = gravity.x;
	float gy = gravity.y;
	float gz = gravity.z;
	return atan(gy / sqrt(gx*gx + gz*gz)) - this->rollOffset;
}

void IMU::getGyro(float * rotationX, float * rotationY, float * rotationZ, bool getNewData) {
	if (getNewData) {
		this->readData();
	}

	VectorInt16 gyro;
	this->mpu->dmpGetGyro(&gyro, this->fifoBuffer);
	float divider = 16.4f;
	*rotationX = float(gyro.x) / divider;
	*rotationY = float(gyro.y) / divider;
	*rotationZ = float(gyro.z) / divider;
}

void IMU::resetFIFO() {
	this->mpu->resetFIFO();
}

void preHeatingThreadFn(IMU * imu) {
	while (!imu->getPreHeatingExitFlag()) {
		imu->readData();
	}
}

bool IMU::getPreHeatingExitFlag() {
	return this->preHeatingExitFlag;
}

void IMU::calibrate() {
	// Zero-out current offsets
	this->yawOffset = 0;
	this->pitchOffset = 0;
	this->rollOffset = 0;

	std::cout << "Pre-heating the IMU (60s)...\n";

	// Create the IMU pre-heating thread
	this->preHeatingExitFlag = false;
	std::thread preHeatingThread(preHeatingThreadFn, this);

	// Wait 30s
	auto now = std::chrono::steady_clock::now();
	std::this_thread::sleep_until(now + std::chrono::seconds(60));

	// Notify user
	std::cout << "Fix the position and press enter\n";
	std::cin.get();

	// Stop pre-heating thread
	this->preHeatingExitFlag = true;
	preHeatingThread.join();

	// Create accumulators for values
	double yawSum = 0;
	double pitchSum = 0;
	double rollSum = 0;
	// Also read counter
	unsigned int iterations = 0;

	// Create timer for 500ms
	now = std::chrono::steady_clock::now();
	auto end = now + std::chrono::milliseconds(500);

	float yaw, pitch, roll;
	// Until that timer...
	while (end > std::chrono::steady_clock::now()) {
		// Get a reading
		this->getYawPitchRoll(&yaw, &pitch, &roll);
		yawSum += yaw;
		pitchSum += pitch;
		rollSum += roll;

		// Increase iteration counter
		++iterations;
	}

	// Calculate average reading and save it
	this->yawOffset = yawSum / iterations;
	this->pitchOffset = pitchSum / iterations;
	this->rollOffset = rollSum / iterations;

	// Re-start pre-heating thread
	this->preHeatingExitFlag = false;
	preHeatingThread = std::thread(preHeatingThreadFn, this);

	// Notify user
	std::cout << "calibration done, offsets:\n";
	std::cout << "\tYaw Offset: " << this->yawOffset << std::endl;
	std::cout << "\tPitch Offset: " << this->pitchOffset << std::endl;
	std::cout << "\tRoll Offset: " << this->rollOffset << std::endl;
	std::cout << "press enter to continue:\n";
	std::cin.get();

	// End pre-heating thread
	this->preHeatingExitFlag = true;
	preHeatingThread.join();
}

void IMU::setOffsets(float yawOffset, float pitchOffset, float rollOffset, bool relative) {
	if (relative) {
		this->yawOffset = this->yawOffset + yawOffset;
		this->pitchOffset = this->pitchOffset + pitchOffset;
		this->rollOffset = this->rollOffset + rollOffset;
	} else {
		this->yawOffset = yawOffset;
		this->pitchOffset = pitchOffset;
		this->rollOffset = rollOffset;
	}
}
