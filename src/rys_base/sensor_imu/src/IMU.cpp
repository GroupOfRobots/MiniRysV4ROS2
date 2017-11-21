#include "IMU.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

IMU::IMU() {
	this->mpu = new MPU6050();
	this->packetSize = 0;
	for (int i = 0; i < 64; ++i) {
		this->fifoBuffer[i] = 0;
	}

	this->yawOffset = 0;
	this->pitchOffset = 0;
	this->rollOffset = 0;
}

IMU::~IMU() {
	delete this->mpu;
}

void IMU::initialize() {
	// initialize device
	this->mpu->initialize();

	// verify connection
	if (!this->mpu->testConnection()) {
		throw(std::string("MPU6050 connection failed"));
	}

	// load and configure the DMP
	uint8_t devStatus = this->mpu->dmpInitialize();

	// setRate accepts factor that works like this: 1kHz / (1 + x); 4 = 200Hz
	/*
	this->mpu->setRate(4);
	this->mpu->setDLPFMode(MPU6050_DLPF_BW_188);
	this->mpu->setDHPFMode(MPU6050_DHPF_2P5);
	*/

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		this->mpu->setDMPEnabled(true);

		// get expected DMP packet size for later comparison
		packetSize = this->mpu->dmpGetFIFOPacketSize();

		// this->mpu->resetFIFO();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		throw(std::string("DMP Initialization failed (code %d)", devStatus));
	}
}

int IMU::fetchData(uint8_t * buffer) {
	// Get current FIFO length
	uint16_t fifoCount = this->mpu->getFIFOCount();

	// If there's not enough
	if (fifoCount < this->packetSize) {
		return -1;
	}

	this->mpu->getFIFOBytes(buffer, this->packetSize);
	this->mpu->resetFIFO();
	return 1;
}

int IMU::getData(ImuData * data) {
	uint8_t rawDataBuffer[this->packetSize];

	// If data is not ready - skip
	if (this->fetchData(rawDataBuffer) < 0) {
		return -1;
	}

	// Get orientation quaternion from the buffer and scale it
	int16_t orientationQuaternion[4];
	this->mpu->dmpGetQuaternion(orientationQuaternion, rawDataBuffer);
	for (int i = 0; i < 4; ++i) {
		// Raw data is in +-32k range with +-2.0 values
		data->orientationQuaternion[i] = static_cast<double>(orientationQuaternion[i]) / 16384.0;
	}

	// Calculate gravity vector based on orientation for use in linear acceleration calculations.
	// This vector is in g-s, thus having a length of 1.
	double gravity[3];
	double qw = data->orientationQuaternion[0];
	double qx = data->orientationQuaternion[1];
	double qy = data->orientationQuaternion[2];
	double qz = data->orientationQuaternion[3];
	gravity[0] = 2 * (qx * qz - qw * qy);
	gravity[1] = 2 * (qw * qx + qy * qz);
	gravity[2] = qw * qw - qx * qx - qy * qy + qz * qz;

	// Get acceleration, subtract gravity (so we get linear acceleration) and scale it to m/s
	int16_t rawAcceleration[3];
	this->mpu->dmpGetAccel(rawAcceleration, rawDataBuffer);
	for (int i = 0; i < 3; ++i) {
		// Raw data is in +-2g range with +-32k values - change that into m/s ((a / 16384 - g) * 9.81)
		/// TODO: if needed, rotate acceleration vector by quaternion BEFORE subtracting gravity
		data->linearAcceleration[i] = (static_cast<double>(rawAcceleration[i]) / 16384.0 - gravity[i]) * 9.81;
	}

	// Get angular velocity from raw data and scale it
	int16_t angularVelocity[3];
	this->mpu->dmpGetGyro(angularVelocity, rawDataBuffer);
	for (int i = 0; i < 3; ++i) {
		// Raw data is in +-2000deg/s range with +-32k values - change into rad/s ((r / 16.384) * M_PI/180)
		data->angularVelocity[i] = static_cast<double>(angularVelocity[i]) * 0.0010652644360316954;
	}

	return 1;
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
