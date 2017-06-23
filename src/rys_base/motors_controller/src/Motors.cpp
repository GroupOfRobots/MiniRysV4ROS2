#include "Motors.h"

#include <iostream>
#include <cmath>

Motors::Motors() {
	this->enabled = false;
	this->distance = 0;
	this->speedLeft = 0;
	this->speedRight = 0;
	this->microstep = 0;
	this->maxMotorSpeed = MAX_MOTOR_SPEED;
}

Motors::~Motors() {
	this->disable();
	this->pruFile.close();
}

void Motors::initialize() {
	this->disable();
}

void Motors::updateOdometry(float dt) {
	this->distance += (this->speedLeft + this->speedRight) / 2 * dt * 0.7;
}

void Motors::setSpeed(float speedLeft, float speedRight, int microstep) {
	if (speedLeft > 1.0f) {
		speedLeft = 1.0f;
	} else if (speedLeft < -1.0f) {
		speedLeft = -1.0f;
	}
	if (speedRight > 1.0f) {
		speedRight = 1.0f;
	} else if (speedRight < -1.0f) {
		speedRight = -1.0f;
	}

	// Validate microstep value
	if (microstep != 1 && (microstep == 0 || microstep % 2 || microstep > 32)) {
		throw(std::string("Bad microstep value!"));
	}

	// Create data frame for PRU
	DataFrame dataFrame;

	dataFrame.enabled = this->enabled;

	// Save microstep value
	this->microstep = microstep;
	dataFrame.microstep = microstep;

	// Calculate max speeds for given microstep value
	this->maxMotorSpeed = MAX_MOTOR_SPEED / microstep;

	// Left motor speed
	// Clip acceleration (change of value), set target speed
	if (speedLeft > (this->speedLeft + MAX_ACCELERATION)) {
		this->speedLeft = this->speedLeft + MAX_ACCELERATION;
	} else if (speedLeft < (this->speedLeft - MAX_ACCELERATION)) {
		this->speedLeft = this->speedLeft - MAX_ACCELERATION;
	} else {
		this->speedLeft = speedLeft;
	}
	// Write direction
	dataFrame.directionLeft = (this->speedLeft >= 0 ? 0 : 1);
	// Calculate and write speed to data frame
	if (this->speedLeft == 0) {
		dataFrame.speedLeft = 0;
	} else {
		dataFrame.speedLeft = this->maxMotorSpeed / std::abs(this->speedLeft);
	}

	// Right motor speed
	// Clip acceleration (change of value), set target speed
	if (speedRight > (this->speedRight + MAX_ACCELERATION)) {
		this->speedRight = this->speedRight + MAX_ACCELERATION;
	} else if (speedRight < (this->speedRight - MAX_ACCELERATION)) {
		this->speedRight = this->speedRight - MAX_ACCELERATION;
	} else {
		this->speedRight = speedRight;
	}
	// Write direction
	dataFrame.directionRight = (this->speedRight >= 0 ? 0 : 1);
	// Calculate and write speed to data frame
	if (this->speedRight == 0) {
		dataFrame.speedRight = 0;
	} else {
		dataFrame.speedRight = this->maxMotorSpeed / std::abs(this->speedRight);
	}

	// Write data frame to file (PRU communication)
	this->writeDataFrame(dataFrame);
}

void Motors::enable() {
	this->enabled = true;
}

void Motors::disable() {
	this->enabled = false;
	DataFrame dataFrame;
	dataFrame.enabled = 0;
	dataFrame.microstep = 1;
	dataFrame.directionLeft = 0;
	dataFrame.directionRight = 0;
	dataFrame.speedLeft = 0;
	dataFrame.speedRight = 0;

	this->writeDataFrame(dataFrame);
}

float Motors::getDistance() {
	return distance;
}

void Motors::resetDistance() {
	distance = 0.0;
}

float Motors::getSpeedLeft() {
	return this->speedLeft;
}

float Motors::getSpeedRight() {
	return this->speedRight;
}

void Motors::writeDataFrame(const DataFrame & frame) {
	std::lock_guard<std::mutex> guard(this->fileAccessMutex);

	// We have to open/close this file each time or it won't "apply" to PRU
	this->pruFile.open(DEVICE_NAME, std::ofstream::out | std::ofstream::binary);
	if (!this->pruFile.is_open() || !this->pruFile.good()) {
		std::cout << "Failed writing to file: " << DEVICE_NAME << std::endl;
	} else {
		this->pruFile.write((char*)(&frame), (int)(sizeof(DataFrame)));
	}
	this->pruFile.close();
}
