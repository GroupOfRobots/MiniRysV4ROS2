#include "Motors.h"

#include <iostream>
#include <cmath>

Motors::Motors() {
	this->distance = 0;
	this->speedLeft = 0;
	this->speedRight = 0;
	this->microstep = 0;
	this->maxMotorSpeed = MAX_MOTOR_SPEED;
	this->minMotorSpeed = MIN_MOTOR_SPEED;
}

Motors::~Motors() {
	this->disable();
	this->pruFile.close();
}

void Motors::initialize() {
	std::ofstream file;
	file.open("/sys/class/gpio/export", std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: /sys/class/gpio/export"));
	}
	file << "78";
	file << "79";
	file.close();

	file.open("/sys/class/gpio/gpio78/direction", std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: /sys/class/gpio/gpio78/direction"));
	}
	file << "out";
	file.close();

	file.open("/sys/class/gpio/gpio79/direction", std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: /sys/class/gpio/gpio79/direction"));
	}
	file << "out";
	file.close();

	this->disable();
}

void Motors::updateOdometry(float dt) {
	this->distance += (this->speedLeft + this->speedRight) / 2 * dt * 0.7;
}

void Motors::setSpeed(float speedLeft, float speedRight, int microstep) {
	if (speedLeft == this->speedLeft && speedRight == this->speedRight && microstep == this->microstep) {
		return;
	}

	DataFrame dataFrame;

	// Calculate max speeds for given microstep value
	if (microstep == 1) {
		this->maxMotorSpeed = MAX_MOTOR_SPEED;
		this->minMotorSpeed = MIN_MOTOR_SPEED;
	} else if (microstep == 2) {
		this->maxMotorSpeed = MAX_MOTOR_SPEED/2;
		this->minMotorSpeed = MIN_MOTOR_SPEED/2;
	} else if (microstep == 3) {
		this->maxMotorSpeed = MAX_MOTOR_SPEED/4;
		this->minMotorSpeed = MIN_MOTOR_SPEED/4;
	} else if (microstep == 4) {
		this->maxMotorSpeed = MAX_MOTOR_SPEED/8;
		this->minMotorSpeed = MIN_MOTOR_SPEED/8;
	} else {
		throw(std::string("Bad microstep value!"));
	}
	this->microstep = microstep;
	dataFrame.microstep = microstep;

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
		dataFrame.speedLeft = this->minMotorSpeed / std::abs(this->speedLeft);
	}
	// Clip value
	if (dataFrame.speedLeft != 0 && dataFrame.speedLeft < this->maxMotorSpeed) {
		dataFrame.speedLeft = (unsigned int)(this->maxMotorSpeed);
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
		dataFrame.speedRight = this->minMotorSpeed / std::abs(this->speedRight);
	}
	// Clip value
	if (dataFrame.speedRight != 0 && dataFrame.speedRight < this->maxMotorSpeed) {
		dataFrame.speedRight = (unsigned int)(this->maxMotorSpeed);
	}

	// Write data frame to file (PRU communication)
	// We have to open/close this file each time or it won't "apply" to PRU
	this->pruFile.open(DEVICE_NAME, std::ofstream::out | std::ofstream::binary);
	if (!this->pruFile.is_open() || !this->pruFile.good()) {
		// throw(std::string("Failed writing to file: ") + std::string(DEVICE_NAME));
		std::cout << "Failed writing to file: " << DEVICE_NAME << std::endl;
	} else {
		this->pruFile.write((char*)(&dataFrame), (int)(sizeof(DataFrame)));
		// std::cout << "Writing: " << (dataFrame.directionLeft == 0 ? '0' : ' ') << (dataFrame.directionLeft == 1 ? '1' : ' ') << " " << dataFrame.speedLeft << std::endl;
	}
	this->pruFile.close();
}

void Motors::enable() {
	std::ofstream file;

	file.open("/sys/class/gpio/gpio78/value", std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: /sys/class/gpio/gpio78/value"));
	}
	file << "0";
	file.close();

	file.open("/sys/class/gpio/gpio79/value", std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: /sys/class/gpio/gpio79/value"));
	}
	file << "0";
	file.close();
}

void Motors::disable() {
	std::ofstream file;

	file.open("/sys/class/gpio/gpio78/value", std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: /sys/class/gpio/gpio78/value"));
	}
	file << "1";
	file.close();

	file.open("/sys/class/gpio/gpio79/value", std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::string("Failed opening file: /sys/class/gpio/gpio79/value"));
	}
	file << "1";
	file.close();
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
