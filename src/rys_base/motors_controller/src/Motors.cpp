#include "Motors.h"

#include <iostream>
#include <cmath>

Motors::Motors() {
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
	{
		std::lock_guard<std::mutex> guard(this->fileAccessMutex);

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
	}

	this->disable();
}

void Motors::updateOdometry(float dt) {
	this->distance += (this->speedLeft + this->speedRight) / 2 * dt * 0.7;
}

void Motors::setSpeed(float speedLeft, float speedRight, int microstep) {
	// Deduplicate calls
	if (speedLeft == this->speedLeft && speedRight == this->speedRight && microstep == this->microstep) {
		return;
	}
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
	if (microstep != 1 && (microstep % 2 || microstep > 8)) {
		throw(std::string("Bad microstep value!"));
	}

	// Create data frame for PRU
	DataFrame dataFrame;

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
	std::lock_guard<std::mutex> guard(this->fileAccessMutex);

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
	std::lock_guard<std::mutex> guard(this->fileAccessMutex);

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
