#include "Controller.h"
#include <iostream>

Controller::Controller() {
	balancing = true;
	lqrEnabled = true;

	speedFilterFactor = 1.0f;
	angleFilterFactor = 1.0f;
	angleFiltered = 0.0f;

	pidSpeedRegulatorEnabled = true;
	pidAngularVelocityFactor = 1.0f;
	pidLinearVelocityFiltered = 0;
	pidSpeedKp = 1;
	pidSpeedKi = 0;
	pidSpeedKd = 0;
	pidSpeedIntegral = 0;
	pidSpeedError = 0;
	pidAngleKp = 1;
	pidAngleKi = 0;
	pidAngleKd = 0;
	pidAngleIntegral = 0;
	pidAngleError = 0;

	lqrAngularVelocityK = 0;
	lqrAngleK = 0;
}

Controller::~Controller() {}

void clipValue(float & value, float max) {
	if (value > max) {
		value = max;
	} else if (value < -max) {
		value = -max;
	}
}

void Controller::init() {
	this->timePoint = std::chrono::high_resolution_clock::now();
	this->timePointPrevious = std::chrono::high_resolution_clock::now();
}

void Controller::setBalancing(bool value) {
	this->balancing = value;
}

void Controller::setLQREnabled(bool value) {
	this->lqrEnabled = value;
}

void Controller::setSpeedFilterFactor(float factor) {
	this->speedFilterFactor = factor;
}

void Controller::setAngleFilterFactor(float factor) {
	this->angleFilterFactor = factor;
}

void Controller::setPIDAngularVelocityFactor(float factor) {
	this->pidAngularVelocityFactor = factor;
}

void Controller::setPIDSpeedRegulatorEnabled(bool enabled) {
	this->pidSpeedRegulatorEnabled = enabled;
}

void Controller::setPIDParameters(float speedKp, float speedKi, float speedKd, float angleKp, float angleKi, float angleKd) {
	this->pidSpeedKp = speedKp;
	this->pidSpeedKi = speedKi;
	this->pidSpeedKd = speedKd;
	this->pidAngleKp = angleKp;
	this->pidAngleKi = angleKi;
	this->pidAngleKd = angleKd;
}

void Controller::setLQRParameters(float angularVelocityK, float angleK) {
	this->lqrAngularVelocityK = angularVelocityK;
	this->lqrAngleK = angleK;
}

void Controller::zeroPIDs() {
	this->pidAngleIntegral = 0;
	this->pidAngleError = 0;
	this->pidSpeedIntegral = 0;
	this->pidSpeedError = 0;
}

float Controller::getSpeedFilterFactor() {
	return this->speedFilterFactor;
}

float Controller::getAngleFilterFactor() {
	return this->angleFilterFactor;
}

bool Controller::getLQREnabled() {
	return this->lqrEnabled;
}

float Controller::getPIDAngularVelocityFactor() {
	return this->pidAngularVelocityFactor;
}

bool Controller::getPIDSpeedRegulatorEnabled() {
	return this->pidSpeedRegulatorEnabled;
}

void Controller::getPIDParameters(float & speedKp, float & speedKi, float & speedKd, float & angleKp, float & angleKi, float & angleKd) {
	speedKp = this->pidSpeedKp;
	speedKi = this->pidSpeedKi;
	speedKd = this->pidSpeedKd;
	angleKp = this->pidAngleKp;
	angleKi = this->pidAngleKi;
	angleKd = this->pidAngleKd;
}

void Controller::getLQRParameters(float & angularVelocityK, float & angleK) {
	angularVelocityK = this->lqrAngularVelocityK;
	angleK = this->lqrAngleK;
}

void Controller::calculateSpeeds(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	clipValue(throttle, 1);
	float throttleRaw = throttle * THROTTLE_MAX;
	clipValue(rotation, 1);
	float rotationRaw = rotation * ROTATION_MAX;

	if (!this->balancing) {
		speedLeftNew = throttleRaw + rotationRaw;
		speedRightNew = throttleRaw - rotationRaw;
		return;
	}

	if (this->lqrEnabled) {
		this->calculateSpeedsLQR(angle, rotationX, speed, throttleRaw, rotationRaw, speedLeftNew, speedRightNew);
	} else {
		this->calculateSpeedsPID(angle, rotationX, speed, throttleRaw, rotationRaw, speedLeftNew, speedRightNew, loopTime);
	}
}

void Controller::calculateSpeedsPID(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	float targetAngle = 0.0f;
	// If speed regulator (first PID layer) is enabled
	if (this->pidSpeedRegulatorEnabled) {
		// Estimate robot's linear velocity based on angle change and speed
		// (Motors' angular velocity = -robot's angular velocity + robot's linear velocity * const)
		// First, calculate robot's angular velocity and normalize it to motors' speed values (thus the constant at the end)
		float angularVelocity = rotationX * this->pidAngularVelocityFactor;

		// Then, subtract the estimated robot's angular velocity from motor's angular velocity
		// What's left is motor's angular velocity responsible for robot's linear velocity
		float linearVelocity = speed - angularVelocity;

		// Also, apply low-pass filter on resulting value
		this->pidLinearVelocityFiltered = linearVelocity * this->speedFilterFactor + this->pidLinearVelocityFiltered * (1.0f - this->speedFilterFactor);

		// First control layer: speed control PID
		// setpoint: user throttle
		// current value: estimated and filtered robot speed
		// output: target robot angle to get the desired speed

		float speedError = throttle - this->pidLinearVelocityFiltered;
		this->pidSpeedIntegral += speedError * loopTime;
		if (this->pidSpeedIntegral > ANGLE_MAX || this->pidSpeedIntegral < -ANGLE_MAX) {
			this->pidSpeedIntegral = 0;
		}

		float speedDerivative = (speedError - this->pidSpeedError) / loopTime;
		this->pidSpeedError = speedError;

		float targetAngle = pidSpeedKp * speedError + pidSpeedKi * this->pidSpeedIntegral + pidSpeedKd * speedDerivative;
		clipValue(targetAngle, ANGLE_MAX);
	}

	// Apply low-pass filter on the angle
	this->angleFiltered = angle * this->angleFilterFactor + this->angleFiltered * (1.0f - this->angleFilterFactor);

	// Second control layer: angle control PID
	// setpoint: robot target angle (from SPEED CONTROL)
	// current value: robot angle (filtered)
	// output: motor speed

	float angleError = targetAngle - this->angleFiltered;
	this->pidAngleIntegral += angleError * loopTime;
	if (this->pidAngleIntegral > SPEED_MAX || this->pidAngleIntegral < -SPEED_MAX) {
		this->pidAngleIntegral = 0;
	}

	float angleDerivative = (angleError - this->pidAngleError) / loopTime;
	this->pidAngleError = angleError;

	float output = pidAngleKp * angleError + pidAngleKi * this->pidAngleIntegral + pidAngleKd * angleDerivative;
	clipValue(output, SPEED_MAX);

	// The rotation part from the user is injected directly into the output
	speedLeftNew = output + rotation;
	speedRightNew = output - rotation;
}

void Controller::calculateSpeedsLQR(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew) {
	// Apply low-pass filter on the angle
	this->angleFiltered = angle * this->angleFilterFactor + this->angleFiltered * (1.0f - this->angleFilterFactor);
	// Calculate output: Motor speed change
	float outputChange = - (this->lqrAngularVelocityK * rotationX + this->lqrAngleK * angle) / 22.5;
	// The rotation part from the user is injected directly into the output
	speedLeftNew = speed + outputChange + rotation;
	speedRightNew = speed + outputChange - rotation;
}
