#include "Controller.h"
#include <iostream>

Controller::Controller() {
	balancing = true;
	lqrEnabled = true;

	speedFilterFactor = 1.0f;
	speedFiltered = 0.0f;
	angleFilterFactor = 1.0f;
	angleFiltered = 0.0f;

	pidSpeedRegulatorEnabled = true;
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

void Controller::setLQRParameters(float linearVelocityK, float angularVelocityK, float angleK) {
	this->lqrLinearVelocityK = linearVelocityK;
	this->lqrAngularVelocityK = angularVelocityK;
	this->lqrAngleK = angleK;
}

void Controller::zeroRegulators() {
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

void Controller::getLQRParameters(float & linearVelocityK, float & angularVelocityK, float & angleK) {
	linearVelocityK = this->lqrLinearVelocityK;
	angularVelocityK = this->lqrAngularVelocityK;
	angleK = this->lqrAngleK;
}

void Controller::calculateSpeeds(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	clipValue(throttle, 1.0f);
	clipValue(rotation, 1.0f);

	if (!this->balancing) {
		speedLeftNew = throttle + rotation;
		speedRightNew = throttle - rotation;
		clipValue(speedLeftNew, 1.0f);
		clipValue(speedRightNew, 1.0f);
		return;
	}

	this->angleFiltered = angle * this->angleFilterFactor + this->angleFiltered * (1.0f - this->angleFilterFactor);
	this->speedFiltered = speed * this->speedFilterFactor + this->speedFiltered * (1.0f - this->speedFilterFactor);

	if (this->lqrEnabled) {
		this->calculateSpeedsLQR(this->angleFiltered, rotationX, this->speedFiltered, throttle, rotation, speedLeftNew, speedRightNew, loopTime);
	} else {
		this->calculateSpeedsPID(this->angleFiltered, rotationX, this->speedFiltered, throttle, rotation, speedLeftNew, speedRightNew, loopTime);
	}
}

void Controller::calculateSpeedsPID(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	float targetAngle = 0.0f;
	// If speed regulator (first PID layer) is enabled
	if (this->pidSpeedRegulatorEnabled) {
		// Estimate robot's linear velocity based on angle change and speed
		// (Motors' angular velocity = -robot's angular velocity + robot's linear velocity * const)
		// What's left is motor's angular velocity responsible for robot's linear velocity
		float linearVelocity = -speed - rotationX / SPEED_TO_DEG;

		// First control layer: speed control PID
		// setpoint: user throttle
		// current value: robot's linear speed
		// output: target robot angle to get the desired speed

		float speedError = throttle - linearVelocity;
		this->pidSpeedIntegral += speedError * loopTime;
		// Integral anti-windup
		if (this->pidSpeedIntegral > ANGLE_MAX || this->pidSpeedIntegral < -ANGLE_MAX) {
			this->pidSpeedIntegral = 0.0f;
		}

		float speedDerivative = (speedError - this->pidSpeedError) / loopTime;
		this->pidSpeedError = speedError;

		float targetAngle = pidSpeedKp * speedError + pidSpeedKi * this->pidSpeedIntegral + pidSpeedKd * speedDerivative;
		clipValue(targetAngle, ANGLE_MAX);
	}

	// Second control layer: angle control PID
	// setpoint: robot target angle (from SPEED CONTROL)
	// current value: robot angle (filtered)
	// output: motor speed

	float angleError = targetAngle - angle;
	this->pidAngleIntegral += angleError * loopTime;
	// Integral anti-windup, 1 is max output value
	if (this->pidAngleIntegral > 1.0f || this->pidAngleIntegral < -1.0f) {
		this->pidAngleIntegral = 0.0f;
	}

	float angleDerivative = (angleError - this->pidAngleError) / loopTime;
	this->pidAngleError = angleError;

	float output = pidAngleKp * angleError + pidAngleKi * this->pidAngleIntegral + pidAngleKd * angleDerivative;
	clipValue(output, 1.0f);

	// The rotation part from the user is injected directly into the output
	speedLeftNew = speed + output + rotation;
	speedRightNew = speed + output - rotation;
}

void Controller::calculateSpeedsLQR(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	// Calculate linear velocity for regulator, 0.05 is wheel radius
	float linearVelocity = (- speed * SPEED_TO_DEG - rotationX) * DEG_TO_RAD * 0.05f;
	// Calculate output: Motor speed change
	float linearVelocityComponent = this->lqrLinearVelocityK * (throttle - linearVelocity);
	float angularVelocityComponent = this->lqrAngularVelocityK * rotationX * DEG_TO_RAD;
	float angleComponent = this->lqrAngleK * angle * DEG_TO_RAD;
	float outputChange = (linearVelocityComponent - angularVelocityComponent - angleComponent) * loopTime * RAD_TO_DEG / SPEED_TO_DEG;
	// The rotation part from the user is injected directly into the output
	speedLeftNew = speed + outputChange + rotation;
	speedRightNew = speed + outputChange - rotation;

	std::cout << "LQR o: " << (speed + outputChange) << std::endl;
}
