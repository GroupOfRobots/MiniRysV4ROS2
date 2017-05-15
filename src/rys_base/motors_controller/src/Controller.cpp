#include "Controller.h"
#include <iostream>

Controller::Controller() {
	angleFilterFactor = 1.0f;
	speedFilterFactor = 1.0f;
	angularVelocityFactor = 0.009f;
	speedRegulatorEnabled = true;
	linearVelocityFiltered = 0;
	speedPIDKp = 1;
	speedPIDKi = 0;
	speedPIDKd = 0;
	speedPIDIntegral = 0;
	speedPIDError = 0;
	anglePrevious = 0;
	angleFiltered = 0;
	anglePIDKp = 1;
	anglePIDKi = 0;
	anglePIDKd = 0;
	anglePIDIntegral = 0;
	anglePIDError = 0;
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

float Controller::speedControl(float value, float setPoint, float dt) {
	float error = setPoint - value;

	this->speedPIDIntegral += error * dt;
	clipValue(this->speedPIDIntegral, ANGLE_MAX);

	float derivative = (error - this->speedPIDError) / dt;
	this->speedPIDError = error;

	float output = speedPIDKp * error + speedPIDKi * this->speedPIDIntegral + speedPIDKd * derivative;
	clipValue(output, ANGLE_MAX);
	return output;
}

float Controller::angleControl(float value, float setPoint, float dt) {
	float error = setPoint - value;

	this->anglePIDIntegral += error * dt;
	clipValue(this->anglePIDIntegral, SPEED_MAX);

	float derivative = (error - this->anglePIDError) / dt;
	this->anglePIDError = error;

	float output = anglePIDKp * error + anglePIDKi * this->anglePIDIntegral + anglePIDKd * derivative;
	clipValue(output, SPEED_MAX);
	return output;
}

void Controller::calculateSpeed(float angle, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	clipValue(throttle, 1);
	float throttleRaw = throttle * THROTTLE_MAX;
	clipValue(rotation, 1);
	float rotationRaw = rotation * ROTATION_MAX;

	float targetAngle = 0.0f;
	if (this->speedRegulatorEnabled) {
		// Estimate robot's linear velocity based on angle change and speed
		// (Motors' angular velocity = -robot's angular velocity + robot's linear velocity * const)

		// First, calculate robot's angular velocity and normalize it to motors' speed values (thus the constant at the end)
		///TODO: find proper const
		float angularVelocity = (angle - this->anglePrevious) / loopTime * this->angularVelocityFactor;
		this->anglePrevious = angle;
		// Then, subtract the estimated robot's angular velocity from motor's angular velocity
		// What's left is motor's angular velocity responsible for robot's linear velocity
		float linearVelocity = speed - angularVelocity;

		// Also, apply low-pass filter on resulting value
		this->linearVelocityFiltered = linearVelocity * this->speedFilterFactor + this->linearVelocityFiltered * (1.0f - this->speedFilterFactor);
		// First control layer: speed control PID
		// input: user throttle (0)
		// setPoint: estimated and filtered robot speed
		// output: target robot angle to get the desired speed
		targetAngle = this->speedControl(this->linearVelocityFiltered, throttleRaw, loopTime);
	}

	// Apply low-pass filter on the angle itself too
	this->angleFiltered = angle * this->angleFilterFactor + this->angleFiltered * (1.0f - this->angleFilterFactor);
	// Second control layer: angle control PID
	// input: robot target angle (from SPEED CONTROL)
	// variable: robot angle
	// output: Motor speed
	float output = this->angleControl(this->angleFiltered, targetAngle, loopTime);

	// The rotation part from the user is injected directly into the output
	speedLeftNew = output + rotationRaw;
	speedRightNew = output - rotationRaw;

	// std::cout << "Controller:";
	// std::cout << " t: " << loopTime;
	// std::cout << " v: " << angularVelocity;
	// std::cout << " s: " << speed;
	// std::cout << " a: " << targetAngle;
	// std::cout << " o: " << output;
	// std::cout << std::endl;
}

void Controller::zeroPIDs() {
	this->anglePIDIntegral = 0;
	this->anglePIDError = 0;
	this->speedPIDIntegral = 0;
	this->speedPIDError = 0;
}

void Controller::setSpeedFilterFactor(float factor) {
	this->speedFilterFactor = factor;
}

void Controller::setAngleFilterFactor(float factor) {
	this->angleFilterFactor = factor;
}

void Controller::setAngularVelocityFactor(float factor) {
	this->angularVelocityFactor = factor;
}

void Controller::setSpeedRegulatorEnabled(bool enabled) {
	this->speedRegulatorEnabled = enabled;
}

void Controller::setSpeedPID(float kp, float ki, float kd) {
	this->speedPIDKp = kp;
	this->speedPIDKi = kd;
	this->speedPIDKd = ki;
}

void Controller::setAnglePID(float kp, float ki, float kd) {
	this->anglePIDKp = kp;
	this->anglePIDKi = kd;
	this->anglePIDKd = ki;
}

float Controller::getSpeedFilterFactor() {
	return this->speedFilterFactor;
}

float Controller::getAngleFilterFactor() {
	return this->angleFilterFactor;
}

float Controller::getAngularVelocityFactor() {
	return this->angularVelocityFactor;
}

bool Controller::getSpeedRegulatorEnabled() {
	return this->speedRegulatorEnabled;
}

void Controller::getSpeedPID(float & kp, float & ki, float & kd) {
	kp = this->speedPIDKp;
	ki = this->speedPIDKi;
	kd = this->speedPIDKd;
}

void Controller::getAnglePID(float & kp, float & ki, float & kd) {
	kp = this->anglePIDKp;
	ki = this->anglePIDKi;
	kd = this->anglePIDKd;
}
