#include "Controller.h"
#include <iostream>

Controller::Controller() {
	speedFilterFactor = 0.95;
	angle = 0;
	anglePrevious = 0;
	anglePIDKp = 1;
	anglePIDKi = 0;
	anglePIDKd = 0;
	anglePIDIntegral = 0;
	anglePIDError = 0;
	speed = 0;
	speedPrevious = 0;
	speedFiltered = 0;
	speedPIDKp = 1;
	speedPIDKi = 0;
	speedPIDKd = 0;
	speedPIDIntegral = 0;
	speedPIDError = 0;
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

void Controller::calculateSpeed(float angle, float speedLeft, float speedRight, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	this->anglePrevious = this->angle;
	this->angle = angle;
	this->speedPrevious = this->speed;
	this->speed = (speedLeft + speedRight) / 2;

	clipValue(throttle, 1);
	int throttleRaw = throttle * THROTTLE_MAX;
	clipValue(rotation, 1);
	int rotationRaw = rotation * ROTATION_MAX;

	this->speedFiltered = this->speedFilterFactor * this->speed + (1.0f - this->speedFilterFactor) * this->speedFiltered;

	/*
	// Estimate robot's velocity based on angle change and speed
	// 90 is an empirical extracted factor to adjust for real units
	float angularVelocity = (this->angle - this->anglePrevious) * 90.0 * loopTime;
	// We use robot_speed(t-1) to compensate the delay
	float estimatedSpeed = -this->speedPrevious - angularVelocity;
	// Low pass / integrating filter on estimated speed
	this->speedFiltered = this->speedFiltered * this->speedFilterFactor + estimatedSpeed * (1.0 - this->speedFilterFactor);
	*/

	// First control layer: speed control PID
	// input: user throttle (0)
	// setPoint: estimated and filtered robot speed
	// output: target robot angle to get the desired speed
	float targetAngle = this->speedControl(this->speedFiltered, throttleRaw, loopTime);

	// Second control layer: angle control PID
	// input: robot target angle (from SPEED CONTROL)
	// variable: robot angle
	// output: Motor speed
	float output = this->angleControl(this->angle, targetAngle, loopTime);

	// The rotation part from the user is injected directly into the output
	speedLeftNew = output + rotationRaw;
	speedRightNew = output - rotationRaw;

	// std::cout << "Controller:";
	std::cout << "a: " << targetAngle;
	std::cout << " o: " << output;
	std::cout << std::endl;
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
