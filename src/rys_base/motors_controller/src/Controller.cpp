#include "Controller.h"
#include <iostream>

Controller::Controller() {
	timePoint = std::chrono::high_resolution_clock::now();
	timePointPrevious = std::chrono::high_resolution_clock::now();
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

float Controller::stabilityControl(float value, float setPoint, float dt) {
	float error = setPoint - value;
	/*if (error > ANGLE_MAX_ERROR) {
		error = ANGLE_MAX_ERROR;
	} else if (error < -ANGLE_MAX_ERROR) {
		error = -ANGLE_MAX_ERROR;
	}*/

	this->anglePIDIntegral += error * dt;
	// Max integral value clipping
	if (this->anglePIDIntegral > ANGLE_MAX_INTEGRAL) {
		this->anglePIDIntegral = ANGLE_MAX_INTEGRAL;
	} else if (this->anglePIDIntegral < -ANGLE_MAX_INTEGRAL) {
		this->anglePIDIntegral = -ANGLE_MAX_INTEGRAL;
	}

	float derivative = (error - this->anglePIDError) / dt;
	this->anglePIDError = error;

	return (anglePIDKp * error + anglePIDKi * this->anglePIDIntegral + anglePIDKd * derivative);
}

float Controller::speedControl(float value, float setPoint, float dt) {
	// Max error value clipping
	float error = setPoint - value;
	/*if (error > SPEED_MAX_ERROR) {
		error = SPEED_MAX_ERROR;
	} else if (error < -SPEED_MAX_ERROR) {
		error = -SPEED_MAX_ERROR;
	}*/

	this->speedPIDIntegral += error * dt;

	// Max integral value clipping
	if (this->speedPIDIntegral > SPEED_MAX_INTEGRAL) {
		this->speedPIDIntegral = SPEED_MAX_INTEGRAL;
	} else if (this->speedPIDIntegral < -SPEED_MAX_INTEGRAL) {
		this->speedPIDIntegral = -SPEED_MAX_INTEGRAL;
	}

	float derivative = (error - this->speedPIDError) / dt;
	this->speedPIDError = error;

	return (speedPIDKp * error + speedPIDKi * this->speedPIDIntegral + speedPIDKd * derivative);
}

void Controller::calculateSpeed(float angle, float speedLeft, float speedRight, int steering, int throttle, float &speedLeftNew, float &speedRightNew, float loopTime) {
	this->anglePrevious = this->angle;
	this->angle = angle;

	// Positive: forward
	this->speedPrevious = this->speed;
	this->speed = (speedLeft + speedRight) / 2;

	// Estimate robot's velocity based on angle change and speed
	// 90 is an empirical extracted factor to adjust for real units
	float angularVelocity = (this->angle - this->anglePrevious) * 90.0 * loopTime;
	// We use robot_speed(t-1) to compensate the delay
	float estimatedSpeed = -this->speedPrevious - angularVelocity;
	// Low pass / integrating filter on estimated speed
	this->speedFiltered = this->speedFiltered * this->speedFilterFactor + estimatedSpeed * (1.0 - this->speedFilterFactor);

	// First control layer: speed control PID
	// input: user throttle (0)
	// setPoint: estimated and filtered robot speed
	// output: target robot angle to get the desired speed
	float targetAngle = speedControl(this->speedFiltered, throttle, loopTime);
	if (targetAngle > MAX_ANGLE) {
		targetAngle = MAX_ANGLE;
	} else if (targetAngle < -MAX_ANGLE) {
		targetAngle = -MAX_ANGLE;
	}

	// Second control layer: stability (angle) control PID
	// input: robot target angle(from SPEED CONTROL)
	// variable: robot angle
	// output: Motor speed
	float output = stabilityControl(angle, targetAngle, loopTime);
	if (output > MAX_OUTPUT) {
		output = MAX_OUTPUT;
	} else if (output < -MAX_OUTPUT) {
		output = -MAX_OUTPUT;
	}

	// The steering part from the user is injected directly into the output
	speedLeftNew = output + steering;
	speedRightNew = output - steering;
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

void Controller::setStabilityPID(float kp, float ki, float kd) {
	this->anglePIDKp = kp;
	this->anglePIDKi = kd;
	this->anglePIDKd = ki;
}
