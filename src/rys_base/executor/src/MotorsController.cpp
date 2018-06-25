#include "MotorsController.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

MotorsController::MotorsController() {
	this->balancing = true;
	this->lqrEnabled = true;

	this->speedFilterFactor = 1.0f;
	this->speedFiltered = 0.0f;
	this->angleFilterFactor = 1.0f;
	this->angleFiltered = 0.0f;

	this->pidSpeedRegulatorEnabled = true;
	this->pidSpeedKp = 0;
	this->pidSpeedInvTi = 0;
	this->pidSpeedTd = 0;
	this->pidSpeedPreviousError1 = 0;
	this->pidSpeedPreviousError2 = 0;
	this->pidAngleKp = 0;
	this->pidAngleInvTi = 0;
	this->pidAngleTd = 0;
	this->pidAnglePreviousError1 = 0;
	this->pidAnglePreviousError2 = 0;
	this->pidPreviousTargetAngle = 0;

	this->lqrLinearVelocityK = 0;
	this->lqrAngularVelocityK = 0;
	this->lqrAngleK = 0;

	this->motorsEnabled = false;
	this->motorSpeedLeft = 0;
	this->motorSpeedRight = 0;
	this->invertLeftSpeed = false;
	this->invertRightSpeed = false;
	this->motorsSwapped = false;

	this->timePoint = std::chrono::high_resolution_clock::now();
	this->timePointPrevious = std::chrono::high_resolution_clock::now();

	this->disableMotors();
}

MotorsController::~MotorsController() {
	this->disableMotors();
	this->motorPruFile.close();
}

void clipValue(float & value, float max) {
	if (value > max) {
		value = max;
	} else if (value < -max) {
		value = -max;
	}
}

void MotorsController::setInvertSpeed(const bool left, const bool right) {
	this->invertLeftSpeed = left;
	this->invertRightSpeed = right;
}

void MotorsController::setMotorsSwapped(const bool motorsSwapped) {
	this->motorsSwapped = motorsSwapped;
}

void MotorsController::setBalancing(bool value) {
	this->balancing = value;
}

void MotorsController::setLQREnabled(bool value) {
	this->lqrEnabled = value;
}

void MotorsController::setSpeedFilterFactor(float factor) {
	this->speedFilterFactor = factor;
}

void MotorsController::setAngleFilterFactor(float factor) {
	this->angleFilterFactor = factor;
}

void MotorsController::setPIDSpeedRegulatorEnabled(bool enabled) {
	this->pidSpeedRegulatorEnabled = enabled;
}

void MotorsController::setPIDParameters(float speedKp, float speedInvTi, float speedTd, float angleKp, float angleInvTi, float angleTd) {
	this->pidSpeedKp = speedKp;
	this->pidSpeedInvTi = speedInvTi;
	this->pidSpeedTd = speedTd;
	this->pidAngleKp = angleKp;
	this->pidAngleInvTi = angleInvTi;
	this->pidAngleTd = angleTd;
}

void MotorsController::setLQRParameters(float linearVelocityK, float angularVelocityK, float angleK) {
	this->lqrLinearVelocityK = linearVelocityK;
	this->lqrAngularVelocityK = angularVelocityK;
	this->lqrAngleK = angleK;
}

void MotorsController::zeroPIDRegulator(){
	this->pidSpeedPreviousError1 = 0;
	this->pidSpeedPreviousError2 = 0;
	this->pidAnglePreviousError1 = 0;
	this->pidAnglePreviousError2 = 0;
}

float MotorsController::getSpeedFilterFactor() {
	return this->speedFilterFactor;
}

float MotorsController::getAngleFilterFactor() {
	return this->angleFilterFactor;
}

bool MotorsController::getLQREnabled() {
	return this->lqrEnabled;
}

bool MotorsController::getPIDSpeedRegulatorEnabled() {
	return this->pidSpeedRegulatorEnabled;
}

void MotorsController::getPIDParameters(float & speedKp, float & speedInvTi, float & speedTd, float & angleKp, float & angleInvTi, float & angleTd) {
	speedKp = this->pidSpeedKp;
	speedInvTi = this->pidSpeedInvTi;
	speedTd = this->pidSpeedTd;
	angleKp = this->pidAngleKp;
	angleInvTi = this->pidAngleInvTi;
	angleTd = this->pidAngleTd;
}

void MotorsController::getLQRParameters(float & linearVelocityK, float & angularVelocityK, float & angleK) {
	linearVelocityK = this->lqrLinearVelocityK;
	angularVelocityK = this->lqrAngularVelocityK;
	angleK = this->lqrAngleK;
}

void MotorsController::calculateSpeeds(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
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

void MotorsController::calculateSpeedsPID(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	float targetAngle = 0.0f;
	float speedError = 0.0f;

	// if (this->pidSpeedRegulatorEnabled && throttle!=0) {
	if (this->pidSpeedRegulatorEnabled) {
		speedError = throttle - (speed - rotationX/SPEED_TO_DEG);

		float speedFactor0 = this->pidSpeedKp * (1 + this->pidSpeedInvTi * loopTime / 2 + this->pidSpeedTd / loopTime);
		float speedFactor1 = this->pidSpeedKp * (this->pidSpeedInvTi * loopTime / 2 - 2 * this->pidSpeedTd / loopTime - 1);
		float speedFactor2 = this->pidSpeedKp * this->pidSpeedTd / loopTime;

		targetAngle = speedFactor0 * speedError + speedFactor1 * this->pidSpeedPreviousError1 + speedFactor2 * this->pidSpeedPreviousError2 + this->pidPreviousTargetAngle;
		clipValue(targetAngle, 0.15);
		// std::cout << speedError << " : " << targetAngle << std::endl;
		// std::cout << targetAngle << std::endl;
	}
	this->pidSpeedPreviousError2 = this->pidSpeedPreviousError1;
	this->pidSpeedPreviousError1 = speedError;
	this->pidPreviousTargetAngle = targetAngle;

	float angleError = targetAngle - angle;

	float angleFactor0 = this->pidAngleKp * (1 + this->pidAngleInvTi * loopTime / 2 + this->pidAngleTd / loopTime);
	float angleFactor1 = this->pidAngleKp * (this->pidAngleInvTi * loopTime / 2 - 2 * this->pidAngleTd / loopTime - 1);
	float angleFactor2 = this->pidAngleKp * this->pidAngleTd / loopTime;

	float output = angleFactor0 * angleError + angleFactor1 * this->pidAnglePreviousError1 + angleFactor2 * this->pidAnglePreviousError2 + speed;
	clipValue(output, 1.0);
	clipValue(rotation, 0.3);

	speedLeftNew = output + rotation;
	speedRightNew = output - rotation;
	clipValue(speedLeftNew, 1.0);
	clipValue(speedRightNew, 1.0);

	this->pidAnglePreviousError2 = this->pidAnglePreviousError1;
	this->pidAnglePreviousError1 = angleError;
}

void MotorsController::calculateSpeedsLQR(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
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
}

void MotorsController::enableMotors() {
	this->motorsEnabled = true;
}

void MotorsController::disableMotors() {
	this->motorsEnabled = false;
	this->motorSpeedLeft = 0;
	this->motorSpeedRight = 0;

	MotorsController::DataFrame dataFrame;
	dataFrame.enabled = 0;
	dataFrame.microstep = 1;
	dataFrame.directionLeft = 0;
	dataFrame.directionRight = 0;
	dataFrame.speedLeft = 0;
	dataFrame.speedRight = 0;

	this->writePRUDataFrame(dataFrame);
}

void MotorsController::setMotorSpeeds(float speedLeft, float speedRight, int microstep, bool ignoreAcceleration) {
	// Validate microstep value
	if (microstep != 1 && (microstep == 0 || microstep % 2 || microstep > 32)) {
		throw(std::domain_error("Bad microstep value! Allowed ones are powers of 2 from 1 to 32"));
	}

	// Create data frame for PRU
	MotorsController::DataFrame dataFrame;

	// Clip speed values
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

	// If needed, invert the speeds
	if (this->invertLeftSpeed) {
		speedLeft = -speedLeft;
	}
	if (this->invertRightSpeed) {
		speedRight = -speedRight;
	}

	if (this->motorsSwapped) {
		float tmp = speedLeft;
		speedLeft = speedRight;
		speedRight = tmp;
	}

	// Set whether the motors are to be enabled
	dataFrame.enabled = this->motorsEnabled;

	// Save microstep value
	dataFrame.microstep = microstep;

	// Calculate max speeds for given microstep value
	int32_t maxMotorSpeed = MAX_MOTOR_SPEED / microstep;

	// Clip speeds according to acceleration limits
	if (ignoreAcceleration) {
		this->motorSpeedLeft = speedLeft;
		this->motorSpeedRight = speedRight;
	} else {
		if (speedLeft > (this->motorSpeedLeft + MAX_ACCELERATION)) {
			this->motorSpeedLeft = this->motorSpeedLeft + MAX_ACCELERATION;
		} else if (speedLeft < (this->motorSpeedLeft - MAX_ACCELERATION)) {
			this->motorSpeedLeft = this->motorSpeedLeft - MAX_ACCELERATION;
		} else {
			this->motorSpeedLeft = speedLeft;
		}
		if (speedRight > (this->motorSpeedRight + MAX_ACCELERATION)) {
			this->motorSpeedRight = this->motorSpeedRight + MAX_ACCELERATION;
		} else if (speedRight < (this->motorSpeedRight - MAX_ACCELERATION)) {
			this->motorSpeedRight = this->motorSpeedRight - MAX_ACCELERATION;
		} else {
			this->motorSpeedRight = speedRight;
		}
	}

	// Write directions
	dataFrame.directionLeft = (this->motorSpeedLeft >= 0 ? 0 : 1);
	dataFrame.directionRight = (this->motorSpeedRight >= 0 ? 0 : 1);
	// Calculate and write speeds to data frame
	if (this->motorSpeedLeft == 0) {
		dataFrame.speedLeft = 0;
	} else {
		dataFrame.speedLeft = maxMotorSpeed / std::abs(this->motorSpeedLeft);
	}
	if (this->motorSpeedRight == 0) {
		dataFrame.speedRight = 0;
	} else {
		dataFrame.speedRight = maxMotorSpeed / std::abs(this->motorSpeedRight);
	}

	// Write data frame to file (PRU communication)
	this->writePRUDataFrame(dataFrame);
}

float MotorsController::getMotorSpeedLeftRaw() const {
	float inversionMultiplier = this->invertLeftSpeed ? -1 : 1;
	if (!this->motorsSwapped) {
		return this->motorSpeedLeft * inversionMultiplier;
	} else {
		return this->motorSpeedRight * inversionMultiplier;
	}
}

float MotorsController::getMotorSpeedRightRaw() const {
	float inversionMultiplier = this->invertRightSpeed ? -1 : 1;
	if (!this->motorsSwapped) {
		return this->motorSpeedRight * inversionMultiplier;
	} else {
		return this->motorSpeedLeft * inversionMultiplier;
	}
}

float MotorsController::getMotorSpeedLeft() const {
	float rawSpeed = getMotorSpeedLeftRaw();
	if (!this->motorsEnabled || rawSpeed == 0) {
		return 0.0f;
	}

	return rawSpeed * PRU_CLOCK/(STEPPER_STEPS_PER_REVOLUTION * MAX_MOTOR_SPEED) * SPEED_MULTIPLIER;
}

float MotorsController::getMotorSpeedRight() const {
	float rawSpeed = getMotorSpeedRightRaw();
	if (!this->motorsEnabled || rawSpeed == 0) {
		return 0.0f;
	}

	return rawSpeed * PRU_CLOCK/(STEPPER_STEPS_PER_REVOLUTION * MAX_MOTOR_SPEED) * SPEED_MULTIPLIER;
}

void MotorsController::writePRUDataFrame(const MotorsController::DataFrame & frame) {
	std::lock_guard<std::mutex> guard(this->fileAccessMutex);

	// We have to open/close this file each time or it won't "apply" to PRU
	this->motorPruFile.open(DEVICE_NAME, std::ofstream::out | std::ofstream::binary);
	if (!this->motorPruFile.is_open() || !this->motorPruFile.good()) {
		this->motorPruFile.close();
		std::string errorString("Failed writing to file: ");
		errorString += std::string(DEVICE_NAME);
		throw(std::runtime_error(errorString));
	} else {
		this->motorPruFile.write((char*)(&frame), (int)(sizeof(MotorsController::DataFrame)));
	}
	this->motorPruFile.close();
}
