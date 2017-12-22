#include "MotorsController.hpp"
#include <cmath>
#include <stdexcept>

MotorsController::MotorsController() {
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

	lqrLinearVelocityK = 0;
	lqrAngularVelocityK = 0;
	lqrAngleK = 0;

	motorsEnabled = false;
	motorSpeedLeft = 0;
	motorSpeedRight = 0;
	invertLeft = false;
	invertRight = false;
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

void MotorsController::init() {
	this->timePoint = std::chrono::high_resolution_clock::now();
	this->timePointPrevious = std::chrono::high_resolution_clock::now();

	this->disableMotors();
}

void MotorsController::setInverting(const bool invertLeft, const bool invertRight) {
	this->invertLeft = invertLeft;
	this->invertRight = invertRight;
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

void MotorsController::setPIDParameters(float speedKp, float speedKi, float speedKd, float angleKp, float angleKi, float angleKd) {
	this->pidSpeedKp = speedKp;
	this->pidSpeedKi = speedKi;
	this->pidSpeedKd = speedKd;
	this->pidAngleKp = angleKp;
	this->pidAngleKi = angleKi;
	this->pidAngleKd = angleKd;
}

void MotorsController::setLQRParameters(float linearVelocityK, float angularVelocityK, float angleK) {
	this->lqrLinearVelocityK = linearVelocityK;
	this->lqrAngularVelocityK = angularVelocityK;
	this->lqrAngleK = angleK;
}

void MotorsController::zeroRegulators() {
	this->pidAngleIntegral = 0;
	this->pidAngleError = 0;
	this->pidSpeedIntegral = 0;
	this->pidSpeedError = 0;
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

void MotorsController::getPIDParameters(float & speedKp, float & speedKi, float & speedKd, float & angleKp, float & angleKi, float & angleKd) {
	speedKp = this->pidSpeedKp;
	speedKi = this->pidSpeedKi;
	speedKd = this->pidSpeedKd;
	angleKp = this->pidAngleKp;
	angleKi = this->pidAngleKi;
	angleKd = this->pidAngleKd;
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
	// To regulate angle we need to reverse it's sign, because the more positive it is the more speed (acceleration) we should output
	// angle = -angle;
	rotationX = -rotationX;

	// Calculate target angle - first initialize it to 0
	float targetAngle = 0.0f;
	// If speed regulator (first PID layer) is enabled
	if (this->pidSpeedRegulatorEnabled) {
		// Estimate robot's linear velocity based on angle change and speed
		// (Motors' angular velocity = -robot's angular velocity + robot's linear velocity * const)
		// What's left is motor's angular velocity responsible for robot's linear velocity
		float linearVelocity = speed - rotationX / SPEED_TO_DEG;

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
	if (this->invertLeft) {
		speedLeft = -speedLeft;
	}
	if (this->invertRight) {
		speedRight = -speedRight;
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
	return this->invertLeft ? -this->motorSpeedLeft : this->motorSpeedLeft;
}

float MotorsController::getMotorSpeedRightRaw() const {
	return this->invertRight ? -this->motorSpeedRight : this->motorSpeedRight;
}

float MotorsController::getMotorSpeedLeft() const {
	if (!this->motorsEnabled || this->motorSpeedLeft == 0) {
		return 0.0f;
	}

	return this->motorSpeedLeft * (this->invertLeft ? -1 : 1) * PRU_CLOCK/(STEPPER_STEPS_PER_REVOLUTION * MAX_MOTOR_SPEED) * SPEED_MULTIPLIER;
}

float MotorsController::getMotorSpeedRight() const {
	if (!this->motorsEnabled || this->motorSpeedRight == 0) {
		return 0.0f;
	}

	return this->motorSpeedRight * (this->invertRight ? -1 : 1) * PRU_CLOCK/(STEPPER_STEPS_PER_REVOLUTION * MAX_MOTOR_SPEED) * SPEED_MULTIPLIER;
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
