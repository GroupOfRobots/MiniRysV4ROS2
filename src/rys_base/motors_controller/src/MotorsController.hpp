#ifndef _MOTORS_CONTROLLER_HPP
#define _MOTORS_CONTROLLER_HPP

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>

#define ANGLE_MAX 15
#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.295779f
#define SPEED_TO_DEG 1200.0f

#define MAX_ACCELERATION 1.0f
// Note: MAX_MOTOR_SPEED is in fact the MINIMUM delay (in PRU ticks) between steps, so to increase the real max speed decrease this constant.
/// TODO: Move this delay/PRU_CLOCK/etc logic into PRU driver and make this controller operate on rev/s.
#define MAX_MOTOR_SPEED 400000.0f
#define PRU_CLOCK 200.0f * 1000.0f * 1000.0f
#define STEPPER_STEPS_PER_REVOLUTION 200.0f
// A 'magic number' that makes the speed calculations right. Required probably because by a bug in PRU firmware.
#define SPEED_MULTIPLIER 0.5f
#define DEVICE_NAME "/dev/rpmsg_pru31"

class MotorsController {
	private:
		/**
		* PRU communication frame.
		*/
		struct DataFrame {
			uint8_t enabled;
			uint8_t microstep;
			uint8_t directionLeft;
			uint8_t directionRight;
			uint32_t speedLeft;
			uint32_t speedRight;
		};

		std::chrono::high_resolution_clock::time_point timePointPrevious;
		std::chrono::high_resolution_clock::time_point timePoint;

		bool balancing;
		bool lqrEnabled;

		float speedFilterFactor;
		float speedFiltered;
		float angleFilterFactor;
		float angleFiltered;

		bool pidSpeedRegulatorEnabled;
		float pidSpeedKp;
		float pidSpeedKi;
		float pidSpeedKd;
		float pidSpeedIntegral;
		float pidSpeedError;
		float pidAngleKp;
		float pidAngleKi;
		float pidAngleKd;
		float pidAngleIntegral;
		float pidAngleError;

		float lqrLinearVelocityK;
		float lqrAngularVelocityK;
		float lqrAngleK;

		bool motorsEnabled;
		float motorSpeedLeft;
		float motorSpeedRight;
		bool invertLeftSpeed;
		bool invertRightSpeed;
		bool motorsSwapped;
		std::ofstream motorPruFile;
		std::mutex fileAccessMutex;

		void writePRUDataFrame(const MotorsController::DataFrame & frame);

		void calculateSpeedsPID(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
		void calculateSpeedsLQR(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
	public:
		MotorsController();
		~MotorsController();

		void setInvertSpeed(const bool left, const bool right);
		void setMotorsSwapped(const bool motorsSwapped);
		void setBalancing(bool value);
		void setLQREnabled(bool value);
		void setSpeedFilterFactor(float factor);
		void setAngleFilterFactor(float factor);
		void setPIDSpeedRegulatorEnabled(bool enabled);
		void setPIDParameters(float speedKp, float speedKi, float speedKd, float angleKp, float angleKi, float kangleK);
		void setLQRParameters(float linearVelocityK, float angularVelocityK, float angleK);
		void zeroRegulators();
		float getSpeedFilterFactor();
		float getAngleFilterFactor();
		bool getLQREnabled();
		bool getPIDSpeedRegulatorEnabled();
		void getPIDParameters(float & speedKp, float & speedKi, float & speedKd, float & angleKp, float & angleKi, float & angleKd);
		void getLQRParameters(float & linearVelocityK, float & angularVelocityK, float & angleK);

		void calculateSpeeds(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);

		void enableMotors();
		void disableMotors();

		/**
		* Clip, limit, process and pass speeds to the PRU.
		* \param speedLeft - target speed of left motor, restricted to <-1; 1> range
		* \param speedRight - target speed of right motor, restricted to <-1; 1> range
		* \param microstep - microstepping to use, valid values are powers of 2 from 1 to 32
		* \param ignoreAcceleration=false - whether to ignore acceleration, defaults to false
		*/
		void setMotorSpeeds(float speedLeft, float speedRight, int microstep, bool ignoreAcceleration = false);
		/**
		* Retrieve last speed of left motor sent to the PRU (after clipping, applying acceleration limits).
		* \return Raw speed of left wheel
		*/
		float getMotorSpeedLeftRaw() const;
		/**
		* Retrieve last speed of right motor sent to the PRU.
		* \return Raw speed of right wheel
		* \sa getMotorSpeedLeftRaw()
		*/
		float getMotorSpeedRightRaw() const;
		/**
		* Retrieve speed of left wheel in revolutions per second.
		* \return Speed of left wheel
		* Maths explained:
		* PRU_CLOCK/MAX_MOTOR_SPEED -> max_steps_per_second
		* max_steps_per_second / STEPPER_STEPS_PER_REVOLUTION -> max_revolutions_per_second
		* max_revolutions_per_second * motorSpeedLeft -> revolutions_per_second
		*/
		float getMotorSpeedLeft() const;
		/**
		* Retrieve speed of right wheel in revolutions per second.
		* \return Speed of right wheel
		* \sa getMotorSpeedLeft()
		*/
		float getMotorSpeedRight() const;
};

#endif
