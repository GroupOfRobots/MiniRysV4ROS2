#ifndef _MOTORS_CONTROLLER_HPP
#define _MOTORS_CONTROLLER_HPP

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>
#include <ratio>

#define ANGLE_MAX 15
#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.295779f
#define SPEED_TO_DEG 1200.0f

#define MAX_ACCELERATION 1.0f
#define MAX_MOTOR_SPEED 300000
#define DEVICE_NAME "/dev/rpmsg_pru31"

class MotorsController {
	private:
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
		std::ofstream motorPruFile;
		std::mutex fileAccessMutex;

		void writePRUDataFrame(const MotorsController::DataFrame & frame);
	public:
		MotorsController();
		~MotorsController();
		void init();
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
		float getPIDAngularVelocityFactor();
		bool getPIDSpeedRegulatorEnabled();
		void getPIDParameters(float & speedKp, float & speedKi, float & speedKd, float & angleKp, float & angleKi, float & angleKd);
		void getLQRParameters(float & linearVelocityK, float & angularVelocityK, float & angleK);

		void calculateSpeeds(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
		void calculateSpeedsPID(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
		void calculateSpeedsLQR(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);

		void enableMotors();
		void disableMotors();
		void setMotorSpeeds(float speedLeft, float speedRight, int microstep, bool ignoreAcceleration = false);
		float getMotorSpeedLeft() const;
		float getMotorSpeedRight() const;
};

#endif
