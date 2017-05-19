#ifndef _CONTROLLER_INCLUDED_
#define _CONTROLLER_INCLUDED_

#include <ctime>
#include <ratio>
#include <chrono>
#include <cstdio>

#define THROTTLE_MAX 580
#define ROTATION_MAX 150
#define ANGLE_MAX 15
#define SPEED_MAX 900
#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.295779f
#define SPEED_TO_DEG 22.5f

class Controller {
	private:
		std::chrono::high_resolution_clock::time_point timePointPrevious;
		std::chrono::high_resolution_clock::time_point timePoint;

		bool balancing;
		bool lqrEnabled;

		float speedFilterFactor;
		float angleFilterFactor;
		float angleFiltered;

		bool pidSpeedRegulatorEnabled;
		float pidAngularVelocityFactor;
		float pidLinearVelocityFiltered;
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
	public:
		Controller();
		~Controller();
		void init();
		void setBalancing(bool value);
		void setLQREnabled(bool value);
		void setSpeedFilterFactor(float factor);
		void setAngleFilterFactor(float factor);
		void setPIDAngularVelocityFactor(float factor);
		void setPIDSpeedRegulatorEnabled(bool enabled);
		void setPIDParameters(float speedKp, float speedKi, float speedKd, float angleKp, float angleKi, float kangleK);
		void setLQRParameters(float linearVelocityK, float angularVelocityK, float angleK);
		void zeroPIDs();
		float getSpeedFilterFactor();
		float getAngleFilterFactor();
		bool getLQREnabled();
		float getPIDAngularVelocityFactor();
		bool getPIDSpeedRegulatorEnabled();
		void getPIDParameters(float & speedKp, float & speedKi, float & speedKd, float & angleKp, float & angleKi, float & angleKd);
		void getLQRParameters(float & linearVelocityK, float & angularVelocityK, float & angleK);

		void calculateSpeeds(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
		void calculateSpeedsPID(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
		void calculateSpeedsLQR(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew);
};

#endif
