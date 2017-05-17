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

class Controller {
	private:
		std::chrono::high_resolution_clock::time_point timePointPrevious;
		std::chrono::high_resolution_clock::time_point timePoint;

		float angleFilterFactor;
		float speedFilterFactor;
		float angularVelocityFactor;

		float linearVelocityFiltered;
		bool speedRegulatorEnabled;
		float speedPIDKp;
		float speedPIDKi;
		float speedPIDKd;
		float speedPIDIntegral;
		float speedPIDError;

		float anglePrevious;
		float angleFiltered;
		float anglePIDKp;
		float anglePIDKi;
		float anglePIDKd;
		float anglePIDIntegral;
		float anglePIDError;

		float lqrAngularVelocityK;
		float lqrAngleK;

		// PI controller implementation (Proportional, integral). DT is in miliseconds
		float speedControl(float value, float setPoint, float dt);
		// PD controller implementation(Proportional, derivative). DT is in miliseconds
		float angleControl(float value, float setPoint, float dt);
	public:
		Controller();
		~Controller();
		void init();
		void setSpeedFilterFactor(float factor);
		void setAngleFilterFactor(float factor);
		void setAngularVelocityFactor(float factor);
		void setSpeedRegulatorEnabled(bool enabled);
		void setSpeedPID(float kp, float ki, float kd);
		void setAnglePID(float kp, float ki, float kd);
		void calculateSpeed(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
		void zeroPIDs();

		float getSpeedFilterFactor();
		float getAngleFilterFactor();
		float getAngularVelocityFactor();
		bool getSpeedRegulatorEnabled();
		void getSpeedPID(float & kp, float & ki, float & kd);
		void getAnglePID(float & kp, float & ki, float & kd);

		void setLQR(float angularVelocityK, float angleK);
		void calculateSpeedLQR(float angle, float rotationX, float speed, float throttle, float rotation, float &speedLeftNew, float &speedRightNew);
};

#endif
