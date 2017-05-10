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

		float speedFilterFactor;

		float angle;
		float anglePrevious;
		float anglePIDKp;
		float anglePIDKi;
		float anglePIDKd;
		float anglePIDIntegral;
		float anglePIDError;

		float speed;
		float speedPrevious;
		float speedFiltered;
		float speedPIDKp;
		float speedPIDKi;
		float speedPIDKd;
		float speedPIDIntegral;
		float speedPIDError;

		// PI controller implementation (Proportional, integral). DT is in miliseconds
		float speedControl(float value, float setPoint, float dt);
		// PD controller implementation(Proportional, derivative). DT is in miliseconds
		float angleControl(float value, float setPoint, float dt);
	public:
		Controller();
		~Controller();
		void init();
		void setSpeedFilterFactor(float factor);
		void setSpeedPID(float kp, float ki, float kd);
		void setAnglePID(float kp, float ki, float kd);
		void calculateSpeed(float angle, float speedLeft, float speedRight, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
		void zeroPIDs();
};

#endif
