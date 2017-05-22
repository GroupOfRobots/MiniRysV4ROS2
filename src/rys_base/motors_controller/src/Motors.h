#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <cstdint>
#include <fstream>
#include <mutex>

#define MAX_ACCELERATION 0.1f
#define MAX_MOTOR_SPEED 400000
#define DEVICE_NAME "/dev/rpmsg_pru31"

struct DataFrame {
	uint32_t speedLeft;
	uint32_t speedRight;
	uint8_t directionLeft;
	uint8_t directionRight;
	uint8_t microstep;
};

class Motors {
	private:
		float speedLeft;
		float speedRight;
		uint8_t microstep;
		float maxMotorSpeed;
		long distance;
		std::ofstream pruFile;
		std::mutex fileAccessMutex;
	public:
		Motors();
		~Motors();
		void initialize();
		void enable();
		void disable();
		void updateOdometry(float);
		// speed from -1.0f to 1.0f
		void setSpeed(float, float, int);
		float getDistance();
		void resetDistance();
		float getSpeedLeft();
		float getSpeedRight();
};

#endif
