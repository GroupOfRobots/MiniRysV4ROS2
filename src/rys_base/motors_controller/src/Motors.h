#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <cstdint>
#include <fstream>

#define MAX_BUFFER_SIZE 512
#define MAX_ACCELERATION 2000
#define MAX_MOTOR_SPEED 3000
#define MIN_MOTOR_SPEED 4000000
#define MAX_USER_SPEED 800
#define MIN_USER_SPEED 0
#define DEVICE_NAME "/dev/rpmsg_pru31"

struct DataFrame {
	unsigned int speedLeft;
	unsigned int speedRight;
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
		float minMotorSpeed;
		long distance;
		std::ofstream pruFile;
	public:
		Motors();
		~Motors();
		void initialize();
		void enable();
		void disable();
		void updateOdometry(float);
		//speed from -1000 to 1000
		void setSpeed(float, float, int);
		float getDistance();
		void resetDistance();
		float getSpeedLeft();
		float getSpeedRight();
};

#endif
