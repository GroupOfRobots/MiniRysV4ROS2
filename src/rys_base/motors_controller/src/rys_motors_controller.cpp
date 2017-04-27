#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rys_messages/msg/imu_yaw_pitch_roll.hpp"

#include "Motors.h"
#include "Controller.h"

volatile float roll;
volatile float pitch;
volatile float yaw;

volatile int steering;
volatile int throttle;

void usleep(const int milliseconds) {
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void imuMessageCallback(const rys_messages::msg::ImuYawPitchRoll::SharedPtr message) {
	roll = message->roll * 180 / M_PI;
	pitch = message->pitch * 180 / M_PI;
	yaw = message->yaw * 180 / M_PI;
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);

	Motors motors;
	Controller controller;
	try {
		motors.initialize();
	} catch (std::string & error) {
		std::cout << "Error initializing: " << error << std::endl;
		return 1;
	}
	usleep(100 * 1000);

	std::cout << "Setup...\n";

	controller.setSpeedPID(0.03, 0.0001, 0.008);
	controller.setStabilityPID(50, 0.05, 20);
	controller.setSpeedFilterFactor(0.95);
	float finalLeftSpeed = 0;
	float finalRightSpeed = 0;

	try {
		motors.enable();
	} catch (std::string & error) {
		std::cout << "Error starting up: " << error << std::endl;
		return 2;
	}

	auto node = rclcpp::node::Node::make_shared("rys_node_motors_controller");
	auto imuSubscriber = node->create_subscription<rys_messages::msg::ImuYawPitchRoll>("rys_queue_sensor_IMU", imuMessageCallback);

	auto previous = std::chrono::high_resolution_clock::now();
	auto now = std::chrono::high_resolution_clock::now();

	rclcpp::rate::WallRate loopRate(100);
	while (rclcpp::ok()) {
		now = std::chrono::high_resolution_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(now - previous);;
		previous = now;
		float loopTime = loopTimeSpan.count();

		motors.updateOdometry(loopTime);

		// Set current position
		if (roll > 40.0 || roll < -40.0) {
			// Laying down, stand up!
			// Direction multiplier
			int multiplier = (roll > 40 ? 1 : -1);

			try {
				// Disable motors, wait 2s
				motors.setSpeed(0.0, 0.0, 1);
				motors.disable();
				usleep(2000 * 1000);

				// Enable motors
				motors.enable();
				usleep(100 * 1000);

				// Drive backwards half-speed, wait 0.5s
				motors.setSpeed(multiplier * 400.0, multiplier * 400.0, 1);
				usleep(500 * 1000);

				// Drive forward full-speed, wait until we've passed '0' point
				motors.setSpeed(-multiplier * 600.0, -multiplier * 600.0, 1);
				while (true) {
					// Passing '0' point depends on from which side we're standing up
					if ((multiplier == 1 && roll <= 0) || (multiplier == -1 && roll >= 0)) {
						std::cout << "Stood up(?), angle: " << roll << std::endl;
						break;
					}
				}

				// Zero-out PID's errors and integrals
				controller.zeroPIDs();
			} catch (std::string & error) {
				std::cout << "Error standing up from laying: " << error << std::endl;
				break;
			}
		} else {
			// Standing up, balance!

			// Calculate target speeds for motors
			controller.calculateSpeed(roll, motors.getSpeedLeft(), motors.getSpeedRight(), steering, throttle, finalLeftSpeed, finalRightSpeed, loopTime);

			// Set target speeds
			try {
				motors.setSpeed(finalLeftSpeed, finalRightSpeed, 4);
			} catch (std::string & error) {
				std::cout << "Error setting motors speed: " << error << std::endl;
				break;
			}
		}

		rclcpp::spin_some(node);
		loopRate.sleep();
	}

	// Disable motors
	try {
		motors.setSpeed(0.0, 0.0, 1);
		motors.disable();
	} catch (std::string & error) {
		std::cout << "Error disabling motors: " << error << std::endl;
		return 3;
	}
	// 1s
	usleep(1000 * 1000);

	return 0;
}
