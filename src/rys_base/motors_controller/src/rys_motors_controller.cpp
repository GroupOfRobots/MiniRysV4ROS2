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
volatile float rollPrevious;
volatile float pitch;
volatile float yaw;

volatile int steering;
volatile int throttle;

void msleep(const int milliseconds) {
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void imuMessageCallback(const rys_messages::msg::ImuYawPitchRoll::SharedPtr message) {
	// std::cout << "Received: " << message->roll << std::endl;
	rollPrevious = roll;

	roll = message->roll * 180 / M_PI;
	pitch = message->pitch * 180 / M_PI;
	yaw = message->yaw * 180 / M_PI;
}

void dataReceiveThreadFn(std::shared_ptr<rclcpp::node::Node> node) {
	auto imuSubscriber = node->create_subscription<rys_messages::msg::ImuYawPitchRoll>("rys_imu", imuMessageCallback, rmw_qos_profile_sensor_data);

	rclcpp::spin(node);
}

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	Motors motors;
	Controller controller;

	std::cout << "Initializing motors...\n";
	try {
		motors.initialize();
	} catch (std::string & error) {
		std::cout << "Error initializing: " << error << std::endl;
		return 1;
	}
	msleep(100);

	std::cout << "Setting up controller...\n";

	controller.setSpeedPID(0.03, 0.0001, 0.008);
	controller.setStabilityPID(50, 0.05, 20);
	controller.setSpeedFilterFactor(0.95);
	float finalLeftSpeed = 0;
	float finalRightSpeed = 0;

	std::cout << "Starting motors...\n";

	try {
		motors.enable();
	} catch (std::string & error) {
		std::cout << "Error starting up: " << error << std::endl;
		return 2;
	}

	auto node = rclcpp::node::Node::make_shared("rys_node_motors_controller");
	std::thread dataReceiveThread(dataReceiveThreadFn, node);

	std::cout << "Running!\n";
	auto previous = std::chrono::high_resolution_clock::now();
	auto now = std::chrono::high_resolution_clock::now();

	rclcpp::rate::WallRate loopRate(100);
	while (rclcpp::ok()) {
		now = std::chrono::high_resolution_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(now - previous);;
		previous = now;
		float loopTime = loopTimeSpan.count();

		// motors.updateOdometry(loopTime);
		std::cout << "Running, time: " << loopTime << ", roll: " << roll << std::endl;

		// Set current position
		if ((roll > 40.0 && rollPrevious > 40.0) || (roll < -40.0 && rollPrevious < -40.0)) {
			// Laying down, stand up!
			std::cout << "Laying down, trying to stand up\n";

			// Direction multiplier
			int multiplier = (roll > 40 ? 1 : -1);

			try {
				// Disable motors, wait 2s
				motors.setSpeed(0.0, 0.0, 1);
				motors.disable();
				msleep(2000);

				// Enable motors
				motors.enable();
				msleep(100);

				// Drive backwards half-speed, wait 0.5s
				motors.setSpeed(multiplier * 400.0, multiplier * 400.0, 1);
				msleep(500);

				// Drive forward full-speed, wait until we've passed '0' point
				motors.setSpeed(-multiplier * 600.0, -multiplier * 600.0, 1);

				rclcpp::rate::WallRate standUpLoopRate(200);
				while (rclcpp::ok()) {
					std::cout << "Standing up, angle: " << roll << std::endl;
					// Passing '0' point depends on from which side we're standing up
					// if (roll > -3 && roll < 3) {
					if ((multiplier == 1 && roll <= 0) || (multiplier == -1 && roll >= 0)) {
						break;
					}
					// rclcpp::spin_some(node);
					standUpLoopRate.sleep();
				}

				std::cout << "Stood up(?), angle: " << roll << std::endl;

				// Zero-out PID's errors and integrals, zero-out loop timer
				controller.zeroPIDs();
				previous = std::chrono::high_resolution_clock::now();
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

		// rclcpp::spin_some(node);
		loopRate.sleep();
	}

	dataReceiveThread.join();

	std::cout << "Quitting, disabling motors\n";
	// Disable motors
	try {
		motors.setSpeed(0.0, 0.0, 1);
		motors.disable();
	} catch (std::string & error) {
		std::cout << "Error disabling motors: " << error << std::endl;
		return 3;
	}
	// 0.5s
	msleep(500);

	return 0;
}
