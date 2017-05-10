#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rys_messages/msg/imu_roll.hpp"
#include "rys_messages/msg/pid_settings.hpp"
#include "std_msgs/msg/bool.hpp"

#include "Motors.h"
#include "Controller.h"

volatile bool enabled;
volatile int enableTimeout = 5000;
std::chrono::time_point<std::chrono::high_resolution_clock> enableTimerEnd;

volatile float roll;
volatile float rollPrevious;
volatile float pitch;
volatile float yaw;

volatile int steering;
volatile int throttle;

Motors motors;
Controller controller;

void msleep(const int milliseconds) {
	auto end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(milliseconds);
	while (rclcpp::ok() && std::chrono::high_resolution_clock::now() < end) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void enableMessageCallback(const std_msgs::msg::Bool::SharedPtr message) {
	if (message->data) {
		enableTimerEnd = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(enableTimeout);
		if (!enabled) {
			std::cout << "Enabling motors...\n";
			motors.enable();
		}
	} else {
		if (enabled) {
			std::cout << "Disabling motors...\n";
		}
		motors.disable();
	}
	enabled = message->data;
}

void imuMessageCallback(const rys_messages::msg::ImuRoll::SharedPtr message) {
	rollPrevious = roll;
	roll = message->roll * 180 / M_PI;
}

void setPIDsMessageCallback(const rys_messages::msg::PIDSettings::SharedPtr message) {
	controller.setSpeedPID(message->speed_p, message->speed_i, message->speed_d);
	controller.setAnglePID(message->angle_p, message->angle_i, message->angle_d);

	std::cout << "Setting PIDs:\n";
	std::cout << "\tSpeed->angle: " << message->speed_p << message->speed_i << message->speed_d << std::endl;
	std::cout << "\tAngle->output: " << message->angle_p << message->angle_i << message->angle_d << std::endl;
}

void dataReceiveThreadFn(std::shared_ptr<rclcpp::node::Node> node) {
	auto enableSubscriber = node->create_subscription<std_msgs::msg::Bool>("rys_enable", enableMessageCallback, rmw_qos_profile_sensor_data);
	auto imuSubscriber = node->create_subscription<rys_messages::msg::ImuRoll>("rys_imu", imuMessageCallback, rmw_qos_profile_sensor_data);
	auto setPIDsSubscriber = node->create_subscription<rys_messages::msg::PIDSettings>("rys_set_pids", setPIDsMessageCallback);

	rclcpp::spin(node);
}

void standUp() {
	// Direction multiplier
	int multiplier = (roll > 40 ? 1 : -1);

	// Disable motors, wait 2s
	motors.setSpeed(0.0, 0.0, 1);
	motors.disable();
	msleep(2000);

	if (!rclcpp::ok()) {
		return;
	}

	// Enable motors
	motors.enable();
	msleep(100);

	if (!rclcpp::ok()) {
		return;
	}

	// Drive backwards half-speed, wait 0.5s
	motors.setSpeed(multiplier * 400.0, multiplier * 400.0, 1);
	msleep(500);

	if (!rclcpp::ok()) {
		return;
	}

	// Drive forward full-speed, wait until we've passed '0' point
	motors.setSpeed(-multiplier * 600.0, -multiplier * 600.0, 1);

	rclcpp::rate::WallRate standUpLoopRate(100);
	while (rclcpp::ok()) {
		// std::cout << "Standing up, angle: " << roll << std::endl;
		// Passing '0' point depends on from which side we're standing up
		if ((multiplier == 1 && roll <= 0) || (multiplier == -1 && roll >= 0)) {
			break;
		}
		standUpLoopRate.sleep();
	}

	std::cout << "Stood up(?), angle: " << roll << std::endl;
}

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	std::cout << "Initializing motors...\n";
	try {
		motors.initialize();
	} catch (std::string & error) {
		std::cout << "Error initializing: " << error << std::endl;
		return 1;
	}
	msleep(100);

	std::cout << "Disabling motors...\n";

	try {
		motors.disable();
	} catch (std::string & error) {
		std::cout << "Error starting up: " << error << std::endl;
		return 2;
	}

	auto node = rclcpp::node::Node::make_shared("rys_node_motors_controller");
	std::thread dataReceiveThread(dataReceiveThreadFn, node);

	std::cout << "Setting up controller...\n";

	controller.init();
	controller.setSpeedPID(0.03, 0.0001, 0.008);
	controller.setAnglePID(50, 0.05, 20);

	controller.setSpeedFilterFactor(0.99);
	float finalLeftSpeed = 0;
	float finalRightSpeed = 0;

	std::cout << "Running!\n";
	auto previous = std::chrono::high_resolution_clock::now();
	auto now = std::chrono::high_resolution_clock::now();

	rclcpp::rate::WallRate loopRate(100);
	while (rclcpp::ok()) {
		now = std::chrono::high_resolution_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(now - previous);;
		previous = now;
		float loopTime = loopTimeSpan.count();

		if (!enabled) {
			motors.disable();
			loopRate.sleep();
			continue;
		}

		// Check enable timer
		if (enableTimerEnd < now) {
			enabled = false;
			motors.disable();
			loopRate.sleep();
			continue;
		}

		// Update "odometry"
		// motors.updateOdometry(loopTime);

		// Detect current position
		if ((roll > 40.0 && rollPrevious > 40.0) || (roll < -40.0 && rollPrevious < -40.0)) {
			// Laying down, stand up!
			std::cout << "Laying down, trying to stand up\n";

			try {
				standUp();

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
