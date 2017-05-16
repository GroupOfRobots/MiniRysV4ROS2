#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rys_interfaces/msg/imu_roll_rotation.hpp"
#include "rys_interfaces/srv/set_regulator_settings.hpp"
#include "rys_interfaces/srv/get_regulator_settings.hpp"

#include "Motors.h"
#include "Controller.h"

volatile bool enabled;
volatile int enableTimeout = 5000;
std::chrono::time_point<std::chrono::high_resolution_clock> enableTimerEnd;

volatile float roll;
volatile float rollPrevious;
volatile float rotationX;

volatile int rotation;
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

void imuMessageCallback(const rys_interfaces::msg::ImuRollRotation::SharedPtr message) {
	rollPrevious = roll;
	roll = message->roll * 180 / M_PI;
	rotationX = message->rotation_x;
}

void setRegulatorSettingsCallback(
	const std::shared_ptr<rmw_request_id_t> requestHeader,
	const std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Request> request,
	std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Response> response
) {
	// Suppress unused parameter warning
	(void) requestHeader;

	// Inform user (for logging purposes)
	std::cout << "Received regulator parameters:\n";
	std::cout << "\t Speed->angle PID:  " << request->speed_kp << " " << request->speed_ki << " " << request->speed_kd << std::endl;
	std::cout << "\t Angle->output PID: " << request->angle_kp << " " << request->angle_ki << " " << request->angle_kd << std::endl;
	std::cout << "\t Speed filter factor: " << request->speed_filter_factor << std::endl;
	std::cout << "\t Angle filter factor: " << request->angle_filter_factor << std::endl;
	std::cout << "\t Angular velocity factor: " << request->angular_velocity_factor << std::endl;
	std::cout << "\t Speed regulator enabled: " << request->speed_regulator_enabled << std::endl;

	// Check values validness
	if (request->speed_kp < 0 || request->speed_kp > 1000 || request->speed_ki < 0 || request->speed_ki > 1000 || request->speed_kd < 0 || request->speed_kd > 1000) {
		response->success = false;
		response->error_text = std::string("Invalid speed PID parameters");
		std::cout << "Invalid speed PID parameters.\n";
		return;
	}
	if (request->angle_kp < 0 || request->angle_kp > 1000 || request->angle_ki < 0 || request->angle_ki > 1000 || request->angle_kd < 0 || request->angle_kd > 1000) {
		response->success = false;
		response->error_text = std::string("Invalid angle PID parameters");
		std::cout << "Invalid angle PID parameters.\n";
		return;
	}

	if (request->speed_filter_factor <= 0 || request->speed_filter_factor > 1 || request->angle_filter_factor <= 0 || request->angle_filter_factor > 1 || request->angular_velocity_factor < 0) {
		response->success = false;
		response->error_text = std::string("Invalid filtering parameters");
		std::cout << "Invalid filtering parameters.\n";
		return;
	}

	// Inform user (for logging purposes)
	std::cout << "Setting regulator parameters!\n";

	// Set values
	controller.setSpeedPID(request->speed_kp, request->speed_ki, request->speed_kd);
	controller.setAnglePID(request->angle_kp, request->angle_ki, request->angle_kd);
	controller.setSpeedFilterFactor(request->speed_filter_factor);
	controller.setAngleFilterFactor(request->angle_filter_factor);
	controller.setAngularVelocityFactor(request->angular_velocity_factor);
	controller.setSpeedRegulatorEnabled(request->speed_regulator_enabled);

	// Set response
	response->success = true;
}

void getRegulatorSettingsCallback(
	const std::shared_ptr<rmw_request_id_t> requestHeader,
	const std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Request> request,
	std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Response> response
) {
	// Suppress unused parameter warning
	(void) requestHeader;
	(void) request;

	response->speed_filter_factor = controller.getSpeedFilterFactor();
	response->angle_filter_factor = controller.getAngleFilterFactor();
	response->angular_velocity_factor = controller.getAngularVelocityFactor();
	response->speed_regulator_enabled = controller.getSpeedRegulatorEnabled();
	controller.getSpeedPID(response->speed_kp, response->speed_ki, response->speed_kd);
	controller.getAnglePID(response->angle_kp, response->angle_ki, response->angle_kd);
}

void dataReceiveThreadFn(std::shared_ptr<rclcpp::node::Node> node) {
	auto enableSubscriber = node->create_subscription<std_msgs::msg::Bool>("rys_control_enable", enableMessageCallback, rmw_qos_profile_sensor_data);
	auto imuSubscriber = node->create_subscription<rys_interfaces::msg::ImuRollRotation>("rys_sensor_imu_roll", imuMessageCallback, rmw_qos_profile_sensor_data);
	auto setRegulatorSettingsServer = node->create_service<rys_interfaces::srv::SetRegulatorSettings>("rys_set_regulator_settings", setRegulatorSettingsCallback);
	auto getRegulatorSettingsServer = node->create_service<rys_interfaces::srv::GetRegulatorSettings>("rys_get_regulator_settings", getRegulatorSettingsCallback);

	rclcpp::spin(node);
}

void standUp() {
	// Direction multiplier
	int multiplier = (roll > 40 ? 1 : -1);

	// Disable motors, wait 1s
	motors.setSpeed(0.0, 0.0, 1);
	motors.disable();
	msleep(1000);
	if (!rclcpp::ok() || !enabled) {
		return;
	}

	// Enable motors
	motors.enable();
	msleep(100);
	if (!rclcpp::ok() || !enabled) {
		return;
	}

	// Drive backwards half-speed, wait 0.5s
	motors.setSpeed(multiplier * 400.0, multiplier * 400.0, 1);
	msleep(500);
	if (!rclcpp::ok() || !enabled) {
		return;
	}

	// Drive forward full-speed, wait until we've passed '0' point
	motors.setSpeed(-multiplier * 600.0, -multiplier * 600.0, 1);
	rclcpp::rate::WallRate standUpLoopRate(100);
	while (rclcpp::ok() && enabled) {
		// std::cout << "Standing up, angle: " << roll << std::endl;
		// Passing '0' point depends on from which side we're standing up
		if ((multiplier == 1 && roll <= 0) || (multiplier == -1 && roll >= 0)) {
			std::cout << "Stood up(?), angle: " << roll << std::endl;
			break;
		}
		standUpLoopRate.sleep();
	}
}

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	auto node = rclcpp::node::Node::make_shared("rys_node_motors_controller");

	std::cout << "Initializing and disabling motors...\n";
	try {
		motors.initialize();
		msleep(100);
		motors.disable();
	} catch (std::string & error) {
		std::cout << "Error initializing motors: " << error << std::endl;
		return 1;
	}

	std::cout << "Initializing data receiving thread...\n";
	std::thread dataReceiveThread(dataReceiveThreadFn, node);

	std::cout << "Setting up controller...\n";
	controller.init();
	controller.setSpeedPID(0.03, 0.0001, 0.008);
	controller.setAnglePID(50, 0.05, 20);

	controller.setAngleFilterFactor(0.95);
	controller.setSpeedFilterFactor(0.95);

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
			// motors.disable();
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
			float speed = (motors.getSpeedLeft() + motors.getSpeedRight()) / 2;
			float finalLeftSpeed = 0;
			float finalRightSpeed = 0;
			controller.calculateSpeed(roll, rotationX, speed, throttle, rotation, finalLeftSpeed, finalRightSpeed, loopTime);

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
