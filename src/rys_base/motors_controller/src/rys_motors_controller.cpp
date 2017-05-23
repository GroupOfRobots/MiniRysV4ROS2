#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rys_interfaces/msg/imu_roll_rotation.hpp"
#include "rys_interfaces/msg/steering.hpp"
#include "rys_interfaces/srv/set_regulator_settings.hpp"
#include "rys_interfaces/srv/get_regulator_settings.hpp"

#include "Motors.h"
#include "Controller.h"

volatile bool enabled;
volatile int enableTimeout = 5000;
std::chrono::time_point<std::chrono::high_resolution_clock> enableTimerEnd;

volatile bool balancing = true;

volatile float roll;
volatile float rollPrevious;
volatile float rotationX;

volatile float rotation;
volatile float throttle;

Motors motors;
Controller controller;

void msleep(const int milliseconds) {
	auto end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(milliseconds);
	while (rclcpp::ok() && std::chrono::high_resolution_clock::now() < end) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void motorsRunTimed(const float leftSpeed, const float rightSpeed, const int microstep, const int milliseconds) {
	auto end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(milliseconds);
	while (rclcpp::ok() && std::chrono::high_resolution_clock::now() < end) {
		motors.setSpeed(leftSpeed, rightSpeed, microstep);
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

void setRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Response> response) {
	// Suppress unused parameter warning
	(void) requestHeader;

	// Inform user (for logging purposes)
	std::cout << "Received regulator parameters:\n";
	std::cout << "\t Speed filter factor: " << request->speed_filter_factor << std::endl;
	std::cout << "\t Angle filter factor: " << request->angle_filter_factor << std::endl;
	std::cout << "\t LQR enabled: " << (request->lqr_enabled ? "True" : "False") << std::endl;
	std::cout << "\t PID: Speed regulator enabled: " << (request->pid_speed_regulator_enabled ? "True" : "False") << std::endl;
	std::cout << "\t PID: 1st stage (speed->angle):  " << request->pid_speed_kp << " " << request->pid_speed_ki << " " << request->pid_speed_kd << std::endl;
	std::cout << "\t PID: 2nd stage (angle->output): " << request->pid_angle_kp << " " << request->pid_angle_ki << " " << request->pid_angle_kd << std::endl;
	std::cout << "\t LQR: Linear velocity K: " << request->lqr_linear_velocity_k << std::endl;
	std::cout << "\t LQR: Angular velocity K: " << request->lqr_angular_velocity_k << std::endl;
	std::cout << "\t LQR: Angle K: " << request->lqr_angle_k << std::endl;

	// Set values
	controller.setSpeedFilterFactor(request->speed_filter_factor);
	controller.setAngleFilterFactor(request->angle_filter_factor);
	controller.setLQREnabled(request->lqr_enabled);
	controller.setPIDSpeedRegulatorEnabled(request->pid_speed_regulator_enabled);
	controller.setPIDParameters(request->pid_speed_kp, request->pid_speed_ki, request->pid_speed_kd, request->pid_angle_kp, request->pid_angle_ki, request->pid_angle_kd);
	controller.setLQRParameters(request->lqr_linear_velocity_k, request->lqr_angular_velocity_k, request->lqr_angle_k);

	// Set response
	response->success = true;
	response->error_text = std::string("success");
}

void getRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Response> response) {
	// Suppress unused parameter warning
	(void) requestHeader;
	(void) request;

	response->speed_filter_factor = controller.getSpeedFilterFactor();
	response->angle_filter_factor = controller.getAngleFilterFactor();

	response->lqr_enabled = controller.getLQREnabled();

	response->pid_speed_regulator_enabled = controller.getPIDSpeedRegulatorEnabled();
	controller.getPIDParameters(response->pid_speed_kp, response->pid_speed_ki, response->pid_speed_kd, response->pid_angle_kp, response->pid_angle_ki, response->pid_angle_kd);
	controller.getLQRParameters(response->lqr_linear_velocity_k, response->lqr_angular_velocity_k, response->lqr_angle_k);
}

void setBalancingMode(const std_msgs::msg::Bool::SharedPtr message) {
	if (message->data != balancing) {
		balancing = message->data;
		std::cout << "Changing balancing mode to: " << balancing << std::endl;
		controller.setBalancing(balancing);
	}
}

void setSteering(const rys_interfaces::msg::Steering::SharedPtr message) {
	throttle = message->throttle;
	rotation = message->rotation;
}

void dataReceiveThreadFn(std::shared_ptr<rclcpp::node::Node> node) {
	auto enableSubscriber = node->create_subscription<std_msgs::msg::Bool>("rys_control_enable", enableMessageCallback);
	auto imuSubscriber = node->create_subscription<rys_interfaces::msg::ImuRollRotation>("rys_sensor_imu_roll", imuMessageCallback, rmw_qos_profile_sensor_data);
	auto balancingModeSubscriber = node->create_subscription<std_msgs::msg::Bool>("rys_control_balancing_enabled", setBalancingMode);
	auto steeringSubscriber = node->create_subscription<rys_interfaces::msg::Steering>("rys_control_steering", setSteering);

	auto setRegulatorSettingsServer = node->create_service<rys_interfaces::srv::SetRegulatorSettings>("rys_set_regulator_settings", setRegulatorSettingsCallback);
	auto getRegulatorSettingsServer = node->create_service<rys_interfaces::srv::GetRegulatorSettings>("rys_get_regulator_settings", getRegulatorSettingsCallback);

	rclcpp::spin(node);
}

void standUp() {
	// Direction multiplier
	int multiplier = (roll > 40 ? 1 : -1);

	// Disable motors, wait 1s
	motorsRunTimed(0.0f, 0.0f, 1, 100);
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

	// Drive backwards half-speed for 0.5s
	motorsRunTimed(multiplier * 0.5f, multiplier * 0.5f, 1, 500);
	if (!rclcpp::ok() || !enabled) {
		return;
	}

	// Drive forward full-speed, wait until we've passed '0' point
	motors.setSpeed(-multiplier * 0.8f, -multiplier * 0.8f, 1);
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

	controller.setBalancing(balancing);
	controller.setLQREnabled(false);

	controller.setSpeedFilterFactor(1);
	controller.setAngleFilterFactor(1);
	controller.setPIDParameters(0.03, 0.0001, 0.008, 50, 0.05, 20);
	controller.setLQRParameters(-3.1623,-0.7968,-32.3996);

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

		// Detect current position, use 2 consecutive reads
		bool layingDown = (roll > 40.0 && rollPrevious > 40.0) || (roll < -40.0 && rollPrevious < -40.0);
		if (balancing && layingDown) {
			// Laying down and wanting to balance, stand up!
			std::cout << "Laying down, trying to stand up\n";

			try {
				standUp();

				// Zero-out regulators: PID's errors and integrals, loop timer etc
				controller.zeroRegulators();
				previous = std::chrono::high_resolution_clock::now();
			} catch (std::string & error) {
				std::cout << "Error standing up from laying: " << error << std::endl;
				break;
			}
		} else {
			// Standing up or not balancing - use controller
			// Calculate target speeds for motors
			float speed = (motors.getSpeedLeft() + motors.getSpeedRight()) / 2;
			float finalLeftSpeed = 0;
			float finalRightSpeed = 0;
			controller.calculateSpeeds(roll, rotationX, speed, throttle, rotation, finalLeftSpeed, finalRightSpeed, loopTime);

			// Set target speeds
			try {
				motors.setSpeed(finalLeftSpeed, finalRightSpeed, 8);
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
