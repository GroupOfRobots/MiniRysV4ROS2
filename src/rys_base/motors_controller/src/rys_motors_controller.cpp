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

#include "MotorsController.hpp"

volatile bool enabled;
volatile int enableTimeout = 5000;
std::chrono::time_point<std::chrono::high_resolution_clock> enableTimerEnd;

volatile bool balancing = true;

volatile float roll;
volatile float rollPrevious;
volatile float rotationX;

volatile float rotation;
volatile float throttle;
volatile unsigned char steeringPrecision;

MotorsController motorsController;

void msleep(const int milliseconds) {
	auto end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(milliseconds);
	while (rclcpp::ok() && std::chrono::high_resolution_clock::now() < end) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void motorsRunTimed(const float leftSpeed, const float rightSpeed, const int microstep, const int milliseconds) {
	auto end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(milliseconds);
	while (rclcpp::ok() && std::chrono::high_resolution_clock::now() < end) {
		motorsController.setMotorSpeeds(leftSpeed, rightSpeed, microstep);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void enableMessageCallback(const std_msgs::msg::Bool::SharedPtr message) {
	if (message->data) {
		enableTimerEnd = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(enableTimeout);
		if (!enabled) {
			std::cout << "Enabling motors...\n";
			motorsController.enableMotors();
		}
	} else {
		if (enabled) {
			std::cout << "Disabling motors...\n";
		}
		motorsController.disableMotors();
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
	motorsController.setSpeedFilterFactor(request->speed_filter_factor);
	motorsController.setAngleFilterFactor(request->angle_filter_factor);
	motorsController.setLQREnabled(request->lqr_enabled);
	motorsController.setPIDSpeedRegulatorEnabled(request->pid_speed_regulator_enabled);
	motorsController.setPIDParameters(request->pid_speed_kp, request->pid_speed_ki, request->pid_speed_kd, request->pid_angle_kp, request->pid_angle_ki, request->pid_angle_kd);
	motorsController.setLQRParameters(request->lqr_linear_velocity_k, request->lqr_angular_velocity_k, request->lqr_angle_k);

	// Set response
	response->success = true;
	response->error_text = std::string("success");
}

void getRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Response> response) {
	// Suppress unused parameter warning
	(void) requestHeader;
	(void) request;

	response->speed_filter_factor = motorsController.getSpeedFilterFactor();
	response->angle_filter_factor = motorsController.getAngleFilterFactor();

	response->lqr_enabled = motorsController.getLQREnabled();

	response->pid_speed_regulator_enabled = motorsController.getPIDSpeedRegulatorEnabled();
	motorsController.getPIDParameters(response->pid_speed_kp, response->pid_speed_ki, response->pid_speed_kd, response->pid_angle_kp, response->pid_angle_ki, response->pid_angle_kd);
	motorsController.getLQRParameters(response->lqr_linear_velocity_k, response->lqr_angular_velocity_k, response->lqr_angle_k);
}

void setBalancingMode(const std_msgs::msg::Bool::SharedPtr message) {
	if (message->data != balancing) {
		balancing = message->data;
		std::cout << "Changing balancing mode to: " << balancing << std::endl;
		motorsController.setBalancing(balancing);
	}
}

void setSteering(const rys_interfaces::msg::Steering::SharedPtr message) {
	throttle = message->throttle;
	rotation = message->rotation;
	steeringPrecision = message->precision;
}

void communicationThreadFn(std::shared_ptr<rclcpp::node::Node> node) {
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
	motorsController.disableMotors();
	msleep(1000);
	if (!rclcpp::ok() || !enabled) {
		return;
	}

	// Enable motors
	motorsController.enableMotors();
	msleep(100);
	if (!rclcpp::ok() || !enabled) {
		return;
	}

	// Drive backwards half-speed for 0.5s
	motorsRunTimed(multiplier * 0.8f, multiplier * 0.8f, 1, 500);
	if (!rclcpp::ok() || !enabled) {
		return;
	}

	// Drive forward full-speed, wait until we've passed '0' point
	motorsRunTimed(-multiplier * 1.0f, -multiplier * 1.0f, 1, 100);
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

	std::cout << "Setting up motors controller...\n";
	try {
		motorsController.init();
	} catch (std::string & error) {
		std::cout << "Error initializing controller: " << error << std::endl;
		return 1;
	}

	motorsController.setBalancing(balancing);
	motorsController.setLQREnabled(false);

	motorsController.setSpeedFilterFactor(1);
	motorsController.setAngleFilterFactor(1);
	motorsController.setPIDParameters(0.03, 0.0001, 0.008, 50, 0.05, 20);
	motorsController.setLQRParameters(-0.0316,-42.3121,-392.3354);

	// This is only needed due to bug in rclcpp in ROS2 beta1.
	// Re-add cslcpp::spin_some(node) to main thread and remove separate data thread when it's fixed (beta2?).
	std::cout << "Initializing data receiving thread...\n";
	std::thread communicationThread(communicationThreadFn, node);

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
			// motorsController.disableMotors();
			loopRate.sleep();
			continue;
		}

		// Check enable timer
		if (enableTimerEnd < now) {
			enabled = false;
			motorsController.disableMotors();
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
				motorsController.zeroRegulators();
				previous = std::chrono::high_resolution_clock::now();
			} catch (std::string & error) {
				std::cout << "Error standing up from laying: " << error << std::endl;
				break;
			}
		} else {
			// Standing up or not balancing - use controller
			// Calculate target speeds for motors
			float speed = (motorsController.getMotorSpeedLeft() + motorsController.getMotorSpeedRight()) / 2;
			float finalLeftSpeed = 0;
			float finalRightSpeed = 0;
			motorsController.calculateSpeeds(roll, rotationX, speed, throttle, rotation, finalLeftSpeed, finalRightSpeed, loopTime);

			// Set target speeds
			try {
				unsigned char microstep = balancing ? 32 : steeringPrecision;
				motorsController.setMotorSpeeds(finalLeftSpeed, finalRightSpeed, microstep);
			} catch (std::string & error) {
				std::cout << "Error setting motors speed: " << error << std::endl;
				break;
			}
		}

		loopRate.sleep();
	}

	communicationThread.join();

	std::cout << "Quitting\n";
	return 0;
}
