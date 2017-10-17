#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>

#include "MotorsControllerNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

MotorsControllerNode::MotorsControllerNode(const char * nodeName, std::chrono::milliseconds rate) : rclcpp::Node(nodeName) {
	this->enabled = false;
	this->balancing = false;
	this->enableTimeout = 5000ms;
	this->enableTimerEnd = std::chrono::high_resolution_clock::now();
	this->previous = std::chrono::high_resolution_clock::now();
	this->now = std::chrono::high_resolution_clock::now();

	this->roll = 0;
	this->rollPrevious = 0;
	this->rotationX = 0;

	this->rotation = 0;
	this->throttle = 0;
	this->steeringPrecision = 1;

	std::cout << "Initializing motors controller...\n";
	this->motorsController = new MotorsController();
	try {
		this->motorsController->init();
	} catch (std::string & error) {
		std::cout << "Error initializing controller: " << error << std::endl;
		throw(std::string("Controller init error"));
	}

	std::cout << "Setting up motors controller...\n";
	this->motorsController->setBalancing(false);
	this->motorsController->setLQREnabled(false);
	this->motorsController->setSpeedFilterFactor(1);
	this->motorsController->setAngleFilterFactor(1);
	this->motorsController->setPIDParameters(0.03, 0.0001, 0.008, 50, 0.05, 20);
	this->motorsController->setLQRParameters(-0.0316,-42.3121,-392.3354);

	std::cout << "Creating ROS subscriptions...\n";
	this->enableSubscriber = this->create_subscription<std_msgs::msg::Bool>("rys_control_enable", std::bind(&MotorsControllerNode::enableMessageCallback, this, _1));
	this->balancingModeSubscriber = this->create_subscription<std_msgs::msg::Bool>("rys_control_balancing_enabled", std::bind(&MotorsControllerNode::setBalancingMode, this, _1));
	this->steeringSubscriber = this->create_subscription<rys_interfaces::msg::Steering>("rys_control_steering", std::bind(&MotorsControllerNode::setSteering, this, _1));
	this->imuSubscriber = this->create_subscription<rys_interfaces::msg::ImuRollRotation>("rys_sensor_imu_roll", std::bind(&MotorsControllerNode::imuMessageCallback, this, _1), rmw_qos_profile_sensor_data);

	std::cout << "Creating ROS services...\n";
	this->setRegulatorSettingsServer = this->create_service<rys_interfaces::srv::SetRegulatorSettings>("rys_set_regulator_settings", std::bind(&MotorsControllerNode::setRegulatorSettingsCallback, this, _1, _2, _3));
	this->getRegulatorSettingsServer = this->create_service<rys_interfaces::srv::GetRegulatorSettings>("rys_get_regulator_settings", std::bind(&MotorsControllerNode::getRegulatorSettingsCallback, this, _1, _2, _3));

	std::cout << "Creating ROS timers...\n";
	this->loopTimer = this->create_wall_timer(rate, std::bind(&MotorsControllerNode::runLoop, this));

	std::cout << "Motors controller working.\n";
}

MotorsControllerNode::~MotorsControllerNode() {
	std::cout << "Deinitializing motors controller...\n";
	delete this->motorsController;
}

void MotorsControllerNode::motorsRunTimed(const float leftSpeed, const float rightSpeed, const int microstep, const int milliseconds) {
	auto end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(milliseconds);
	while (rclcpp::ok() && std::chrono::high_resolution_clock::now() < end) {
		this->motorsController->setMotorSpeeds(leftSpeed, rightSpeed, microstep);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void MotorsControllerNode::enableMessageCallback(const std_msgs::msg::Bool::SharedPtr message) {
	std::cout << "Received motor toggle request\n";

	if (message->data) {
		this->enableTimerEnd = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(enableTimeout);
		if (!this->enabled) {
			std::cout << "Enabling motors...\n";
			this->motorsController->enableMotors();
		}
	} else {
		if (this->enabled) {
			std::cout << "Disabling motors...\n";
		}
		this->motorsController->disableMotors();
	}
	this->enabled = message->data;
}

void MotorsControllerNode::imuMessageCallback(const rys_interfaces::msg::ImuRollRotation::SharedPtr message) {
	this->rollPrevious = roll;
	this->roll = message->roll * 180 / M_PI;
	this->rotationX = message->rotation_x;
}

void MotorsControllerNode::setRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Response> response) {
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
	this->motorsController->setSpeedFilterFactor(request->speed_filter_factor);
	this->motorsController->setAngleFilterFactor(request->angle_filter_factor);
	this->motorsController->setLQREnabled(request->lqr_enabled);
	this->motorsController->setPIDSpeedRegulatorEnabled(request->pid_speed_regulator_enabled);
	this->motorsController->setPIDParameters(request->pid_speed_kp, request->pid_speed_ki, request->pid_speed_kd, request->pid_angle_kp, request->pid_angle_ki, request->pid_angle_kd);
	this->motorsController->setLQRParameters(request->lqr_linear_velocity_k, request->lqr_angular_velocity_k, request->lqr_angle_k);

	// Set response
	response->success = true;
	response->error_text = std::string("success");
}

void MotorsControllerNode::getRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Response> response) {
	std::cout << "Received get regulator settings request\n";

	// Suppress unused parameter warning
	(void) requestHeader;
	(void) request;

	response->speed_filter_factor = this->motorsController->getSpeedFilterFactor();
	response->angle_filter_factor = this->motorsController->getAngleFilterFactor();

	response->lqr_enabled = this->motorsController->getLQREnabled();

	response->pid_speed_regulator_enabled = this->motorsController->getPIDSpeedRegulatorEnabled();
	this->motorsController->getPIDParameters(response->pid_speed_kp, response->pid_speed_ki, response->pid_speed_kd, response->pid_angle_kp, response->pid_angle_ki, response->pid_angle_kd);
	this->motorsController->getLQRParameters(response->lqr_linear_velocity_k, response->lqr_angular_velocity_k, response->lqr_angle_k);
}

void MotorsControllerNode::setBalancingMode(const std_msgs::msg::Bool::SharedPtr message) {
	std::cout << "Received balancing mode set request\n";
	if (message->data != this->balancing) {
		this->balancing = message->data;
		std::cout << "Changing balancing mode to: " << this->balancing << std::endl;
		this->motorsController->setBalancing(this->balancing);
	}
}

void MotorsControllerNode::setSteering(const rys_interfaces::msg::Steering::SharedPtr message) {
	std::cout << "Received set steering request\n";
	this->throttle = message->throttle;
	this->rotation = message->rotation;
	this->steeringPrecision = message->precision;
}

void MotorsControllerNode::standUp() {
	// Direction multiplier
	int multiplier = (this->roll > 40 ? 1 : -1);

	// Disable motors, wait 1s
	this->motorsRunTimed(0.0f, 0.0f, 1, 100);
	this->motorsController->disableMotors();
	this->motorsRunTimed(0.0f, 0.0f, 1, 1000);
	if (!rclcpp::ok() || !this->enabled) {
		return;
	}

	// Enable motors
	this->motorsController->enableMotors();
	this->motorsRunTimed(0.0f, 0.0f, 1, 100);
	if (!rclcpp::ok() || !this->enabled) {
		return;
	}

	// Drive backwards half-speed for 0.5s
	this->motorsRunTimed(multiplier * 0.8f, multiplier * 0.8f, 1, 500);
	if (!rclcpp::ok() || !this->enabled) {
		return;
	}

	// Drive forward full-speed, wait until we've passed '0' point
	this->motorsRunTimed(-multiplier * 1.0f, -multiplier * 1.0f, 1, 100);
	rclcpp::rate::WallRate standUpLoopRate(100);
	while (rclcpp::ok() && this->enabled) {
		// Passing '0' point depends on from which side we're standing up
		if ((multiplier * this->roll) <= 0) {
			std::cout << "Stood up(?), angle: " << this->roll << std::endl;
			break;
		}
		standUpLoopRate.sleep();
	}
}

void MotorsControllerNode::runLoop() {
	std::cout << "Loop!\n";

	this->now = std::chrono::high_resolution_clock::now();
	auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->now - this->previous);
	float loopTime = loopTimeSpan.count();
	this->previous = this->now;

	if (!this->enabled) {
		return;
	}

	// Check enable timer
	if (this->enableTimerEnd < this->now) {
		this->enabled = false;
		this->motorsController->disableMotors();
		return;
	}

	// Detect current position, use 2 consecutive reads
	bool layingDown = (this->roll > 40.0 && this->rollPrevious > 40.0) || (this->roll < -40.0 && this->rollPrevious < -40.0);
	if (this->balancing && layingDown) {
		// Laying down and wanting to balance, stand up!
		std::cout << "Laying down, trying to stand up\n";

		try {
			standUp();

			// Zero-out regulators: PID's errors and integrals, loop timer etc
			this->motorsController->zeroRegulators();
			this->previous = std::chrono::high_resolution_clock::now();
		} catch (std::string & error) {
			std::cout << "Error standing up from laying: " << error << std::endl;
			throw(std::string("Error standing up from laying"));
		}
	} else {
		// Standing up or not balancing - use controller
		// Calculate target speeds for motors
		float speed = (this->motorsController->getMotorSpeedLeft() + this->motorsController->getMotorSpeedRight()) / 2;
		float finalLeftSpeed = 0;
		float finalRightSpeed = 0;
		this->motorsController->calculateSpeeds(this->roll, this->rotationX, speed, this->throttle, this->rotation, finalLeftSpeed, finalRightSpeed, loopTime);

		// Set target speeds
		try {
			unsigned char microstep = this->balancing ? 32 : this->steeringPrecision;
			this->motorsController->setMotorSpeeds(finalLeftSpeed, finalRightSpeed, microstep);
		} catch (std::string & error) {
			std::cout << "Error setting motors speed: " << error << std::endl;
			throw(std::string("Error setting motors speed"));
		}
	}
}
