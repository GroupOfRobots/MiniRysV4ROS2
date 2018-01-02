#include "MotorsControllerNode.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <stdexcept>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

MotorsControllerNode::MotorsControllerNode(
	const std::string & robotName,
	const std::string & nodeName,
	std::chrono::milliseconds rate,
	float wheelRadius,
	float baseWidth,
	unsigned int odometryRate
) : rclcpp::Node(nodeName, robotName, true), wheelRadius(wheelRadius), baseWidth(baseWidth), odometryRate(odometryRate) {
	this->enabled = false;
	this->balancing = false;
	this->enableTimeout = 5000ms;
	this->enableTimerEnd = std::chrono::high_resolution_clock::now();
	this->previous = std::chrono::high_resolution_clock::now();
	this->timeNow = std::chrono::high_resolution_clock::now();

	this->roll = 0;
	this->rollPrevious = 0;
	this->rotationX = 0;

	this->rotation = 0;
	this->throttle = 0;
	this->steeringPrecision = 1;

	this->odometryPublishCounter = 0;
	this->odometryFrame = KDL::Frame();
	this->odometryTwist = KDL::Twist();
	this->odoSeq = 0;

	std::cout << "[MOTORS] Initializing motors controller...\n";
	this->motorsController = new MotorsController();
	try {
		this->motorsController->init();
	} catch (const std::exception & error) {
		std::cout << "[MOTORS] Error initializing controller: " << error.what() << std::endl;
		throw(error);
	}

	this->motorsController->setInvertSpeed(false, true);
	this->motorsController->setMotorsSwapped(true);
	this->motorsController->setBalancing(false);
	this->motorsController->setLQREnabled(false);
	this->motorsController->setSpeedFilterFactor(1);
	this->motorsController->setAngleFilterFactor(1);
	this->motorsController->setPIDParameters(0.03, 0.0001, 0.008, 50, 0.05, 20);
	this->motorsController->setLQRParameters(-0.0316,-42.3121,-392.3354);

	std::cout << "[MOTORS] Motors controller initialized\n";

	this->motorsEnableSubscriber = this->create_subscription<std_msgs::msg::Bool>("/" + robotName + "/control/enable_motors", std::bind(&MotorsControllerNode::enableMessageCallback, this, _1));
	this->balancingEnableSubscriber = this->create_subscription<std_msgs::msg::Bool>("/" + robotName + "/control/enable_balancing", std::bind(&MotorsControllerNode::setBalancingMode, this, _1));
	this->steeringSubscriber = this->create_subscription<rys_interfaces::msg::Steering>("/" + robotName + "/control/steering", std::bind(&MotorsControllerNode::setSteering, this, _1));
	this->imuSubscriber = this->create_subscription<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", std::bind(&MotorsControllerNode::imuMessageCallback, this, _1), rmw_qos_profile_sensor_data);

	const rmw_qos_profile_t odometryQosProfile = {
		RMW_QOS_POLICY_HISTORY_KEEP_LAST,
		200,
		RMW_QOS_POLICY_RELIABILITY_RELIABLE,
		RMW_QOS_POLICY_DURABILITY_VOLATILE,
		false
	};

	this->odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/" + robotName + "/control/odometry", odometryQosProfile);

	this->setRegulatorSettingsServer = this->create_service<rys_interfaces::srv::SetRegulatorSettings>("/" + robotName + "/control/regulator_settings/set", std::bind(&MotorsControllerNode::setRegulatorSettingsCallback, this, _1, _2, _3));
	this->getRegulatorSettingsServer = this->create_service<rys_interfaces::srv::GetRegulatorSettings>("/" + robotName + "/control/regulator_settings/get", std::bind(&MotorsControllerNode::getRegulatorSettingsCallback, this, _1, _2, _3));

	this->loopTimer = this->create_wall_timer(rate, std::bind(&MotorsControllerNode::runLoop, this));

	std::cout << "[MOTORS] Node ready\n";
}

MotorsControllerNode::~MotorsControllerNode() {
	std::cout << "[MOTORS] Deinitializing motors controller...\n";
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
	if (message->data) {
		this->enableTimerEnd = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(enableTimeout);
		if (!this->enabled) {
			std::cout << "[MOTORS] Enabling motors...\n";
			this->motorsController->enableMotors();
		}
	} else {
		if (this->enabled) {
			std::cout << "[MOTORS] Disabling motors...\n";
		}
		this->motorsController->disableMotors();
	}
	this->enabled = message->data;
	this->previous = std::chrono::high_resolution_clock::now();
}

void MotorsControllerNode::imuMessageCallback(const sensor_msgs::msg::Imu::SharedPtr message) {
	double qx = message->orientation.x;
	double qy = message->orientation.y;
	double qz = message->orientation.z;
	double qw = message->orientation.w;

	this->rollPrevious = roll;
	this->roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
}

void MotorsControllerNode::setRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Response> response) {
	// Suppress unused parameter warning
	(void) requestHeader;

	// Inform user (for logging purposes)
	std::cout << "[MOTORS] Received regulator parameters:\n";
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
	std::cout << "[MOTORS] Received get regulator settings request\n";

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
	std::cout << "[MOTORS] Received balancing mode set request\n";
	if (message->data != this->balancing) {
		this->balancing = message->data;
		std::cout << "[MOTORS] Changing balancing mode to: " << this->balancing << std::endl;
		this->motorsController->setBalancing(this->balancing);
	}
}

void MotorsControllerNode::setSteering(const rys_interfaces::msg::Steering::SharedPtr message) {
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

	rclcpp::WallRate standUpLoopRate(100);
	while (rclcpp::ok() && this->enabled) {
		// Passing '0' point depends on from which side we're standing up
		if ((multiplier * this->roll) <= 0) {
			std::cout << "[MOTORS] Stood up(?), angle: " << this->roll << std::endl;
			break;
		}
		standUpLoopRate.sleep();
	}
}

void MotorsControllerNode::runLoop() {
	this->timeNow = std::chrono::high_resolution_clock::now();
	auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->timeNow - this->previous);
	float loopTime = loopTimeSpan.count();
	this->previous = this->timeNow;

	if (!this->enabled) {
		return;
	}

	// Check enable timer
	if (this->enableTimerEnd < this->timeNow) {
		this->enabled = false;
		this->motorsController->disableMotors();
		return;
	}

	// Detect current position, use 2 consecutive reads
	bool layingDown = (this->roll > 40.0 && this->rollPrevious > 40.0) || (this->roll < -40.0 && this->rollPrevious < -40.0);
	if (this->balancing && layingDown) {
		// Laying down and wanting to balance, stand up!
		std::cout << "[MOTORS] Laying down, trying to stand up\n";

		try {
			standUp();

			// Zero-out regulators: PID's errors and integrals, loop timer etc
			this->motorsController->zeroRegulators();
			this->previous = std::chrono::high_resolution_clock::now();
		} catch (const std::exception & error) {
			std::cout << "[MOTORS] Error standing up from laying: " << error.what() << std::endl;
			throw(error);
		}
	} else {
		// Standing up or not balancing - use controller
		// Calculate target speeds for motors
		float leftSpeed = this->motorsController->getMotorSpeedLeftRaw();
		float rightSpeed = this->motorsController->getMotorSpeedRightRaw();
		float linearSpeed = (leftSpeed + rightSpeed) / 2;
		float finalLeftSpeed = 0;
		float finalRightSpeed = 0;
		this->motorsController->calculateSpeeds(this->roll, this->rotationX, linearSpeed, this->throttle, this->rotation, finalLeftSpeed, finalRightSpeed, loopTime);

		// Save current speeds in units suitable for odometry (m/s)
		leftSpeed = this->motorsController->getMotorSpeedLeft() * this->wheelRadius * 2.0 * M_PI;
		rightSpeed = this->motorsController->getMotorSpeedRight() * this->wheelRadius * 2.0 * M_PI;

		// Set target speeds
		try {
			unsigned char microstep = this->balancing ? 32 : this->steeringPrecision;
			this->motorsController->setMotorSpeeds(finalLeftSpeed, finalRightSpeed, microstep);
		} catch (const std::exception & error) {
			std::cout << "[MOTORS] Error setting motors speed: " << error.what() << std::endl;
			throw(error);
		}

		// Odometry
		// First, create update frame and twist and apply them onto saved ones
		KDL::Frame updateFrame;
		KDL::Twist updateTwist;

		// Second, calculate the difference frame by forward kinematics
		if (leftSpeed == rightSpeed) {
			// Equal speeds <=> only linear movement (along Y axis for Y-forward-oriented mode)
			updateFrame = KDL::Frame(KDL::Vector(0, leftSpeed * loopTime, 0));
			updateTwist.vel.y(leftSpeed);
		} else {
			// Full forward kinematics for differential robot
			float linearVelocity = (rightSpeed + leftSpeed) / 2;
			float angularVelocity = (rightSpeed - leftSpeed) / baseWidth;
			float rotationPointDistance = linearVelocity / angularVelocity;
			float rotationAngle = angularVelocity * loopTime;

			// Mobile robots traditionally are Y-forward-oriented
			float deltaX = rotationPointDistance * std::sin(rotationAngle);
			float deltaY = rotationPointDistance * (1.0 - std::cos(rotationAngle));

			// Those are for X-forward-oriented (kept here for reference)
			// float deltaX = rotationPointDistance * (std::cos(rotationAngle) - 1.0);
			// float deltaY = rotationPointDistance * std::sin(rotationAngle);

			updateFrame = KDL::Frame(KDL::Rotation::RotZ(rotationAngle), KDL::Vector(deltaX, deltaY, 0));
			// For X-forward-oriented x and y would be swapped here
			updateTwist.vel.x(linearVelocity * std::sin(rotationAngle));
			updateTwist.vel.y(linearVelocity * std::cos(rotationAngle));
			updateTwist.rot.z(angularVelocity);
		}

		// Third, update odometry frame and twist
		this->odometryFrame = this->odometryFrame * updateFrame;
		this->odometryTwist = this->odometryTwist + updateTwist;

		// Fourth, publish odometry data (if needed)
		odometryPublishCounter++;
		if (odometryPublishCounter >= odometryRate) {
			// Create an odometry message
			auto odometryMessage = std::make_shared<nav_msgs::msg::Odometry>();

			// Fill out the header
			odometryMessage->header.stamp = this->now();
			odometryMessage->header.frame_id = "odom";
			odometryMessage->child_frame_id = "base_link";

			// Actual covariance is unknown - fill with zeros
			for (unsigned int i = 0; i < odometryMessage->pose.covariance.size(); ++i) {
				odometryMessage->pose.covariance[i] = 0.0;
			}
			for (unsigned int i = 0; i < odometryMessage->twist.covariance.size(); ++i) {
				odometryMessage->twist.covariance[i] = 0.0;
			}

			// Third, update the odometry frame and put it into the message
			odometryMessage->pose.pose.position.x = this->odometryFrame.p.x();
			odometryMessage->pose.pose.position.y = this->odometryFrame.p.y();
			odometryMessage->pose.pose.position.z = this->odometryFrame.p.z();
			double rotX, rotY, rotZ, rotW;
			this->odometryFrame.M.GetQuaternion(rotX, rotY, rotZ, rotW);
			odometryMessage->pose.pose.orientation.x = rotX;
			odometryMessage->pose.pose.orientation.y = rotY;
			odometryMessage->pose.pose.orientation.z = rotZ;
			odometryMessage->pose.pose.orientation.w = rotW;

			odometryMessage->twist.twist.linear.x = this->odometryTwist.vel.x();
			odometryMessage->twist.twist.linear.y = this->odometryTwist.vel.y();
			odometryMessage->twist.twist.linear.z = this->odometryTwist.vel.z();
			odometryMessage->twist.twist.angular.x = this->odometryTwist.rot.x();
			odometryMessage->twist.twist.angular.y = this->odometryTwist.rot.y();
			odometryMessage->twist.twist.angular.z = this->odometryTwist.rot.z();

			// Publish the odometry message
			this->odometryPublisher->publish(odometryMessage);

			// Cleanup
			odometryPublishCounter = 0;
			this->odometryFrame = KDL::Frame();
			this->odometryTwist = KDL::Twist();

			// Debug log, enable if odometry frames are suspected to be lost
			// std::cout << "s: " << this->odoSeq++ << std::endl;
		}
	}
}
