#include "IMUNode.hpp"

#include <iostream>
#include <memory>

IMUNode::IMUNode(
	const std::string & robotName,
	const std::string & nodeName,
	const std::chrono::milliseconds loopDuration,
	const std::chrono::milliseconds calibrationDuration,
	const int imuCalibrationOffsets[6]
) : rclcpp::Node(nodeName, robotName, true) {
	this->calibration = false;
	this->calibrationValuesSum = 0;
	this->calibrationIterations = 0;
	this->calibrationDuration = calibrationDuration;
	this->calibrationEndTime = std::chrono::high_resolution_clock::now();

	std::cout << "[IMU] Initializing IMU...\n";
	this->imu = new IMU();
	this->imu->initialize();
	this->imu->setOffsets(imuCalibrationOffsets);
	std::cout << "[IMU] IMU initialized\n";

	this->imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", rmw_qos_profile_sensor_data);
	this->calibrationSubscription = this->create_subscription<std_msgs::msg::Empty>("/" + robotName + "/control/imu/calibrate", std::bind(&IMUNode::imuCalibrateCallback, this, std::placeholders::_1));
	this->timer = this->create_wall_timer(loopDuration, std::bind(&IMUNode::timerCallback, this));
	std::cout << "[IMU] Node ready\n";
}

IMUNode::~IMUNode() {
	delete this->imu;
}

void IMUNode::imuCalibrateCallback(const std_msgs::msg::Empty::SharedPtr message) {
	// Prevent unused parameter warning
	(void)message;

	std::cout << "[IMU] Calibration: collecting data (" << this->calibrationDuration.count() << "ms)...\n";
	this->calibrationEndTime = std::chrono::high_resolution_clock::now() + this->calibrationDuration;
	this->calibrationValuesSum = 0;
	this->calibrationIterations = 0;
	this->calibration = true;
}

void IMUNode::timerCallback() {
	auto message = std::make_shared<sensor_msgs::msg::Imu>();

	message->header.stamp = rclcpp::Time::now();
	message->header.frame_id = "MPU6050";

	IMU::ImuData data;
	int result;
	try {
		result = this->imu->getData(&data);
	} catch (std::string & error) {
		std::cout << "[IMU] Error getting IMU reading: " << error << std::endl;
		return;
	}

	if (result < 0) {
		return;
	}

	message->orientation.x = data.orientationQuaternion[1];
	message->orientation.y = data.orientationQuaternion[2];
	message->orientation.z = data.orientationQuaternion[3];
	message->orientation.w = data.orientationQuaternion[0];
	message->angular_velocity.x = data.angularVelocity[0];
	message->angular_velocity.y = data.angularVelocity[1];
	message->angular_velocity.z = data.angularVelocity[2];
	message->linear_acceleration.x = data.linearAcceleration[0];
	message->linear_acceleration.y = data.linearAcceleration[1];
	message->linear_acceleration.z = data.linearAcceleration[2];
	for (int i = 0; i < 9; ++i) {
		message->orientation_covariance[i] = 0;
		message->angular_velocity_covariance[i] = 0;
		message->linear_acceleration_covariance[i] = 0;
	}

	this->imuPublisher->publish(message);

	/// TODO: adjust to new measurement/message type
	/*
	if (this->calibration) {
		this->calibrationValuesSum += roll;
		this->calibrationIterations++;
		if (std::chrono::high_resolution_clock::now() >= this->calibrationEndTime) {
			this->calibration = false;
			float averageRoll = this->calibrationValuesSum / this->calibrationIterations;
			this->imu->setOffsets(0, 0, averageRoll);
			std::cout << "[IMU] Calibration: data collected, average offset: " << averageRoll << std::endl;
		}
	}
	*/
}
