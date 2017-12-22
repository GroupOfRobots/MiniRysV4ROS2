#include "IMUNode.hpp"

#include <iostream>
#include <memory>

IMUNode::IMUNode(
	const std::string & robotName,
	const std::string & nodeName,
	const std::chrono::milliseconds loopDuration,
	const std::chrono::milliseconds calibrationDuration,
	const int imuCalibrationOffsets[6],
	const int infrequentPublishRate
) : rclcpp::Node(nodeName, robotName, true), infrequentPublishRate(infrequentPublishRate), infrequentPublishCount(0) {
	std::cout << "[IMU] Initializing IMU...\n";
	this->imu = new IMU();
	this->imu->initialize();
	// this->imu->setOffsets(imuCalibrationOffsets);
	std::cout << "[IMU] IMU initialized\n";

	this->imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", rmw_qos_profile_sensor_data);
	this->imuInfrequentPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imuInfrequent", rmw_qos_profile_sensor_data);
	this->timer = this->create_wall_timer(loopDuration, std::bind(&IMUNode::timerCallback, this));
	std::cout << "[IMU] Node ready\n";
}

IMUNode::~IMUNode() {
	delete this->imu;
}

void IMUNode::timerCallback() {
	auto message = std::make_shared<sensor_msgs::msg::Imu>();

	message->header.stamp = this->now();
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

	this->infrequentPublishCount++;
	if (this->infrequentPublishCount >= this->infrequentPublishRate) {
		this->infrequentPublishCount = 0;
		this->imuInfrequentPublisher->publish(message);
	}
}
