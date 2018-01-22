#include "IMUNode.hpp"
#include <stdexcept>
#include <iostream>
#include <memory>
#include <cmath>

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
	std::cout << "[IMU] IMU initialized\n";

	this->imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", rmw_qos_profile_sensor_data);
	this->imuInfrequentPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imuInfrequent", rmw_qos_profile_sensor_data);
	this->timer = this->create_wall_timer(loopDuration, std::bind(&IMUNode::publishData, this));
	std::cout << "[IMU] Node ready\n";
}

IMUNode::~IMUNode() {
	delete this->imu;
}

void IMUNode::publishData() {
	auto message = std::make_shared<sensor_msgs::msg::Imu>();

	message->header.stamp = this->now();
	message->header.frame_id = "MPU6050";

	IMU::ImuData data;
	int result;
	try {
		result = this->imu->getData(&data);
	} catch (const std::exception & error) {
		std::cout << "[IMU] Error getting IMU reading: " << error.what() << std::endl;
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

	double qx = data.orientationQuaternion[1];
	double qy = data.orientationQuaternion[2];
	double qz = data.orientationQuaternion[3];
	double qw = data.orientationQuaternion[0];
	double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));

	std::cout << "IMU; roll = " << roll << ";";
	std::cout << "wx = " << message->angular_velocity.x << ";";
	std::cout << "wy = " << message->angular_velocity.y << ";";
	std::cout << "wz = " << message->angular_velocity.z << ";";
	std::cout << std::endl;

	this->imuPublisher->publish(message);

	this->infrequentPublishCount++;
	if (this->infrequentPublishCount >= this->infrequentPublishRate) {
		this->infrequentPublishCount = 0;
		this->imuInfrequentPublisher->publish(message);
	}
}
