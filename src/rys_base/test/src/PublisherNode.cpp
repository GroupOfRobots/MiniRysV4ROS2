#include "PublisherNode.hpp"
#include <stdexcept>
#include <iostream>
#include <memory>

#include <sched.h>
#include <sys/mman.h>

PublisherNode::PublisherNode(
	const std::string & robotName,
	const std::string & nodeName,
	const bool useIPC,
	const std::chrono::milliseconds loopDuration
) : rclcpp::Node(nodeName, robotName, useIPC) {

	const rmw_qos_profile_t imuQosProfile = {
		RMW_QOS_POLICY_HISTORY_KEEP_LAST,
		100,
		RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
		RMW_QOS_POLICY_DURABILITY_VOLATILE,
		false
	};

	this->previous = std::chrono::high_resolution_clock::now();
	this->timeNow = std::chrono::high_resolution_clock::now();

	this->numOfImuMessages = 0;
	this->frequency = 0.0;

	this->testPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", rmw_qos_profile_sensor_data);
	// this->testPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", imuQosProfile);
	// this->testPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu");
	this->timer = this->create_wall_timer(loopDuration, std::bind(&PublisherNode::publishData, this));
	std::cout << "[Publisher] Node ready\n";
}

void PublisherNode::publishData() {
	this->numOfImuMessages++;
	if (this->numOfImuMessages > 9999) {
		this->previous = this->timeNow;
		this->timeNow = std::chrono::high_resolution_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->timeNow - this->previous);
		float loopTime = loopTimeSpan.count();
		this->frequency = this->numOfImuMessages/loopTime;
		std::cout <<"[Publisher] Message frequency: " << this->frequency << std::endl;
		this->numOfImuMessages = 0;
	}
	auto message = std::make_shared<sensor_msgs::msg::Imu>();

	message->header.stamp = this->now();
	message->header.frame_id = "Publisher";

	message->orientation.x = 0;
	message->orientation.y = 0;
	message->orientation.z = 0;
	message->orientation.w = 0;
	message->angular_velocity.x = 0;
	message->angular_velocity.y = 0;
	message->angular_velocity.z = 0;
	message->linear_acceleration.x = 0;
	message->linear_acceleration.y = 0;
	message->linear_acceleration.z = 0;
	for (int i = 0; i < 9; ++i) {
		message->orientation_covariance[i] = 0;
		message->angular_velocity_covariance[i] = 0;
		message->linear_acceleration_covariance[i] = 0;
	}

	this->testPublisher->publish(message);
	// std::cout << "[PublisherNode] Published Data" << std::endl;
}
