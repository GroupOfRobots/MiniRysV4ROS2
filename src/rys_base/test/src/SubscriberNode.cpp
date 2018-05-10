#include "SubscriberNode.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <stdexcept>

#include <sched.h>
#include <sys/mman.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

SubscriberNode::SubscriberNode(
	const std::string & robotName,
	const std::string & nodeName,
	const bool useIPC
) : rclcpp::Node(nodeName, robotName, useIPC) {
	this->previous = std::chrono::high_resolution_clock::now();
	this->timeNow = std::chrono::high_resolution_clock::now();

	const rmw_qos_profile_t imuQosProfile = {
		RMW_QOS_POLICY_HISTORY_KEEP_LAST,
		100,
		RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
		RMW_QOS_POLICY_DURABILITY_VOLATILE,
		false
	};

	this->testSubscriber = this->create_subscription<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", std::bind(&SubscriberNode::testMessageCallback, this, _1), rmw_qos_profile_sensor_data);
	// this->testSubscriber = this->create_subscription<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", std::bind(&SubscriberNode::testMessageCallback, this, _1), imuQosProfile);
	// this->testSubscriber = this->create_subscription<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imu", std::bind(&SubscriberNode::testMessageCallback, this, _1));

	std::cout << "[Subscriber] Node ready\n";

	this->numOfImuMessages = 0;
	this->frequency = 0.0;
}

void SubscriberNode::testMessageCallback(const sensor_msgs::msg::Imu::SharedPtr message) {
	this->numOfImuMessages++;
	if (this->numOfImuMessages > 9999) {
		this->previous = this->timeNow;
		this->timeNow = std::chrono::high_resolution_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->timeNow - this->previous);
		float loopTime = loopTimeSpan.count();
		this->frequency = this->numOfImuMessages/loopTime;
		std::cout <<"[Subscriber] Message frequency: " << this->frequency << std::endl;
		this->numOfImuMessages = 0;
	}
	return;
}