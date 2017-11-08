#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "TemperatureNode.hpp"

using namespace std::chrono_literals;

TemperatureNode::TemperatureNode(
	const char * nodeName,
	const char * topicName,
	std::chrono::milliseconds rate,
	const uint8_t inputNumber,
	const float coefficient,
	const float criticalLevel
) : rclcpp::Node(nodeName) {
	this->coefficient = coefficient;
	this->filename = std::string("/sys/devices/platform/ocp/44e0d000.tscadc/TI-am335x-adc/iio:device0/in_voltage") + std::to_string(inputNumber) + std::string("_raw");
	this->criticalLevel = criticalLevel;

	std::string criticalTopicName = std::string(topicName) + std::string("_critical");

	this->publisherTemperature = this->create_publisher<std_msgs::msg::Float32>(topicName);
	this->publisherCriticalTemperature = this->create_publisher<std_msgs::msg::Bool>(criticalTopicName.c_str());
	this->timer = this->create_wall_timer(rate, std::bind(&TemperatureNode::publishData, this));
}

TemperatureNode::~TemperatureNode() {}

void TemperatureNode::publishData() {
	auto message = std_msgs::msg::Float32();
	auto messageCriticalLevel = std_msgs::msg::Bool();

	const int readings = 5;

	std::ifstream file;
	float voltageSum = 0;
	int rawValue = 0;

	for (int i = 0; i < readings; ++i) {
		file.open(this->filename, std::ios::in);
		file >> rawValue;
		file.close();
		voltageSum += static_cast<float>(rawValue) / coefficient;
		rclcpp::sleep_for(100ms);
	}

	// Divide sum by number of readings and multiply by 100
	// The sensor is 10mV/C, so to convert volts to degrees
	message.data = (voltageSum / readings) * 100;

	if (message.data > this->criticalLevel) {
		messageCriticalLevel.data = true;
	} else {
		messageCriticalLevel.data = false;
	}

	this->publisherCriticalTemperature->publish(messageCriticalLevel);
	this->publisherTemperature->publish(message);

	std::cout << "Publishing temperature: " << message.data << " | critical: " << messageCriticalLevel.data << std::endl;
}
