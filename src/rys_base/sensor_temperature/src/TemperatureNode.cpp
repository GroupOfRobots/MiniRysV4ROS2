#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "TemperatureNode.hpp"

using namespace std::chrono_literals;

TemperatureNode::TemperatureNode(
	const std::string & robotName,
	const std::string & nodeName,
	std::chrono::milliseconds rate,
	const uint8_t inputNumber,
	const float coefficient,
	const float criticalLevel
) : rclcpp::Node(nodeName, robotName, true) {
	this->coefficient = coefficient;
	this->criticalLevel = criticalLevel;
	this->filename = std::string("/sys/devices/platform/ocp/44e0d000.tscadc/TI-am335x-adc/iio:device0/in_voltage") + std::to_string(inputNumber) + std::string("_raw");

	this->publisher = this->create_publisher<rys_interfaces::msg::TemperatureStatus>("/" + robotName + "/sensor/temperature", rmw_qos_profile_default);
	this->timer = this->create_wall_timer(rate, std::bind(&TemperatureNode::publishData, this));
	std::cout << "[TEMP] Node ready\n";
}

TemperatureNode::~TemperatureNode() {}

void TemperatureNode::publishData() {
	auto message = std::make_shared<rys_interfaces::msg::TemperatureStatus>();

	message->header.stamp = this->now();
	message->header.frame_id = "LM35";

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
	message->temperature = (voltageSum / readings) * 100;

	if (message->temperature > this->criticalLevel) {
		message->temperature_critical = true;
	} else {
		message->temperature_critical = false;
	}

	this->publisher->publish(message);
}
