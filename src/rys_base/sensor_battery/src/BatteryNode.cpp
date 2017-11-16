#include <iostream>
#include <fstream>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "BatteryNode.hpp"

BatteryNode::BatteryNode(
	const std::string & robotName,
	const std::string & nodeName,
	std::chrono::milliseconds rate,
	const uint8_t inputNumbers[3],
	const float coefficients[3],
	const float lowLevel
) : rclcpp::Node(nodeName, robotName, true) {
	for (int i = 0; i < 3; ++i) {
		this->coefficients[i] = coefficients[i];
		this->filenames[i] = std::string("/sys/devices/platform/ocp/44e0d000.tscadc/TI-am335x-adc/iio:device0/in_voltage") + std::to_string(inputNumbers[i]) + std::string("_raw");
	}
	this->lowLevel = lowLevel;

	this->publisher = this->create_publisher<rys_interfaces::msg::BatteryStatus>("/" + robotName + "/sensor/battery", rmw_qos_profile_default);
	this->timer = this->create_wall_timer(rate, std::bind(&BatteryNode::publishData, this));
	std::cout << "[BATT] Node ready\n";
}

BatteryNode::~BatteryNode() {}

void BatteryNode::publishData() {
	auto message = std::make_shared<rys_interfaces::msg::BatteryStatus>();

	message->header.stamp = rclcpp::Time::now();
	message->header.frame_id = "ADC";

	std::ifstream file;
	float voltages[3];
	int rawValue = 0;

	for (int i = 0; i < 3; ++i) {
		file.open(this->filenames[i], std::ios::in);
		file >> rawValue;
		file.close();
		voltages[i] = static_cast<float>(rawValue) / coefficients[i];
	}

	message->voltage_cell1 = voltages[0];
	message->voltage_cell2 = voltages[1] - voltages[0];
	message->voltage_cell3 = voltages[2] - voltages[1];

	if (message->voltage_cell1 < this->lowLevel || message->voltage_cell2 < this->lowLevel || message->voltage_cell3 < this->lowLevel) {
		message->voltage_low = true;
	} else {
		message->voltage_low = false;
	}

	this->publisher->publish(message);
}
