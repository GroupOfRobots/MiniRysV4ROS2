#include <iostream>

#include "VL53L0XNode.hpp"

VL53L0XNode::VL53L0XNode(
	const char * nodeName,
	const char * publishTopicName,
	const std::chrono::milliseconds loopDuration,
	const uint8_t pins[5],
	const uint8_t addresses[5]
) : rclcpp::Node(nodeName) {
	// Create sensor objects
	for (int i = 0; i < 5; ++i) {
		this->sensors[i] = new VL53L0X(pins[i]);
		this->sensorInitialized[i] = false;
		try {
			this->sensors[i]->powerOff();
		} catch (std::string & errorString) {
			std::cerr << "Error disabling sensor " << i << ": " << errorString << std::endl;
		}
	}

	if (!rclcpp::ok()) {
		return;
	}

	// Initialize the sensors
	for (int i = 0; i < 5; ++i) {
		try {
			this->sensors[i]->init();
			this->sensors[i]->setTimeout(200);
			this->sensors[i]->setMeasurementTimingBudget(20000);
			this->sensors[i]->setAddress(addresses[i]);
			this->sensorInitialized[i] = true;
		} catch (std::string & errorString) {
			std::cerr << "Error initializing sensor " << i << ": " << errorString << std::endl;
		}
	}

	if (!rclcpp::ok()) {
		return;
	}

	// Start continuous back-to-back measurement
	for (int i = 0; rclcpp::ok() && i < 5; ++i) {
		if (this->sensorInitialized[i]) {
			std::cout << "Starting VL53L0X sensor: " << i << "\n";
			this->sensors[i]->startContinuous();
		}
	}

	this->publisher = this->create_publisher<rys_interfaces::msg::Ranges>(publishTopicName, rmw_qos_profile_sensor_data);
	this->timer = this->create_wall_timer(loopDuration, std::bind(&VL53L0XNode::sensorsReadAndPublishData, this));
}

VL53L0XNode::~VL53L0XNode() {
	for (int i = 0; i < 5; ++i) {
		if (sensorInitialized[i]) {
			sensors[i]->stopContinuous();
		}
		sensors[i]->powerOff();
		delete sensors[i];
	}
}

int VL53L0XNode::readSensor(int sensorIndex) {
	if (!sensorInitialized[sensorIndex]) {
		return -1;
	}

	int value = -1;
	// Actual reading can throw
	try {
		value = sensors[sensorIndex]->readRangeContinuousMillimeters();
	} catch (std::string & error) {
		std::cout << "Error reading distances from sensor " << sensorIndex << ": " << error << std::endl;
		return -1;
	}

	// Checking timeout can not
	if (sensors[sensorIndex]->timeoutOccurred()) {
		std::cout << "Sensor " << sensorIndex << " timeout!\n";
		return -1;
	}

	return value;
}

void VL53L0XNode::sensorsReadAndPublishData() {
	auto message = std::make_shared<rys_interfaces::msg::Ranges>();

	// Read sensors - offloaded to separate function as there are identical checks for every sensor
	message->front = this->readSensor(0);
	message->back = this->readSensor(1);
	message->top = this->readSensor(2);
	message->left = this->readSensor(3);
	message->right = this->readSensor(4);

	this->publisher->publish(message);
}
