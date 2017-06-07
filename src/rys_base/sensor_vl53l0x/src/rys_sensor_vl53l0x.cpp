#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/ranges.hpp"
#include "VL53L0X.hpp"

// in order: front, back, top, left, right
// P8 connector, pins 7 through 11
const uint8_t pins[5] = { 66, 67, 69, 68, 88 };
const uint8_t addresses[5] = {
	VL53L0X_ADDRESS_DEFAULT + 2,
	VL53L0X_ADDRESS_DEFAULT + 4,
	VL53L0X_ADDRESS_DEFAULT + 6,
	VL53L0X_ADDRESS_DEFAULT + 10,
	VL53L0X_ADDRESS_DEFAULT + 12
};
bool sensorInitialized[5];
VL53L0X* sensors[5];

int readSensor(int sensorIndex) {
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

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	auto node = rclcpp::node::Node::make_shared("rys_node_sensors_vl53l0x");
	auto sensorsPublisher = node->create_publisher<rys_interfaces::msg::Ranges>("rys_sensor_vl53l0x", rmw_qos_profile_sensor_data);

	auto message = std::make_shared<rys_interfaces::msg::Ranges>();

	std::cout << "Initializing VL53L0X sensors...\n";

	for (int i = 0; i < 5; ++i) {
		sensors[i] = new VL53L0X(pins[i]);
		try {
			sensors[i]->powerOff();
		} catch (std::string & errorString) {
			std::cerr << "Error disabling sensor " << i << ": " << errorString << std::endl;
		}
	}

	if (!rclcpp::ok()) {
		return 0;
	}

	for (int i = 0; i < 5; ++i) {
		try {
			sensors[i]->init();
			sensors[i]->setTimeout(200);
			sensors[i]->setMeasurementTimingBudget(20000);
			sensors[i]->setAddress(addresses[i]);
			sensorInitialized[i] = true;
		} catch (std::string & errorString) {
			std::cerr << "Error initializing sensor " << i << ": " << errorString << std::endl;
		}
	}


	// Start continuous back-to-back measurement
	for (int i = 0; rclcpp::ok() && i < 5; ++i) {
		if (sensorInitialized[i]) {
			std::cout << "Starting VL53L0X sensor: " << i << "\n";
			sensors[i]->startContinuous();
		}
	}

	std::cout << "Working!\n";

	rclcpp::rate::WallRate loopRate(50);
	while (rclcpp::ok()) {
		// Read sensors - offloaded to separate function as there are identical checks for every sensor
		message->front = readSensor(0);
		message->back = readSensor(1);
		message->top = readSensor(2);
		message->left = readSensor(3);
		message->right = readSensor(4);

		sensorsPublisher->publish(message);

		rclcpp::spin_some(node);
		loopRate.sleep();
	}

	std::cout << "Closing!\n";

	// Clean-up: delete objects, set GPIO/XSHUT pins to low.
	for (int i = 0; i < 5; ++i) {
		if (sensorInitialized[i]) {
			sensors[i]->stopContinuous();
		}
		sensors[i]->powerOff();
		delete sensors[i];
	}
}
