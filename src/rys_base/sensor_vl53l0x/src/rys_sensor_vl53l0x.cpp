#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/ranges.hpp"
#include "VL53L0X.hpp"

// in order: front, back, top, left, right
const uint8_t pins[5] = { 0, 1, 2, 3, 4};
const uint8_t addresses[5] = {
	VL53L0X_ADDRESS_DEFAULT + 2,
	VL53L0X_ADDRESS_DEFAULT + 4,
	VL53L0X_ADDRESS_DEFAULT + 6,
	VL53L0X_ADDRESS_DEFAULT + 10,
	VL53L0X_ADDRESS_DEFAULT + 12
};

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	auto node = rclcpp::node::Node::make_shared("rys_node_motors_vl53l0x");
	auto sensorsPublisher = node->create_publisher<rys_interfaces::msg::Ranges>("rys_sensor_sonars", rmw_qos_profile_sensor_data);

	auto message = std::make_shared<rys_interfaces::msg::Ranges>();

	std::cout << "Initializing VL53L0X sensors...\n";

	VL53L0X* sensors[5];
	for (int i = 0; i < 5; ++i) {
		sensors[i] = new VL53L0X(pins[i]);
		sensors[i]->powerOff();
	}

	if(!rclcpp::ok()) {
		return 0;
	}

	for (int i = 0; i < 5; ++i) {
		sensors[i]->init();
		sensors[i]->setTimeout(200);
		sensors[i]->setMeasurementTimingBudget(20000);
		sensors[i]->setAddress(addresses[i]);
	}
	// Start continuous back-to-back measurement
	for (int i = 0; rclcpp::ok() && i < 5; ++i) {
		sensors[i]->startContinuous();
	}

	std::cout << "Working!\n";

	rclcpp::rate::WallRate loopRate(50);
	while(rclcpp::ok()) {
		try {
			// This can throw
			message->front = sensors[0]->readRangeContinuousMillimeters();
			message->back = sensors[1]->readRangeContinuousMillimeters();
			message->top = sensors[2]->readRangeContinuousMillimeters();
			message->left = sensors[3]->readRangeContinuousMillimeters();
			message->right = sensors[4]->readRangeContinuousMillimeters();

			if (sensors[0]->timeoutOccurred()) {
				message->front = -1;
			}
			if (sensors[1]->timeoutOccurred()) {
				message->back = -1;
			}
			if (sensors[2]->timeoutOccurred()) {
				message->top = -1;
			}
			if (sensors[3]->timeoutOccurred()) {
				message->left = -1;
			}
			if (sensors[4]->timeoutOccurred()) {
				message->right = -1;
			}
		} catch (std::string & error) {
			std::cout << "Error reading distances from sonars: " << error << std::endl;
		}

		sensorsPublisher->publish(message);

		rclcpp::spin_some(node);
		loopRate.sleep();
	}


	// Clean-up: delete objects, set GPIO/XSHUT pins to low.
	for (int i = 0; i < 5; ++i) {
		sensors[i]->stopContinuous();
		delete sensors[i];
	}
}
