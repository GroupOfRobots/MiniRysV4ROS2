#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#define I2C_DEV_PATH "/dev/i2c-2"
#include "RangesNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	// front, back, top, left, right
	// TIMER4, TIMER7, TIMER5, GPIO_50, GPIO_51
	// GPIO2_2, GPIO2_3, GPIO2_5, GPIO_1_18, GPIO1_19
	const uint8_t pins[5] = {
		67,
		51,
		66,
		69,
		50
	};
	const uint8_t addresses[5] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6,
		VL53L0X_ADDRESS_DEFAULT + 10,
		VL53L0X_ADDRESS_DEFAULT + 12
	};

	auto node = std::make_shared<RangesNode>("rys", "sensor_ranges", true, 20ms, pins, addresses);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
