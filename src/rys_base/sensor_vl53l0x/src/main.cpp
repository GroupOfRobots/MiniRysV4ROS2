#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#define I2C_DEV_PATH "/dev/i2c-2"
#include "VL53L0XNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	const uint8_t pins[5] = { 66, 67, 69, 68, 88 };
	const uint8_t addresses[5] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6,
		VL53L0X_ADDRESS_DEFAULT + 10,
		VL53L0X_ADDRESS_DEFAULT + 12
	};

	rclcpp::spin(std::make_shared<VL53L0XNode>("rys_node_sensor_vl53l0x", "rys_sensor_vl53l0x", 20ms, pins, addresses));
	rclcpp::shutdown();

	return 0;
}
