#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "BatteryNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	const uint8_t inputNumbers[3] = { 3, 1, 6 };
	const float coefficients[3] = { 734.4895, 340.7509, 214.1773 };

	auto node = std::make_shared<BatteryNode>("rys", "sensor_battery", true, 1000ms, inputNumbers, coefficients);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
