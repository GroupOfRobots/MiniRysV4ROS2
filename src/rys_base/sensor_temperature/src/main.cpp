#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "TemperatureNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	const uint8_t inputNumber = 5;
	const float coefficient = 564.7637;

	rclcpp::spin(std::make_shared<TemperatureNode>("rys_node_sensor_temperature", "rys_sensor_temperature", 2000ms, inputNumber, coefficient));
	rclcpp::shutdown();

	return 0;
}
