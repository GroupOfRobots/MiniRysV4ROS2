#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "DWMNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DWMNode>("rys", "sensor_dwm1000", 1000ms);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
