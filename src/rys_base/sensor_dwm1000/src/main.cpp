#include <chrono>
#include "DWMNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DWMNode>("rys_node_sensor_dwm1000", "rys_sensor_dwm1000", 1000ms));
	rclcpp::shutdown();

	return 0;
}
