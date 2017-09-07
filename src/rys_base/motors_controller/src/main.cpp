#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "MotorsControllerNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MotorsControllerNode>("rys_node_motors_controller", 10ms));
	rclcpp::shutdown();

	return 0;
}
