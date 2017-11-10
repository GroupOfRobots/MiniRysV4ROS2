#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "MotorsControllerNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MotorsControllerNode>("rys", "motors_controller", 10ms);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
