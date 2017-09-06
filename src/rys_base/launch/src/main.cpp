#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "../../motors_controller/src/MotorsControllerNode.hpp"
#include "../../sensor_dwm1000/src/DWMNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;

	auto motorsNode = std::make_shared<MotorsControllerNode>("rys_node_motors_controller", 10ms);
	auto dwmNode = std::make_shared<DWMNode>("rys_node_sensor_dwm1000", "rys_sensor_dwm1000", 1000ms);
	executor.add_node(motorsNode);
	executor.add_node(dwmNode);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
