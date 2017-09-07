#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "rys_motors_controller/MotorsControllerNode.hpp"
#include "rys_sensor_dwm1000/DWMNode.hpp"
#include "rys_sensor_imu/IMUNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;

	auto motorsNode = std::make_shared<MotorsControllerNode>("rys_node_motors_controller", 10ms);
	auto dwmNode = std::make_shared<DWMNode>("rys_node_sensor_dwm1000", "rys_sensor_dwm1000", 1000ms);
	auto imuNode = std::make_shared<IMUNode>("rys_node_sensor_imu", "rys_sensor_imu_roll", "rys_control_imu_calibrate", 10ms, 3000ms);
	executor.add_node(motorsNode);
	executor.add_node(dwmNode);
	executor.add_node(imuNode);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
