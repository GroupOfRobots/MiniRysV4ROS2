#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "IMUNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IMUNode>("rys_node_sensor_imu", "rys_sensor_imu_roll", "rys_control_imu_calibrate", 10ms, 3000ms));
	rclcpp::shutdown();

	return 0;
}
