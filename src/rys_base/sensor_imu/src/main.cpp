#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "IMUNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	auto node = std::make_shared<IMUNode>("rys", "sensor_imu", 10ms, 3000ms);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
