#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "IMUNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	int imuOffsets[6] = {831, 1493, 1086, -155, -24, 19};
	auto node = std::make_shared<IMUNode>("rys", "sensor_imu", true, 10ms, 3000ms, imuOffsets);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
