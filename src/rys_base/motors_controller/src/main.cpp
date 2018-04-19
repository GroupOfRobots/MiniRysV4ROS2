#include <chrono>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "MotorsControllerNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	double wheelRadius = 0.056;
	double baseWidth = 0.135;
	float temporaryValue = 0.0;

	for (int i = 2; i <= argc; i+=2) {
		if (!std::strcmp(argv[i-1], "-r")){
			wheelRadius = atof(argv[i]);
		} else if (!std::strcmp(argv[i-1], "-w")){
			baseWidth = atof(argv[i]);
		} else if (!std::strcmp(argv[i-1], "-t")){
			temporaryValue = atof(argv[i]);
		}
	}

	std::cout << "[MOTORS] wheelRadius = " << wheelRadius << "; baseWidth = " << baseWidth << ";\n";

	auto node = std::make_shared<MotorsControllerNode>("rys", "motors_controller", true, 10ms, wheelRadius, baseWidth, 0, temporaryValue);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
