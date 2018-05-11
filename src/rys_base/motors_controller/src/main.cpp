#include <chrono>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "MotorsControllerNode.hpp"
#include <sched.h>
#include <sys/mman.h>

using namespace std::chrono_literals;

void setRTPriority() {
	struct sched_param schedulerParams;
	schedulerParams.sched_priority = sched_get_priority_max(SCHED_FIFO)-1;
	std::cout << "[MAIN] Setting RT scheduling, priority " << schedulerParams.sched_priority << std::endl;
	if (sched_setscheduler(0, SCHED_FIFO, &schedulerParams) == -1) {
		std::cout << "[MAIN] WARNING: Setting RT scheduling failed: " << std::strerror(errno) << std::endl;
		return;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		std::cout << "[MAIN] WARNING: Failed to lock memory: " << std::strerror(errno) << std::endl;
	}
}

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
	setRTPriority();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
