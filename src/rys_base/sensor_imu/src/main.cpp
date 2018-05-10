#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "IMUNode.hpp"
#include <sched.h>
#include <sys/mman.h>

using namespace std::chrono_literals;

void setRTPriority() {
	struct sched_param schedulerParams;
	schedulerParams.sched_priority = sched_get_priority_max(SCHED_FIFO);
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
	int imuOffsets[6] = {831, 1493, 1086, -155, -24, 19};
	auto node = std::make_shared<IMUNode>("rys", "sensor_imu", true, 5ms, 3000ms, imuOffsets);
	setRTPriority();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
