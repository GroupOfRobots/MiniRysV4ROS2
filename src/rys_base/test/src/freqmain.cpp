#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "FrequencyCounter.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <sched.h>
#include <sys/mman.h>

using namespace std::chrono_literals;

void setRTPriority() {
	struct sched_param schedulerParams;
	schedulerParams.sched_priority = sched_get_priority_max(SCHED_RR)-1;
	std::cout << "[MAIN] Setting RT scheduling, priority " << schedulerParams.sched_priority << std::endl;
	if (sched_setscheduler(0, SCHED_RR, &schedulerParams) == -1) {
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

	const std::string robotName = "rys";
	const std::string nodeName = "counter";
	std::chrono::milliseconds loopDuration = 2ms;
	for (int i = 2; i <= argc; i+=2) {
		if (!std::strcmp(argv[i-1], "-t")){
			loopDuration = std::chrono::milliseconds(atoi(argv[i]));
		}
	}

	auto counter = std::make_shared<FrequencyCounter>();
	auto timer_callback = 
		[&counter](){
			counter->count();
		};
	auto node = rclcpp::Node::make_shared(nodeName, robotName, true);
	auto timer = node->create_wall_timer(loopDuration, timer_callback);

	setRTPriority();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
