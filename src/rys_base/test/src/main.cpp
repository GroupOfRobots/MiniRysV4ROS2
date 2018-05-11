#include <chrono>
#include <cstring>
#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include "rclcpp/rclcpp.hpp"

#include "SubscriberNode.hpp"
#include "PublisherNode.hpp"

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
	rclcpp::executors::SingleThreadedExecutor executor;

	std::string robotName("rys");
	bool useIPC = false;

	auto pubNode = std::make_shared<PublisherNode>("rys", "publisher", useIPC, 2ms);
	auto subNode = std::make_shared<SubscriberNode>("rys", "subscriber", useIPC);

	setRTPriority();

	executor.add_node(pubNode);
	// executor.add_node(subNode);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
