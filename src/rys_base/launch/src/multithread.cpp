#include <chrono>
#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include "rclcpp/rclcpp.hpp"

#include "rys_motors_controller/MotorsControllerNode.hpp"
// #include "rys_sensor_battery/BatteryNode.hpp"
// #include "rys_sensor_dwm1000/DWMNode.hpp"
#include "rys_sensor_imu/IMUNode.hpp"
// #include "rys_sensor_temperature/TemperatureNode.hpp"
// #include "rys_sensor_ranges/RangesNode.hpp"

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
	rclcpp::executors::MultiThreadedExecutor executor;

	std::string robotName("rys");
	bool useIPC = false;

	double wheelRadius = 0.056;
	double baseWidth = 0.135;

	int imuOffsets[6] = {831, 1493, 1086, -155, -24, 19};

	// const uint8_t batteryInputNumbers[3] = { 3, 1, 6 };
	// const float batteryCoefficients[3] = { 734.4895, 340.7509, 214.1773 };
	// const uint8_t temperatureInputNumber = 5;
	// const float temperatureCoefficient = 564.7637;

	// front, back, top, left, right
	/*
	const uint8_t vl53l0xPins[5] = { 67, 51, 66, 69, 50 };
	const uint8_t vl53l0xAddresses[5] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6,
		VL53L0X_ADDRESS_DEFAULT + 10,
		VL53L0X_ADDRESS_DEFAULT + 12
	};
	*/

	// auto batteryNode = std::make_shared<BatteryNode>(robotName, "sensor_battery", useIPC, 1000ms, batteryInputNumbers, batteryCoefficients);
	// auto dwmNode = std::make_shared<DWMNode>(robotName, "sensor_dwm1000", useIPC, 1000ms);
	auto imuNode = std::make_shared<IMUNode>(robotName, "sensor_imu", useIPC, 20ms, 3000ms, imuOffsets);
	// auto temperatureNode = std::make_shared<TemperatureNode>(robotName, "sensor_temperature", useIPC, 2000ms, temperatureInputNumber, temperatureCoefficient);
	// auto rangesNode = std::make_shared<RangesNode>(robotName, "sensor_ranges", useIPC, 20ms, vl53l0xPins, vl53l0xAddresses);
	auto motorsNode = std::make_shared<MotorsControllerNode>(robotName, "motors_controller", useIPC, 10ms, wheelRadius, baseWidth);

	executor.add_node(motorsNode);
	// executor.add_node(batteryNode);
	// executor.add_node(dwmNode);
	executor.add_node(imuNode);
	// executor.add_node(temperatureNode);
	// executor.add_node(rangesNode);

	setRTPriority();

	executor.spin();

	rclcpp::shutdown();
	return 0;
}
