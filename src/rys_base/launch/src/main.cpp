#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "rys_motors_controller/MotorsControllerNode.hpp"
#include "rys_sensor_battery/BatteryNode.hpp"
#include "rys_sensor_dwm1000/DWMNode.hpp"
#include "rys_sensor_imu/IMUNode.hpp"
#include "rys_sensor_vl53l0x/VL53L0XNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;

	const uint8_t batteryInputNumbers[3] = { 3, 1, 6 };
	const float batteryCoefficients[3] = { 734.4895, 340.7509, 214.1773 };

	const uint8_t vl53l0xPins[5] = { 66, 67, 69, 68, 88 };
	const uint8_t vl53l0xAddresses[5] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6,
		VL53L0X_ADDRESS_DEFAULT + 10,
		VL53L0X_ADDRESS_DEFAULT + 12
	};

	auto motorsNode = std::make_shared<MotorsControllerNode>("rys_node_motors_controller", 10ms);
	auto batteryNode = std::make_shared<BatteryNode>("rys_node_sensor_battery", "rys_sensor_battery", 1000ms, batteryInputNumbers, batteryCoefficients);
	auto dwmNode = std::make_shared<DWMNode>("rys_node_sensor_dwm1000", "rys_sensor_dwm1000", 1000ms);
	auto imuNode = std::make_shared<IMUNode>("rys_node_sensor_imu", "rys_sensor_imu_roll", "rys_control_imu_calibrate", 10ms, 3000ms);
	auto vl53l0xNode = std::make_shared<VL53L0XNode>("rys_node_sensor_vl53l0x", "rys_sensor_vl53l0x", 20ms, vl53l0xPins, vl53l0xAddresses);
	executor.add_node(motorsNode);
	executor.add_node(batteryNode);
	executor.add_node(dwmNode);
	executor.add_node(imuNode);
	executor.add_node(vl53l0xNode);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
