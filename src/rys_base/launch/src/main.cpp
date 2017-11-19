#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "rys_motors_controller/MotorsControllerNode.hpp"
#include "rys_sensor_battery/BatteryNode.hpp"
#include "rys_sensor_dwm1000/DWMNode.hpp"
#include "rys_sensor_imu/IMUNode.hpp"
#include "rys_sensor_temperature/TemperatureNode.hpp"
#include "rys_sensor_vl53l0x/VL53L0XNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;

	const uint8_t batteryInputNumbers[3] = { 3, 1, 6 };
	const float batteryCoefficients[3] = { 734.4895, 340.7509, 214.1773 };
	const uint8_t temperatureInputNumber = 5;
	const float temperatureCoefficient = 564.7637;

	// front, back, top, left, right
	const uint8_t vl53l0xPins[5] = { 67, 51, 66, 69, 50 };
	const uint8_t vl53l0xAddresses[5] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6,
		VL53L0X_ADDRESS_DEFAULT + 10,
		VL53L0X_ADDRESS_DEFAULT + 12
	};

	auto motorsNode = std::make_shared<MotorsControllerNode>("rys", "motors_controller", 10ms);
	auto batteryNode = std::make_shared<BatteryNode>("rys", "sensor_battery", 1000ms, batteryInputNumbers, batteryCoefficients);
	auto dwmNode = std::make_shared<DWMNode>("rys", "sensor_dwm1000", 1000ms);
	auto imuNode = std::make_shared<IMUNode>("rys", "sensor_imu", 1ms, 3000ms);
	auto temperatureNode = std::make_shared<TemperatureNode>("rys", "sensor_temperature", 2000ms, temperatureInputNumber, temperatureCoefficient);
	auto vl53l0xNode = std::make_shared<VL53L0XNode>("rys", "sensor_ranges", 20ms, vl53l0xPins, vl53l0xAddresses);

	executor.add_node(motorsNode);
	executor.add_node(batteryNode);
	executor.add_node(dwmNode);
	executor.add_node(imuNode);
	executor.add_node(temperatureNode);
	executor.add_node(vl53l0xNode);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
