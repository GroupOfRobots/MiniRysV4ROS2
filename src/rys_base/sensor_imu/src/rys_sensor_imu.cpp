#include <chrono>
#include <iostream>
#include <memory>

#include "IMU.h"
#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/imu_roll_rotation.hpp"
#include "std_msgs/msg/empty.hpp"

const int rate = 100;
const int calibrationDuration = 3000;

bool calibration = false;
std::chrono::time_point<std::chrono::high_resolution_clock> calibrationEndTime;
float calibrationValuesSum;
unsigned long int calibrationIterations;

void imuCalibrateCallback(const std_msgs::msg::Empty::SharedPtr message) {
	// Prevent unused parameter warning
	(void)message;

	std::cout << "Calibration: collecting data (" << calibrationDuration << "ms)...\n";
	calibrationEndTime = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(calibrationDuration);
	calibrationValuesSum = 0;
	calibrationIterations = 0;
	calibration = true;
}

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	auto node = rclcpp::node::Node::make_shared("rys_node_sensor_imu");
	auto imuCalibrationSubscriber = node->create_subscription<std_msgs::msg::Empty>("rys_control_imu_calibrate", imuCalibrateCallback);
	auto imuPublisher = node->create_publisher<rys_interfaces::msg::ImuRollRotation>("rys_sensor_imu_roll", rmw_qos_profile_sensor_data);

	auto message = std::make_shared<rys_interfaces::msg::ImuRollRotation>();

	std::cout << "Initializing IMU...\n";

	IMU imu;
	try {
		imu.initialize(rate);
		imu.resetFIFO();
	} catch (std::string & error) {
		std::cout << "Error initializing: " << error << std::endl;
		return 1;
	}

	std::cout << "Working!\n";

	rclcpp::rate::WallRate loopRate(rate);
	while (rclcpp::ok()) {
		try {
			float roll = imu.getRoll();
			message->roll = roll;

			float rotationX, rotationY, rotationZ;
			imu.getGyro(&rotationX, &rotationY, &rotationZ);
			message->rotation_x = rotationX;
			message->rotation_y = rotationY;
			message->rotation_z = rotationZ;

			if (calibration) {
				calibrationValuesSum += roll;
				calibrationIterations++;
				if (std::chrono::high_resolution_clock::now() >= calibrationEndTime) {
					calibration = false;
					float averageRoll = calibrationValuesSum / calibrationIterations;
					imu.setOffsets(0, 0, averageRoll);
					std::cout << "Calibration: data collected, average offset: " << averageRoll << std::endl;
				}
			}
		} catch (std::string & error) {
			std::cout << "Error getting IMU reading: " << error << std::endl;
			break;
		}

		imuPublisher->publish(message);
		rclcpp::spin_some(node);
		loopRate.sleep();
	}

	return 0;
}
