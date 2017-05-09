#include <chrono>
#include <iostream>
#include <memory>

#include "IMU.h"
#include "rclcpp/rclcpp.hpp"
#include "rys_messages/msg/imu_roll.hpp"
#include "std_msgs/msg/empty.hpp"

const int rate = 100;
const float filteringFactor = 0.98f;
const int calibrationDuration = 5000;

bool calibration = false;
std::chrono::time_point<std::chrono::high_resolution_clock> calibrationEndTime;
float calibrationValuesSum;
unsigned long int calibrationIterations;

void imuCalibrateCallback(const std_msgs::msg::Empty::SharedPtr message) {
	calibrationEndTime = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(calibrationDuration);
	calibrationValuesSum = 0;
	calibrationIterations = 0;
	calibration = true;
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);

	IMU imu;
	try {
		imu.initialize(rate);
		imu.resetFIFO();
	} catch (std::string & error) {
		std::cout << "Error initializing: " << error << std::endl;
		return 1;
	}

	auto node = rclcpp::node::Node::make_shared("rys_node_sensor_imu");
	auto imuCalibrationSubscriber = node->create_subscription<std_msgs::msg::Empty>("rys_imu_calibrate", imuCalibrateCallback);
	auto imuPublisher = node->create_publisher<rys_messages::msg::ImuRoll>("rys_imu", rmw_qos_profile_sensor_data);

	auto msg = std::make_shared<rys_messages::msg::ImuRoll>();

	float previousValue = 0;
	rclcpp::rate::WallRate loopRate(rate);
	while (rclcpp::ok()) {
		try {
			float roll = imu.getRoll();
			msg->roll = filteringFactor * roll + (1.0f - filteringFactor) * previousValue;
			previousValue = msg->roll;

			if (calibration) {
				calibrationValuesSum += roll;
				calibrationIterations++;
				if (std::chrono::high_resolution_clock::now() >= calibrationEndTime) {
					calibration = false;
					float averageRoll = calibrationValuesSum / calibrationIterations;
					imu.setOffsets(0, 0, averageRoll);
				}
			}
		} catch (std::string & error) {
			std::cout << "Error getting IMU reading: " << error << std::endl;
			break;
		}

		imuPublisher->publish(msg);
		rclcpp::spin_some(node);
		loopRate.sleep();
	}

	return 0;
}
