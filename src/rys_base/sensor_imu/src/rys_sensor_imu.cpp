#include <iostream>
#include <memory>

#include "IMU.h"
#include "rclcpp/rclcpp.hpp"
#include "rys_messages/msg/imu_yaw_pitch_roll.hpp"

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);

	IMU imu;
	try {
		imu.initialize();
		imu.resetFIFO();
		imu.calibrate();
	} catch (std::string & error) {
		std::cout << "Error initializing: " << error << std::endl;
		return 1;
	}
	usleep(100 * 1000);

	auto node = rclcpp::node::Node::make_shared("rys_node_sensor_IMU");
	auto imuPublisher = node->create_publisher<rys_messages::msg::ImuYawPitchRoll>("rys_queue_sensor_IMU", rmw_qos_profile_sensor_data);

	auto msg = std::make_shared<rys_messages::msg::ImuYawPitchRoll>();

	while (rclcpp::ok()) {
		try {
			imu.getYawPitchRoll(&(msg->yaw), &(msg->pitch), &(msg->roll));
		} catch (std::string & error) {
			std::cout << "Error getting IMU reading: " << error << std::endl;
			break;
		}

		imuPublisher->publish(msg);
		rclcpp::spin_some(node);
	}

	return 0;
}
