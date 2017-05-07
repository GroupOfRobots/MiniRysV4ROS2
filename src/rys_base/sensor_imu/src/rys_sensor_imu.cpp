#include <iostream>
#include <memory>

#include "IMU.h"
#include "rclcpp/rclcpp.hpp"
#include "rys_messages/msg/imu_roll.hpp"

int main(int argc, char * argv[]) {
	const int rate = 100;
	const float filteringFactor = 0.95f;

	rclcpp::init(argc, argv);

	IMU imu;
	try {
		imu.initialize(rate);
		imu.resetFIFO();
		// imu.calibrate();
	} catch (std::string & error) {
		std::cout << "Error initializing: " << error << std::endl;
		return 1;
	}
	// usleep(100 * 1000);

	auto node = rclcpp::node::Node::make_shared("rys_node_sensor_imu");
	auto imuPublisher = node->create_publisher<rys_messages::msg::ImuRoll>("rys_imu", rmw_qos_profile_sensor_data);

	auto msg = std::make_shared<rys_messages::msg::ImuRoll>();

	float previousValue = 0;
	rclcpp::rate::WallRate loopRate(rate);
	while (rclcpp::ok()) {
		try {
			msg->roll = filteringFactor * imu.getRoll() + (1.0f - filteringFactor) * previousValue;
			previousValue = msg->roll;
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
