#ifndef _IMU_NODE_HPP_
#define _IMU_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
#include "./IMU.hpp"

class IMUNode : public rclcpp::Node {
	private:
		IMU * imu;
		const int infrequentPublishRate;
		int infrequentPublishCount;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuInfrequentPublisher;

		void publishData();
	public:
		IMUNode(
			const std::string & robotName,
			const std::string & nodeName,
			const std::chrono::milliseconds loopDuration,
			const std::chrono::milliseconds calibrationDuration,
			const int imuCalibrationOffsets[6],
			const int infrequentPublishRate = 50
		);
		~IMUNode();
};

#endif
