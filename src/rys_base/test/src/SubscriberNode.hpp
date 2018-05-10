#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"

class SubscriberNode : public rclcpp::Node {
	private:
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr testSubscriber;

		std::chrono::time_point<std::chrono::high_resolution_clock> previous;
		std::chrono::time_point<std::chrono::high_resolution_clock> timeNow;

		int numOfImuMessages;
		float frequency;

		void testMessageCallback(const sensor_msgs::msg::Imu::SharedPtr message);
	public:
		SubscriberNode(
			const std::string & robotName,
			const std::string & nodeName,
			const bool useIPC
		);
};
