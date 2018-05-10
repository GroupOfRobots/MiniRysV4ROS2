#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"

class PublisherNode : public rclcpp::Node {
	private:
		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr testPublisher;

		std::chrono::time_point<std::chrono::high_resolution_clock> previous;
		std::chrono::time_point<std::chrono::high_resolution_clock> timeNow;

		int numOfImuMessages;
		float frequency;

		void publishData();
	public:
		PublisherNode(
			const std::string & robotName,
			const std::string & nodeName,
			const bool useIPC,
			const std::chrono::milliseconds loopDuration
		);
};
