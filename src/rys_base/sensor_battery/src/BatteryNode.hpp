#ifndef _BATTERY_NODE_HPP_
#define _BATTERY_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/battery_status.hpp"

class BatteryNode : public rclcpp::Node {
	private:
		float coefficients[3];
		std::string filenames[3];
		float lowLevel;
		bool isCritical;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<rys_interfaces::msg::BatteryStatus>::SharedPtr publisher;

		void publishData();
	public:
		BatteryNode(
			const std::string & robotName,
			const std::string & nodeName,
			std::chrono::milliseconds rate,
			const uint8_t inputNumbers[3],
			const float coefficients[3],
			const float lowLevel = 3.3f
		);
		~BatteryNode();
};

#endif
