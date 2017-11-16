#ifndef _TEMPERATURE_NODE_HPP_
#define _TEMPERATURE_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/temperature_status.hpp"

class TemperatureNode : public rclcpp::Node {
	private:
		float coefficient;
		std::string filename;
		float criticalLevel;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<rys_interfaces::msg::TemperatureStatus>::SharedPtr publisher;

		void publishData();
	public:
		TemperatureNode(
			const std::string & robotName,
			const std::string & nodeName,
			std::chrono::milliseconds rate,
			const uint8_t inputNumber,
			const float coefficient,
			const float criticalLevel = 70.0f
		);
		~TemperatureNode();
};

#endif
