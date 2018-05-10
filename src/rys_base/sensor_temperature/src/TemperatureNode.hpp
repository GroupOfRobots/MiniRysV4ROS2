#ifndef _TEMPERATURE_NODE_HPP_
#define _TEMPERATURE_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/temperature_status.hpp"

class TemperatureNode : public rclcpp::Node {
	private:
		std::string filename;
		float coefficient;
		float criticalLevel;
		float hysteresis;
		bool isCritical;
		int readings;
		int currentReadings;
		float voltageSum;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<rys_interfaces::msg::TemperatureStatus>::SharedPtr publisher;

		void readData();
		void publishData(const float temperature);
	public:
		TemperatureNode(
			const std::string & robotName,
			const std::string & nodeName,
			const bool useIPC,
			std::chrono::milliseconds rate,
			const uint8_t inputNumber,
			const float coefficient,
			const float criticalLevel = 60.0f,
			const float hysteresis = 10.0f,
			const int readings = 5
		);
		~TemperatureNode();
};

#endif
