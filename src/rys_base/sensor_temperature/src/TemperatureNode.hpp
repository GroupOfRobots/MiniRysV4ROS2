#ifndef _TEMPERATURE_NODE_HPP_
#define _TEMPERATURE_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

class TemperatureNode : public rclcpp::Node {
	private:
		float coefficient;
		std::string filename;
		float criticalLevel;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisherTemperature;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisherCriticalTemperature;

		void publishData();
	public:
		TemperatureNode(const char * nodeName, const char * topicName, std::chrono::milliseconds rate, const uint8_t inputNumber, const float coefficient, const float criticalLevel = 70.0f);
		~TemperatureNode();
};

#endif
