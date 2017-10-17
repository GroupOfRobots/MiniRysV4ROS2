#ifndef _DWM_NODE_HPP_
#define _DWM_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "DWM.hpp"

class DWMNode : public rclcpp::Node {
	private:
		DWM * dwm;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

		void publishData();
	public:
		DWMNode(const char * nodeName, const char * topicName, std::chrono::milliseconds rate);
		~DWMNode();
};

#endif
