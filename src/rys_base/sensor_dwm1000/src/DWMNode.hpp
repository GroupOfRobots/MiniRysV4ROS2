#ifndef _DWM_NODE_HPP_
#define _DWM_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "DWM.hpp"

class DWMNode : public rclcpp::Node {
	private:
		DWM * dwm;
		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher;
		void publishData();
	public:
		// DWMNode(const char * nodeName, const int rate);
		DWMNode(const char * nodeName, const char * topicName, std::chrono::milliseconds rate);
		~DWMNode();
};

#endif
