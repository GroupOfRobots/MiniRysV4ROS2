#include <iostream>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "DWMNode.hpp"

DWMNode::DWMNode(const char * nodeName, const char * topicName, std::chrono::milliseconds rate) : rclcpp::Node(nodeName) {
	// this->dwm = new DWM(115, 20);
	// this->dwm->initialize(true);

	this->publisher = this->create_publisher<std_msgs::msg::Float64>(topicName);
	this->timer = this->create_wall_timer(rate, std::bind(&DWMNode::publishData, this));
}

DWMNode::~DWMNode() {
	delete this->dwm;
}

void DWMNode::publishData() {
	auto message = std_msgs::msg::Float64();

	// message.data = this->dwm->readRange();
	message.data = 0.1;
	// std::cout << "DWM1000: publishing message with data: " << message.data << std::endl;

	this->publisher->publish(message);
}
