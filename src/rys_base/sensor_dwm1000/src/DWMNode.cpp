#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "DWMNode.hpp"

DWMNode::DWMNode(const char * nodeName, const char * topicName, std::chrono::milliseconds rate) : rclcpp::Node(nodeName) {
	this->publisher = this->create_publisher<std_msgs::msg::Empty>(topicName);
	this->timer = this->create_wall_timer(rate, std::bind(&DWMNode::publishData, this));

	this->dwm = new DWM(115, 20);
	this->dwm->initialize();
}

DWMNode::~DWMNode() {
	delete this->dwm;
}

void DWMNode::publishData() {
	auto message = std_msgs::msg::Empty();

	uint8_t * data = this->dwm->receiveFrame();
	std::cout << "DWM1000: data: " << data << std::endl;

	std::cout << "DWM1000: publishing (empty) message\n";
	this->publisher->publish(message);
}
