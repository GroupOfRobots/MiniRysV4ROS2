#include <iostream>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "DWMNode.hpp"

DWMNode::DWMNode(const char * nodeName, const char * topicName, std::chrono::milliseconds rate) : rclcpp::Node(nodeName) {
	// this->dwm = new DWM(115, 20);
	// this->dwm->initialize(true);

	this->publisher = this->create_publisher<rys_interfaces::msg::LocationRanges>(topicName);
	this->timer = this->create_wall_timer(rate, std::bind(&DWMNode::publishData, this));
}

DWMNode::~DWMNode() {
	delete this->dwm;
}

void DWMNode::publishData() {
	auto message = rys_interfaces::msg::LocationRanges();

	// TODO: implement ranging once beacons (anchors) work
	// float ranges[4] = this->dwm->readRange();
	message.range0 = 0.0;
	message.range1 = 0.0;
	message.range2 = 0.0;
	message.range3 = 0.0;
	// std::cout << "DWM1000: publishing distances: " << message.range0 << std::endl;

	this->publisher->publish(message);
}
