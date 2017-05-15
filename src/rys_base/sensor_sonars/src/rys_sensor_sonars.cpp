#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "Sonars.h"
#include "rys_interfaces/msg/sonars.hpp"

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);
	auto node = rclcpp::node::Node::make_shared("rys_node_motors_controller");
	auto sensorsPublisher = node->create_publisher<rys_interfaces::msg::Sonars>("rys_sensor_sonars", rmw_qos_profile_sensor_data);

	auto message = std::make_shared<rys_interfaces::msg::Sonars>();

	std::cout << "Initializing sonars...\n";
	Sonars sonars;

	std::cout << "Working!\n";

	rclcpp::rate::WallRate loopRate(30);
	while(rclcpp::ok()) {
		unsigned int front, back, top;
		try {
			// This can throw
			sonars.getDistances(&front, &back, &top);

			// This should not
			message->front = front;
			message->back = back;
			message->top = top;

			std::cout << "Readings: " << front << " " << back << " " << top << std::endl;
			sensorsPublisher->publish(message);
		} catch (std::string & error) {
			std::cout << "Error reading distances from sonars: " << error << std::endl;
		}

		rclcpp::spin_some(node);
		loopRate.sleep();
	}
}
