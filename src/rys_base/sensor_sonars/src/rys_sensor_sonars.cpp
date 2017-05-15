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
		sonars.getDistances(&front, &back, &top);
		message->front = front;
		message->back = back;
		message->top = top;

		sensorsPublisher->publish(message);
		rclcpp::spin_some(node);
		loopRate.sleep();
	}
}
